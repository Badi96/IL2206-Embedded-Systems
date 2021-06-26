#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"
//Included by us
//#include "alt_types.h"
//#include <time.h>
//#include <sys/alt_timestamp.h>
//#include <sys/alt_cache.h>


#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x01

/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001

/* LED Patterns */

#define LED_RED_LOAD 0x00000008 // Start led for extra load
#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear



// LEDS for positions
#define LED_RED_10 0x00020000
#define LED_RED_11 0x00010000
#define LED_RED_12 0x00008000
#define LED_RED_13 0x00004000
#define LED_RED_14 0x00002000
#define LED_RED_15 0x00001000

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0002 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonTask_Stack[TASK_STACKSIZE];
OS_STK SwitchTask_Stack[TASK_STACKSIZE];
OS_STK WatchdogTask_Stack[TASK_STACKSIZE];
OS_STK OverloadTask_Stack[TASK_STACKSIZE];
OS_STK ExtraloadTask_Stack[TASK_STACKSIZE];

// Task Priorities

#define STARTTASK_PRIO    5
#define WATCHDOGTASK_PRIO	 7
#define EXTRALOADTASK_PRIO	8
#define BUTTONTASK_PRIO		9
#define SWITCHTASK_PRIO    10 
#define VEHICLETASK_PRIO  	11
#define CONTROLTASK_PRIO  	13
#define OVERLOADTASK_PRIO	15




// Task Periods

#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define BUTTON_PERIOD  300
#define OVERLOAD_PERIOD  300
#define WATCHDOG_PERIOD  300
#define EXTRALOAD_PERIOD  300

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;
OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_Overload;

// Semaphores

OS_EVENT *VehicleSem;
OS_EVENT *ControlSem;
OS_EVENT *ButtonSem;
OS_EVENT *SwitchSem;
OS_EVENT *OverloadSem;
OS_EVENT *WatchdogSem;

// SW-Timer
OS_TMR *VehicleTmr;
OS_TMR *ControlTmr;
OS_TMR *ButtonTmr;
OS_TMR *SwitchTmr;
OS_TMR *WatchdogTmr;

/*
 * Types
 */
enum active {on = 2, off = 1};

enum active gas_pedal = off;
enum active brake_pedal = off;
enum active top_gear = off;
enum active engine = off;
enum active cruise_control = off; 

/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs
INT16S target_velocity;
alt_u32 ticks;
alt_u32 time_1;
alt_u32 time_2;
alt_u32 timer_overhead;


void SignalVehicleSem(void)
{
    OSSemPost(VehicleSem); // Vehicle task - GO
}

void SignalControlSem(void)
{
    OSSemPost(ControlSem); // Control task - GO
}


void SignalButtonSem(void)
{
    OSSemPost(ButtonSem);  // Button task - GO
}

void SignalSwitchSem(void)
{
	OSSemPost(SwitchSem); 	// Switch Task - GO
}

void SignalWatchdogSem(void)
{
	OSSemPost(WatchdogSem); 	// Watchdog Task - GO
}



int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */
  
  return delay;
}

static int b2sLUT[] = {0x40, //0
		       0x79, //1
		       0x24, //2
		       0x30, //3
		       0x19, //4
		       0x12, //5
		       0x02, //6
		       0x78, //7
		       0x00, //8
		       0x18, //9
		       0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);
  
  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8S target)
{
  int tmp = target;
  //printf("Target vlosicity: %d\n", tmp);
  printf(target);
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(target < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);
  
  out = int2seven(0) << 14 |
    out_high << 7  |
    out_low;
	
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);
	
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
	if(position <=50)
	{
		led_red = led_red & ~LED_RED_10;
		led_red = led_red & ~LED_RED_11;
		led_red = led_red & ~LED_RED_12;
		led_red = led_red & ~LED_RED_13;
		led_red = led_red & ~LED_RED_14;
		led_red = led_red & ~LED_RED_15;
	}
	if(position >= 0 && position < 400)
	{
		led_red = led_red | LED_RED_10;
	}
	if(position >= 400 && position < 800)
	{
		led_red = led_red | LED_RED_11;
	}
	if(position >= 800 && position < 1200)
	{
		led_red = led_red | LED_RED_12;
	}
	if(position >= 1200 && position < 1600)
	{
		led_red = led_red | LED_RED_13;
	}
	if(position >= 1600 && position < 2000)
	{
		led_red = led_red | LED_RED_14;
	}
	if(position >= 2000 && position < 2400)
	{
		led_red = led_red | LED_RED_15;
	}
}

/*
 * The function 'adjust_position()' adjusts the position depending on the
 * acceleration and velocity.
 */
INT16U adjust_position(INT16U position, INT16S velocity,
		       INT8S acceleration, INT16U time_interval)
{
  INT16S new_position = position + velocity * time_interval / 1000;

  if (new_position > 2400) {
    new_position -= 2400;
  } else if (new_position < 0){
    new_position += 2400;
  }
  
  show_position(new_position);
  return new_position;
}
 
/*
 * The function 'adjust_velocity()' adjusts the velocity depending on the
 * acceleration.
 */
INT16S adjust_velocity(INT16S velocity, INT8S acceleration,  
		       enum active brake_pedal, INT16U time_interval)
{
  INT16S new_velocity;
  INT8U brake_retardation = 200; // should be 200

  if (brake_pedal == off)
    new_velocity = velocity  + (float) (acceleration * time_interval) / 10000.0;
  else {
    if (brake_retardation * time_interval / 1000 > velocity)
      new_velocity = 0;
    else
      new_velocity = velocity - brake_retardation * time_interval / 1000;
  }
  
  return new_velocity;
}


void ButtonTask(void* pdata){
	INT8U err;
    //INT16S targetVelocity;
    INT16S* currentVelocity;
    int pressed_buttons;
	void* msg;
	printf("Button task created!\n");
	
	while(1)
	{
		OSSemPend(ButtonSem,0,&err); //Button task - NO GO
		pressed_buttons=buttons_pressed();
		if(pressed_buttons & CRUISE_CONTROL_FLAG)  //If button for cruise controll is pressed
		{ 
			if(cruise_control == on)
			{
				cruise_control = off;
				target_velocity = NULL;
				IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,0x00);
				led_green = led_green & ~LED_GREEN_2; // Cruise control LED off
				//printf("Cruise OFF\n");
			}
			else if(cruise_control == off && gas_pedal == off && brake_pedal == off)
			{	
				msg = OSMboxPend(Mbox_Velocity, 0, &err);
				currentVelocity=(INT16S*) msg;
			    if(top_gear==on && *currentVelocity >= 20) // If top gear is on and vel>=20m/s
				{
				cruise_control = on;
				show_target_velocity(*currentVelocity); // Shows target velocity
				target_velocity=*currentVelocity;
				//printf("Cruise ON\n");
				led_green = led_green | LED_GREEN_2; // Cruise control LED on
				}
			}
		}

		if(pressed_buttons & GAS_PEDAL_FLAG)  //If button for gas is pressed
		{
        	if(gas_pedal == off && engine == on) 
			{
            	gas_pedal = on;
				//printf("GAS_ON\n");
				cruise_control = off;
            	led_green = led_green | LED_GREEN_6; // Gas LED on
				led_green = led_green & ~LED_GREEN_2;// Cruise control LED off
				IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,0x00); // Turns of 7seg
        	}
   			else 
			{	
				if(gas_pedal = on)
				{
				gas_pedal = off;
				//printf("GAS_OFF\n");
            	led_green = led_green & ~LED_GREEN_6; // Gas LED off
            	}
        	}
    	}

		if(pressed_buttons & BRAKE_PEDAL_FLAG)  //If button for brake is pressed
		{
        	if(brake_pedal == off && gas_pedal == off) 
			{
            	brake_pedal = on;
				OSMboxPost(Mbox_Brake, (void *) brake_pedal);
				//printf("BRAKE_ON\n");
				cruise_control = off;
				gas_pedal = off;
            	led_green = led_green | LED_GREEN_4; // LED 4 on
				led_green = led_green & ~LED_GREEN_2;// LED 2 off
				led_green = led_green & ~LED_GREEN_6; // Gas LED off
				IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,0x00); // Turns of 7seg
        	}
   			else 
			{	
				
				brake_pedal = off;
				gas_pedal = off;
				OSMboxPost(Mbox_Brake, (void *) brake_pedal);
				//printf("BRAKE_OFF\n");
            	led_green = led_green & ~LED_GREEN_4; // LED 4 off
            	
        	}
    	}
	}
  
}

void SwitchTask(void* pdata) {
	printf("Switch task created!\n");
	INT8U err;
    //INT16S targetVelocity;
    INT16S* current_velocity;
	int switches;
	while(1) 
	{
		OSSemPend(SwitchSem, 0, &err); //Switch task - no Go
		switches = switches_pressed(); 
		if(switches & ENGINE_FLAG) //If switch for engine is on
		{
			if(engine == off)
			{
				engine = on;
				led_red = led_red | LED_RED_0; // Engine LED on
				//printf("ENGINE ON\n");
			}
		}
		else
		{
			if(engine == on)
			{
				current_velocity = OSMboxPend(Mbox_Velocity, 0, &err);
				if(*current_velocity == 0)
				{
					engine = off;	
					led_red = led_red & ~LED_RED_0; // Engine LED off
					//printf("ENGINE OFF\n");
				}
				else
				{
					//printf("VELOCITY IS TO HIGH/n");
				}
			}
				
		}
		
		
		if(switches & TOP_GEAR_FLAG) //If switch for top gear is on
		{
			if(top_gear == off)
			{
				led_red = led_red | LED_RED_1; // Top gear LED on
				top_gear = on;
				//printf("TOP GEAR ON\n");
			}
		}
		else
		{			
			if(top_gear == on)
			{	
				top_gear = off;	
				cruise_control = off;
				led_red = led_red & ~LED_RED_1;		  // Top gear LED off	
				led_green = led_green & ~LED_GREEN_2; // Cruise control LED off
				IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,0x00); // Turns of 7seg
				//printf("TOP GEAR OFF\n");
			}
		}
		
	}
}

void OverloadTask(void* pdata)
{
	printf("Overload task created!\n");
	INT8U err;
    INT8U send = 1;

        while (1)
        {
            OSSemPend(OverloadSem, 0, &err); // Oveload task - no GO
			//printf("BBBBBBBBBBBBBBBBB");
            OSMboxPost(Mbox_Overload, (void *) &send);
			
        }
}

void WatchdogTask(void* pdata)
{
	printf("Watchdog task created!\n");
	INT8U err;
	void* msg;
	INT8U* out;
	INT8U* tmp = 0;
	while(1)
	{
		OSSemPend(WatchdogSem, 0, &err); //Watchdog task - no Go
		//printf("AAAAAAAAAAAAAAAA");
		msg = OSMboxAccept(Mbox_Overload);
		out = (INT8U*) msg;
		//printf("AAAAAAAAAAAAAAAA %d\n", *out);
		if(*out == 1)
		{		
			printf("NO OVERLOAD\n");
		}	
		else
		{	
			printf("OVERLOAD\n");
		}
		OSSemPost(OverloadSem);
		tmp = 1; 
	}
}



void ExtraloadTask(void* pdata)
{	
  	INT16U extraload;
	INT16U factor = EXTRALOAD_PERIOD / 100; //  1% extra load = 3ms
  	INT32S switches;
	led_red = led_red | LED_RED_LOAD; // Engine LED on
  	while(1)
  	{
    	switches = (switches_pressed() >> 4) & 0x3F; // Integer corresponding to switches turned on
		//printf("AAAAAAAAAAAAAAAAA %d\n", switches);
    	extraload = (INT8U) switches << 1; // extra_load = 2 * <value of switches>
		
    	if(extraload > 100)
		{
        	extraload = 100; // 100% is max load
		}    	
		// set LEDR9-LEDR4 to 0 and then mask with switches
    	led_red = (led_red & ~0x3F0) | (switches << 4);
    	IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red);
    
    	extraload *= factor; // extraload is from 0-300ms
    	OSTimeDlyHMSM(0,0,0, (EXTRALOAD_PERIOD-extraload)); //time delay varyes from 0-300 ms
  	} 
}

/*
 * The task 'VehicleTask' updates the current velocity of the vehicle
 */
void VehicleTask(void* pdata)
{ 
  INT8U err;  
  void* msg;
  INT8U* throttle; 
  INT8S acceleration;  
  INT16U position = 0; 
  INT16S velocity = 0;    
  enum active brake_pedal = off;

  printf("Vehicle task created!\n");

  while(1)
    {
      err = OSMboxPost(Mbox_Velocity, (void *) &velocity);
      
      OSSemPend(VehicleSem, 0, &err);
      //OSTimeDlyHMSM(0,0,0,VEHICLE_PERIOD); 

      /* Non-blocking read of mailbox: 
	 - message in mailbox: update throttle
	 - no message:         use old throttle
      */
      msg = OSMboxPend(Mbox_Throttle, 1, &err); 
      if (err == OS_NO_ERR)
		{ 
			throttle = (INT8U*) msg;
			//printf("BOOOOOOBS %d\n", *throttle);
		}
      msg = OSMboxPend(Mbox_Brake, 1, &err); 
      if (err == OS_NO_ERR) 
	brake_pedal = (enum active) msg;
	
	

      // vehichle cannot effort more than 80 units of throttle
      if (*throttle > 80) *throttle = 80;

      // brakes + wind
      if (brake_pedal == off)
	{
		//printf("OFFFFFFFFFFF");
	  acceleration = (*throttle) - 1*velocity;
	 
	  if (400 <= position && position < 800)
	    acceleration -= 2; // traveling uphill 2
	  else if (800 <= position && position < 1200)
	    acceleration -= 4; // traveling steep uphill 4
	  else if (1600 <= position && position < 2000) 
	    acceleration += 4; //traveling downhill 4
	  else if (2000 <= position)
	    acceleration += 2; // traveling steep downhill 2
	}
      else
	{
		//printf("ONNNNNNNN");
		acceleration = -4*velocity;
	}
		
      
      printf("Position: %d m\n", position);
      printf("Velocity: %d m/s\n", velocity);
      printf("Accell: %d m/s2\n", acceleration);
      printf("Throttle: %d V\n", *throttle);

	  velocity = adjust_velocity(velocity, acceleration, brake_pedal, VEHICLE_PERIOD);
	  position = adjust_position(position, velocity, acceleration, VEHICLE_PERIOD);
      //position = position + velocity * VEHICLE_PERIOD / 1000;
      //velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;

	  //show_position(position); // Show position with 5 red LEDs

      if(position > 2400)
	position = 0;

      show_velocity_on_sevenseg((INT8S) velocity);
	  
    }
} 
 
/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  INT16S* error_velocity;
  void* msg_vel;
  void* msg_target;
  INT16S* current_velocity;

  printf("Control Task created!\n");

  while(1)
    {
      current_velocity = (INT16S*) OSMboxPend(Mbox_Velocity, 0, &err);
	  OSSemPend(ControlSem, 1, &err);

      if(target_velocity == NULL)
	  {
	   		error_velocity = 0;
	  }
	  else
	  {
	  		*error_velocity = target_velocity - *current_velocity;
			
	  }
	  
	
	  if(cruise_control == on)
	  {
			
			if(*error_velocity <= -4)
			{
				throttle = 0;
			}
			else if(*error_velocity >= 4)
			{
				throttle = 80;
			}
			else
			{
				throttle = 10;
			}
	  }
	  else
	  {
	  		if(gas_pedal == off)
	  		{
	  			throttle = 0;
	  		}
	 		else
	  		{
				throttle = 80;
	  		}
	  }

      err = OSMboxPost(Mbox_Throttle, (void *) &throttle);
	
      
      //OSTimeDlyHMSM(0,0,0, CONTROL_PERIOD);
	  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
	  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red);
    }
}





/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */
  
  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
		       delay,
		       alarm_handler,
		       context) < 0)
    {
      printf("No system clock available!n");
    }

  /* 
   * Create and start Software Timer 
   */
  ControlTmr = OSTmrCreate(0, (CONTROL_PERIOD/100), OS_TMR_OPT_PERIODIC, SignalControlSem, NULL, NULL, &err);
  VehicleTmr = OSTmrCreate(0, (VEHICLE_PERIOD/100), OS_TMR_OPT_PERIODIC, SignalVehicleSem, NULL, NULL, &err);
  ButtonTmr = OSTmrCreate(0, (BUTTON_PERIOD/100), OS_TMR_OPT_PERIODIC, SignalButtonSem, NULL, NULL, &err);
  SwitchTmr = OSTmrCreate(0, (BUTTON_PERIOD/100), OS_TMR_OPT_PERIODIC, SignalSwitchSem, NULL, NULL, &err);
  WatchdogTmr = OSTmrCreate(0, (WATCHDOG_PERIOD/100), OS_TMR_OPT_PERIODIC, SignalWatchdogSem, NULL, NULL, &err);

   OSTmrStart(ControlTmr, &err);
   OSTmrStart(VehicleTmr, &err);
   OSTmrStart(ButtonTmr, &err);
   OSTmrStart(SwitchTmr, &err);
   OSTmrStart(WatchdogTmr, &err);

  //Create semaphores
  VehicleSem = OSSemCreate(1);
  ControlSem = OSSemCreate(1);
  ButtonSem = OSSemCreate(1);
  SwitchSem = OSSemCreate(1);
  OverloadSem = OSSemCreate(1);
  WatchdogSem = OSSemCreate(1);
  
  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Overload = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
   
  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
			ControlTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			CONTROLTASK_PRIO,
			CONTROLTASK_PRIO,
			(void *)&ControlTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
			VehicleTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			VEHICLETASK_PRIO,
			VEHICLETASK_PRIO,
			(void *)&VehicleTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
			ButtonTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&ButtonTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			BUTTONTASK_PRIO,
			BUTTONTASK_PRIO,
			(void *)&ButtonTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);
  
  err = OSTaskCreateExt(
			SwitchTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&SwitchTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			SWITCHTASK_PRIO,
			SWITCHTASK_PRIO,
			(void *)&SwitchTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
			OverloadTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&OverloadTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			OVERLOADTASK_PRIO,
			OVERLOADTASK_PRIO,
			(void *)&OverloadTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
			WatchdogTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&WatchdogTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			WATCHDOGTASK_PRIO,
			WATCHDOGTASK_PRIO,
			(void *)&WatchdogTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);
  
  err = OSTaskCreateExt(
			ExtraloadTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&ExtraloadTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			EXTRALOADTASK_PRIO,
			EXTRALOADTASK_PRIO,
			(void *)&ExtraloadTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);
  
  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");
 
  OSTaskCreateExt(
		  StartTask, // Pointer to task code
		  NULL,      // Pointer to argument that is
		  // passed to task
		  (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
		  // of task stack 
		  STARTTASK_PRIO,
		  STARTTASK_PRIO,
		  (void *)&StartTask_Stack[0],
		  TASK_STACKSIZE,
		  (void *) 0,  
		  OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	 
  OSStart();
  
  return 0;
}
