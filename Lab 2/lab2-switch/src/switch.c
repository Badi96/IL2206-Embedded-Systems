// File: TwoTasks.c 

#include <stdio.h>
#include "includes.h"
#include <string.h>
#include "system.h"
#include "alt_types.h"
#include <time.h>
//#include <sys/alt_timestamp.h>
//#include <sys/alt_cache.h>
#include <altera_avalon_performance_counter.h>

#define DEBUG 1

/* Definition of Task Stacks */
/* Stack grows from HIGH to LOW memory */
#define   TASK_STACKSIZE       2048
OS_STK    task1_stk[TASK_STACKSIZE];
OS_STK    task2_stk[TASK_STACKSIZE];
OS_STK    stat_stk[TASK_STACKSIZE];

// Declaration of semaphores
OS_EVENT *semaphore1;
OS_EVENT *semaphore2;
OS_EVENT *semaphore3;

// States for tasks
int task1state = 0;
int task2state = 0;

//OS_TMR *Tmr;

alt_u64 time1,time2,cycles;

/* Definition of Task Priorities */
#define TASK1_PRIORITY      6  // highest priority
#define TASK2_PRIORITY      7
#define TASK_STAT_PRIORITY 12  // lowest priority 

void printStackSize(char* name, INT8U prio) 
{
  INT8U err;
  OS_STK_DATA stk_data;
    
  err = OSTaskStkChk(prio, &stk_data);
  if (err == OS_NO_ERR) {
    if (DEBUG == 1)
      printf("%s (priority %d) - Used: %d; Free: %d\n", 
	     name, prio, stk_data.OSUsed, stk_data.OSFree);
  }
  else
    {
      if (DEBUG == 1)
	printf("Stack Check Error!\n");    
    }
}

float microseconds(int ticks)
{
  return (float) 1000000 * (float) ticks / (float) alt_get_cpu_freq();
}

/* Prints a message and sleeps for given time interval */
void task1(void* pdata)
{
  INT8U err;
  while (1)
    { 
	  time1=perf_get_total_time ((void*)PERFORMANCE_COUNTER_BASE);
	  PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
	  
	  
	  OSSemPend(semaphore1, 0, &err); // Task 1 wants to have an exclusive access (Wait)
      //printf("Task 0 - State %d\n",task1state);
	  if (task1state == 0){
	  task1state++;
      //printf("Task 0 - State %d\n",task1state);
	  }
	  else{
      task1state--;
      //printf("Task 0 - State %d\n",task1state);
      }
	  OSSemPost(semaphore2); // Task 1 signalerar till semaphore 2 (Task2 can now proceed)
     	//OSTimeDlyHMSM(0, 0, 0, 11); // Context Switch to next task
				   //* Task will go to the ready state
				  // * after the specified delay
				  // */
    }
}

/* Prints a message and sleeps for given time interval */
void task2(void* pdata)
{
  INT8U err;
  float time;
  int i = 0;
  float sum_time = 0.0;
  float average_time = 0.0;
  while (1)
    { 
   	  time2=perf_get_total_time ((void*)PERFORMANCE_COUNTER_BASE);
	  PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
	  cycles = time2 - time1;	
	  time=microseconds(cycles);
	  

	  
      if(i<10)
	  {
      		if(time<40.0 && time>39.0)
	  	    {
				i+=1;
				sum_time=time+sum_time;
	        }
	  }
	  else
	  {
		average_time=sum_time/i;
		printf("Averaged time for 10 iterations: %.6f\n", average_time);
		i=0;
		sum_time=0.0;
		average_time=0.0;
	  }	

	  
	  OSSemPend(semaphore2, 0, &err); // Task 2 wants to have an exclusive access (Wait)
      //printf("Task 1 - State %d\n",task2state);
	  if (task2state == 0){
	  task2state++;
	  //printf("Task 1 - State %d\n",task2state);
	  }
	  else{
      task2state--;
      //printf("Task 1 - State %d\n",task2state);
      }
	  OSSemPost(semaphore1); // Task 2 signalerar till semaphore 3 (Task3 can now proceed)
      //OSTimeDlyHMSM(0, 0, 0, 4);
    }
}

/* Printing Statistics */
void statisticTask(void* pdata)
{
  INT8U err;
  while(1)
    {
	  OSSemPend(semaphore3, 0, &err); // Task 3 wants to have an exclusive access (Wait)
      printStackSize("Task1", TASK1_PRIORITY);
      printStackSize("Task2", TASK2_PRIORITY);
      printStackSize("StatisticTask", TASK_STAT_PRIORITY);
	  OSSemPost(semaphore1); // Task 3 signalerar till semaphore 1 (Task1 can now proceed)
    }
}

/* The main function creates two task and starts multi-tasking */
int main(void)
{

  semaphore1 = OSSemCreate(1); 
  semaphore2 = OSSemCreate(0);
  semaphore3 = OSSemCreate(0);
  printf("Lab 3 - Two Tasks\n");

  OSTaskCreateExt
    ( task1,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task1_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK1_PRIORITY,               // Desired Task priority
      TASK1_PRIORITY,               // Task ID
      &task1_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
      );
	   
  OSTaskCreateExt
    ( task2,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task2_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK2_PRIORITY,               // Desired Task priority
      TASK2_PRIORITY,               // Task ID
      &task2_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                       
      );  

  if (DEBUG == 1)
    {
      OSTaskCreateExt
	( statisticTask,                // Pointer to task code
	  NULL,                         // Pointer to argument passed to task
	  &stat_stk[TASK_STACKSIZE-1],  // Pointer to top of task stack
	  TASK_STAT_PRIORITY,           // Desired Task priority
	  TASK_STAT_PRIORITY,           // Task ID
	  &stat_stk[0],                 // Pointer to bottom of task stack
	  TASK_STACKSIZE,               // Stacksize
	  NULL,                         // Pointer to user supplied memory (not needed)
	  OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
	  OS_TASK_OPT_STK_CLR           // Stack Cleared                              
	  );
    }  

  OSStart();
  return 0;
}
