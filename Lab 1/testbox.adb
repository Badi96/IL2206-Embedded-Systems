with Ada.Text_IO; use Ada.Text_IO;
with Semaphores; use Semaphores;

procedure Test_Box is
   Box: CountingSemaphore(3,0);
   
   task type Wait_task;
   task type Signal_task;
   
   task body Wait_Task is
   begin
      Box.Wait;
      
   
   
