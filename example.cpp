//#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include "AccelStepper.hpp"
//#define NUM_THREADS    5

/*
void *PrintHello(void *threadid)
{
  long tid;
  tid = (long)threadid;
  sleep(3);
  printf("Hello World! It's me, thread #%ld!\n", tid);

  pthread_exit(NULL);
}
*/

 AccelStepper stepper1(4, 2, 3, 4, 5);

int main (int argc, char *argv[])
{
//  pthread_t threads[NUM_THREADS];
//  int rc;
//  long t;
/*
  for(t=0; t<NUM_THREADS; t++){
      printf("In main: creating thread %ld\n", t);
      rc = pthread_create(&threads[t], NULL, PrintHello, (void *)t);
      if (rc){
        printf("ERROR; return code from pthread_create() is %d\n", rc);
      }
      sleep(1);
  }
  */


  stepper1.setMaxSpeed(500.0);
  stepper1.setAcceleration(200.0);
  stepper1.setSpeed(400);
  stepper1.moveTo(0);
  stepper1.runToPosition();
  for(;;)
  {
	  stepper1.moveTo(-1000);
	  stepper1.runToPosition();
  }

  /* Last thing that main() should do */
//  pthread_exit(NULL);
  return 0;
}


