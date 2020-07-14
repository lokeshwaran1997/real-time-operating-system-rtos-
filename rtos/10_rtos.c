
// Submit only two .c files in an e-mail to me (not in a compressed file):
// 10_rtos.c//LOKESHWARAN RAJENDRAN
// 10_tm4c123gh6pm_startup_ccs.c
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Green:  PE2
// Yellow: PE3
// Orange: PE4
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <stdio.h>
#define alpha 0.9
uint32_t totaltime=0;
/********RELATED TO STRING PARSING *******/
unsigned char str[81];    //global variable str
uint8_t argc;//related to string parsing for UART
uint8_t pos[81];
unsigned char type[81];
/*****************************************/
uint8_t minargs;
char b;
uint32_t getSp();
uint8_t getSVC();
//Used for register saving during SVC call
uint32_t getR0();
uint32_t getR1();
uint32_t getR2();
uint32_t r0;
uint32_t r1;
uint32_t r2;
volatile uint32_t time_out = 0;
/***Supervisor call macros*****/
#define  SVC_yield 1
#define  SVC_sleep 2
#define  SVC_wait 3
#define  SVC_post 4
#define  SVC_destroy 5
#define  SVC_CREATETHREAD 6
#define SVC_CPU 7
/******************************/
uint8_t p= 1;  //priority scheduling
uint8_t preempt=1;
uint8_t pre=1;//preemption flag for every 1 millisecond interrupt
uint8_t c=0;
uint8_t pi=1;

// REQUIRED: corrected bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))
//PUSHBUTTONS
#define PB0   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PB1   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PB2   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PB3   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define PB4   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;      //no of process waiting
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread
uint8_t state;
struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // -8=highest to 7=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    int8_t skipvalue;              //used for priority scheduling
    uint32_t percentage;             //this is used for ps command percentage
    uint32_t start_time;             //using for process start time
    uint32_t runtime;                //difference of time for process
    uint32_t totaltime;
} tcb[MAX_TASKS];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
//    // REQUIRED: initialize systick for 1ms system timer
//    NVIC_ST_RELOAD_R |= 0x9C3F;    //(40Mhz/1000)-1=39999 value
//    NVIC_ST_CURRENT_R |=0;
//    NVIC_ST_CTRL_R |= 0x00000007;  //clk enable,interrupt enable,system clock set
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{

           uint8_t i,flag=0;
           bool ok;
           char* print[5];
           static uint8_t task = 0xFF;
           ok = false;
           while (!ok)
           {
               task++;
               if (task >= MAX_TASKS-1)
                   task = 0;

               if(p==1) //Priority Scheduling
               {
                   if(flag==0)
                   {
                     for(i=0;i<9;i++)
                      {
                       tcb[i].skipvalue=tcb[i].currentPriority;
                      }
                      flag=1;
                    }
                        if(tcb[task].skipvalue==0)
                          {
                             ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
                             tcb[task].skipvalue=tcb[task].currentPriority;
//                             itoa(task,print);
//                             putsUart0(print);
                           }
                        else
                           {
                             tcb[task].skipvalue--;
                            }
                 }
                else  //round robin mode based on P value
                 {
                   ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
                 }
            }

           return task;
}
void rtosStart()
{
    // REQUIRED: add code to call the first task to be run
    uint32_t a;
    _fn fn;

    taskCurrent = rtosScheduler();
    a=getSp();
    setSp(tcb[taskCurrent].sp);
    preempt=1;
    tcb[taskCurrent].state = STATE_READY;
    // Add code to initialize the SP with tcb[task_current].sp;
    fn=(_fn) tcb[taskCurrent].pid;
    // REQUIRED: initialize systick for 1ms system timer
        NVIC_ST_RELOAD_R |= 0x9C3F;    //(40Mhz/1000)-1=39999 value
        NVIC_ST_CURRENT_R |=0;
        NVIC_ST_CTRL_R |= 0x00000007;  //clk enable,interrupt enable,system clock set
    tcb[taskCurrent].start_time = NVIC_ST_CURRENT_R;
    (*fn)();


}

bool createThread(_fn fn, char name[], int priority)
{
    __asm("   MOV R3,R0");       //moving the parameters into these regs which we can use in svc create thread
    __asm("   MOV R4,R1");
    __asm("   MOV R5,R2");


    __asm("   SVC #6");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm("   SVC #5");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{

}

struct semaphore* createSemaphore(uint8_t count)
{

    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
    }
    return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    __asm("   SVC #1");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
  __asm("   SVC #2");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm("   SVC #3");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm("   SVC #4");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint32_t total = 0;
    int i=0;
    tcb[taskCurrent].runtime +=  tcb[taskCurrent].start_time;
     for(i=0;i<MAX_TASKS;i++)
     {

         if (tcb[i].state==3)
             {
                tcb[i].ticks--;
                if (tcb[i].ticks == 0)
                {
                tcb[i].state=2;
                }
             }

     }

     if(time_out >= 1000)
     {
     time_out = 0;
    for( i=0; i<MAX_TASKS;i++)
    {
    tcb[i].totaltime = (alpha * tcb[i].totaltime) + ((1-alpha)*tcb[i].runtime);
    tcb[i].runtime = 0;
    total += tcb[i].totaltime;
    }
     }
    else
        time_out++;
     tcb[taskCurrent].start_time = NVIC_ST_CURRENT_R;
     if((preempt && pre)==1)
         NVIC_INT_CTRL_R |= 0x10000000;        //PENDsV IS SET
}
// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{	//STEP 1 FOR CONTEXT SWITCHING
	//SAVING SPECIAL REGISTERS AS PER M4F STANDARD FOR CURRENT PROCESS
	//R4-R11 ARE USER SAVE REGISTERS
	//R0-3 AND R12,LR,PC,XPSR ARE SAVED BY HARDWARE AUTOMATICALLY
	//ORDER OF PUSHING 
	//1.HARDWARE PUSH
	//2.COMPILER PUSH 
	//3.USER PUSH
        __asm("   PUSH {R4}");
        __asm("   PUSH {R5}");
        __asm("   PUSH {R6}");
        __asm("   PUSH {R7}");
        __asm("   PUSH {R8}");
        __asm("   PUSH {R9}");
        __asm("   PUSH {R10}");
        __asm("   PUSH {R11}");

        tcb[taskCurrent].sp=getSp();//SAVE SP OF CURRENT THREAD
        setSp(a);//SWITCH TO SYSTEM STACK
        if(NVIC_ST_CURRENT_R < tcb[taskCurrent].start_time)
           tcb[taskCurrent].runtime += tcb[taskCurrent].start_time - NVIC_ST_CURRENT_R;
	//STEP 2---->CALL SCHEDULER
        taskCurrent = rtosScheduler();
        if(((NVIC_INT_CTRL_R>>26)&0x01) == 0)
        tcb[taskCurrent].start_time = NVIC_ST_CURRENT_R;
	//STEP 3------>CALL NEW PROCESS
	//IF CASE EXECUTES, IF NEW PROCESS IS ALREADY RUN
        if (tcb[taskCurrent].state == STATE_READY)
        {
            setSp(tcb[taskCurrent].sp);
        }
	//ELSE CASE RUNS IF NEW PROCESS IS UNRUN STATE
	//SO WE NEED TO SEED THE STACK WITH GARBAGE TO PROVIDE ILLUSION TO COMPILER THAT IT HAS ALREADY RUN BEFORE
	//ORDER FOLLOWS AS PER M4F ARCHITECTURE STANDARD TO PUSH ON TO THE STACK
        else
        {
            tcb[taskCurrent].state = STATE_READY;
            stack[taskCurrent][255]=0x01000000; //xpsr
           stack[taskCurrent][254]= (uint32_t*) tcb[taskCurrent].pid; //pc
           stack[taskCurrent][253]=1; //R12
           stack[taskCurrent][252]=1; //R3
           stack[taskCurrent][251]=2; //R2
           stack[taskCurrent][250]=3; //R1
           stack[taskCurrent][249]=7; //R0
           stack[taskCurrent][248]=4; //R4
           stack[taskCurrent][247]=0xFFFFFFF9; //LR
           stack[taskCurrent][246]=10; 
           stack[taskCurrent][245]=11; 
           stack[taskCurrent][244]=12; 
           stack[taskCurrent][243]=13; 
           stack[taskCurrent][242]=14; 
           stack[taskCurrent][241]=15; 
           stack[taskCurrent][240]=6; 
           stack[taskCurrent][239]=13; 
           stack[taskCurrent][238]=14; 
           stack[taskCurrent][237]=15; 
           stack[taskCurrent][236]=6; 
           tcb[taskCurrent].sp=&stack[taskCurrent][236];//SETTING THE SP WITH THE BASE OF THE STACK

           setSp(tcb[taskCurrent].sp);

        }
	
        __asm("   POP    {R4-R11}");//POP EVERYTHING

}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives

void svCallIsr()
{
	  uint32_t d;
          r0=getR0();
//          r1=getR1();
//          r2=getR2();
          struct semaphore* pSemaphore;
          if(taskCurrent == 1)
              r0 = r0;
          d=getSVC();
    switch(d)
    {
       case SVC_yield:
     {
        NVIC_INT_CTRL_R |= 0x10000000;        //PENDsV IS SET
        break;
      }
       case SVC_sleep:
     {
        tcb[taskCurrent].ticks=r0;
        tcb[taskCurrent].state=STATE_DELAYED;
        NVIC_INT_CTRL_R |= 0x10000000;        //PENDsV IS SET
        break;
       }
       case SVC_wait:
       {

           pSemaphore=r0;
           if (pSemaphore->count > 0)
           {
               pSemaphore->count--;
           }
           else
           {
              pSemaphore->processQueue[pSemaphore->queueSize++]=tcb[taskCurrent].pid;
               tcb[taskCurrent].state=STATE_BLOCKED;
               tcb[taskCurrent].semaphore=pSemaphore;
               NVIC_INT_CTRL_R |= 0x10000000;        //PENDsV IS SET
           }
           if(pi==1 && tcb[taskCurrent].state == STATE_BLOCKED)//FOR PRIORITY INVERSION 
           {
               int m;
               for(m=0;m<MAX_TASKS;m++)
               {
                if(tcb[m].semaphore==pSemaphore)
                  {
                     if(tcb[m].state == STATE_READY)
                     if(tcb[taskCurrent].currentPriority < tcb[m].currentPriority)
                        {
                         tcb[m].currentPriority = tcb[taskCurrent].currentPriority;
                         break;
                        }
                    }
              }
           }
           break;

       }
       case SVC_post:
       {
           uint8_t i;
           pSemaphore=r0;
           if(pSemaphore->queueSize > 0)
           {
               for(i=0;i<MAX_TASKS;i++)
               {

                   if (tcb[i].pid==pSemaphore->processQueue[0])
                       tcb[i].state=STATE_READY;
                }
               for(i=0;i<MAX_QUEUE_SIZE-1;i++)
                {

                   pSemaphore->processQueue[i]= pSemaphore->processQueue[i+1];
                 }
                   pSemaphore->queueSize--;
            }
           else
            {
               pSemaphore->count++;
             }
           if(pi==1)//FOR PRIORITY INVERSION
           {
               tcb[taskCurrent].currentPriority=tcb[taskCurrent].priority+8;
           }
           break;
        }
       case SVC_destroy:
       {
           int p;
           p=r0;
           int n,i,j;

               struct semaphore* pSemaphore;
               for(n=0;n<10;n++)
               {
                   if(tcb[n].pid == p)
                   {
                   pSemaphore=tcb[n].semaphore;
                   tcb[n].state=STATE_INVALID;
                   tcb[n].pid=0;
                   for(j=0;j<5;j++)
                   {
                          pSemaphore->processQueue[j]=0;

                              for(i=0;i<MAX_QUEUE_SIZE-1;i++)
                                  {

                                     pSemaphore->processQueue[i]= pSemaphore->processQueue[i+1];
                                   }
                                     pSemaphore->queueSize--;
                   }
                  }
               }
             break;
            }
       case SVC_CREATETHREAD:
       {
           void* fn;
           fn=getR3();
           char *name;
           name= getR4();
          int priority;
          priority=getR5();
           bool ok = false;
             uint8_t i = 0;
             bool found = false;
             // REQUIRED: store the thread name
             // add task if room in task list
             if (taskCount < MAX_TASKS)
             {
                 // make sure fn not already in list (prevent reentrancy)
                 while (!found && (i < MAX_TASKS))
                 {
                     found = (tcb[i++].pid ==  fn);
                 }
                 if (!found)
                 {
                     // find first available tcb record
                     i = 0;
                     while (tcb[i].state != STATE_INVALID) {i++;}
                     tcb[i].state = STATE_UNRUN;
                     tcb[i].pid = fn;
                     tcb[i].sp = &stack[i][255];
                     tcb[i].priority =priority ;
                     tcb[i].currentPriority = priority+8;
                     tcb[i].runtime = 0;
                     tcb[i].totaltime = 0;
                     strcpy(tcb[i].name,name);
                     // increment task count
                     taskCount++;
                     ok = true;
                 }
             }
             // REQUIRED: allow tasks switches again
             return ok;
             break;
       }
       case SVC_CPU:
       {
           totaltime=0;
           int i,j,k;
           int d2,d3;
           char* name;
           char print[6],print2[6];
           putsUart0("\r\n");
           putsUart0("it comes here");
           putsUart0("\r\nprocessname      pidno      priority\r\n");
          for(i=0;i<9;i++)
          {
             putsUart0(tcb[i].name);
               putsUart0("      \t   ");
               d2=tcb[i].pid;
               itoa(d2,print);
               putsUart0(print);
               putsUart0("  \t   ");
               d3=tcb[i].priority;
               itoa(d3,print);
               putsUart0(print);
               putsUart0("\r\t\t\t\t\t\t");
                              tcb[i].percentage=(tcb[i].totaltime / 4000) ;
                              itoa(tcb[i].percentage/100,print);
                              itoa(tcb[i].percentage%100,print2);
                             putsUart0(print);
                             putcUart0('.');
                             putsUart0(print2);
                             putsUart0("\r\n");

          }
          // cpu percentage

           break;
       }
    default:
        break;
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
  // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
  //           5 pushbuttons, and uart
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
       SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

       // Set GPIO ports to use AP (not needed since default configuration -- for clarity)
       SYSCTL_GPIOHBCTL_R = 0;

       // Enable GPIO port A,E and F peripherals
          SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE;
       //Configuring LED's
          GPIO_PORTF_DIR_R = 0x04;      //bit PF2 is set for blue_led
          GPIO_PORTF_DR2R_R = 0x04;     //drive strength to 2mA
          GPIO_PORTF_DEN_R = 0x04;     //enabling led

          GPIO_PORTE_DIR_R = 0x1E;      //bit PE1,PE2,PE3,PE4 is set for RED_led,GREEN,YELLOW,ORANGE
          GPIO_PORTE_DR2R_R = 0x1E;     //drive strength to 2mA
          GPIO_PORTE_DEN_R = 0x1E;     //enabling led

          //ENABLING PUSH_BUTTONS
         // GPIO_PORTA_DIR_R = 0x7C;      //bit PA2,PA3,PA4,PA5,PA6
         // GPIO_PORTA_DR2R_R = 0x7C;     //drive strength to 2mA
          GPIO_PORTA_DEN_R = 0x7C;      // Enable push_buttons
          GPIO_PORTA_PUR_R = 0x7C;      // enable internal pull-up for push button

          // Configure UART0 pins
             SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
             GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
             GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
             GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

             // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
             UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
             UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
             UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
             UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
             UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
             UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module



}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                              // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}



// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    uint8_t j=0;
    uint8_t j0=0;
    uint8_t j1=0;
    uint8_t j2=0;
    uint8_t j3=0;
    uint8_t j4=0;

           if(PB0==0)
              {
                  j0=1;
              }
              else
              {
                  j0=0;
              }
             if(PB1==0)
              {
                  j1=2;
              }
             else
             {
                 j1=0;
             }

              if(PB2==0)
              {
                  j2=4;
              }
              else
              {
                  j2=0;
              }

              if(PB3==0)
              {
                  j3=8;
              }
              else
              {
                  j3=0;
              }

              if(PB4==0)
              {
                  j4=16;
              }
              else
              {
                  j4=0;
              }

                       j=j1+j2+j3+j4+j0;

   return j;


}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
uint32_t getSp()
{
    __asm("   MOV  R0, SP");

}
void setSp(uint32_t x)
{

   // __asm("   ADD  SP, #8");
    __asm("   MOV  SP, R0");
    __asm("   SUB  SP, #8");
}

uint8_t getSVC()
{
        __asm("   MOV  R7, SP");
        __asm("   ADD  R7,#72");
        __asm("   LDR  R0,[R7]");
        __asm("   SUB  R0,#2");
        __asm("   LDR  R0,[R0]");
        __asm("   AND  R0,#0xFF");

}
//UART BLOCKING FUNCTIONS
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}
// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
     while (UART0_FR_R & UART_FR_RXFE)
     {
     yield();
     }
           return UART0_DR_R & 0xFF;


}

//this thread is for idle2
void idle2()
{

    while(true)
    {
        YELLOW_LED = 1;
        waitMicrosecond(1000);
        YELLOW_LED = 0;
        yield();
    }
}
// Added steps 2,3 and 4 in here
char getchaar()
{
uint8_t count=0;
while(1)
{
    b=getcUart0();

       if(b==0x8)                   //backspace
       {
           if(count>0)
           {
               count=count-1;
           }
               else
               {
                  b= getcUart0();
                }
       }
       else if(b==0x0D)                       // carriage return
           {
               count=0;
               break;
           }

       else
               {
               str[count]=tolower(b);
               count=count+1;
               }
        }
    if(count==81)
    {
        count=0;
    }

return b;
}

char parsestr();
char parsestr()
{
    uint16_t i;
    uint16_t j=-1;
     uint16_t Length;
    uint8_t flag=0;
    Length=strlen(str);
    for(i=0;i<Length;i++)
    {
     if(isspace(str[i]))
     {
       flag=0;
     }
     if((isalpha(str[i])|| isdigit(str[i])) && (flag==0))
     {
         j=j+1;
         flag=1;
         if((str[i]>='a' && str[i]<='z') || (str[i]>='A' && str[i]<='Z'))
         {
             pos[j]=i;
             type[j]='alpha';
         }
         else if(str[i]>=48 && str[i]<=57)
         {
             pos[j]=i;
             type[j]='n';
         }
     }
    }

    argc=strlen(type);
    putsUart0("\n");
    for(i=0;i<Length;i++)
    {
        if((isspace(str[i]))||(str[i]>=33 && str[i]<=47) || (str[i]>=58 && str[i]<=64) || (str[i]>=91 && str[i]<=96)||(str[i]>=123 && str[i]<=126))
         str[i]='\0';
    }
    for(i=0;i<argc;i++)
    {
        putsUart0("\r\n");
        putsUart0(&str[pos[i]]);
    }

 }

bool iscommand(char* verb,uint8_t minargs)
{

if((strcmp(verb,&str[pos[0]])==0)  && argc>=minargs)
        {
            return true;
        }
else
{
    return false;
}
}

uint32_t getR0()
{

}
//uint32_t getR1()
//{
//    __asm("   MOV   R0,R1");
//}
//uint32_t getR2()
//{
//    __asm("   MOV   R0,R2");

//}
//this functions are used for create thread in SVC
uint32_t getR3()
{
    __asm("   MOV   R0,R3");
}
uint32_t getR4()
{
    __asm("   MOV   R0,R4");
}
uint32_t getR5()
{
    __asm("   MOV   R0,R5");
}
//this itoa() function is referenced from https://clc-wiki.net/wiki/K&R2_solutions
void itoa(int n,char s[]);
void reverse(char s[]);
void itoa(int n,char s[])
{
    int i,sign;
    sign=n;
    i=0;
    do{
        s[i++]=abs(n%10) + '0';
    }while(n/=10);
    if(sign<0)
        s[i++]='-';
    s[i]='\0';
    reverse(s);

}
void reverse(char s[])
{
    int c,i,j;
    for(i=0,j=strlen(s)-1;i<j;i++,j--)
    {
        c=s[i];
        s[i]=s[j];
        s[j]=c;
    }
}
// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{

    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
             RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            createThread(flash4Hz, "Flash4Hz", 0);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

void shell()
{
    char d;
    uint8_t i;
    while (true)
    {
        // REQUIRED: add processing for the shell commands through the UART here
            putsUart0("\n\r");
            putsUart0("\r\n Enter the string:");
            d=getchaar();
            putsUart0("\r\n");
            putsUart0("\r\nThe output is...");
            parsestr();
            if(iscommand("scheduler",2)==true)
            {
            putsUart0("\r\n");
            putsUart0("scheduler on");
            p=1;

            for(i=0;i<81;i++)
            {
                pos[i]=0;
                type[i]=0;
                str[i]='\0';
                argc=0;
            }
           }
            if(iscommand("sched",2)==true)
                       {
                       putsUart0("\r\n");
                       putsUart0("scheduler off");
                       p=0;

                       for(i=0;i<81;i++)
                       {
                           pos[i]=0;
                           type[i]=0;
                           str[i]='\0';
                           argc=0;
                       }
                      }
            if(iscommand("pidof",1)==true)
                         {
                                    int i=0;
                                    char buffer[20];
                                    int pid;
                                  for(i=0;i<10;i++)
                                  {
                                     if (strcasecmp(tcb[i].name,&str[pos[1]])==0)
                                     {
                                         pid=tcb[i].pid;
                                          itoa(pid,buffer);
                                          putsUart0("\r\n");
                                          putsUart0(buffer);
                                      }
                                  }
                           }
                                  if(iscommand("preemption",1)==true)
                                      {
                                                         putsUart0("\r\n");
                                                         putsUart0("preemption on");
                                                         pre=1;
                                                         preempt=1;
                                                         for(i=0;i<81;i++)
                                                         {
                                                             pos[i]=0;
                                                             type[i]=0;
                                                             str[i]='\0';
                                                             argc=0;
                                                         }
                                       }
                                  if(iscommand("preempt",0)==true)
                                  {
                                    putsUart0("\r\n");
                                    putsUart0("preemption off");
                                    pre=0;
                                    preempt=0;

                                    for(i=0;i<81;i++)
                                      {
                                       pos[i]=0;
                                       type[i]=0;
                                       str[i]='\0';
                                       argc=0;
                                        }
                                    }
                                  if(iscommand("reboot",0)==true)
                                                               {
                                                                   putsUart0("\r\n");
                                                                   putsUart0("reboot happens");

                                                                   for(i=0;i<81;i++)
                                                                    {
                                                                         pos[i]=0;
                                                                         type[i]=0;
                                                                         str[i]='\0';
                                                                         argc=0;
                                                                    }
                                                                   ResetISR();
                                                                }
                                  if(iscommand("ipcs",0)==true)
                                  {
                                      char buffer[20];
                                      int z,z1,z2,z3;
                                      putsUart0("\r\n");

                                      putsUart0("list of semaphores count queueSize waitingPid\r\n");
                                     // putsUart0("   KeyPressed     KeyReleased    FlashReq    Resource\r\n");
                                      for(i=0;i<MAX_SEMAPHORES-1;i++)
                                      {
                                          switch(i)
                                          {
                                          case 0:{
                                                   putsUart0("keypressed");

                                                   }break;
                                          case 1:{
                                              putsUart0("keyreleased");
                                                 }break;
                                          case 2:{
                                              putsUart0("flashreq");
                                                 }break;
                                          case 3:{
                                              putsUart0("resource");
                                                 }break;

                                          }
                                         z=semaphores[i].count;
                                         itoa(z,buffer);
                                         putsUart0(" \t     ");
                                         putsUart0(buffer);
                                         z1=semaphores[i].queueSize;
                                         itoa(z,buffer);
                                         putsUart0(" \t     ");
                                         putsUart0(buffer);
                                         z2=semaphores[i].processQueue[0];
                                         itoa(z2,buffer);
                                         putsUart0("  \t    ");
                                         putsUart0(buffer);
                                         putsUart0("\r\n");

                                       }
                                      for(i=0;i<81;i++)
                                        {
                                          pos[i]=0;
                                          type[i]=0;
                                          str[i]='\0';
                                          argc=0;
                                        }
                                  }


                                      if(iscommand("pi",0)==true)                 //priority inheritance on
                                       {
                                           putsUart0("\r\n");
                                           putsUart0("Priority Inheritance On");
                                           pi=1;
                                           for(i=0;i<81;i++)
                                           {
                                             pos[i]=0;
                                             type[i]=0;
                                             str[i]='\0';
                                             argc=0;
                                            }
                                        }
                                      if(iscommand("pioff",0)==true)                    //priority inheritance off
                                       {
                                        putsUart0("\r\n");
                                        putsUart0("Priority Inheritance Off");
                                        pi=0;
                                        for(i=0;i<81;i++)
                                         {
                                           pos[i]=0;
                                           type[i]=0;
                                           str[i]='\0';
                                           argc=0;
                                          }
                                        }
                                      if(iscommand("kill",1)==true)         //kills a process
                                        {
                                            int i;
                                            putsUart0("\r\n");
                                            putsUart0("Terminated");
                                            for(i=0;i<10;i++)
                                          {
                                            if(strcasecmp(tcb[i].name,&str[pos[1]])==0)
                                                destroyThread(tcb[i].pid);
                                          }
                                            for(i=0;i<81;i++)
                                            {
                                             pos[i]=0;
                                             type[i]=0;
                                             str[i]='\0';
                                              argc=0;
                                               }
                                           }
                                      if(iscommand("ps",0)==true)         //process status cmd
                                       {
                                        putsUart0("\r\n");
                                        putsUart0("Process Status");
                                        __asm("   SVC #7");
                                        for(i=0;i<81;i++)
                                         {
                                           pos[i]=0;
                                           type[i]=0;
                                           str[i]='\0';
                                           argc=0;
                                           }
                                         }
            for(i=0;i<81;i++)
                   {
                       pos[i]=0;
                       type[i]=0;
                       str[i]='\0';
                       argc=0;
                     }

    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
  {
    bool ok;

    // Initialize hardware
    initHw();
    rtosInit();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1);
    keyReleased = createSemaphore(0);
    flashReq = createSemaphore(5);
    resource = createSemaphore(1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7);
    // Add other processes
   ok &= createThread(lengthyFn, "LengthyFn", 4);
    ok &= createThread(flash4Hz, "Flash4Hz", 0);
    ok &= createThread(oneshot, "OneShot", -4);
   ok &= createThread(readKeys, "ReadKeys", 4);
    ok &= createThread(debounce, "Debounce", 4);
    ok &= createThread(important, "Important", -8);
    ok &= createThread(uncooperative, "Uncoop", 2);
    ok &= createThread(shell, "Shell", 0);

    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}


