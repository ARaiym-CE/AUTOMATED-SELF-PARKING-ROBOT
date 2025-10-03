// Init_Timers.c
// Runs on MSP432
// Provide functions that initialize Timers
// Prof. Christman
// 6/28/2023

/* These functions will be used for labs 3-8 in CPET253

There is an initialization function for each instantiation of TimerA;
TimerA0, TimerA1, TimerA2, TimerA3

*/

#include "msp.h"


// ------------TimerA0_Init------------
// Before initializing the timer, do the calculations:
//      Elapsed Time = clock counts * (N/Clock Frequency)
//      clock counts cannot exceed 65,535
//      N is the clock scaler

// Input: none
// Output: none
void TimerA0_Init(void){
    //**********First initialize TimerA0 for PWM
    //Since the motors are connected to P2.6 and P2.7, use TimerA0, compare blocks 3 & 4****
    //stop the timer
        TA0CTL &= ~0x0030;
    //choose SMCLK for the clock source
        TA0CTL |= 0x0200;
        TA0CTL &= ~0x0100;
    //choose clock divider of 2
        TA0CTL |= 0x0040; //0b0000 0010 0100 0000
    //Outmode 7: reset/set
        //TA0CCTL1 |= 0x00E0;
        TA0CCTL3 = 0x00E0;
        TA0CCTL4 = 0x00E0;

        TA0CCR0 = 59999;
}

// ------------TimerA1_Init------------
// Before initializing the timer, do the calculations:
//      Elapsed Time = clock counts * (N/Clock Frequency)
//      clock counts cannot exceed 65,535
//      N is the clock scaler

// Input: none
// Output: none
void TimerA1_Init(void){
  // In the TA1CTL register:
  // Disable timer for initialization
  // Choose a clock source
  // Select a clock scaler if needed
  //    Enable overflow interrupt if needed
  // In the TA1EX0 register:
  //    Select a second clock scaler if needed
  // For EACH CCRn being used, in the TA1CCTLn register
  // set the PWM outmode if needed
  //    Enable the CCRn match interrupt if needed
  // For PWM, Load TA1CCR0 with clock
  // counts for desired period
}

// ------------TimerA2_Init------------
// Before initializing the timer, do the calculations:
//      Elapsed Time = clock counts * (N/Clock Frequency)
//      clock counts cannot exceed 65,535
//      N is the clock scaler

// Input: none
// Output: none
void TimerA2_Init(void){
  // In the TA2CTL register:
  //    Disable timer for initialization
  //    Choose a clock source
  //    Select a clock scaler if needed
  //    Enable overflow interrupt if needed
  // In the TA2EX0 register:
  //    Select a second clock scaler if needed
  // For EACH CCRn being used, in the TA2CCTLn register
  //    set the PWM outmode if needed
  //    Enable the CCRn match interrupt if needed
  // For EACH CCRn being used, Load TA2CCRn with clock
  //    counts for desired delay

    //SMCLK, N = 8, stop mode
    TA2CTL |= 0x02C0;//0000 0010 1100 0000
    TA2CTL &= ~0x0130;//0000 0001 0011 0000
}


// ------------TimerA3_Init------------
// Before initializing the timer, do the calculations:
//      Elapsed Time = clock counts * (N/Clock Frequency)
//      clock counts cannot exceed 65,535
//      N is the clock scaler

// Input: none
// Output: none
void TimerA3_Init(void){
  // In the TA3CTL register:
  // Disable timer for initialization
  // Choose a clock source
  // Select a clock scaler if needed
  //    Enable overflow interrupt if needed
  // In the TA3EX0 register:
  //    Select a second clock scaler if needed
  // For EACH CCRn being used, in the TA3CCTLn register
  // set the PWM outmode if needed
  //    Enable the CCRn match interrupt if needed
  // For PWM, Load TA3CCR0 with clock
  // counts for desired period
    TA3CTL &= ~0x0030;   // Stop the timer

      TA3CTL = 0b0000001010010000; // Bits 4,7,9 Up mode. divider of 4, SMCLK selected
      /*
      TA3CTL |= 0x0200;    // Use SMCLK as the clock source
      TA3CTL &= ~0x0180;   // Clear unnecessary bits for clock source configuration
  */

      // Select a clock scaler if needed
       //TA3CTL |= 0x0040;    // Set clock prescaler to divide by 8 (ID__3)
       //TA3CTL &= ~0x0080;   // Ensure no higher prescaler is selected
       // In the TA3EX0 register:
         TA3EX0 &= ~0x0007;   // No secondary clock scaling needed

         // For EACH CCRn being used, in the TA3CCTLn register
       // below is right
         // For PWM, Load TA3CCR0 with clock counts for desired period
           TA3CCR0 = 59999;     // 20ms period
           TA3CCR2 = 4499;      // 1.5ms pulse for neutral servo position (0 degrees)
           TA3CCTL2 |= 0x00E0;  // Set CCR3 to Reset/Set mode (OUTMOD_7)
  }


