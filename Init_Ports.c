

// Init_Ports.c
// Runs on MSP432
// Provide functions that initialize ports 
// Prof. Christman
// 1/8/2023

/* These functions will be used in all of the labs for CPET253

There is an initialization function for each of the ports 1-10 and J.

The initializations will be different for each lab.
*/

#include "msp.h"


// ------------Port1_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port1_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state

    P1DIR &= 0b11101101;
    P1REN |= 0b00010010; // Enables Pull Up or Down Resistors for BIT4 and BIT1
    P1OUT |= 0b00010010; // Specify Pull Up for P1.4 and P1.1

}

// ------------Port2_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port2_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state

    P2DIR |= 0b11000000;
    P2SEL1 = 0;
    P2SEL0 = 0b11000000;

}

// ------------Port3_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port3_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state

    P3DIR |= 0b11000000;
}

// ------------Port4_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port4_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state

 /*   P4DIR &= 0b11101101; // Sets P4.0, 4.2, 4.3, and 4.5-4.7 as INPUTS
    P4REN |= 0b11101101; // Enables Pull Up or Down Resistors for BIT 0, 2, 3, and 5-7
    P4OUT |= 0b11101101; // Specify Pull Up for BITS 0, 2, 3, and 5-7

    P4SEL0 &= ~0b11101101;
    P4SEL1 &= ~0b11101101;
*/
}

// ------------Port5_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port5_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state

  //  P5DIR |= 0b00110000;
    //           ^^
    //           ||
    //           ||
    //           ++---------- BIT5 and BIT4 set as OUTPUTS
    //                        All others as INPUTS
    P5DIR |= 0x30;//0011 0000. Set left and right motor direction as outputs
       P5DIR |= 0x08;//0000 1000. Set pin 3 as an output for IR sensors


}

// ------------Port6_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port6_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state

    P6DIR |= 0b00000100;
    //             ^^
    //             ||
    //             ||
    //             ++---------- BIT2 OUTPUTS
    //                          BIT3 and All others as INPUTS

    P6SEL1 &= 0b00001100; //Normal Mode
    P6SEL0 &= 0b00001100;

}

// ------------Port7_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port7_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
    P7DIR &= ~0xFF;//Make all port 7 pins inputs
}

// ------------Port8_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port8_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state

    P8DIR |= BIT2;
    P8SEL0 |= BIT2;
    P8SEL1 &= ~BIT2;


}

// ------------Port9_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port9_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state

    //P9DIR |= 0b00000100;
//                  ^
//                  |
//                  |
//                  +---------- BIT6 and BIT7 set as OUTPUTS
//                              All others as INPUTS
    ///P9SEL1 &= 0b00000100;
  //  P9SEL0 |= 0b00000100;
    P9DIR |= 0x04;//0000 0100
//    P9DIR |= BIT3;
//    P9SEL0 |= BIT3;
//    P9SEL1 &= ~BIT3;

}

// ------------Port10_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port10_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}

// ------------PortJ_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void PortJ_Init(void){
  // Use the port direction register to configure pins as inputs and output
  //    -A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  //    -A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}
