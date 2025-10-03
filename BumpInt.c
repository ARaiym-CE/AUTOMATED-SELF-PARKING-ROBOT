// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches on the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"

 uint8_t  right = 1;
 uint8_t  left = 0 ;

 uint8_t wasInterrupt = 0;
  uint8_t count = 0;
uint8_t direction;
// extern
 // dirction
// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
void BumpInt_Init(void){


    P4DIR &= ~0xED;  // 0b1110_1101 - Configure P4.7, P4.6, P4.5, P4.3, P4.2, P4.0 as inputs
        P4REN |= 0xED;   // 0b1110_1101 - Enable pull-up resistors
        P4OUT |= 0xED;   // 0b1110_1101 - Set pull-ups (default HIGH)
        P4SEL0 &= ~0xED; // use as GPIO
        P4SEL1 &= ~0xED;//

        P4IE |= 0xED;    // 0b1110_1101 - Enable interrupts for bump sensors
        P4IES |= 0xED;   // 0b1110_1101 - Set interrupt on falling edge

        NVIC->ISER[1] |= 0x40; // Enable Port 4 interrupts (Interrupt 38, Bit 6)
        P4IFG &= 0x00;  // 0b1110_1101 - Clear all interrupt flags

}

// triggered on touch, falling edge
void PORT4_IRQHandler(void){
   // uint16_t status = P4IV ;
   // wasInterruput = 0; store it in same waay in the lecture
    // write this as part of Lab 5

    uint8_t status = P4IV;

    if (status) wasInterrupt = 1;// Set a generic interrupt flag (optional)

           if (status == 0x02) { // B0 pressed (P4.0)
               count += 3; // the name of count is the same of the one in the top so it has to be the same in the one in the main file.
               direction = right;
           }
           else if (status == 0x06) { // B1 pressed (P4.2)
               count += 2; //
               direction = right;
           }
           else if (status == 0x08) { // B2 pressed (P4.3)
               count += 1;
               direction = right;
           }
           else if (status == 0x0C) { // B3 pressed (P4.5)
               count -= 1;
               direction = left;
           }
           else if (status == 0x0E) { // B4 pressed (P4.6)
               count -= 2;

               direction = left;
           }
           else if (status == 0x10) { // B5 pressed (P4.7)
               count -= 3;
               direction = left;
           }
           else {
               // No interrupt detected (optional)

    }
    P4IFG = 0;//  clear all the flages here .

}


