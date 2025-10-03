/****************************************************************************************
         CPET253Final_Project

Ali Raiymkulov

 This program uses a state machine to control the TI-RSLK robot to drive
 in a pattern of forward, right turn, backward, left turn, forward, right turn, ..

 To control the motors on the TI-RSLK robot, there are three outputs that need
 to be driven.
    :Pin    :Description            :Notes
    :=======:=======================:=========================
    : P5.5  : Right motor direction : 0=forwards, 1=backwards
    : P3.6  : Right motor sleep     : 0=sleep, 1=awake
    : P2.6  : Right motor PWM       : 0=stop, PWM signal = go
    : P5.4  : Left motor direction  : 0=forwards, 1=backwards
    : P3.7  : Left motor sleep      : 0=sleep, 1= awakea
    : P2.7  : Left motor PWM        : 0=stop, PWM signal = go

 Functions in this code:
     -Clock_Init48MHz() - function provided by TI to set system clock
     -Clock_Delay1ms(time) - built in function that delays time ms
     -Port2_Init();
     -Port3_Init();
     -Port5_Init();
     -TimerA0_Init();
     -MotorForward(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -MotorBackward(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -MotorTurnRight(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -MotorTurnLeft(volatile uint16_t rightDuty, volatile uint16_t leftDuty );

The state machine has 4 states; forward, right, left, backward
use FSM to make a pattern: Forward, right turn 90 degrees, backwards, left turn 90, forward...
*******************************************************************************************/

#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/motor.h"
#include "../inc/Init_Ports.h"
#include "../inc/Init_Timers.h"
#include "../inc/Reflectance.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/UART1.h"
#include "../inc/SSD1306.h"

extern uint8_t  count;// used the same from the other file
extern uint8_t  wasInterrupt;// extern means to use the same from the other file
extern uint8_t  right;// the right is for the buper
extern uint8_t  left ;//  the left is for the bumper which is the left side of the bumper
extern uint8_t direction;// it showes the robtic which side the bumbper was hitten

#define TRIGGER 0x04
#define ECHO 0x08
#define SERVO_CENTER 4500
#define SERVO_RIGHT 9000
#define SERVO_LEFT 2000

#define OBSTACLE_THRESHOLD 10

#define microsecondsToClockCycles(a) ( (a) * 1.5 )       //assume 12Mhz clock divided by 8
#define clockCyclesToMicroseconds(a) ( (a) / 1.5 )       // 1.5 clock cycles = 1us

void Servo(uint16_t angle);
uint32_t pulseIn (void);

bool isAvoiding = false;
bool ExitThat = false;
bool StartParking = false;


void Servo(uint16_t angle_count) {
    TA3CCR0 = 59999;
    TA3CCR2 = angle_count;
    TA3CTL |= 0x0010; // bit 4 up mode
    TA3CTL &= ~0x0020; // clear bit 5
    return;
}

void ServoInit(void) {
    Servo(4500);
    Clock_Delay1ms(1000);
    TA3CTL &= ~0x0030; // bits 4 and 5 makes it up down mode
    return;
}

uint32_t pulseIn(void) {
    uint16_t width = 0;
    uint16_t maxcount = 56999;

    TA2CTL |= 0x0020;
    TA2CTL &= ~0x0010;
    TA2R = 0;

    while (!(P6IN & ECHO)) {
        if (TA2R > maxcount) return 0;
    }

    TA2R = 0;

    while (P6IN & ECHO) {
        if (TA2R > maxcount) return 0;
    }

    width = TA2R;
    TA2CTL &= ~0x0030;
    return width / 1.5;
}

uint16_t distanceInCm(void) {
    uint16_t distance;
    float time;

    P6OUT |= TRIGGER;      //drive trigger pin p6.2 high
    Clock_Delay1us(10);      //wait 10 us
    P6OUT &= ~TRIGGER;      //drive trigger pin low
    time = pulseIn();
    distance = (time*0.034/2);      //calculate distance using s=t * 0.034/2. t comes from pulseIn() function
    if(distance == 0){
        distance = 400;
     }      // if no echo (distance = 0), assume object is at farthest distance
    return(distance);      //return the distance
}

// ---------- State Machines ----------
void FollowLine(void) {
    static enum motor_states {FORWARD, CORRECT_RIGHT, CORRECT_LEFT} state = FORWARD;
    static enum motor_states prevState = !FORWARD;
    bool isNewState = (state != prevState);
    prevState = state;

    int32_t raw = Reflectance_Read(1000);
    int32_t pos = Reflectance_Position(raw);

    switch (state) {
        case FORWARD:
            if (isNewState) Motor_Forward(5999, 3999);
            if (pos < -95) state = CORRECT_RIGHT;
            else if (pos > 95) state = CORRECT_LEFT;
            break;

        case CORRECT_RIGHT:
            if (isNewState) Motor_Forward(2499, 6999);
            if (pos >= 0) state = FORWARD;
            break;

        case CORRECT_LEFT:
            if (isNewState) Motor_Forward(8999, 2499);
            if (pos <= 0) state = FORWARD;
            break;
    }

    Clock_Delay1ms(10);
}

void Bumpers(void){
    enum motor_states {FWD, EXIT} state, prevState;

    state = FWD;
    prevState = !FWD;
    uint16_t stateTimer = 0;
    bool isNewState;

    while(StartParking){
        isNewState = (state != prevState);
        prevState = state;

        switch(state){
        case FWD:
            if(isNewState){
                wasInterrupt = 0;
                stateTimer = 0;
                Motor_Forward(12999, 12999);
            }
        stateTimer ++;
            if(wasInterrupt == 1){
                Motor_Stop();
                state = EXIT;
            }
            break;
        case EXIT:
            Motor_Stop();
            StartParking = 0;

        }// end switch
    }// end while
}

void ObstacleAvoidance(void) {
        ServoInit();//center the servo using the ServoInit() function
        uint16_t distance, right_wall, left_wall;

        enum motor_states {Backward, SweepRight, SweepLeft, Exit, Message} prevState, state; //These are the states of the state machine

        state = Backward;          //start in FORWARD state
        prevState = !Backward;   //used to know when the state has changed
        uint16_t stateTimer = 0;           //used to stay in a state
        bool isNewState;              //true when the state has switched
        bool ExitAvoidance;
        ExitAvoidance = true;



        while(ExitAvoidance) {

            isNewState = (state != prevState);
            prevState = state;
            distance = distanceInCm();
            switch (state) {
            case Backward:
                if(isNewState){
                    stateTimer = 0;
                Motor_Backward(4999, 4999);
                Clock_Delay1ms(100);
                }
                     state = Message;
                break;
            case Message:
                if(isNewState){
                    stateTimer = 0;
                }
                SSD1306_ClearBuffer();
                SSD1306_DrawString(0, 32,"-- Starting Sweep! --------",WHITE);
                SSD1306_DisplayBuffer();
                Clock_Delay1ms(2000);
                    state = SweepRight;
                break;
            case SweepRight:
                if(isNewState){
                    stateTimer = 0;
                    Servo (9000);
                    right_wall = distanceInCm();
                    Motor_Stop();
                }
                Clock_Delay1ms(2000);
                    state = SweepLeft;
            case SweepLeft:
                if(isNewState){
                    stateTimer = 0;
                    Servo (1200);
                    left_wall = distanceInCm();
                    Motor_Stop();
                }
                Clock_Delay1ms(1500);
                if(right_wall <= left_wall){// turn left
                    stateTimer = 0;
                    ServoInit();
                    Motor_Left(12999, 0);
                    Clock_Delay1ms(1200);
                }
                if(left_wall <= right_wall){// turn right
                    stateTimer = 0;
                    ServoInit();
                    Motor_Right(0, 12999);
                    Clock_Delay1ms(1200);
                }
                    state = Exit;
            case Exit:
                SSD1306_ClearBuffer();
                SSD1306_DrawString(0, 32,"-- Starting Parking! ------",WHITE);
                SSD1306_DisplayBuffer();
                ExitAvoidance = false;
                isAvoiding = false;
                ExitThat = true;
                StartParking = true;

        }  //while
    }
    }

void main(void) {
   WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
    Clock_Init48MHz();

    // Init ports & timers
    Port2_Init(); Port3_Init(); Port5_Init();
    Port6_Init(); Port7_Init(); Port9_Init();
    Port8_Init();
    UART0_Init();
    SSD1306_Init(SSD1306_SWITCHCAPVCC);
    EnableInterrupts();

    TimerA0_Init();
    TimerA2_Init();
    TimerA3_Init();

    DisableInterrupts();
    BumpInt_Init();
    EnableInterrupts();

    ServoInit();

    while (1) {
        if (isAvoiding) {
            ObstacleAvoidance();
        }
        else if (ExitThat){
            Bumpers();
        }
         else {
            if (distanceInCm() <= OBSTACLE_THRESHOLD) {
                Motor_Stop();
                isAvoiding = true;
            } else {
                FollowLine();
            }
        }
    }
}// end main
