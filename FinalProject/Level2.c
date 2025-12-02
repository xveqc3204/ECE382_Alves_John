#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/Motor.h"
#include "../inc/LaunchPad.h"
#include "../inc/Nokia5110.h"
#include "../inc/TimerA1.h"
#include "../inc/TimerA2.h"
#include "../inc/ADC14.h"
#include "../inc/IRDistance.h"
#include "../inc/LPF.h"
#include "../inc/Classifier.h"
#include "Level2.h"

#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ((X) > (Max) ? (Max) : (X)))

// PWM parameters for controlling motor speeds.
#define PWM_AVERAGE 550                 // Average PWM for balancing //550, 575
#define SWING 200
#define PWMIN (PWM_AVERAGE - SWING)     // Minimum PWM threshold
#define PWMAX (PWM_AVERAGE + SWING)     // Maximum PWM threshold
#define GAIN_DIVIDER 100

//Turning parameters to easily adjust
#define TURNING_SPEED_OUTER 500 //TUNE //500
#define TURNING_SPEED_INNER 275 //TUNE //275
#define TURN_DURATION_MS 700 //TUNE
#define CYCLES_TO_BE_CERTAIN 4 // Variable that is multiple by 20ms to determine if the scenario is certain of being a turn

//Distance Variables
#define GOAL_THRESHOLD 250 //TUNE
#define TARGET_WALL_DIST 228 // Distance from wall for wall following

// States For State Machine (Extend/ Modified beyond what was presented in final destination)
typedef enum {
    GOING_FORWARD,
    TURNING_LEFT,
    TURNING_RIGHT,
    AT_DESTINATION,
    STOP
} RobotState_t;

// Static (Global) Important Variables

static RobotState_t CurrentState = GOING_FORWARD;
static RobotState_t NextState = GOING_FORWARD;

// Variables to try and combat instability of scenarios/ states
static uint8_t scenarioStableCount = 0;
static scenario_t stableScenario = Straight;

//Maze tracking globals, to track and determine where I am at in the maze and ultimately to decide which wall to follow
static uint8_t leftTurnCount = 0;
static uint8_t rightTurnCount = 0;
static bool followRightWall = true;

// IR distance variables to store readings from wall sensors (in mm).
static int32_t Left, Center, Right;          // Distances to the left, center, and right walls
static int32_t Error = 0;                    // Error signal for wall following

// Scenario for the classifier
static scenario_t CurrentScenario = Straight;

// Controller Variables
static int16_t Kp = 70; //TUNE
// LCD update rate: used to track number of controller executions for display.
static uint8_t NumControllerExecuted = 0;

// Turn time variables to track how long I have been turning, tried gathering actual times instead of using counter
static uint32_t systemTime_ms = 0;
static uint32_t turnStartTime_ms = 0;

// Periodic ADC sampling function for IR sensors.
// This function should be triggered periodically by TimerA ISR.
static void IRsampling(void){

    uint16_t raw17, raw14, raw16;               // Variables to store raw ADC values for each sensor
    ADC_In17_14_16(&raw17, &raw14, &raw16);     // Read ADC values from channels 17, 14, and 16

    uint32_t nr = LPF_Calc(raw17);              // Apply low-pass filter (LPF) to smooth right sensor data
    uint32_t nc = LPF_Calc2(raw14);             // Apply LPF to smooth center sensor data
    uint32_t nl = LPF_Calc3(raw16);             // Apply LPF to smooth left sensor data

    Left = LeftConvert(nl);                     // Convert smoothed left data to distance (or other scaled units)
    Center = CenterConvert(nc);                 // Convert smoothed center data to distance
    Right = RightConvert(nr);                   // Convert smoothed right data to distance

    CurrentScenario = Classify(Left, Center, Right);
}

// Proportional controller function to keep the robot centered between two walls using IR sensors.
// Runs at 100 Hz (configured by TimerA ISR).
static void ControllerL2(void){

    // Previous Scenario for State Machine
    static scenario_t PreviousScenario = Straight;

    // Controller runs ever 20ms so counter below capture necessary arithmetic
    static uint16_t ledFlashCount = 0;
    systemTime_ms += 20;
    NumControllerExecuted++;

    // Calculate error dependent on if I am following the right wall or left wall
    if (followRightWall){
        // Right > TARGET_WALL_DIST -> negative error -> veer left, vice versa
        Error = TARGET_WALL_DIST - Right; // TARGET_WALL_DIST mm distance from wall
    }
    else {
        // Left > TARGET_WALL_DIST -> positive error -> veer right, vice versa
        Error = Left - TARGET_WALL_DIST; // TARGET_WALL_DIST mm distance from wall
    }


    // Combat's jumpy classification to ensure that the readings will be consistent and decrease likelihood of turning when no supposed to
    if((CurrentScenario == stableScenario) && scenarioStableCount < 255){ // the < checks to ensure uint8_t doesnt get messed up by overflow
        scenarioStableCount++;
    }
    else{
        stableScenario = CurrentScenario;
        scenarioStableCount = 0;
    }

    //State Machine
    switch (CurrentState) {

            case GOING_FORWARD:  // Moving forward

                NextState = GOING_FORWARD;

                // Calculate the left and right motor duty cycles based on proportional control
                uint16_t leftDuty_permil = PWM_AVERAGE - (Kp*Error) / GAIN_DIVIDER;   // Adjust left motor speed based on error
                uint16_t rightDuty_permil = PWM_AVERAGE + (Kp*Error) / GAIN_DIVIDER;  // Adjust right motor speed based on error

                // Ensure the calculated PWM duty cycles are within the motor's operational range
                leftDuty_permil = MINMAX(PWMIN,PWMAX,leftDuty_permil);
                rightDuty_permil = MINMAX(PWMIN,PWMAX,rightDuty_permil);

                Motor_Forward(leftDuty_permil, rightDuty_permil);

                // Emergency stop just in case, have ran into too many walls
                if (Center < 50){
                    Motor_Stop(0, 0);
                    NextState = STOP;
                    break;
                }

                // Checks if the cases for being at the home point exists
                if ((Center < (GOAL_THRESHOLD)) && (CurrentScenario == Blocked)){
                        NextState = AT_DESTINATION;
                        break;
                }

                if((PreviousScenario == Straight) && (stableScenario != Straight) && (scenarioStableCount >= CYCLES_TO_BE_CERTAIN)){ // Checks if a transition from going forward to something else has occurred

                    scenarioStableCount = 0; //Checks to see if the robot's reading of its classification is certain. CYCLE_TO_BE_CERTAIN * 20ms = ___ ms of reading a value -> Certain

                    if (stableScenario == LeftTurn){
                        leftTurnCount++;
                        turnStartTime_ms = systemTime_ms;
                        Motor_Forward(TURNING_SPEED_INNER, TURNING_SPEED_OUTER); // Turn Left in a smooth arc pattern
                        NextState = TURNING_LEFT;
                    }
                    else if((stableScenario == RightTurn) || (stableScenario == TeeJoint)){
                        rightTurnCount++;
                        turnStartTime_ms = systemTime_ms;
                        Motor_Forward(TURNING_SPEED_OUTER, TURNING_SPEED_INNER-20); // Turn Right in a smooth arc pattern
                        NextState = TURNING_RIGHT;
                    }

                }

                break;

            case TURNING_LEFT:  // Turning left

                Motor_Forward(TURNING_SPEED_INNER, TURNING_SPEED_OUTER); // Turn left in a smooth arc pattern

                if ((systemTime_ms - turnStartTime_ms) > TURN_DURATION_MS){
                    NextState = GOING_FORWARD;
                }
                else{
                    NextState = TURNING_LEFT; // Keep turning left if not complete yet
                }
                break;

            case TURNING_RIGHT:  // Turning right

                Motor_Forward(TURNING_SPEED_OUTER, TURNING_SPEED_INNER-20); // Turn right in a smooth arc pattern

                if ((systemTime_ms - turnStartTime_ms) > TURN_DURATION_MS){
                    NextState = GOING_FORWARD;
                }
                else{
                    NextState = TURNING_RIGHT; // Keep turning right if not complete yet
                }
                break;

            case AT_DESTINATION:
                Motor_Stop(0,0); //Stop motors

                ledFlashCount++;
                if (ledFlashCount <= 25){ // 25*20ms = 500ms; 1/500ms = 2Hz
                    LaunchPad_RGB(RED);
                }
                else if (ledFlashCount <= 50){
                    LaunchPad_RGB(BLUE);
                }
                else{
                    ledFlashCount = 0;
                }

                NextState = AT_DESTINATION;
                break;

            case STOP:
                Motor_Stop(0,0); //Stop motors
                NextState = STOP;
                break;
        }

    //Checks to see if im in the part of the maze where I want to switch walls
    if ((rightTurnCount >= 1) && (leftTurnCount >= 3) && followRightWall){
        followRightWall = false; // follows left wall now to avoid right joint on the final stretch of the maze
    }

    // Previous scencario logic
    if (stableScenario == Straight) {
        PreviousScenario = Straight;
    }

    // Reseta when exiting turn states
    if ((CurrentState == TURNING_LEFT || CurrentState == TURNING_RIGHT) &&
        NextState == GOING_FORWARD) {
        PreviousScenario = Straight;  // force reset after turn
    }

    CurrentState = NextState;
}

char* stateNameArr[] = {"FRWD", "LTRN", "RTRN", "DEST", "STOP"};
char* scenarioNameArr[] = {
    "Error", "L2Close", "R2Close", "LR2Close", "C2Close",
    "LC2Close", "RC2Close", "AllClose", "Straight", "LeftTurn",
    "RightTurn", "TeeJoint", "LeftJoint", "RightJoint", "CrossRoad", "Blocked"
};

// Clears and initializes the LCD display with default text and formatting
static void LCDClear(void) {

    // Set the contrast level; 0xB1 works well on the red SparkFun display.
    // Adjust this from 0xA0 (lighter) to 0xCF (darker) if needed.
    uint8_t const contrast = 0xB0;
    Nokia5110_SetContrast(contrast);  // Apply the contrast setting
    Nokia5110_Clear();                // Clear the entire display screen

    // Display Level on LCD
    Nokia5110_SetCursor2(0,0);
    Nokia5110_OutString("Level 2");

    // Set labels and units for left, center, right IR sensor values, and error.
    Nokia5110_SetCursor2(2,1); Nokia5110_OutString("State:");
    Nokia5110_SetCursor2(3,1); Nokia5110_OutString("L:");
    Nokia5110_SetCursor2(4,1); Nokia5110_OutString("C:");
    Nokia5110_SetCursor2(5,1); Nokia5110_OutString("R:");
    Nokia5110_SetCursor2(6,1); Nokia5110_OutString("Scenario:");
}

// Updates the LCD display with real-time data values for Kp, left, center, right distances, and error.
static void LCDOut(void) {
    // Turn counts and wall side
    Nokia5110_SetCursor2(1, 1);
    Nokia5110_OutString("L:");
    Nokia5110_OutUDec(leftTurnCount, 1);
    Nokia5110_OutString(" R:");
    Nokia5110_OutUDec(rightTurnCount, 1);
    Nokia5110_OutString(" RW?:");
    Nokia5110_OutUDec(followRightWall, 1);

    // Current state
    Nokia5110_SetCursor2(2, 1);
    Nokia5110_OutString("St:");
    Nokia5110_OutString(stateNameArr[CurrentState]);
    Nokia5110_OutString("  ");

    // Left distance
    Nokia5110_SetCursor2(3, 1);
    Nokia5110_OutString("L:");
    Nokia5110_OutSDec(Left, 4);
    Nokia5110_OutString("mm");

    // Center distance
    Nokia5110_SetCursor2(4, 1);
    Nokia5110_OutString("C:");
    Nokia5110_OutSDec(Center, 4);
    Nokia5110_OutString("mm");

    // Right distance
    Nokia5110_SetCursor2(5, 1);
    Nokia5110_OutString("R:");
    Nokia5110_OutSDec(Right, 4);
    Nokia5110_OutString("mm");

    // Current & previous scenario
    Nokia5110_SetCursor2(6, 1); // fix this cursor to encompass L C and R after debugging
    Nokia5110_OutString("Cur:");
    Nokia5110_OutUDec(CurrentScenario, 2);
//    Nokia5110_OutString(" Prv:");
//    Nokia5110_OutUDec(PreviousScenario, 2);
}

void Level2(void) {
    DisableInterrupts();             // Disable interrupts during initialization
    Clock_Init48MHz();               // Set the system clock to 48 MHz
    LaunchPad_Init();                // Initialize LaunchPad buttons and LEDs
    Motor_Init();                    // Initialize motor controls
    Nokia5110_Init();                // Initialize the Nokia LCD
    LCDClear();                      // Clear the LCD screen

    // Flashes LEDs to delay before starting to prevent slippage from pressing on and robot moving right away
    for (int i = 0; i < 5; i++) {
        LaunchPad_RGB(GREEN);
        Clock_Delay1ms(100);  // Green for 100ms
        LaunchPad_RGB(RGB_OFF);
        Clock_Delay1ms(100);  // Off for 100ms
        // Total: 5 x 200ms = 1s
    }
    LaunchPad_RGB(RGB_OFF);  // Ensure off before starting

    // Use TimerA2 to run the controller at 50 Hz (every 20ms)
    uint16_t const period_4us = 5000;       // Timer period to achieve 20ms (5000 x 4us)
    TimerA2_Init(&ControllerL2, period_4us);  // Initialize TimerA2 for controller

    // Use TimerA1 to sample the IR sensors at 2000 Hz
    uint16_t const period_2us = 250;        // Timer period to achieve 0.5ms (250 x 2us)
    TimerA1_Init(&IRsampling, period_2us);  // Initialize TimerA1 for controller

    // Initialize ADC channels for sensors on pins 17, 14, and 16
    ADC0_InitSWTriggerCh17_14_16();
    uint16_t raw17, raw14, raw16;
    ADC_In17_14_16(&raw17, &raw14, &raw16); // Initial ADC sampling for calibration

    LPF_Init(raw17, 64);     // Initialize low-pass filter for right sensor (P9.0/channel 17)
    LPF_Init2(raw14, 64);    // Initialize LPF for center sensor (P4.1/channel 12)
    LPF_Init3(raw16, 64);    // Initialize LPF for left sensor (P9.1/channel 16)

    // Set rate for updating the LCD display: updates every 5 controller cycles (10 Hz)
    uint16_t const LcdUpdateRate = 5;

    NumControllerExecuted = 0;        // Reset execution count

    EnableInterrupts();               // Enable global interrupts to start periodic tasks

    CurrentState = GOING_FORWARD;

    while(1) {
        // Enter low-power mode, waiting for interrupts
        WaitForInterrupt();

        // Update the LCD display every 10 Hz (5 controller runs)
        // Note: Avoid adding LCDOut inside the ISR since Nokia5110 is a slow device.
        if (NumControllerExecuted == LcdUpdateRate) {
            LCDOut();                  // Call function to output data on the LCD
            NumControllerExecuted = 0; // Reset count after LCD update
        }

    }
}
