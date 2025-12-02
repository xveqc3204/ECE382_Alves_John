#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/Motor.h" // Lab 13 - Motors
#include "../inc/Bump.h" // Lab 14 - Bump Sensors
#include "../inc/Tachometer.h" // Lab 14 - Distance
#include "../inc/LaunchPad.h" // LEDs
#include "../inc/UART0.h" // For debugging if necessary
#include "../inc/Nokia5110.h"
#include "../inc/TimerA2.h"
#include "Level1.h"

#define HOME_TOLERANCE 40 // How close robot get's to the destination
#define APPROX_DIST_MM 30 // Distance in mm used to calculate whether it's better to go left or right
#define x_destination 0 // Fixed x coordinate in mm for the final location
#define y_destination 0 // Fixed y coordinate in mm for the final location

#define MINMAX(Min, Max, X) ((X) < (Min) ? (Min) : ((X) > (Max) ? (Max) : (X)))

// Controller Define item
#define PWM_AVERAGE 300                 // Average PWM for balancing
#define SWING 250                       // Maximum correction swing
#define PWMIN (PWM_AVERAGE - SWING)     // Minimum PWM threshold
#define PWMAX (PWM_AVERAGE + SWING)     // Maximum PWM threshold
#define GAIN_DIVIDER 100
#define KP 250                          // Proportional gain 250/100 = 2.5


// States For State Machine (Extend/ Modified beyond what was presented in final destination)
typedef enum {
    STOP,
    GOING_FORWARD,
    GOING_BACKWARD,
    TURNING_LEFT,
    TURNING_RIGHT,
    AT_DESTINATION,
} RobotState_t;

// Structure to define robot movement commands
typedef struct command {
    uint16_t left_permil;     // PWM duty cycle for the left wheel
    uint16_t right_permil;    // PWM duty cycle for the right wheel
    void (*MotorFunction)(uint16_t, uint16_t);  // Motor function to call (e.g., Motor_Forward)
    int32_t dist_mm;          // Wheel displacement in mm
} command_t;

// Control parameters for various states (distances and duty cycles)
#define FRWD_DIST       700   // Replace this line for forward distance
#define BKWD_DIST       90    // Replace this line for backward distance
#define TR90_DIST       100    // Replace this line for 90 degree turn
#define TR60_DIST       74    // Replace this line for 60 degree turn
#define TR30_DIST       37    // Replace this line for 30 degree turn

#define NUM_STATES      6     // Number of robot states
#define LEN_STR_STATE   5     // String length for state names
static char strState[NUM_STATES][LEN_STR_STATE] = {"STOP", "FRWD", "BKWD", "LFTR", "RGTR", "DEST"};  // State names

// Static (Global) Important Variables

static RobotState_t CurrentState = GOING_FORWARD;
static RobotState_t NextState = GOING_FORWARD;

//Position Items
static int16_t x_position = 360;
static int16_t y_position = 0;

//Distance Items
static uint16_t distanceTraveled = 0;
static int16_t LeftDistance_mm = 0;
static int16_t RightDistance_mm = 0;
static int16_t PrevLeftDistance_mm = 0;
static int16_t PrevRightDistance_mm = 0;

// Essentially Counts left turns; 0->North; 1->West; 2->South; 3->East
static uint8_t heading = 0;

// Updates our values relative to the given heading, has separate case for backwards
void updatePosition(int16_t distance_mm, bool backwards_case){
    uint8_t heading_backwards = (heading + 2) % 4;
    uint8_t corrected_heading = (backwards_case) ? heading_backwards : heading;
    switch (corrected_heading) {
        case 0: // Going North (+y Direction)
            y_position += distance_mm;
            break;
        case 1: // Going West (-x Direction)
            x_position -= distance_mm;
            break;
        case 2: // Going South (-y Direction)
            y_position -= distance_mm;
            break;
        case 3: // Going East (+x Direction)
            x_position += distance_mm;
            break;
    }
}

RobotState_t findOptimalTurn(void){

    // Essentially calculating the cardinal direction if I turn left or right and if that minimizes distance relative to the destination
    int16_t west_approx = abs((x_position - APPROX_DIST_MM) - x_destination);
    int16_t east_approx = abs((x_position + APPROX_DIST_MM) - x_destination);
    int16_t south_approx = abs((y_position - APPROX_DIST_MM) - y_destination);
    int16_t north_approx = abs((y_position + APPROX_DIST_MM) - y_destination);

    switch (heading) {
        case 0: // Facing North, look at turning Left-(West) or Right-(East)
            return (west_approx < east_approx) ? TURNING_LEFT : TURNING_RIGHT;

        case 1: // Facing West, look at turning Left-(South) or Right-(North)
            return (south_approx < north_approx) ? TURNING_LEFT : TURNING_RIGHT;

        case 2: // Facing South, look at turning Left-(East) or Right-(West)
            return (east_approx < west_approx) ? TURNING_LEFT : TURNING_RIGHT;

        case 3: // Facing East, look at turning Left-(North) or Right-(South)
            return (north_approx < south_approx) ? TURNING_LEFT : TURNING_RIGHT;

        default:
            return TURNING_LEFT;
    }
}

//Checks if the robot has made it to the desired point by a margin of 50mm in x and y direction
bool isAtDestination(void){
    return (abs(x_position-x_destination) < HOME_TOLERANCE) && (abs(y_position-y_destination) < HOME_TOLERANCE);
}

// Control commands for each state
command_t ControlCommands[NUM_STATES] = {
    {0,   0,   &Motor_Stop,        0},          // Stop indefinitely
    {0,   0,   &Motor_Forward,     FRWD_DIST},  // Move forward until bump sensor triggered, Motor speeds 0 bc it's calculated by controller
    {200, 200, &Motor_Backward,    BKWD_DIST},  // Move backward for 90mm
    {200, 200, &Motor_TurnLeft,    TR30_DIST},  // Turn left
    {200, 200, &Motor_TurnRight,   TR30_DIST},  // Turn right
    {0,   0,   &Motor_Stop,        0},          // At destination
};

// Clear the LCD and display initial state
static void LCDClearL1(void) {
    Nokia5110_Clear();
    Nokia5110_SetCursor2(0, 1);
    Nokia5110_OutString("Level1");
    Nokia5110_SetCursor2(1, 1);
    Nokia5110_OutString("ST:");
    Nokia5110_SetCursor2(2, 1);
    Nokia5110_OutString("Dist:");
    Nokia5110_SetCursor2(3, 1);
    Nokia5110_OutString("X:");
    Nokia5110_SetCursor2(4, 1);
    Nokia5110_OutString("Y:");
    Nokia5110_SetCursor2(5, 1);
    Nokia5110_OutString("H:");
}

// Update the LCD
static void LCDOutL1(void) {
    // Display state
    Nokia5110_SetCursor2(1, 4);
    Nokia5110_OutString(strState[CurrentState]);
    Nokia5110_OutString(" ");  // Clear extra char

    // Display distance
    Nokia5110_SetCursor2(2, 5);
    Nokia5110_OutUDec(distanceTraveled,5);
    Nokia5110_OutString("   ");

    // Display x coord
    Nokia5110_SetCursor2(3, 3);
    Nokia5110_OutSDec(x_position,5);
    Nokia5110_OutString("   ");

    // Display y coord
    Nokia5110_SetCursor2(4, 3);
    Nokia5110_OutSDec(y_position,5);
    Nokia5110_OutString("   ");

    // Display heading
    Nokia5110_SetCursor2(5, 3);
    Nokia5110_OutUDec(heading,5);
}

// Counter for how many times the controller function has been called
static uint8_t NumControllerExecuted = 0;  // Updated every 20ms

// Main control logic for the robot, executed by the TimerA2 ISR every 20ms
static void ControllerL1(void) {

    static uint16_t timer_20ms = 0;   // Timer to track elapsed time
    static uint8_t bumpRead = 0x00;   // Stores the bump sensor reading

    // Controller runs ever 20ms so counter below capture necessary arithmetic
    static uint16_t ledFlashCount = 0;

    NumControllerExecuted++;  // Increment counter every time the controller runs

    // Get the PWM duty cycles for the current state
    uint16_t left_permil = ControlCommands[CurrentState].left_permil;
    uint16_t right_permil = ControlCommands[CurrentState].right_permil;

    // FSM Output: Execute the motor command for the current state
    // Write your code here
    ControlCommands[CurrentState].MotorFunction(left_permil, right_permil);

    // State transition logic based on bump sensors and distance
    bumpRead = Bump_Read();  // Read bump sensor status
    Tachometer_GetDistances(&LeftDistance_mm, &RightDistance_mm);  // Get current wheel distances

    //Calculate Delta Distances so Distance is calculated properly
    int16_t deltaLeft = LeftDistance_mm - PrevLeftDistance_mm;
    int16_t deltaRight = RightDistance_mm - PrevRightDistance_mm;
    int16_t deltaAvgDist = (deltaLeft + deltaRight) / 2;

    //Set Previous Distance to Current Distance
    PrevLeftDistance_mm = LeftDistance_mm;
    PrevRightDistance_mm = RightDistance_mm;

    //Find Magnitude
    int16_t LMag = (LeftDistance_mm < 0) ? -LeftDistance_mm : LeftDistance_mm;
    int16_t RMag = (RightDistance_mm < 0) ? -RightDistance_mm : RightDistance_mm;

    switch (CurrentState) {

        case STOP:  // Remain in Stop state indefinitely
            NextState = STOP;
            break;

        case GOING_FORWARD:  // Moving forward

            updatePosition(deltaAvgDist,0);
            distanceTraveled += abs(deltaAvgDist);

            //Controller to help robot move straight
            int16_t Error = LeftDistance_mm - RightDistance_mm; //error associated with left and right wheel
            int16_t leftDuty_permil = PWM_AVERAGE - (KP*Error) / GAIN_DIVIDER;   // Adjust left motor speed based on error
            int16_t rightDuty_permil = PWM_AVERAGE + (KP*Error) / GAIN_DIVIDER;  // Adjust right motor speed based on error
            leftDuty_permil = MINMAX(PWMIN,PWMAX,leftDuty_permil);
            rightDuty_permil = MINMAX(PWMIN,PWMAX,rightDuty_permil);
            Motor_Forward(leftDuty_permil, rightDuty_permil); // Set motor speeds to maintain center position


            if(isAtDestination()){
                NextState = AT_DESTINATION;
                break;
            }

            if ((bumpRead & 0x01) || (bumpRead & 0x02)){ // B1 || B2
                // B1 -> 30 deg Left; B2 -> 60 deg Left
                ControlCommands[TURNING_LEFT].dist_mm = (bumpRead & 0x01) ? TR30_DIST : TR60_DIST;
                NextState = TURNING_LEFT;
            }
            else if ((bumpRead & 0x04) || (bumpRead & 0x08)){ // B3 || B4
                NextState = GOING_BACKWARD;
            }
            else if ((bumpRead & 0x10) || (bumpRead & 0x20)){ // B5 || B6
                // B6 -> 30 deg Right; B5 -> 60 deg Right
                ControlCommands[TURNING_RIGHT].dist_mm = (bumpRead & 0x20) ? TR30_DIST : TR60_DIST;
                NextState = TURNING_RIGHT;
            }
            else{
                NextState = GOING_FORWARD;
            }
            break;

        case GOING_BACKWARD:  // Moving backward

            if ((LMag >= ControlCommands[GOING_BACKWARD].dist_mm) || (RMag >= ControlCommands[GOING_BACKWARD].dist_mm)) { // Finished Backing Up in Reverse
                ControlCommands[TURNING_LEFT].dist_mm  = TR90_DIST;
                ControlCommands[TURNING_RIGHT].dist_mm  = TR90_DIST;

                // Calculate distance of moving backwards
                int16_t backwardsDist = (LMag + RMag) / 2;

                // Update Distance in Backwards direction
                updatePosition(backwardsDist,true);
                distanceTraveled += abs(backwardsDist);

                NextState = findOptimalTurn();
            }
            else{
                NextState = GOING_BACKWARD;
            }
            break;

        case TURNING_LEFT:  // Turning left
            if (RMag >= ControlCommands[TURNING_LEFT].dist_mm) {  // Finished left turn
                if (ControlCommands[TURNING_LEFT].dist_mm == TR90_DIST){ // Ensures that a Full 90 degrees is the turn so the heading will properly adapt
                    heading = (heading + 1) % 4;  // Update heading and Resets Heading 4->0 to maintain cyclical nature
                }
                // For small changes of direction per Lab 16's FSM, won't change heading
                NextState = GOING_FORWARD;  // Move forward again
            }
            else{
                NextState = TURNING_LEFT;
            }
            break;

        case TURNING_RIGHT:  // Turning right
            if (LMag >= ControlCommands[TURNING_RIGHT].dist_mm) {  // Finished right turn
                if (ControlCommands[TURNING_RIGHT].dist_mm == TR90_DIST){ // Ensures that a Full 90 degrees is the turn so the heading will properly adapt
                    heading = (heading + 3) % 4;  // Update heading and Resets Heading 4->0 to maintain cyclical nature, heading+3 here is the equivalent to -1 mod 4
                }
                // For small changes of direction per Lab 16's FSM, won't change heading
                NextState = GOING_FORWARD;  // Move forward again
            }
            else{
                NextState = TURNING_RIGHT;
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
        default:
            NextState = STOP;
            break;
    }

    // Update the timer or reset if transitioning to a new state
    if (CurrentState == NextState) {
        timer_20ms++;  // Stay in current state, increment timer
    } else {
        timer_20ms = 0;  // New state, reset timer and distance measurements
        Tachometer_ResetSteps();
        LeftDistance_mm = 0;
        RightDistance_mm = 0;
        PrevLeftDistance_mm = 0;
        PrevRightDistance_mm = 0;
    }

    // Set the current state to the next state for the next iteration
    CurrentState = NextState;
}

void Level1(void){
    // ========== Initialization Phase ==========
    DisableInterrupts();    // Disable interrupts during initialization
    Clock_Init48MHz();      // Set the system clock to 48 MHz
    LaunchPad_Init();       // Initialize the LaunchPad hardware (buttons, LEDs)
    Bump_Init();            // Initialize bump sensors
    Motor_Init();           // Initialize motor driver
    Nokia5110_Init();       // Initialize Nokia 5110 LCD display
    Tachometer_Init();      // Initialize tachometers for wheel distance measurement

    // Set LCD contrast
    uint8_t const contrast = 0xB0;
    Nokia5110_SetContrast(contrast);  // Adjust LCD contrast

    // Set TimerA2 to call the Controller3() function every 20 ms (50 Hz)
    const uint16_t period_4us = 5000;       // 20 ms period
    TimerA2_Init(&ControllerL1, period_4us); // Initialize TimerA2

    Tachometer_ResetSteps();  // Reset tachometer distance measurements

    // Set the LCD update rate
    const uint16_t LcdUpdateRate = 25;    // 50 Hz / 25 = 2 Hz
    LCDClearL1();  // Clear the LCD and display initial state

    EnableInterrupts();  // Enable interrupts

    // ========== Main Loop ==========
    while(1) {

        // Enter low-power mode while waiting for the next interrupt
        WaitForInterrupt();

        // Update the LCD
        if (NumControllerExecuted == LcdUpdateRate) {
            LCDOutL1();  // Output current state and motor data to the LCD
            NumControllerExecuted = 0;  // Reset the execution count
        }
    }
}



