// Tachometer.c
// Runs on MSP432
// Provide mid-level functions that initialize ports, take
// angle and distance measurements, and report total travel
// statistics.
// Daniel Valvano
// December 20, 2018
// Updated by Stan Baek, July 23, 2022

// Comments revised by Stan Baek
// October 22, 2024


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

// Left Encoder A connected to P10.5 (J5)
// Left Encoder B connected to P5.2 (J2.12)
// Right Encoder A connected to P10.4 (J5)
// Right Encoder B connected to P5.0 (J2.13)

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/TA3InputCapture.h"
#include "Tachometer.h"

// Global variables to track the previous and current times of interrupts
// from both the right and left wheel encoders
static uint16_t PreviousRightTime = 0;  // Stores last interrupt time for right encoder
static uint16_t CurrentRightTime = 0;   // Stores current interrupt time for right encoder
static uint16_t PreviousLeftTime = 0;   // Stores last interrupt time for left encoder
static uint16_t CurrentLeftTime = 0;    // Stores current interrupt time for left encoder

// Step counters for the right and left encoders, tracking total movement
static int16_t TachoRightSteps = 0;     // Right wheel step count, positive = forward, negative = backward
static int16_t TachoLeftSteps = 0;      // Left wheel step count, positive = forward, negative = backward

// Enum to store the last known direction of both the right and left wheels
enum TachDirection static TachoRightDir = STOPPED;  // Default: not moving
enum TachDirection static TachoLeftDir = STOPPED;   // Default: not moving


// This function is called by the TimerA3 CCR0 ISR when the right encoder detects movement.
// It calculates the time between interrupts, determines the direction of movement,
// and updates the step counter accordingly.
// Input: currenttime is the time at which the interrupt occurs.
static void tachometerRightInt(uint16_t currenttime){
    PreviousRightTime = CurrentRightTime;   // Store the previous interrupt time
    CurrentRightTime = currenttime;         // Update to current interrupt time

    // Check the state of Encoder B to determine movement direction
    if(P5->IN & 0x01) {
        // If Encoder B is high, the wheel moved forward
        TachoRightSteps++;                  // Increment step count for forward movement
        TachoRightDir = FORWARD;            // Set direction to forward
    } else {
        // If Encoder B is low, the wheel moved backward
        TachoRightSteps--;                  // Decrement step count for backward movement
        TachoRightDir = REVERSE;            // Set direction to reverse
    }
}


// This function is called by the TimerA3 CCR1 ISR when the left encoder detects movement.
// It performs the same operations as the right interrupt handler.
// Input: currenttime is the time at which the interrupt occurs.
static void tachometerLeftInt(uint16_t currenttime){
    PreviousLeftTime = CurrentLeftTime;    // Store the previous interrupt time
    CurrentLeftTime = currenttime;         // Update to current interrupt time

    // Check the state of Encoder B to determine movement direction
    if(P5->IN & 0x04) {
        // If Encoder B is high, the wheel moved forward
        TachoLeftSteps++;                  // Increment step count for forward movement
        TachoLeftDir = FORWARD;            // Set direction to forward
    } else {
        // If Encoder B is low, the wheel moved backward
        TachoLeftSteps--;                  // Decrement step count for backward movement
        TachoLeftDir = REVERSE;            // Set direction to reverse
    }
}


// ------------Tachometer_Init------------
// Initialize the GPIO pins and TimerA3 to interface with the rotary encoders.
// The rotary encoders provide step and direction data for the left and right wheels.
// Input: none
// Output: none
void Tachometer_Init(void){

    // Configure P5.0 (Right Encoder B) and P5.2 (Left Encoder B) as GPIO inputs.
    // These inputs will be used to determine the direction of wheel rotation.
    P5->SEL0 &= ~0x05;  // Disable alternate functions for P5.0 and P5.2
    P5->SEL1 &= ~0x05;  // Set P5.0 and P5.2 to GPIO mode
    P5->DIR &= ~0x05;   // Set P5.0 and P5.2 as input pins

    // Initialize TimerA3 for input capture mode to measure time intervals between encoder pulses.
    // This will allow us to measure speed and count steps for both wheels.
    TimerA3Capture_Init(&tachometerRightInt, &tachometerLeftInt);
}


// ------------Tachometer_ResetSteps------------
// Resets the step counters for both the right and left wheels to zero.
// This is useful when starting a new measurement or task.
// Input: none
// Output: none
void Tachometer_ResetSteps(void) {
    TachoRightSteps = 0;  // Reset right wheel step count
    TachoLeftSteps = 0;   // Reset left wheel step count
}

// ------------Tachometer_GetSpeeds------------
// Retrieves the periods (time between interrupts) for both the left and right wheels.
// The periods are used to calculate the wheel speeds.
// Input: leftPeriod_2_3rd_us  - pointer to store the left wheel period in units of 2/3 microseconds
//        rightPeriod_2_3rd_us - pointer to store the right wheel period in units of 2/3 microseconds
// Output: none
// Assumes: Tachometer_Init() and Clock_Init48MHz() have been called.
void Tachometer_GetSpeeds(uint16_t *leftPeriod_2_3rd_us, uint16_t *rightPeriod_2_3rd_us) {
    *leftPeriod_2_3rd_us = (CurrentLeftTime - PreviousLeftTime);    // Calculate left wheel period
    *rightPeriod_2_3rd_us = (CurrentRightTime - PreviousRightTime); // Calculate right wheel period
}

// ------------Tachometer_GetDirections------------
// Retrieves the last known direction of movement for both the left and right wheels.
// Input: leftDir  - pointer to store the left wheel's direction (FORWARD/REVERSE/STOPPED)
//        rightDir - pointer to store the right wheel's direction (FORWARD/REVERSE/STOPPED)
// Output: none
void Tachometer_GetDirections(enum TachDirection *leftDir, enum TachDirection *rightDir) {
    *leftDir = TachoLeftDir;   // Get left wheel direction
    *rightDir = TachoRightDir; // Get right wheel direction
}

// ------------Tachometer_GetSteps------------
// Retrieves the total number of steps (degree-based) taken by both the left and right wheels.
// One step is equivalent to one degree of rotation.
// Input: leftSteps_deg  - pointer to store the number of steps (degrees) for the left wheel
//        rightSteps_deg - pointer to store the number of steps (degrees) for the right wheel
// Output: none
void Tachometer_GetSteps(int16_t* leftSteps_deg, int16_t* rightSteps_deg) {
    *leftSteps_deg = TachoLeftSteps;   // Get total steps for left wheel
    *rightSteps_deg = TachoRightSteps; // Get total steps for right wheel
}

// ------------Tachometer_GetDistances------------
// Calculates the distances traveled by the left and right wheels based on the step count.
// Converts steps to distance using a predefined step-to-distance conversion factor (STEP2DISTANCE).
// Input: leftDistance_mm  - pointer to store the distance traveled by the left wheel (in mm)
//        rightDistance_mm - pointer to store the distance traveled by the right wheel (in mm)
// Output: none
void Tachometer_GetDistances(int16_t* leftDistance_mm, int16_t* rightDistance_mm) {
    *leftDistance_mm = TachoLeftSteps * STEP2DISTANCE;   // Convert left steps to distance (mm)
    *rightDistance_mm = TachoRightSteps * STEP2DISTANCE; // Convert right steps to distance (mm)
}


// ------------Tachometer_Get------------
// Retrieves the full set of tachometer data: period, direction, and steps for both wheels.
// Input: leftPeriod_2_3rd_us  - pointer to store the left wheel period (units of 2/3 microseconds)
//        leftDir              - pointer to store the left wheel direction (FORWARD/REVERSE/STOPPED)
//        leftSteps_deg        - pointer to store the left wheel step count (in degrees)
//        rightPeriod_2_3rd_us - pointer to store the right wheel period (units of 2/3 microseconds)
//        rightDir             - pointer to store the right wheel direction (FORWARD/REVERSE/STOPPED)
//        rightSteps_deg       - pointer to store the right wheel step count (in degrees)
// Output: none
void Tachometer_Get(uint16_t* leftPeriod_2_3rd_us, enum TachDirection* leftDir, int16_t* leftSteps_deg,
                   uint16_t* rightPeriod_2_3rd_us, enum TachDirection* rightDir, int16_t* rightSteps_deg) {
    // Retrieve left wheel data
    *leftPeriod_2_3rd_us = (CurrentLeftTime - PreviousLeftTime);  // Calculate left period
    *leftDir = TachoLeftDir;   // Get left direction
    *leftSteps_deg = TachoLeftSteps;   // Get left step count

    // Retrieve right wheel data
    *rightPeriod_2_3rd_us = (CurrentRightTime - PreviousRightTime); // Calculate right period
    *rightDir = TachoRightDir;  // Get right direction
    *rightSteps_deg = TachoRightSteps;  // Get right step count
}
