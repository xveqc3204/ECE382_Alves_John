/**
 * @file      Tachometer.h
 * @brief     Take tachometer angle and distance measurements
 * @details   Provide mid-level functions that initialize ports,
 * take angle and distance measurements, and report total travel
 * statistics.
 * @version   TI-RSLK MAX v1.1
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2019 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      June 28, 2019
 *

<table>
<caption id="tach_interface">Romi Encoder connections</caption>
<tr><th>MSP432  <th>Romi Encoder  <th>comment
<tr><td>P10.5 (J5) <td>ELA           <td>Left Encoder A, used for speed
<tr><td>P5.2 (J5) <td>ELB           <td>Left Encoder B, used for direction
<tr><td>P10.4 (J5)<td>ERA           <td>Right Encoder A, used for speed
<tr><td>P5.0 (J5)<td>ERB           <td>Right Encoder B, used for direction
</table>
 ******************************************************************************/

// Comments revised by Stan Baek
// October 21, 2024

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

/*!
 * @defgroup RSLK_Input_Output
 * @brief
 * @{*/

#ifndef TACHOMETER_H_
#define TACHOMETER_H_


// Left Encoder A (Speed) connected to P10.5 (J5)
// Left Encoder B (Displacement) connected to P5.2 (J2.12)
// Right Encoder A (Speed) connected to P10.4 (J5)
// Right Encoder B (Displacement) connected to P5.0 (J2.13)


/**
 * \brief Specifies the direction of motor rotation relative to the front of the robot.
 */
enum TachDirection {
  FORWARD,  // Wheel moving the robot forward
  STOPPED,  // Wheel stopped
  REVERSE   // Wheel moving the robot backward
};

// Convert number of pulses (timer period) to RPM.
// speed (RPM) = PULSE2RPM / pulses
#define PULSE2RPM   250000


// Convert number of steps (angle in degrees) to distance in mm.
// distance (mm) = steps * STEP2DISTANCE
// Wheel circumference = 220 mm.
// Gear ratio = 120:1, 3 N poles in tachometer = 360 pulses per wheel rotation.
// 1 wheel step = 1 degree rotation of the wheel.
// 1 full wheel revolution (360 degrees) = 220 mm of displacement.
#define STEP2DISTANCE  220/360 // no parentheses should be added, e.g., (220/360)


/**
 * Initializes GPIO pins for input to determine motor direction and
 * sets up the input capture interface for speed measurement.
 * @param none
 * @return none
 * @brief Initializes the tachometer interface.
 */
void Tachometer_Init(void);


/**
 * Retrieves the most recent tachometer speed measurements.
 * @param leftPeriod_2_3rd_us Pointer to store the left wheel tachometer period (in units of 2/3 microseconds).
 * @param rightPeriod_2_3rd_us Pointer to store the right wheel tachometer period (in units of 2/3 microseconds).
 * @return none
 * @note Assumes Tachometer_Init() and Clock_Init48MHz() have been called.
 * @brief Retrieves the most recent tachometer speeds.
 */
void Tachometer_GetSpeeds(uint16_t *leftPeriod_2_3rd_us, uint16_t *rightPeriod_2_3rd_us);


/**
 * Retrieves the most recent direction of the wheels.
 * @param leftDir Pointer to store the direction of the left wheel (FORWARD, REVERSE, or STOPPED).
 * @param rightDir Pointer to store the direction of the right wheel (FORWARD, REVERSE, or STOPPED).
 * @return none
 * @note Assumes Tachometer_Init() and Clock_Init48MHz() have been called.
 * @brief Retrieves the most recent tachometer directions.
 */
void Tachometer_GetDirections(enum TachDirection *leftDir, enum TachDirection *rightDir);


/**
 * Retrieves the number of forward steps taken by each wheel.
 * @param leftSteps_deg Pointer to store the number of forward steps for the left wheel (1 full wheel rotation = 360 steps).
 * @param rightSteps_deg Pointer to store the number of forward steps for the right wheel (1 full wheel rotation = 360 steps).
 * @return none
 * @note Assumes Tachometer_Init() and Clock_Init48MHz() have been called.
 * @brief Retrieves the most recent tachometer step count.
 */
void Tachometer_GetSteps(int16_t *leftSteps_deg, int16_t *rightSteps_deg);


/**
 * Retrieves the distances traveled by the wheels since the last reset.
 * @param leftDistance_mm Pointer to store the distance traveled by the left wheel in mm.
 * @param rightDistance_mm Pointer to store the distance traveled by the right wheel in mm.
 * @return none
 * @note Assumes Tachometer_Init() and Clock_Init48MHz() have been called.
 * @brief Retrieves the distances traveled by the wheels.
 */
void Tachometer_GetDistances(int16_t* leftDistance_mm, int16_t* rightDistance_mm);


/**
 * Resets the tachometer step counts for both wheels.
 * @param none
 * @return none
 * @note Assumes Tachometer_Init() and Clock_Init48MHz() have been called.
 * @brief Resets the tachometer step counts.
 */
void Tachometer_ResetSteps(void);


/**
 * Retrieves the most recent tachometer measurements, including speed, direction, and steps.
 * @param leftPeriod_2_3rd_us Pointer to store the left wheel tachometer period (in units of 2/3 microseconds).
 * @param leftDir Pointer to store the direction of the left wheel.
 * @param leftSteps Pointer to store the number of forward steps for the left wheel.
 * @param rightPeriod_2_3rd_us Pointer to store the right wheel tachometer period (in units of 2/3 microseconds).
 * @param rightDir Pointer to store the direction of the right wheel.
 * @param rightSteps Pointer to store the number of forward steps for the right wheel.
 * @return none
 * @note Assumes Tachometer_Init() and Clock_Init48MHz() have been called.
 * @brief Retrieves the most recent tachometer data for both wheels.
 */
void Tachometer_Get(uint16_t *leftPeriod_2_3rd_us,
                    enum TachDirection *leftDir,
                    int16_t *leftSteps,
                    uint16_t *rightPeriod_2_3rd_us,
                    enum TachDirection *rightDir,
                    int16_t *rightSteps);


#endif /* TACHOMETER_H_ */
