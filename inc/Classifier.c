/*
 * Classifier.c
 * Runs on MSP432
 *
 *
 * Conversion function for a GP2Y0A21YK0F IR sensor and classification function
 *  to take 3 distance readings and determine what the current state of the robot is.
 *
 *  Created on: Jul 24, 2020
 *  Author: Captain Steven Beyer
 *
 * This example accompanies the book
 * "Embedded Systems: Introduction to Robotics,
 * Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 * For more information about my classes, my research, and my books, see
 * http://users.ece.utexas.edu/~valvano/
 *
 * Simplified BSD License (FreeBSD License
 *
 * Copyright (c) 2019, Jonathan Valvano, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 */

#include <stdint.h>
#include "../inc/Classifier.h"


// Complete the following lines
#define SIDEMIN    212   // smallest side distance to the wall in mm
#define SIDEMAX    354   // largest side distance to wall in mm
#define CENTERMIN  150   // min distance to the wall in the front
#define CENTEROPEN 600   // distance to the wall between open/blocked
#define IRMIN      50    // min possible reading of IR sensor
#define IRMAX      800   // max possible reading of IR sensor


/* Classify
* Utilize three distance values from the left, center, and right
* distance sensors to determine and classify the situation into one
* of many scenarios (enumerated by scenario)
*
* Input
*   int32_t Left: distance measured by left sensor
*   int32_t Center: distance measured by center sensor
*   int32_t Right: distance measured by right sensor
*
* Output
*   scenario_t: value representing the scenario the robot
*       currently detects (e.g. RightTurn, LeftTurn, etc.)
*/
scenario_t Classify(int32_t left_mm, int32_t center_mm, int32_t right_mm) {

    scenario_t result = ClassificationError;
    int state = 0;

    //ClassificationError (0) if any of the sensors report distances less than 50 mm or greater than 800 mm.
    if ((left_mm < IRMIN)|| (center_mm < IRMIN) || (right_mm < IRMIN) ||
       (left_mm > IRMAX) || (center_mm > IRMAX) || (right_mm > IRMAX)){
        return ClassificationError;
    }

    //LeftTooClose (1) if the left sensor reports a distance less than 212 mm.
    if (left_mm < SIDEMIN){
        state += LeftTooClose;
    }
    //RightTooClose (2) if the right sensor reports a distance less than 212 mm.
    if (right_mm < SIDEMIN){
        state += RightTooClose;
    }
    //CenterTooClose (4) if the center sensor reports a distance less than 150 mm.
    if (center_mm < CENTERMIN){
        state += CenterTooClose;
    }

    //In the case where it does not read anything, return ClassificationError
    if(state){
        return (scenario_t)state;
    }

    //{Blocked, Right Turn, Left Turn, Tee Joint}: When the center sensor measures a distance less than 600 mm.
    if (center_mm < CENTEROPEN){
        if ((left_mm >= SIDEMAX) && (right_mm >= SIDEMAX)){
            result = TeeJoint;
        }
        else if (right_mm >= SIDEMAX){
            result = RightTurn;
        }
        else if (left_mm >= SIDEMAX){
            result = LeftTurn;
        }
        else if (center_mm >= CENTERMIN){
            result = Blocked;
        }
    }
    //{Straight, Right Joint, Left Joint, Cross Road}: When the center sensor measures a distance greater than or equal to 600 mm.
    if (center_mm >= CENTEROPEN){
        if ((left_mm >= SIDEMAX) && (right_mm >= SIDEMAX)){
            result = CrossRoad;
        }
        else if (right_mm >= SIDEMAX){
            result = RightJoint;
        }
        else if (left_mm >= SIDEMAX){
            result = LeftJoint;
        }
        else if (center_mm >= CENTERMIN){
            result = Straight;
        }
    }

    return result;
}

