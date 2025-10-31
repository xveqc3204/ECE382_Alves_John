/**
 * @file      TA3InputCapture.h
 * @brief     Initialize Timer A3
 * @details   Use Timer A3 in capture mode to request interrupts on rising
 * edges of P10.4 (TA3CCP0) and P8.2 (TA3CCP2) and call user functions.<br>
 * Use Timer A3 in capture mode to request interrupts on rising
 * edges of P10.4 (TA3CCP0) and P10.5 (TA3CCP1) and call user functions.
 * @version   TI-RSLK MAX v1.1
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2019 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      June 28, 2019
 ******************************************************************************/

// Comments revised by Stan Baek
// October 21, 2024tan Baek
// October 21, 2024

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019

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

// external signal connected to P10.5 (TA3CCP1) (trigger on rising edge)
// external signal connected to P10.4 (TA3CCP0) (trigger on rising edge)

#ifndef TA3INPUTCAPTURE_H_
#define TA3INPUTCAPTURE_H_

/**
 * Initializes Timer A3 in edge time mode to trigger interrupts on
 * the rising edges of P10.4 (TA3CCR0) and P10.5 (TA3CCR1). When an interrupt occurs,
 * the corresponding user-defined function is called with the current timer value.
 *
 * @param task0 Pointer to the function that will be called when a rising edge is detected on P10.4 (TA3CCR0).
 *              The function takes a 16-bit timer value (in units of 2/3 microseconds) as its parameter.
 * @param task1 Pointer to the function that will be called when a rising edge is detected on P10.5 (TA3CCR1).
 *              The function takes a 16-bit timer value (in units of 2/3 microseconds) as its parameter.
 *
 * @return None.
 *
 * @note Assumes that the low-speed subsystem master clock (SMCLK) is running at 12 MHz.
 *
 * @brief Initializes Timer A3 to trigger interrupts on rising edges of signals from P10.4 and P10.5.
 */
void TimerA3Capture_Init(void(*task0)(uint16_t time), void(*task1)(uint16_t time));


void TimerA3Capture_Init2(void(*task0)(uint16_t time), void(*task1)(uint16_t time), uint8_t priority);

#endif /* TA3INPUTCAPTURE_H_ */
