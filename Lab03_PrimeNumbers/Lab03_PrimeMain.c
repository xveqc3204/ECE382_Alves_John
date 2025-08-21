// main.c
// Runs on MSP432
// Determine whether a number is prime or composite.
// Stan Baek
// July 11, 2021
/*
Simplified BSD License (FreeBSD License)
Copyright (c) 2021, Stan Baek, All rights reserved.

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


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"


/**
 * This code performs prime number checking on an array of positive integers
 * and stores the results in another array. The process involves iterating
 * through the array of integers and determining whether each number is prime
 * or composite. It does so by checking if the number is divisible by any
 * integer between 2 and half of the number's value (i.e., n/2).
 */
void main(void) {

    // Nums: An array of positive integers to test for prime or composite numbers.
    // The value 0 at the end of the array marks the end of the array.
    uint32_t Nums[16] = {1, 2, 7, 10, 15, 62, 97, 282, 408, 467, 967, 6687, 25913, 25481, 118061, 0};

    // The result should be {-1, 1, 1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, x};
    // where 1 indicates a prime number, -1 indicates a composite number,
    // and x denotes a "don't care" value.
    int8_t Res[16];  // Result array; 1 for prime, -1 for composite numbers.

    const uint32_t* NumsAddr = Nums;    // Pointer to the first element of the Nums array.
    int8_t* ResAddr = Res;              // Pointer to the first element of the Res array.

    while (*NumsAddr) {     // Loop through the numbers until reaching the value 0.

        if (*NumsAddr == 1) {
            // Special case: 1 is neither prime nor composite.
            *ResAddr = -1;
        } else {
            // Check if the current number is prime by testing divisibility.
            // Only need to check divisibility from 2 to n/2.
            // There are more efficient algorithms, but let's not worry.
            uint8_t is_prime = 1;   // Assume the number is prime.
            uint32_t m = *NumsAddr >> 1; // Set m to n/2 (integer division).

            for (uint32_t i = 2; i <= m; ++i) {  // Check divisibility by i = 2, 3, ..., n/2.
                 if (*NumsAddr % i == 0) {      // If divisible by i, it's composite.
                     is_prime = -1;             // Mark as composite.
                     break;                     // No need to check further divisors.
                 }
            }

            *ResAddr = is_prime;
        }
        NumsAddr++; // Move to the next number in the Nums array.
        ResAddr++;  // Move to the next result slot in the Res array.
    }

    while(1);   // stay here forever


}

