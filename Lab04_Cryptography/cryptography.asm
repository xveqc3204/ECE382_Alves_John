; cryptography.asm
; Runs on any Cortex M
; Basic XOR encryption and decryption functions for cryptography.
; Capt Steven Beyer
; July 23 2020
; Updated by Stan Baek
; June 03 2021
;
;Simplified BSD License (FreeBSD License)
;Copyright (c) 2020, Steven Beyer, All rights reserved.
;
;Redistribution and use in source and binary forms, with or without modification,
;are permitted provided that the following conditions are met:
;
;1. Redistributions of source code must retain the above copyright notice,
;   this list of conditions and the following disclaimer.
;2. Redistributions in binary form must reproduce the above copyright notice,
;   this list of conditions and the following disclaimer in the documentation
;   and/or other materials provided with the distribution.
;
;THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
;USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;The views and conclusions contained in the software and documentation are
;those of the authors and should not be interpreted as representing official
;policies, either expressed or implied, of the FreeBSD Project.


       .thumb
       .text
       .align 2
       .global Encrypt
       .global Decrypt
       .global XOR_bytes

EOM    .byte  '#'


;------------ Encrypt ------------
; This subroutine takes the address of a plain text message and a key,
;   then encrypts the message by performing an XOR operation.
;   The encrypted result is stored at the specified memory location.
;   The encryption stops when EOM is encountered.
;
; Inputs:
;   R0 - Address of the plain text message
;   R1 - Key for XOR encryption
;   R2 - Address to store the encrypted message
;
; Return:
;   None
Encrypt:   .asmfunc
; ============ Add your code and comments below ===================

    ; Use EOM defined at line 44 for '#'
    PUSH {LR}
    PUSH {R4-R11}

    LDRB R4, EOM
    MOV  R5,  R0 ;unencrypted
    MOV  R6,  R1  ;key
    MOV  R7,  R2 ;EMAddr

Loop1:
    LDRB R8, [R5], #1 ;uncrypted++
    MOV  R0,  R8 ;unencrypted -> input
    MOV  R1,  R6 ;key -> input
    BL XOR_bytes        ;XOR_Bytes

    STRB R0, [R7], #1 ;store XORed byte to EMAddr then ++

    CMP  R8, R4         ;if bit is equal to #
    BNE  Loop1

    POP {R4-R11}
    POP {LR}
    BX   LR

; =============== End of your code ================================
    .endasmfunc


;------------ Decrypt ------------
; This subroutine takes the address of an encrypted message and a key,
;   then decrypts the message by performing an XOR operation.
;   The decrypted result is stored at the specified memory location.
;   The decryption stops when the EOM is encountered.
;
; Inputs:
;   R0 - Address of the encrypted message
;   R1 - Key for XOR decryption
;   R2 - Address to store the decrypted message
;
; Return:
;   None
Decrypt:    .asmfunc
; ============ Add your code and comments below ===================

    PUSH {LR}
    PUSH {R4-R11}

    LDRB R4, EOM
    MOV  R5,  R0  ;unencrypted
    MOV  R6,  R1  ;key
    MOV  R7,  R2  ;DMAddr

Loop2:
    LDRB R8, [R5], #1
    MOV  R0,  R8
    MOV  R1,  R6
    BL XOR_bytes         ;XOR_Bytes

    STRB R0, [R7], #1

    CMP  R0, R4          ;if bit is equal to #
    BNE  Loop2

    POP {R4-R11}
    POP {LR}
    BX  LR

; =============== End of your code ================================
    .endasmfunc
    .end
