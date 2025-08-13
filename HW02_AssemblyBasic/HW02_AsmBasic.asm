; Assembly Basic
; Runs on any Cortex M
; Stan Baek
; 11 May 2022
;
;Simplified BSD License (FreeBSD License)
;Copyright (c) 2022, Stan Baek All rights reserved.
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

; =================================================================
; CAREFULLY READ EVERY COMMENT TO ENSURE WHAT EACH LINE IS DOING!
; Note: The comments here show lexicon for beginners in Assembly programming,
; so you should not follow that style.
; =================================================================

        .thumb
        .global  main       ; make the symbol visible to the linker.

main:  .asmfunc             ; begin assembly main function

        ; Observe the R0-R8 values
        MOV     R0, #0x1212     ; R0 <= 0x1212
        MOV     R1, #0xFF00     ; R1 <= 0xFF00
        AND     R2, R0, R1      ; R2 <= R0 AND R1
        ORR     R3, R0, R1      ; R3 <= R0 OR R1
        EOR     R4, R0, R1      ; R4 <= R0 ^ R1
        BIC     R5, R0, R1      ;

        MOV     R0, #100        ; R0 <= 100
        MOV     R1, R0          ; R1 <= R0
        ADD     R1, #2          ; R1 <= R1 + 2
        ADD     R2, R0, R1      ; R2 <= R0 + R1
        SUB     R2, R2, #100    ; R2 <= R2 - 100
        SUB     R1, R2, R0      ; R1 <= R2 - R0

        MOV     R0, #382        ; R0 <= 382
        MOV     R1, #10         ; R1 <= 10
        UDIV    R2, R0, R1      ; R2 <= R0/R1
        MUL     R3, R2, R1      ; R3 <= R2*R1
        SUB     R4, R0, R3      ; R3 <= 382 % 10 (modulo or remainder)

L1      MOV     R0, #0
Loop1   CMP     R0, #5          ; Observe PSR after executing this instruction.
        BEQ     L2              ; if R0 == 5, go to L2
        ADD     R0, #1          ; R0 <= R0++
        B       Loop1           ; Go to Loop1

L2      MOV     R0, #5
Loop2   SUB     R0, #1
        CMP     R0, #1          ; Observe PSR after executing this instruction.
        BHS     Loop2           ; Brach Higher or Same, if R0 >= 0, go to Loop2
                                ; If R0 < 0, exit the loop.


Stall   B   Stall               ; stall here - stay here forever, equivalent to while(1); in C
                                ; Inspect the memory at the address stored in ResAddr
       .endasmfunc

       .end

