;/*
;* swmul.inc
;*
;*  Created: 19.01.2015 19:45:23
;*   Author: Dominik
;*/ 

;---------------------------------------------------------------------------------
;										Unsigned
;---------------------------------------------------------------------------------

;#################################################################################
;                                      MUL16
;#################################################################################
MUL16: ; R16:R17 = Multiplikand, R18:R19 = Multiplicator
PUSH R23
PUSH R22
PUSH R17
PUSH R19

PUSH R18
PUSH R16
	CLR R0
	CLR R1 

	LDI R22, 16

	TST R16	; Test for Minus
	BRPL MUL16_NO_COM1
	COM R16
	COM R17
	
	LDI R23, 1
	ADD R17, R23
	LDI R23, 0
	ADC R16, R23
	MUL16_NO_COM1: 
	TST R18	; Test for Minus
	BRPL MUL16_NO_COM2	 
	COM R18
	COM R19
	LDI R23, 1
	ADD R19, R23
	LDI R23, 0
	ADC R18, R23
	MUL16_NO_COM2:

MUL16_LOOP:
	LSL R1
	ROL R0

	LSL R19
	ROL R18
	BRCC MUL16_NULL

	ADD R1, R17
	ADC R0, R16
MUL16_NULL:
	DEC R22
BRNE MUL16_LOOP
	
	POP R16
	TST R16	; Test for Minus
	BRPL MUL16_NOT_NEG1
	COM R0
	COM R1
	LDI R23, 1
	ADD R1, R23
	LDI R23, 0
	ADC R0, R23

MUL16_NOT_NEG1:
	POP R18
	TST R18	; Test for Minus
	BRPL MUL16_NOT_NEG2
	COM R0
	COM R1

	LDI R23, 1
	ADD R1, R23
	LDI R23, 0
	ADC R0, R23
MUL16_NOT_NEG2:
	POP R19
	POP R17
	POP R22
	POP R23
RET
;---------------------------------------------------------------------------------
;										Unsigned
;---------------------------------------------------------------------------------

;#################################################################################
;                                      UMUL8
;#################################################################################
UMUL8: ; R16 = Multiplikand, R17 = Multiplicator
PUSH R16
PUSH R17
PUSH R24
PUSH R2
	CLR R0
	CLR R1 
	CLR R2
	LDI R24, 8
UMUL8_LOOP:
	LSL R1
	ROL R0
	
	LSL R17
	BRCC UMUL8_NULL
	ADD R1, R16
	ADC R0, R2
UMUL8_NULL:
	DEC R24
	BRNE UMUL8_LOOP
POP R2
POP R24
POP R17
POP R16
RET
;#################################################################################
;                                      UMUL16
;#################################################################################
UMUL16: ; R16, R17 = Multiplikand, R18, R19 = Multiplicator
PUSH R16
PUSH R17
PUSH R18
PUSH R19
PUSH R24
	CLR R0
	CLR R1 
	CLR R2 
	CLR R3
	CLR R4
	LDI R24, 16
UMUL16_LOOP:
	LSL R3
	ROL R2
	ROL R1
	ROL R0

	LSL R19
	ROL R18
	BRCC UMUL16_NULL
	ADD R3, R17
	ADC R2, R16
	ADC R1, R4
UMUL16_NULL:
	DEC R24
	BRNE UMUL16_LOOP
POP R24
POP R19
POP R18
POP R17
POP R16
RET ; Return R0 - R3
;#################################################################################
;                                      UMUL24
;#################################################################################
UMUL24: ; R16, R17, R18 = Multiplikand, R19, R20, R21 = Multiplicator
PUSH R6
PUSH R22
	CLR R0
	CLR R1 
	CLR R2 
	CLR R3
	CLR R4
	CLR R5
	CLR R6

	LDI R22, 24
UMUL24_LOOP:
	LSL R5
	ROL R4
	ROL R3
	ROL R2
	ROL R1
	ROL R0

	LSL R21
	ROL R20
	ROL R19
	BRCC UMUL24_NULL
	ADD R5, R18
	ADC R4, R17
	ADC R3, R16
	ADC R2, R6
UMUL24_NULL:
	DEC R22
	BRNE UMUL24_LOOP
POP R22
POP R6
RET
;#################################################################################
;                                      UMUL32
;#################################################################################
UMUL32: ; R16, R17, R18, R19 = Multiplikand, R20, R21, R22, R23 = Multiplicator
PUSH R24
PUSH R8
	CLR R0
	CLR R1 
	CLR R2 
	CLR R3
	CLR R4
	CLR R5
	CLR R6
	CLR R7
	CLR R8

	LDI R24, 32
UMUL32_LOOP:
	LSL R7
	ROL R6
	ROL R5
	ROL R4
	ROL R3
	ROL R2
	ROL R1
	ROL R0

	LSL R23
	ROL R22
	ROL R21
	ROL R20
	BRCC UMUL32_NULL
	ADD R7, R19
	ADC R6, R18
	ADC R5, R17
	ADC R4, R16
	ADC R3, R8
UMUL32_NULL:
	DEC R24
	BRNE UMUL32_LOOP
POP R8
POP R24
RET

