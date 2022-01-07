;/*
;* hwmul.inc
;*
;*  Created: 19.01.2015 19:45:10
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
	MUL R16, R17
	PUSH R0
	MOV R0, R1
	POP R1
RET
;#################################################################################
;                                      UMUL16
;#################################################################################
UMUL16: ; R16, R17 = Multiplikand, R18, R19 = Multiplicator
PUSH R4
PUSH R5
PUSH R6
	CLR R2
	MUL R17, R19
		CLR R3
		CLR R4
		MOV R5, R1
		MOV R6, R0
	MUL R17, R18
		ADD R5, R0
		ADC R4, R1
		ADC R3, R2
	MUL R16, R19
		ADD R5, R0
		ADC R4, R1
		ADC R3, R2
	MUL R16, R18
		ADD R4, R0
		ADC R3, R1
	MOV R0, R3
	MOV R1, R4
	MOV R2, R5
	MOV R3, R6
POP R6
POP R5
POP R4
RET ; Return R0 - R3
;#################################################################################
;                                      UMUL24
;#################################################################################
UMUL24: ; R16, R17, R18 = Multiplikand, R19, R20, R21 = Multiplicator
PUSH R6
PUSH R7
PUSH R8
	CLR R2
	MUL R18, R21
		CLR R3
		CLR R4
		CLR R5
		CLR R6
		MOV R7, R1
		MOV R8, R0
	MUL R18, R20
		ADD R7, R0
		ADC R6, R1
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R18, R19
		ADD R6, R0
		ADC R5, R1
		ADC R4, R2
		ADC R3, R2
	MUL R17, R21
		ADD R7, R0
		ADC R6, R1
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R17, R20
		ADD R6, R0
		ADC R5, R1
		ADC R4, R2
		ADC R3, R2
	MUL R17, R19
		ADD R5, R0
		ADC R4, R1
		ADC R3, R2
	MUL R16, R21	
		ADD R6, R0
		ADC R5, R1
		ADC R4, R2
		ADC R3, R2
	MUL R16, R20
		ADD R5, R0
		ADC R4, R1
		ADC R3, R2
	MUL R16, R19
		ADD R4, R0
		ADC R3, R1
	MOV R0, R3 
	MOV R1, R4
	MOV R2, R5
	MOV R3, R6
	MOV R4, R7
	MOV R5, R8
POP R8
POP R7
POP R6
RET
;#################################################################################
;                                      UMUL32
;#################################################################################
UMUL32: ; R16, R17, R18, R19 = Multiplikand, R20, R21, R22, R23 = Multiplicator
PUSH R8
PUSH R9
PUSH R10
	CLR R2
	MUL R19, R23
		CLR R3
		CLR R4
		CLR R5
		CLR R6
		CLR R7
		CLR R8
		MOV R9,  R1
		MOV R10, R0
	MUL R19, R22
		ADD R9, R0
		ADC R8, R1
		ADC R7, R2
		ADC R6, R2
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R19, R21
		ADD R8, R0
		ADC R7, R1
		ADC R6, R2
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R19, R20
		ADD R7, R0
		ADC R6, R1
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R18, R23
		ADD R9, R0
		ADC R8, R1
		ADC R7, R2
		ADC R6, R2
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R18, R22
		ADD R8, R0
		ADC R7, R1
		ADC R6, R2
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R18, R21
		ADD R7, R0
		ADC R6, R1
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R18, R20
		ADD R6, R0
		ADC R5, R1
		ADC R4, R2
		ADC R3, R2
	MUL R17, R23
		ADD R8, R0
		ADC R7, R1
		ADC R6, R2
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R17, R22
		ADD R7, R0
		ADC R6, R1
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R17, R21
		ADD R6, R0
		ADC R5, R1
		ADC R4, R2
		ADC R3, R2
	MUL R17, R20
		ADD R5, R0
		ADC R4, R1
		ADC R3, R2
	MUL R16, R23
		ADD R7, R0
		ADC R6, R1
		ADC R5, R2
		ADC R4, R2
		ADC R3, R2
	MUL R16, R22
		ADD R6, R0
		ADC R5, R1
		ADC R4, R2
		ADC R3, R2
	MUL R16, R21
		ADD R5, R0
		ADC R4, R1
		ADC R3, R2
	MUL R16, R20
		ADD R4, R0
		ADC R3, R1
	MOV R0, R3 
	MOV R1, R4
	MOV R2, R5
	MOV R3, R6
	MOV R4, R7
	MOV R5, R8
	MOV R6, R9
	MOV R7, R10
POP R10
POP R9
POP R8
RET

