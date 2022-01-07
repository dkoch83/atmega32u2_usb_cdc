;/*
;* math.inc
;*
;*  Created: 15.07.2014 21:09:47
;*   Author: Dominik
;*/ 


.IFDEF HWMUL
	.INCLUDE "./include/hwmul.inc"
.ELSE
	.INCLUDE "./include/swmul.inc"
.ENDIF


;#################################################################################
;                                      DIV16
;#################################################################################
DIV16: ; R16 H, R17 L = Dividend, R18 H , R19 L = Divisor
PUSH R17
PUSH R19
PUSH R22
PUSH R23
PUSH R18
PUSH R16
	CLR R0	; Ergebnis 
	CLR R1  
	CLR R2	; Rest
	CLR R3
	CLR R4	; TMP 1
	CLR R5
	LDI R22, 16 ; 16 Bit Division

	TST R16	; Test for Minus
	BRPL DIV16_NO_COM1
	COM R16
	COM R17
	
	LDI R23, 1
	ADD R17, R23
	LDI R23, 0
	ADC R16, R23
	DIV16_NO_COM1: 
	TST R18	; Test for Minus
	BRPL DIV16_NO_COM2	 
	COM R18
	COM R19

	LDI R23, 1
	ADD R19, R23
	LDI R23, 0
	ADC R18, R23
	DIV16_NO_COM2:
DIV16_LOOP:
	LSL R17
	ROL R16

	ROL R5
	ROL R4
	
	LSL R1
	ROL R0

	CP  R5, R19
	CPC R4, R18
	BRLO DIV16_ZERO

	SUB R5, R19
	SBC R4, R18

	INC R1
DIV16_ZERO:
	DEC R22
	BRNE DIV16_LOOP
	MOV R2, R4
	MOV R3, R5

	POP R16
	TST R16	; Test for Minus
	BRPL DIV16_NOT_NEG1
	COM R0
	COM R1
	LDI R23, 1
	ADD R1, R23
	LDI R23, 0
	ADC R0, R23

	COM R2
	COM R3
	LDI R23, 1
	ADD R3, R23
	LDI R23, 0
	ADC R2, R23
DIV16_NOT_NEG1:
	POP R18
	TST R18	; Test for Minus
	BRPL DIV16_NOT_NEG2
	COM R0
	COM R1
	LDI R23, 1
	ADD R1, R23
	LDI R23, 0
	ADC R0, R23

	COM R2
	COM R3
	LDI R23, 1
	ADD R3, R23
	LDI R23, 0
	ADC R2, R23
DIV16_NOT_NEG2:
POP R23
POP R22
POP R19
POP R17
RET
;#################################################################################
;                                      UDIV8
;#################################################################################
UDIV8: ; R16 = Dividend, R17 = Divisor
PUSH R16
PUSH R17
PUSH R24
	CLR R0	; Ergebnis 
	CLR R1  ; Rest
	LDI R24, 8 ; 8 Bit Division
UDIV8_LOOP:
	LSL R16

	ROL R1
	
	LSL R0
	
	CP R1, R17
	BRLO UDIV8_ZERO
	SUB R1, R17
	INC R0
UDIV8_ZERO:
	DEC R24
	BRNE UDIV8_LOOP
POP R24
POP R17
POP R16
RET
;#################################################################################
;                                      UDIV16
;#################################################################################
UDIV16: ; R16 H, R17 L = Dividend, R18 H , R19 L = Divisor
PUSH R16
PUSH R17
PUSH R18
PUSH R19
PUSH R24
	CLR R0	; Ergebnis 
	CLR R1  
	CLR R2	; Rest
	CLR R3
	LDI R24, 16 ; 16 Bit Division
UDIV16_LOOP:
	LSL R17
	ROL R16

	ROL R3
	ROL R2
	
	LSL R1
	ROL R0

	CP  R3, R19
	CPC R2, R18
	BRLO UDIV16_ZERO

	SUB R3, R19
	SBC R2, R18

	INC R1
UDIV16_ZERO:
	DEC R24
	BRNE UDIV16_LOOP
POP R24
POP R19
POP R18
POP R17
POP R16
RET
;#################################################################################
;                                      UDIV24
;#################################################################################
UDIV24: ; R16 H, R17, R18 L = Dividend, R19 H , R20, R21 L = Divisor
PUSH R16
PUSH R17
PUSH R18
PUSH R19
PUSH R20
PUSH R21
PUSH R24
	CLR R0	; Ergebnis 
	CLR R1  
	CLR R2	
	CLR R3	; Rest
	CLR R4
	CLR R5
	LDI R24, 24 ; 24 Bit Division
UDIV24_LOOP:
	LSL R18
	ROL R17
	ROL R16

	ROL R5
	ROL R4
	ROL R3
	
	LSL R2
	ROL R1
	ROL R0

	CP	R5, R21
	CPC R4, R20
	CPC R3, R19
	BRLO UDIV24_ZERO

	SUB R5, R21
	SBC R4, R20
	SBC R3, R19

	INC R2
UDIV24_ZERO:
	DEC R24
	BRNE UDIV24_LOOP
POP R24
POP R21
POP R20
POP R19
POP R18
POP R17
POP R16
RET
;#################################################################################
;                                      UDIV32
;#################################################################################
UDIV32: ; R16 H, R17, R18, R19 L = Dividend, R20 H, R21, R22, R23 L = Divisor
PUSH R8
PUSH R9
PUSH R10
PUSH R11
PUSH R16
PUSH R17
PUSH R18
PUSH R19
PUSH R20
PUSH R21
PUSH R22
PUSH R23
PUSH R24
	CLR R0	; Ergebnis 
	CLR R1  
	CLR R2	
	CLR R3	
	CLR R4	; Rest
	CLR R5
	CLR R6	
	CLR R7	
	CLR R8	; TMP 1
	CLR R9
	CLR R10
	CLR R11
	LDI R24, 32 ; 32 Bit Division
UDIV32_LOOP:
	LSL R19
	ROL R18
	ROL R17
	ROL R16

	ROL R11
	ROL R10
	ROL R9
	ROL R8

	LSL R3
	ROL R2
	ROL R1
	ROL R0

	CP	R11, R23
	CPC R10, R22
	CPC  R9, R21
	CPC  R8, R20
	BRLO UDIV32_ZERO

	SUB R11, R23
	SBC R10, R22
	SBC R9 , R21
	SBC R8 , R20

	INC R3
UDIV32_ZERO:
	DEC R24
	BRNE UDIV32_LOOP
	MOV R4, R8
	MOV R5, R9
	MOV R6, R10
	MOV R7, R11
POP R24
POP R23
POP R22
POP R21
POP R20
POP R19
POP R18
POP R17
POP R16
POP R11
POP R10
POP R9
POP R8
RET
;#################################################################################
;                                      UDIV64
;#################################################################################
.DSEG
DIV64_COUNT: .BYTE 1
.CSEG
UDIV64: ; H R16:R17:R18:R19:R20:R21:R22:R23 L = Dividend, H R24:R25:R26:R27:R28:R29:R30:R31 L = Divisor
PUSH R16
PUSH R17
PUSH R18
PUSH R19
PUSH R20
PUSH R21
PUSH R22
PUSH R23
PUSH R24
PUSH R25
PUSH R26
PUSH R27
PUSH R28
PUSH R29
PUSH R30
PUSH R31
	CLR R0	; Ergebnis 
	CLR R1
	CLR R2
	CLR R3
	CLR R4
	CLR R5
	CLR R6
	CLR R7
	CLR R8	; Rest ; TMP 1
	CLR R9
	CLR R10
	CLR R11
	CLR R12
	CLR R13
	CLR R14
	CLR R15

	PUSH R24
	LDI R24, 64 ; 64 Bit Division
	STS DIV64_COUNT, R24
	POP R24

	UDIV64_LOOP:
		LSL R23
		ROL R22
		ROL R21
		ROL R20
		ROL R19
		ROL R18
		ROL R17
		ROL R16

		ROL R15
		ROL R14
		ROL R13
		ROL R12
		ROL R11
		ROL R10
		ROL R9
		ROL R8

		LSL R7
		ROL R6
		ROL R5
		ROL R4
		ROL R3
		ROL R2
		ROL R1
		ROL R0

		CP R15, R31
		CPC R14, R30
		CPC R13, R29
		CPC R12, R28
		CPC	R11, R27
		CPC R10, R26
		CPC  R9, R25
		CPC  R8, R24
		BRLO UDIV64_ZERO

		SUB R15, R31
		SBC R14, R30
		SBC R13, R29
		SBC R12, R28
		SBC R11, R27
		SBC R10, R26
		SBC R9 , R25
		SBC R8 , R24

		INC R7
	UDIV64_ZERO:
		PUSH R24
		LDS R24, DIV64_COUNT
		DEC R24
		STS DIV64_COUNT, R24
		POP R24
		BRNE UDIV64_LOOP
POP R31
POP R30
POP R29
POP R28
POP R27
POP R26
POP R25
POP R24
POP R23
POP R22
POP R21
POP R20
POP R19
POP R18
POP R17
POP R16
RET