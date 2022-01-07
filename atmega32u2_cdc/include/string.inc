;/*
;* string.inc
;*
;*  Created: 19.08.2014 21:11:57
;*   Author: Dominik
;*/ 

.ifndef string_include
.set string_include = 1


;###############################################################################
;								HEX_STR
;###############################################################################
; From Buffer block size must be divisible by 1
;
; OUT BUFFER SIZE must ((BUFFER FROM SIZE * 3) + 1
;
HEX_STR_CODE: .DB "0123456789ABCDEF"

HEX_STR: ; XH:XL = Pointer FROM BUFFER, YH:YL = Pointer OUT BUFFER, R16 = BUFFER FROM SIZE
PUSH XH
PUSH XL
PUSH YH
PUSH YL
PUSH ZH
PUSH ZL
PUSH R16
PUSH R17
PUSH R18
PUSH R19
PUSH R0
	MOV R19, R16

	LDI ZH, HIGH(HEX_STR_CODE)
	LDI ZL, LOW(HEX_STR_CODE)

	LDI R18, 0x00

	TST R16
	BREQ HEX_STR_END

	CLR R16
	HEX_STR_LOOP1:
		LD R18, X+
		
		MOV R17, R18
		SWAP R17
		ANDI R17, 0x0F
		CALL PGM_READ_BYTE ;ZH:ZL = Pointer Address, R16:R17 = Byte Adress, Return = R0
		ST Y+, R0

		MOV R17, R18
		ANDI R17, 0x0F
		CALL PGM_READ_BYTE ;ZH:ZL = Pointer Address, R16:R17 = Byte Adress, Return = R0
		ST Y+, R0

		;Concat " "
		LDI R18, 0x20
		ST Y+, R18

		DEC R19
	BREQ HEX_STR_END 
	RJMP HEX_STR_LOOP1
	HEX_STR_END:
	LDI R16, 0
	ST -Y, R16
POP R0
POP R19
POP R18
POP R17
POP R16
POP ZL
POP ZH
POP YL
POP YH
POP XL
POP XH
RET
;############################################################################
;                                INT8_TO_STR
;############################################################################
INT8_TO_STR: ; R16 = INT8, X = STR_POINTER
	PUSH R10
	PUSH R23
	PUSH R24
	PUSH R25
	PUSH R18
	PUSH R17
	PUSH R16

	MOV R18, R16

	CLR R10
	LDI R24, 10
	LDI R25, 0
	LDI R23, '0'

	TST R16
	BRMI INT8_TO_STR_NEG1
	RJMP INT8_TO_STR_NEG1_END
	INT8_TO_STR_NEG1:
		POP R16
		COM R16
		INC R16
		INC R10
		PUSH R16
	INT8_TO_STR_NEG1_END:

	CP R16, R24
	BRLO INT8_TO_STR_LOOP1_END
	MOV R17, R24
	INT8_TO_STR_LOOP1:
		RCALL UDIV8 ; R16 = Dividend, R17 = Divisor
		INC R10
		MOV R16, R0
		CP R16, R24
		BRSH INT8_TO_STR_LOOP1
		POP R16
		PUSH R16
	INT8_TO_STR_LOOP1_END:
	INC R10
	INC R10

	ADD XL, R10
	ADC XH, R25
	ST -X, R25
	DEC R10

	TST R18
	BRMI INT8_TO_STR_NEG2
	RJMP INT8_TO_STR_NEG2_END
	INT8_TO_STR_NEG2:
		DEC R10
	INT8_TO_STR_NEG2_END:

	INT8_TO_STR_LOOP2:
		MOV R17, R24
		RCALL UDIV8 ; R16 = Dividend, R17 = Divisor
		ADD R1, R23
		ST -X, R1
		MOV R16, R0
		DEC R10
		BRNE INT8_TO_STR_LOOP2
	INT8_TO_STR_LOOP2_END:
	
	POP R16

	TST R18
	BRMI INT8_TO_STR_NEG3
	RJMP INT8_TO_STR_NEG3_END
	INT8_TO_STR_NEG3:
		LDI R18, '-'
		ST -X, R18
	INT8_TO_STR_NEG3_END:

	POP R17
	POP R18
	POP R25
	POP R24
	POP R23
	POP R10
RET
;############################################################################
;                                UINT8_TO_STR
;############################################################################
UINT8_TO_STR: ; R16 = INT8, X = STR_POINTER
	PUSHX
	PUSH R10
	PUSH R23
	PUSH R24
	PUSH R25
	PUSH R16
	

	CLR R10
	LDI R24, 10
	LDI R25, 0
	LDI R23, '0'

	CP R16, R24
	BRLO UINT8_TO_STR_LOOP1_END
	MOV R17, R24
	UINT8_TO_STR_LOOP1:
		RCALL UDIV8 ; R16 = Dividend, R17 = Divisor
		INC R10
		MOV R16, R0
		CP R16, R24
		BRSH UINT8_TO_STR_LOOP1
		POP R16
		PUSH R16
	UINT8_TO_STR_LOOP1_END:
	INC R10
	INC R10
	ADD XL, R10
	ADC XH, R25
	ST -X, R25
	DEC R10

	UINT8_TO_STR_LOOP2:
		MOV R17, R24
		RCALL UDIV8 ; R16 = Dividend, R17 = Divisor
		ADD R1, R23
		ST -X, R1
		MOV R16, R0
		DEC R10
		BRNE UINT8_TO_STR_LOOP2
	UINT8_TO_STR_LOOP2_END:
	POP R16
	POP R25
	POP R24
	POP R23
	POP R10
	POPX
RET
;############################################################################
;                                UINT16_TO_STR
;############################################################################
UINT16_TO_STR: ; R16:R17 = INT16, X = STR_POINTER
	PUSH R10
	PUSH R23
	PUSH R24
	PUSH R25

	PUSH R18
	PUSH R19
	PUSH R16
	PUSH R17

	CLR R10
	LDI R24, 10
	LDI R25, 0
	LDI R23, '0'

	CP R17, R24
	CPC R16, R25
	BRLO INT16_TO_STR_LOOP1_END

	CLR R18
	MOV R19, R24
	INT16_TO_STR_LOOP1:
		RCALL UDIV16 ; R16 H, R17 L = Dividend, R18 H , R19 L = Divisor
		INC R10
		MOV R16, R0
		MOV R17, R1
		CP R17, R24
		CPC R16, R25
		BRSH INT16_TO_STR_LOOP1

		POP R17
		POP R16

		PUSH R16
		PUSH R17

	INT16_TO_STR_LOOP1_END:
	INC R10
	INC R10

	ADD XL, R10
	ADC XH, R25

	ST -X, R25
	DEC R10
	INT16_TO_STR_LOOP2:
		RCALL UDIV16 ; R16 H, R17 L = Dividend, R18 H , R19 L = Divisor
		ADD R3, R23
		ST -X, R3
		MOV R16, R0
		MOV R17, R1
		DEC R10
		BRNE INT16_TO_STR_LOOP2
	INT16_TO_STR_LOOP2_END:

	POP R17
	POP R16
	POP R19
	POP R18

	POP R25
	POP R24
	POP R23
	POP R10
RET
;############################################################################
;                                UINT24_TO_STR
;############################################################################
UINT24_TO_STR: ; R16:R17:R18 = UINT24, X = STR_POINTER
PUSH R10
PUSH R23
PUSH R24
PUSH R25

PUSH R21
PUSH R20
PUSH R19
PUSH R18
PUSH R17
PUSH R16

	CLR R10
	LDI R24, 10
	LDI R25, 0
	LDI R23, '0'

	CP R18, R24
	CPC R17, R25
	CPC R16, R25
	BRLO UINT24_TO_STR_LOOP1_END

	CLR R19
	CLR R20
	MOV R21, R24
	UINT24_TO_STR_LOOP1:
		RCALL UDIV24 ; R16 H, R17, R18 L = Dividend, R19 H , R20, R21 L = Divisor
		INC R10
		MOV R16, R0
		MOV R17, R1
		MOV R18, R2
		
		CP R18, R24
		CPC R17, R25
		CPC R16, R25
		BRSH UINT24_TO_STR_LOOP1

		POP R16
		POP R17
		POP R18

		PUSH R18
		PUSH R17
		PUSH R16

	UINT24_TO_STR_LOOP1_END:
	INC R10
	INC R10
	ADD XL, R10
	ADC XH, R25

	ST -X, R25
	DEC R10
	UINT24_TO_STR_LOOP2:
		RCALL UDIV24 ; R16 H, R17, R18 L = Dividend, R19 H , R20, R21 L = Divisor
		ADD R5, R23
		ST -X, R5
		MOV R16, R0
		MOV R17, R1
		MOV R18, R2
		DEC R10
		BRNE UINT24_TO_STR_LOOP2
	UINT24_TO_STR_LOOP2_END:
POP R16
POP R17
POP R18
POP R19
POP R20
POP R21

POP R25
POP R24
POP R23
POP R10
RET
;############################################################################
;                                UINT32_TO_STR
;############################################################################
UINT32_TO_STR: ; R16:R17:R18:R19 = UINT32, X = STR_POINTER
PUSH R10
PUSH R25
PUSH R24
PUSH R23
PUSH R22
PUSH R21
PUSH R20
PUSH R19
PUSH R18
PUSH R17
PUSH R16

	CLR R10
	LDI R24, 10
	LDI R25, 0
	
	CP R19, R24
	CPC R18, R25
	CPC R17, R25
	CPC R16, R25
	BRLO UINT32_TO_STR_LOOP1_END

	CLR R20
	CLR R21
	CLR R22
	MOV R23, R24
	UINT32_TO_STR_LOOP1:
		RCALL UDIV32 ; R16 H, R17, R18, R19 L = Dividend, R20 H, R21, R22, R23 L = Divisor
		INC R10
		MOV R16, R0
		MOV R17, R1
		MOV R18, R2
		MOV R19, R3

		CP R19, R24
		CPC R18, R25
		CPC R17, R25
		CPC R16, R25
		BRSH UINT32_TO_STR_LOOP1

		POP R16
		POP R17
		POP R18
		POP R19

		PUSH R19
		PUSH R18
		PUSH R17
		PUSH R16

	UINT32_TO_STR_LOOP1_END:
	INC R10
	INC R10
	ADD XL, R10
	ADC XH, R25

	ST -X, R25
	DEC R10
	UINT32_TO_STR_LOOP2:
		MOV R23, R24
		RCALL UDIV32 ; R16 H, R17, R18, R19 L = Dividend, R20 H, R21, R22, R23 L = Divisor
		LDI R23, '0'
		ADD R7, R23
		ST -X, R7
		MOV R16, R0
		MOV R17, R1
		MOV R18, R2
		MOV R19, R3
		DEC R10
		BRNE UINT32_TO_STR_LOOP2
	UINT32_TO_STR_LOOP2_END:
POP R16
POP R17
POP R18
POP R19
POP R20
POP R21
POP R22

POP R23
POP R24
POP R25
POP R10
RET

;############################################################################
;                                UINT64_TO_STR
;############################################################################
.DSEG

.CSEG
UINT64_TO_STR: ; R16:R17:R18:R19:R20:R21:R22:R23 = UINT64, X = STR_POINTER
PUSH R10

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

	CLR R10
	LDI R24, 10
	LDI R25, 0

	CP R23, R24
	CPC R22, R25
	CPC R21, R25
	CPC R20, R25
	CPC R19, R25
	CPC R18, R25
	CPC R17, R25
	CPC R16, R25
	BRLO UINT64_TO_STR_LOOP1_END

	UINT64_TO_STR_LOOP1:
		CLR R24
		CLR R25
		CLR R26
		CLR R27
		CLR R28
		CLR R29
		CLR R30
		LDI R31, 10

		PUSH R10
		RCALL UDIV64 ; H R16:R17:R18:R19:R20:R21:R22:R23 L = Dividend, H R24:R25:R26:R27:R28:R29:R30:R31 L = Divisor
		POP R10
		INC R10
		MOV R16, R0
		MOV R17, R1
		MOV R18, R2
		MOV R19, R3
		MOV R20, R4
		MOV R21, R5
		MOV R22, R6
		MOV R23, R7

		LDI R24, 10
		LDI R25, 0
		CP R23, R24
		CPC R22, R25
		CPC R21, R25
		CPC R20, R25
		CPC R19, R25
		CPC R18, R25
		CPC R17, R25
		CPC R16, R25
		BRSH UINT64_TO_STR_LOOP1
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
	UINT64_TO_STR_LOOP1_END:
	INC R10
	INC R10

	LDI R25, 0

	ADD XL, R10
	ADC XH, R25

	ST -X, R25

	UINT64_TO_STR_LOOP2:
		PUSH R10
		PUSH XL
		PUSH XH

		CLR R24
		CLR R25
		CLR R26
		CLR R27
		CLR R28
		CLR R29
		CLR R30
		LDI R31, 10		

		RCALL UDIV64 ; H R16:R17:R18:R19:R20:R21:R22:R23 L = Dividend, H R24:R25:R26:R27:R28:R29:R30:R31 L = Divisor
		
		POP XH
		POP XL
		POP R10
		
		LDI R23, '0'
		ADD R15, R23
		ST -X, R15
		MOV R16, R0
		MOV R17, R1
		MOV R18, R2
		MOV R19, R3
		MOV R20, R4
		MOV R21, R5
		MOV R22, R6
		MOV R23, R7
		DEC R10
		BRNE UINT64_TO_STR_LOOP2
	UINT64_TO_STR_LOOP2_END:
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
POP R10
RET
;############################################################################
;                                STR_TO_INT8
;############################################################################
STR_TO_INT8: ; XH:XL = STR_POINTER | Return = R0
PUSH R1
PUSH R16
PUSH R17
PUSH R18
PUSH XL
PUSH XH
	CLR R18

	CALL STR_LEN8 ; XH:XL = Stinrg Pointer | Return = R0
	TST R0
	BREQ STR_TO_INT8_END

	MOV R16, R0

	CPI R16, 3
	BREQ STR_TO_INT8_3

	CPI R16, 2
	BREQ STR_TO_INT8_2

	CPI R16, 1
	BREQ STR_TO_INT8_1

	RJMP STR_TO_INT8_END

	STR_TO_INT8_3:
	LD R16, X+
	LDI R17, '0'
	SUB R16, R17
	LDI R17, 100
	CALL UMUL8 ; R16 = Multiplikand, R17 = Multiplicator
	ADD R18, R1

	STR_TO_INT8_2:
	LD R16, X+
	LDI R17, '0'
	SUB R16, R17
	LDI R17, 10
	CALL UMUL8 ; R16 = Multiplikand, R17 = Multiplicator
	ADD R18, R1

	STR_TO_INT8_1:
	LD R16, X+
	LDI R17, '0'
	SUB R16, R17
	ADD R18, R16

	MOV R0, R18
STR_TO_INT8_END:
POP XH
POP XL
POP R16
POP R17
POP R18
POP R1
RET
;############################################################################
;                                 STR_LEN
;############################################################################
STR_LEN: ; XH:XL = Stinrg Pointer | Return = R1:R0 = Str_LEN
PUSH XH
PUSH XL
PUSH YH
PUSH YL
PUSH R16
	CLR YL
	CLR YH

	STR_LEN_LOOP:
		LD R16, X+
		TST R16
		BREQ STR_LEN_LOOP_END

		ADIW Y, 1
		TST R16
	BRNE STR_LEN_LOOP
	STR_LEN_LOOP_END:
	MOV R0, YL
	MOV R1, YH
STR_LEN_END:
POP R16
POP YL
POP YH
POP XL
POP XH
RET ; R1:R0 = Str_LEN
;############################################################################
;                                 STR_LEN8
;############################################################################
STR_LEN8: ; XH:XL = Stinrg Pointer | Return = R0
PUSH XH
PUSH XL
PUSH R16
	CLR R0
	STR_LEN8_LOOP:
		LD R16, X+
		TST R16
		BREQ STR_LEN8_LOOP_END
		; Len + 1
		INC R0
		BRVS STR_LEN8_LOOP_END
	RJMP STR_LEN8_LOOP
	STR_LEN8_LOOP_END:
POP R16
POP XL
POP XH
RET
;############################################################################
;                                 STR_LEN16
;############################################################################
STR_LEN16: ; XH:XL = Stinrg Pointer | Return = R1:R0
	PUSH XH
	PUSH XL
	PUSH R3
	PUSH R16
	PUSH R17
		CLR R0
		CLR R1
		LDI R16, 0x01
		CLR R17
		STR_LEN16_LOOP:
			LD R3, X+
			TST R3
			BREQ STR_LEN16_LOOP_END
			ADD R0, R16
			ADC R1, R17
			TST R3
		BRNE STR_LEN16_LOOP
		STR_LEN16_LOOP_END:
	POP R17
	POP R16
	POP R3
	POP XL
	POP XH
RET
;############################################################################
;                                 FIND_CHAR
;############################################################################
FIND_CHAR: ; XH:XL = Str Pointer, R16 = Char
	PUSH R16
	PUSH R17

	PUSH XH
	PUSH XL
		FIND_CHAR_LOOP:
			LD R17, X+
			TST R17
			BREQ FIND_CHAR_STR_END
			CP R17, R16
		BRNE FIND_CHAR_LOOP

		SBIW XH:XL, 1

		MOV R0, XH
		MOV R1, XL

		MOV R16, XH
		MOV R17, XL

		POP XL
		POP XH

		SUB R17, XL
		SBC R16, XH
		
		MOV R2, R16
		MOV R3, R17
		
		CLR R16
		LDI R17, 1

		ADD R3, R17
		ADC R2, R16

		RJMP FIND_CHAR_STR_END_END
		FIND_CHAR_STR_END:
			CLR R0
			CLR R1
			CLR R2
			CLR R3
		FIND_CHAR_STR_END_END:
	POP R17
	POP R16
RET ; R0:R1 = Char Pointer, R2:R3 = POS
;############################################################################
;                                 STR_COPY
;############################################################################
STR_COPY: ; XH:XL = SOURCE_POINTER, YH:YL = DEST_POINTER, R16:R17 = FROM_POS, R18:R19 = TO_POS
	PUSH XH
	PUSH XL
	PUSH YH
	PUSH YL
	PUSH R16
	PUSH R17
	PUSH R18
	PUSH R19
	PUSH R20
	PUSH R21
		ADD XL, R17
		ADC XH, R16

		SUB R19, R17
		SBC R18, R16

		MOV R16, R18
		MOV R17, R19

		CP R17, R21
		CPC R16, R21
		BREQ STR_COPY_LOOP_END

		STR_COPY_LOOP:
			LD R20, X+
			ST Y+, R20
			
			LDI R21, 1
			SUB R17, R21
			CLR R21
			SBC R16, R21
			
			CP R17, R21
			CPC R16, R21
		BRNE STR_COPY_LOOP

		CLR R21
		ST Y, R21
		STR_COPY_LOOP_END:
	POP R21 
	POP R20
	POP R19
	POP R18
	POP R17
	POP R16
	POP YL
	POP YH
	POP XL
	POP XH
RET
;############################################################################
;                                 STR_FIND
;############################################################################
STR_FIND: ; XH:XL = SOURCE, YH:YL = SRING
	PUSH XH
	PUSH XL
	PUSH YH
	PUSH YL
	PUSH R16
	PUSH R17
	PUSH R18
	PUSH R4
	PUSH R5
	PUSH R6
	PUSH R7

	MOV R6, XH 
	MOV R7, XL

	MOV R4, YH
	MOV R5, YL

		STR_FIND_START:
		MOV YH, R4
		MOV YL, R5
		LD R16, Y+
		STR_FIND_LOOP:
			LD R17, X+
			TST R17
			BREQ STR_FIND_END_NORES
			CP R17, R16
		BRNE STR_FIND_LOOP

		SBIW X, 1
		MOV R0, XH
		MOV R1, XL
		ADIW X, 1

		MOV R2, XH
		MOV R3, XL

		SUB R3, R7
		SBC R2, R6

		STR_FIND_CHECK:
			LD R16, Y+
			LD R17, X+

			TST R16
			BREQ STR_FIND_END
			TST R17
			BREQ STR_FIND_END_NORES

			CP R17, R16		
		BREQ STR_FIND_CHECK
		RJMP STR_FIND_START

		RJMP STR_FIND_END
		STR_FIND_END_NORES:
			CLR R0
			CLR R1
			CLR R2
			CLR R3
		STR_FIND_END:
	POP R7
	POP R6
	POP R5
	POP R4
	POP R18
	POP R17
	POP R16
	POP YL
	POP YH
	POP XL
	POP XH
RET ; R0:R1 = FINT_POINTER, R2:R3 = STR_POS
;############################################################################
;                                 STR_SEARCH
;############################################################################
STR_SEARCH: ; XH:XL = SOURCE, YH:YL = SRING
	PUSH XH
	PUSH XL
	PUSH YH
	PUSH YL
	PUSH R16
	PUSH R17
	PUSH R18
	PUSH R4
	PUSH R5
	PUSH R6
	PUSH R7

	MOV R6, XH 
	MOV R7, XL
	MOV R4, YH
	MOV R5, YL
		STR_SEARCH_START:
		MOV YH, R4
		MOV YL, R5
		LD R16, Y+
		RCALL CHAR_UPPER

		STR_SEARCH_LOOP:
			LD R17, X+
			TST R17
			BREQ STR_SEARCH_END_NORES

			PUSH R16
			MOV R16, R17
			RCALL CHAR_UPPER
			MOV R17, R16
			POP R16

			CP R17, R16
		BRNE STR_SEARCH_LOOP

		SBIW X, 1
		MOV R0, XH
		MOV R1, XL
		ADIW X, 1

		MOV R2, XH
		MOV R3, XL

		SUB R3, R7
		SBC R2, R6

		STR_SEARCH_CHECK:
			LD R16, Y+
			RCALL CHAR_UPPER

			LD R17, X+
			PUSH R16
			MOV R16, R17
			RCALL CHAR_UPPER
			MOV R17, R16
			POP R16

			TST R16
			BREQ STR_SEARCH_END
			TST R17
			BREQ STR_SEARCH_END_NORES

			CP R17, R16		
		BREQ STR_SEARCH_CHECK
		RJMP STR_SEARCH_START

		RJMP STR_SEARCH_END
		STR_SEARCH_END_NORES:
			CLR R0
			CLR R1
			CLR R2
			CLR R3
		STR_SEARCH_END:
	POP R7
	POP R6
	POP R5
	POP R4
	POP R18
	POP R17
	POP R16
	POP YL
	POP YH
	POP XL
	POP XH
RET ; R0:R1 = FINT_POINTER, R2:R3 = STR_POS
;############################################################################
;                                 STR_UPPER
;############################################################################
STR_UPPER: ; XH:XL = STR_POINTER
	PUSH XH
	PUSH XL
	PUSH R16
	
	STR_UPPER_LOOP:
		LD R16, X
		TST R16
		BREQ STR_UPPER_END
		RCALL CHAR_UPPER
		ST X+, R16
	RJMP STR_UPPER_LOOP
	STR_UPPER_END:
	POP R16
	POP XL
	POP XH
RET
;############################################################################
;                                 STR_LOWER
;############################################################################
STR_LOWER:
	PUSH XH
	PUSH XL
	PUSH R16
	
	STR_LOWER_LOOP:
		LD R16, X
		TST R16
		BREQ STR_LOWER_END
		RCALL CHAR_LOWER
		ST X+, R16
	RJMP STR_LOWER_LOOP
	STR_LOWER_END:
	POP R16
	POP XL
	POP XH
RET
;############################################################################
;                                 CHAR_UPPER
;############################################################################
CHAR_UPPER: ; R16 = CHAR
	PUSH R17
	PUSH R18

	LDI R18, 'z'
	INC R18
	LDI R17, 'a'

	CP R16, R17
	BRSH CHAR_UPPER_BIGER_A
	RJMP CHAR_UPPER_DONOT

	CHAR_UPPER_BIGER_A:
		CP R16, R18
		BRLO CHAR_UPPER_DO
		RJMP CHAR_UPPER_DONOT
	CHAR_UPPER_DO:
		CBR R16, 0b00100000
	CHAR_UPPER_DONOT:
	POP R18
	POP R17
RET
;############################################################################
;                                 CHAR_LOWER
;############################################################################
CHAR_LOWER:  ; R16 = CHAR
	PUSH R17
	PUSH R18

	LDI R18, 'Z'
	INC R18
	LDI R17, 'A'

	CP R16, R17
	BRSH CHAR_LOWER_BIGER_A
	RJMP CHAR_LOWER_DONOT

	CHAR_LOWER_BIGER_A:
		CP R16, R18
		BRLO CHAR_LOWER_DO
		RJMP CHAR_LOWER_DONOT
	CHAR_LOWER_DO:
		SBR R16, 0b00100000
	CHAR_LOWER_DONOT:
	POP R18
	POP R17
RET
;############################################################################
;                                 STR_REPLACE
;############################################################################
STR_REPLACE:

RET
;############################################################################
;                                 STR_CUT
;############################################################################
STR_CUT:

RET
;############################################################################
;                                 STR_RTRIM
;############################################################################
STR_RTRIM: ; XH:XL STING_POINTER, R16 = CHAR TO TRIM
PUSH XH
PUSH XL
PUSH R16
PUSH R17
PUSH R18

	RCALL STR_LEN
	ADD XL, R1
	ADC XH, R0

	LDI R18, 0x00
	STR_RTRIM_LOOP:
		LD R17, -X
		CP R16, R17
		BREQ STR_RTRIM_DEL
		BRNE STR_RTRIM_LOOP_END
		STR_RTRIM_DEL:
			ST X, R18
		RJMP STR_RTRIM_LOOP
	STR_RTRIM_LOOP_END:

POP R18
POP R17
POP R16
POP XL
POP XH
RET
;############################################################################
;									CPSTR
;############################################################################
CPSTR: ; X = STR1, Y = STR2 | R0 = TRUE(1)|FALSE(0)
PUSHX
PUSHY
;PUSH R0
PUSH R1
PUSH R2
PUSH R3
	RCALL STR_LEN
	MOV R2, R0
	MOV R3, R1
	
	PUSHX
		MOV XH, YH
		MOV XL, YL
		RCALL STR_LEN
	POPX

	CP R0, R2
	CPC R1, R3
	BRNE CPSTR_FALSE

	CPSTR_LOOP:
		LD R0, X+
		LD R1, Y+

		TST R0
		BREQ CPSTR_LOOP_END

		CP R0, R1
		BREQ CPSTR_LOOP
		BRNE CPSTR_FALSE
	CPSTR_LOOP_END:
	RJMP CPSTR_TRUE

	RJMP CPSTR_FALSE_END
	CPSTR_FALSE:
		CLR R0
	CPSTR_FALSE_END:

	RJMP CPSTR_TRUE_END
	CPSTR_TRUE:
		CLR R0
		INC R0
	CPSTR_TRUE_END:

	CPSTR_EXIT:
POP R3
POP R2
POP R1
;POP R0
POPY
POPX
RET



.ENDIF