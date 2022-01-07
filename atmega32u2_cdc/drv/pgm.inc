;/*
; * pgm.inc
; *
; *  Created: 01.06.2014 18:34:39
; *   Author: Dominik
; */ 

; Copy this Section to Main.asm and uncomment
; ------ Start Section -----

;Uncommnet to DISABLE Functions
;.set NO_PGM_READ_BYTE		= 1
;.set NO_PGM_READ_WORD		= 1
;.set NO_PGM_READ_BUFFER	= 1
;.set NO_PGM_READ_STR		= 1
;.set NO_PGM_STRLEN			= 1
;.set NO_PGM_FIND_CHAR		= 1
;.include "./drv/pgm.inc"

; ------ End Section ------

.ifndef pgm_include
.set pgm_include = 1

.ifndef NO_PGM_READ_WORD
.message "PGM_READ_WORD"
PGM_READ_WORD: ;(Z) Pointer Address, OUT (R1) HIGH Byte, (R0) LOW Byte
	PUSH ZL
	PUSH ZH

	; Shift Left
	LSL ZL
	ROL ZH

	; Read LOW Byte
	LPM R0, Z+
	; Read HIGH Byte
	LPM R1, Z

	POP ZH
	POP ZL
RET
.else 
.message "PGM_READ_WORD			DISABLED"
.endif


.ifndef NO_PGM_READ_BYTE
.message "PGM_READ_BYTE"
PGM_READ_BYTE: ;ZH:ZL = Pointer Address, R16:R17 = Byte Adress
	PUSH ZL
	PUSH ZH

	; Shift Left
	LSL ZL
	ROL ZH

	ADD ZL, R17
	ADC ZH, R16

	LPM R0, Z

	POP ZH
	POP ZL
RET ;R0 = OUT
;###############################################################################
;								PGM_READ_BYTE8
;###############################################################################
PGM_READ_BYTE8: ;ZH:ZL = Pointer Address, R16 = Byte Adress
PUSH ZL
PUSH ZH
PUSH R16
	; Shift Left
	LSL ZL
	ROL ZH

	ADD ZL, R16
	LDI R16, 0x00
	ADC ZH, R16

	LPM R0, Z
PGM_READ_BYTE8_END:
POP R16
POP ZH
POP ZL
RET ;R0 = OUT
.else 
.message "PGM_READ_BYTE			DISABLED"
.endif


.ifndef NO_PGM_READ_BUFFER
.message "PGM_READ_BUFFER"
PGM_READ_BUFFER: ; (Z) Pointer PGM, (Y) Pointer SRAM, (X) Read Bytes
	PUSH ZL
	PUSH ZH
	PUSH YL
	PUSH YH
	PUSH XL
	PUSH XH
	
	; Shift Left
	LSL ZL
	ROL ZH
	LOOP_PGM_READ_BUFFER:
		; Read HIGH Byte
		LPM R0, Z+
		ST Y+, R0
		SBIW X, 1
		BREQ FINISH_PGM_READ_BUFFER

		; Read LOW Byte
		LPM R0, Z+
		ST Y+, R0
		SBIW X, 1
		BRNE LOOP_PGM_READ_BUFFER
	FINISH_PGM_READ_BUFFER:
	POP XH
	POP XL
	POP YH
	POP YL
	POP ZH
	POP ZL
RET
.else 
.message "PGM_READ_BUFFER		DISABLED"
.endif

.ifndef NO_PGM_READ_STR
.message "PGM_READ_STR"
PGM_READ_STR: ; (Z) Pointer PGM, (Y) Pointer SRAM, (R0) (OUT) String Length
	PUSH ZL
	PUSH ZH
	PUSH YL
	PUSH YH
	PUSH R23
	CLR R0	; Reset R0 Counter to Zero
	
	; Shift Left
	LSL ZL
	ROL ZH
	LOOP_PGM_READ_STR:
		; Read HIGH Byte
		LPM R23, Z+
		ST Y+, R23
		TST R23
		BREQ FINISH_PGM_READ_STR
		INC R0
		BRVS FINISH_PGM_READ_STR
	RJMP LOOP_PGM_READ_STR

	FINISH_PGM_READ_STR:
	POP R23
	POP YH
	POP YL
	POP ZH
	POP ZL
RET
.else 
.message "PGM_READ_STR		DISABLED"
.endif

.ifndef NO_PGM_STRLEN
.message "PGM_STRLEN"
PGM_STRLEN: ; (Z) Pointer PGM, (R1:R0) (OUT) String Length
	PUSH ZL
	PUSH ZH
	PUSH R3
	PUSH R4

	CLR R3  ; Set R3 to 0 for ADD
	CLR R4
	INC R4	; Set R4 to 1 for ADD
	 
	CLR R0	; Clear for Strlen LOW Byte
	CLR R1  ; Clear for Strlen HIGH Byte

	; Shift Left
	LSL ZL
	ROL ZH
	LOOP_PGM_STRLEN:
		; Read Byte
		LPM R23, Z+
		TST R23
		BREQ FINISH_PGM_STRLEN
		ADD R0, R4
		ADC R1, R3
	RJMP LOOP_PGM_STRLEN
	FINISH_PGM_STRLEN:
	POP R4
	POP R3
	POP ZH
	POP ZL
RET
.else 
.message "PGM_STRLEN		DISABLED"
.endif

.ifndef NO_PGM_FIND_CHAR
.message "PGM_FIND_CHAR"
;###############################################################################
;								PGM_FIND_CHAR
;###############################################################################
PGM_FIND_CHAR: ; (Z) Pointer PGM, R16 = CHAR to FIND, R0:R1 = CHAR POS
PUSH ZL
PUSH ZH
PUSH R20
PUSH R21
PUSH R22
PUSH R23

	CLR R0	; Reset R0 Counter to Zero
	CLR R1	; Reset R1 Counter to Zero

	LDI R20, 0xFF
	MOV R0, R20
	MOV R1, R20

	LDI R20, 0x00
	LDI R21, 0x01

	; Shift Left
	LSL ZL
	ROL ZH
	LOOP_PGM_FIND_CHAR:
		; Read Byte
		LPM R23, Z+
		ADD R1, R21
		ADC R0, R20
		CPI R23, 0x00
		BREQ FINISH_PGM_FIND_CHAR
		CP R23, R16
		BREQ END_PGM_FIND_CHAR
	RJMP LOOP_PGM_FIND_CHAR

	FINISH_PGM_FIND_CHAR:
		LDI R20, 0xFF
		MOV R0, R20
		MOV R1, R20
	END_PGM_FIND_CHAR:
POP R23
POP R22
POP R21
POP R20
POP ZH
POP ZL
RET
;###############################################################################
;								PGM_FIND_CHAR8
;###############################################################################
PGM_FIND_CHAR8: ; (Z) Pointer PGM, R16 = CHAR to FIND, R0 = CHAR POS
PUSH ZL
PUSH ZH
PUSH R16
PUSH R17
	LDI R17, 0xFF
	MOV R0, R17

	; Shift Left
	LSL ZL
	ROL ZH
	LOOP_PGM_FIND_CHAR8:
		; Read Byte
		LPM R17, Z+
		INC R0
		CPI R17, 0x00
		BREQ FINISH_PGM_FIND_CHAR8
		CP R17, R16
		BREQ END_PGM_FIND_CHAR8
	RJMP LOOP_PGM_FIND_CHAR8

	FINISH_PGM_FIND_CHAR8:
		LDI R17, 0xFF
		MOV R0, R17
	END_PGM_FIND_CHAR8:
POP R17
POP R16
POP ZH
POP ZL
RET
.else 
.message "PGM_FIND_CHAR		DISABLED"
.endif


.endif ; include pgm_include