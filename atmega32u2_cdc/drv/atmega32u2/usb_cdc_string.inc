;/*
; * usb_cdc_string.inc
; *
; *  Created: 27.04.2016 21:01:04
; *   Author: Dominik Koch
; */ 


.ifndef USB_CDC_STRING_INCLUDE
.set USB_CDC_STRING_INCLUDE = 1
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
;                                 STR_TO_INT8
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
;                                INT8_TO_STR
;############################################################################
INT8_TO_STR: ; R16 = INT8, X = STR_POINTER
	PUSH R0
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
	POP R0
RET
;#################################################################################
;                                      BIN_STR
;#################################################################################
; STR Buffer = In Buffer * 9 + 1 
BIN_STR: ; X = Buffer_Pointer, Y = STR_POINTER, R16 = Buffer_Size
PUSH R16
PUSH R17
PUSH R18
PUSH R19
PUSH XL
PUSH XH
PUSH YL
PUSH YH
	CPI R16, 0x00
	BREQ BIN_STR_END

	BIN_STR_BYTE_LOOP:
		LD R19, X+
		LDI R17, 8
		BIN_STR_BIN_LOOP:
			LSL R19
			BRCS BIN_STR_BIN_LOOP_1
			BRCC BIN_STR_BIN_LOOP_0

			RJMP BIN_STR_BIN_LOOP_0_END
			BIN_STR_BIN_LOOP_0:
				LDI R18, '0'
				ST Y+, R18
			BIN_STR_BIN_LOOP_0_END:

			RJMP BIN_STR_BIN_LOOP_1_END
			BIN_STR_BIN_LOOP_1:
				LDI R18, '1'
				ST Y+, R18
			BIN_STR_BIN_LOOP_1_END:
		DEC R17
		BRNE BIN_STR_BIN_LOOP
		BIN_STR_BIN_LOOP_END:

		; Space all 8 BIT
		LDI R18, ' '
		ST Y+, R18
	DEC R16
	BRNE BIN_STR_BYTE_LOOP
	BIN_STR_BYTE_LOOP_END:

	; NULL Terminate String
	LDI R18, 0x00
	ST Y+, R18
BIN_STR_END:
POP YH
POP YL
POP XH
POP XL
POP R19
POP R18
POP R17
POP R16
RET
;#################################################################################
;                                      UMUL8
;#################################################################################
UMUL8: ; R16 = Multiplikand, R17 = Multiplicator | Return = R0:R1 
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
;###############################################################################
;								HEX_STR
;###############################################################################
; From Buffer block size must be divisible by 1
;
; OUT BUFFER SIZE must ((BUFFER FROM SIZE * 3) + 1
;
HEX_STR_CODE: .DB "0123456789ABCDEF"

HEX_STR: ; XH:XL = Pointer FROM BUFFER, YH:YL = Pointer OUT BUFFER, R16 = BUFFER FROM SIZE, R17 = Block
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
PUSH R20
PUSH R0
	MOV R18, R16
	MOV R19, R17
	CLR R20

	LDI ZH, HIGH(HEX_STR_CODE)
	LDI ZL, LOW(HEX_STR_CODE)

	TST R16
	BREQ HEX_STR_END

	CLR R16
	HEX_STR_LOOP1:
		LD R17, X+
		
		MOV R16, R17
		SWAP R16
		ANDI R16, 0x0F
		CALL PGM_READ_BYTE8 ;ZH:ZL = Pointer Address, R16 = Byte Adress, Return = R0
		ST Y+, R0

		MOV R16, R17
		ANDI R16, 0x0F
		CALL PGM_READ_BYTE8 ;ZH:ZL = Pointer Address, R16 = Byte Adress, Return = R0
		ST Y+, R0
		
		INC R20
		CP R20, R19
		BRSH HEX_STR_SPACHE
		RJMP HEX_STR_SPACHE_END
		HEX_STR_SPACHE:
		CLR R20
		;Concat " "
		LDI R17, 0x20
		ST Y+, R17
		HEX_STR_SPACHE_END:

		DEC R18
	BREQ HEX_STR_END 
	RJMP HEX_STR_LOOP1
	HEX_STR_END:
	LDI R16, 0
	ST -Y, R16
POP R0
POP R20
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
;###############################################################################
;								STR_HEX
;###############################################################################
;HEX_STR_CODE: .DB "0123456789ABCDEF"
STR_HEX: ; XH:XL = STR_Pointer, YH:YL = Out_Buffer_Pointer, R16 = Max_Bytes, RETURN = R0 = Out_Buffer_Bytes
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
	CLR R18
	MOV R19, R16

	TST R19
	BRNE STR_HEX_MAX_BYTES_END
	STR_HEX_MAX_BYTES:
		LDI R19, 0xFF
	STR_HEX_MAX_BYTES_END:

	LDI ZH, HIGH(HEX_STR_CODE)
	LDI ZL, LOW(HEX_STR_CODE)

	STR_HEX_LOOP1:
		LD R16, X+
		CPI R16, 0x00
		BREQ STR_HEX_LOOP1_END
		
		RCALL CHAR_UPPER ; R16 = CHAR

		CPI R16, ' '
		BREQ STR_HEX_LOOP1

		CLR R17

		RCALL PGM_FIND_CHAR8 ; (Z) Pointer PGM, R16 = CHAR to FIND, R0 = CHAR POS
		OR R17, R0

		LD R16, X+
		CPI R16, 0x00
		BREQ STR_HEX_STORE_FINISH
		
		CPI R16, ' '
		BREQ STR_HEX_STORE

		SWAP R17

		RCALL PGM_FIND_CHAR8 ; (Z) Pointer PGM, R16 = CHAR to FIND, R0 = CHAR POS
		OR R17, R0

		STR_HEX_STORE:
			ST Y+, R17
			INC R18
			DEC R19
			BREQ STR_HEX_LOOP1_END
	RJMP STR_HEX_LOOP1
	STR_HEX_STORE_FINISH:
		ST Y+, R17
		INC R18
	STR_HEX_LOOP1_END:
	MOV R0, R18
STR_HEX_END:
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
;###############################################################################
;								PRINT_PGM_STR
;###############################################################################
PRINT_PGM_STR: ; Z = STR_POINTER
PUSH XH
PUSH XL
PUSH YH
PUSH YL
PUSH ZH
PUSH ZL
PUSH R0
	LDI YL, LOW(USB_TX_BUFFER_P)
	LDI YH, HIGH(USB_TX_BUFFER_P)
	LDD XL, Y+0
	LDD XH, Y+1
	MOVW Y, X

	; Save Y
	PUSH YL
	PUSH YH

	;Set Pointer to Buffer Begin
	ADIW Y, 6

	CALL PGM_READ_STR ; (Z) Pointer PGM, (Y) Pointer SRAM, (R0) (OUT) String Length

	; Resore Y
	POP YH
	POP YL

	; Save Str Length
	STD Y+2, R0
	CLR R0
	STD Y+3, R0

	; Set Str Pos to Zero	
	STD Y+4, R0
	STD Y+5, R0	 

	CALL USB_CDC_TX_WAIT
PRINT_PGM_STR_END:
POP R0
POP ZL
POP ZH
POP YL
POP YH
POP XL
POP XH
RET
;###############################################################################
;								PRINT_RAM_STR
;###############################################################################
PRINT_RAM_STR: ; X = STR_POINTER
PUSH XL
PUSH XH
PUSH YL
PUSH YH
PUSH ZL
PUSH ZH
PUSH R16
PUSH R17
	; Save X
	MOVW R17:R16, X

	LDI YL, LOW(USB_TX_BUFFER_P)
	LDI YH, HIGH(USB_TX_BUFFER_P)
	LDD XL, Y+0
	LDD XH, Y+1
	MOVW Y, X

	; Save Y
	PUSH YL
	PUSH YH

	;Set Pointer to Buffer Begin
	ADIW Y, 6

	; Restore X
	MOVW X, R17:R16

	; Clear Z
	CLR ZL
	CLR ZH

	PRINT_RAM_STR_COPY:
		LD R16, X+
		ST Y+, R16
		ADIW Z, 1 
		TST R16
	BRNE PRINT_RAM_STR_COPY
	PRINT_RAM_STR_COPY_END:
	
	; Resore Y
	POP YH
	POP YL

	SBIW Z, 1
	; Save Str Length
	STD Y+2, ZL
	STD Y+3, ZH

	; Set Str Pos to Zero
	CLR R16
	STD Y+4, R16
	STD Y+5, R16	
	CALL USB_CDC_TX_WAIT
PRINT_RAM_STR_END:
POP R17
POP R16
POP ZH
POP ZL
POP YH
POP YL
POP XH
POP XL
RET
;###############################################################################
;								PGM_READ_STR
;###############################################################################
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
		; Read Byte
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
;###############################################################################
;								PRINT_HEX
;###############################################################################
PRINT_HEX: ; X = Buffer_Pointer , R16 = Buffer_Size, R17 = Block_Size ( FF FF or FFFF FFFF )
/*
PUSH XH
PUSH XL
PUSH YH
PUSH YL
PUSH R16
PUSH R17
PUSH R0
PUSH R1
	LDI YL, LOW(CDC_TX_BUFFER)
	LDI YH, HIGH(CDC_TX_BUFFER)
	CALL HEX_STR ; XH:XL = Pointer FROM BUFFER, YH:YL = Pointer OUT BUFFER, R16 = BUFFER FROM SIZE
	MOVW X, Y
	CALL STR_LEN16
	ADD XL, R1
	ADC XH, R0
	LDI R16, '\r'
	ST X+, R16
	LDI R16, 0x00
	ST X+, R16
	inc R1
	STS CDC_TX_SIZEL, R1
	STS CDC_TX_SIZEH, R0
	CALL USB_CDC_TX_WAIT
PRINT_HEX_END:
POP R1
POP R0
POP R17
POP R16
POP YL
POP YH
POP XL
POP XH
*/
RET
;###############################################################################
;								PRINT_NL
;###############################################################################
PRINT_NL:
PUSH XH
PUSH XL
PUSH YH
PUSH YL
PUSH R16


	LDI YL, LOW(USB_TX_BUFFER_P)
	LDI YH, HIGH(USB_TX_BUFFER_P)
	LDD XL, Y+0
	LDD XH, Y+1
	MOVW Y, X

	; Save Y
	PUSH YL
	PUSH YH

	ADIW Y, 6
	
	LDI R16, '\r'
	ST Y+, R16
	LDI R16, '\n'
	ST Y+, R16
	LDI R16, 0x00
	ST Y+, R16
	
	; Restore Y
	POP YH
	POP YL

	;Save Str Length
	LDI R16, 2
	STD Y+2, R16
	LDI R16, 0x00
	STD Y+3, R16
	
	; Zero Buffer Pos
	STD Y+4, R16
	STD Y+5, R16
	
	CALL USB_CDC_TX_WAIT
PRINT_NL_END:
POP R16
POP YL
POP YH
POP XL
POP XH
RET
;###############################################################################
;								PRINT_HEX_STR
;###############################################################################
;	OUTPUT		FF FF FF FF ....
;
PRINT_HEX_STR: ; X = Buffer_Pointer , R16 = Buffer_Size, R17 = Block_Size ( FF FF or FFFF FFFF )
/*
PUSH XH
PUSH XL
PUSH YH
PUSH YL
PUSH R16
PUSH R17
PUSH R0
PUSH R1
	LDI YL, LOW(CDC_TX_BUFFER)
	LDI YH, HIGH(CDC_TX_BUFFER)
PUSH YH
PUSH YL
PUSH XH
PUSH XL
	CALL HEX_STR ; XH:XL = Pointer FROM BUFFER, YH:YL = Pointer OUT BUFFER, R16 = BUFFER FROM SIZE
	MOVW X, Y
	CALL STR_LEN16
	ADD XL, R1
	ADC XH, R0

	LDI R17, ' '
	ST X+, R17

	MOVW Y, X
POP XL
POP XH
	PRINT_HEX_STR_STR_LOOP:
		LD R17, X+

		CPI R17, 0x7F
		BRSH PRINT_HEX_STR_STR_LOOP_DOT

		CPI R17, 0x20
		BRLO PRINT_HEX_STR_STR_LOOP_DOT

		RJMP PRINT_HEX_STR_STR_LOOP_DOT_END
		PRINT_HEX_STR_STR_LOOP_DOT:
			LDI R17, '.'
		PRINT_HEX_STR_STR_LOOP_DOT_END:

		ST Y+, R17
		DEC R16
	BRNE PRINT_HEX_STR_STR_LOOP
	PRINT_HEX_STR_STR_LOOP_END:
	LDI R17, 0x00
	ST Y+, R17
; POP Y to X
POP XL
POP XH
	
	CALL STR_LEN16
	ADD XL, R1
	ADC XH, R0

	LDI R16, '\r'
	ST X+, R16
	LDI R16, 0x00
	ST X+, R16
	INC R1
	STS CDC_TX_SIZEL, R1
	STS CDC_TX_SIZEH, R0
	
	CALL USB_CDC_TX_WAIT
PRINT_HEX_STR_END:
POP R1
POP R0
POP R17
POP R16
POP YL
POP YH
POP XL
POP XH
*/
RET
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
;###############################################################################
;								COPY_RAM_TO_RAM
;###############################################################################
COPY_RAM_TO_RAM: ; X = From_Pointer, Y = To_Pointer, Z = Copy_Size
PUSH R16
PUSH XL
PUSH XH
PUSH YL
PUSH YH
PUSH ZL
PUSH ZH
	CLR R16

	; Test for Zero
	CP  ZL, R16
	CPC ZH, R16
	BREQ COPY_RAM_TO_RAM_END

	COPY_RAM_TO_RAM_LOOP:
		LD R16, X+
		ST Y+, R16
		SBIW Z, 1
	BRNE COPY_RAM_TO_RAM_LOOP
COPY_RAM_TO_RAM_END:
POP ZH
POP ZL
POP YH
POP YL
POP XH
POP XL
POP R16
RET
;###############################################################################
;								IS_STR_HEX
;###############################################################################
STR_IS_STR_HEX:	.db " 0123456789ABCDEF",0x00
IS_STR_HEX: ; X = STR_Pointer, Retrun = R0 = [1=True | 0=False]
PUSH R16
PUSH XL
PUSH XH
PUSH ZL
PUSH ZH
	LDI ZL, LOW(STR_IS_STR_HEX)
	LDI ZH, HIGH(STR_IS_STR_HEX)

	IS_STR_HEX_LOOP:
		LD R16, X+
		CPI R16, 0x00
		BREQ IS_STR_HEX_LOOP_END

		CALL CHAR_UPPER ; R16 = CHAR
		CALL PGM_FIND_CHAR8 ; (Z) Pointer PGM, R16 = CHAR to FIND, R0 = CHAR POS
		MOV R16, R0
		CPI R16, 0xFF
		BREQ IS_STR_HEX_FALSE
	RJMP IS_STR_HEX_LOOP
	IS_STR_HEX_LOOP_END:

	CLR R0
	INC R0
	RJMP IS_STR_HEX_FALSE_END
	IS_STR_HEX_FALSE:
		CLR R0
	IS_STR_HEX_FALSE_END:
IS_STR_HEX_END:
POP ZH
POP ZL
POP XH
POP XL
POP R16
RET
;###############################################################################
;								IS_INT_STR
;###############################################################################
STR_IS_STR_INT:	.db "0123456789",0x00,0x00
IS_STR_INT: ; X = STR_Pointer, Retrun = R0 = [1=True | 0=False]
PUSH R16
PUSH XL
PUSH XH
PUSH ZL
PUSH ZH
	LDI ZL, LOW(STR_IS_STR_INT)
	LDI ZH, HIGH(STR_IS_STR_INT)

	IS_STR_INT_LOOP:
		LD R16, X+
		CPI R16, 0x00
		BREQ IS_STR_INT_LOOP_END

		CALL CHAR_UPPER ; R16 = CHAR
		CALL PGM_FIND_CHAR8 ; (Z) Pointer PGM, R16 = CHAR to FIND, R0 = CHAR POS
		MOV R16, R0
		CPI R16, 0xFF
		BREQ IS_STR_INT_FALSE
	RJMP IS_STR_INT_LOOP
	IS_STR_INT_LOOP_END:

	CLR R0
	INC R0
	RJMP IS_STR_INT_FALSE_END
	IS_STR_INT_FALSE:
		CLR R0
	IS_STR_INT_FALSE_END:
IS_STR_INT_END:
POP ZH
POP ZL
POP XH
POP XL
POP R16
RET

.endif 