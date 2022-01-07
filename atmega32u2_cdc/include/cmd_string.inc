;/*
; * cmd_string.inc
; *
; *  Created: 14.03.2016 17:49:04
; *   Author: Dominik Koch
; */ 

.ifndef cmd_string_include
.set cmd_string_include = 1

;##################################################
;			USB_CDC_STRING_TEST_CONTROL_CHAR
;##################################################
.set CKEY_NONE		=	 0x00
.set CKEY_UP		=	 0x01
.set CKEY_Down		=	 0x02
.set CKEY_Left		=	 0x03
.set CKEY_Right		=	 0x04
.set CKEY_Backspace =	 0x05
.set CKEY_ENTF		=	 0x06
.set CKEY_TAB		=	 0x07
;-----------------
.set CKEY_F1		=	 0x10
.set CKEY_F2		=	 0x11
.set CKEY_F3		=	 0x12
.set CKEY_F4		=	 0x13
.set CKEY_F5		=	 0x14
.set CKEY_F6		=	 0x15
.set CKEY_F7		=	 0x16
.set CKEY_F8		=	 0x17
.set CKEY_F9		=	 0x18
.set CKEY_F10		=	 0x19
.set CKEY_F11		=	 0x1A
.set CKEY_F12		=	 0x1B
;-----------------
.set CKEY_EINFG		=	 0x20
.set CKEY_POS1		=	 0x21
.set CKEY_END		=	 0x22
.set CKEY_Bild_UP	=	 0x23
.set CKEY_Bild_Down =	 0x24
;-------------------

.CSEG
;##################################################
;				COMMAND_CMP_PGM_RAM
;##################################################
COMMAND_CMP_PGM_RAM: ; Z=PGM_POINTER  Y=RAM_POINTER | Return R0 = [1=True | 0=False]
PUSH R18
PUSH R19
PUSH XL
PUSH XH
	PUSH ZL
	PUSH ZH
	PUSH YL
	PUSH YH
	PUSH XL
	PUSH XH

	CALL PGM_STRLEN ; (Z) Pointer PGM, (R0) (OUT) String Length
	MOV R18, R0

	MOVW X, Y
	CALL STR_LEN8 ; XH:XL = Stinrg Pointer | Return = R0
	MOV R19, R0

	POP XH
	POP XL
	POP YH
	POP YL
	POP ZH
	POP ZL

	CLR R0

	; Save Y
	MOVW X, Y

	CP R18, R0
	BREQ COMMAND_CMP_PGM_RAM_FALSE

	CP R19, R0
	BREQ COMMAND_CMP_PGM_RAM_FALSE

	CP R19, R18
	BRLO COMMAND_CMP_PGM_RAM_FALSE

	; Shift Left
	LSL ZL
	ROL ZH
	COMMAND_CMP_PGM_RAM_LOOP:
		LPM R18, Z+
		LD  R19, Y+

		CPI R18, 0x00
		BREQ COMMAND_CMP_PGM_RAM_TRUE

		CP R18, R19
		BRNE COMMAND_CMP_PGM_RAM_FALSE

	RJMP COMMAND_CMP_PGM_RAM_LOOP

	COMMAND_CMP_PGM_RAM_TRUE:
	SBIW Y, 1
	INC R0
	RJMP COMMAND_CMP_PGM_RAM_END

	COMMAND_CMP_PGM_RAM_FALSE:
		MOVW Y, X

COMMAND_CMP_PGM_RAM_END:
POP XH
POP XL
POP R19
POP R18
RET
;##################################################
;				COMMAND_PGM_EXEC
;##################################################
.MACRO COMMAND_PGM_EXEC ; (PGM_CMD_STRING_POINTER, FUNCTION_POINTER)
                        ; Y = String RAM Pointer
						; Return = R0 = [1=True | 0=False]
	LDI ZL,  LOW(@0)
	LDI ZH, HIGH(@0)
	LDI XL,  LOW(@1)
	LDI XH, HIGH(@1)
	CALL COMMAND_PGM_EXEC_ ;Z = PGM_CMP_STRING | Y = RAM_STRING | X = Functions_Pointer | RETURN R0 = [1=True | 0=False]
.ENDMACRO

COMMAND_PGM_EXEC_: ;Z = PGM_CMP_STRING | Y = RAM_STRING | X = Functions_Pointer | RETURN R0 = [1=True | 0=False]
PUSH ZL
PUSH ZH
	CALL COMMAND_CMP_PGM_RAM ; Z=PGM_POINTER  Y=RAM_POINTER | Return R0 = [1=True | 0=False]
	MOVW Z, X
	PUSH R0
	SBRC R0, 0
	ICALL
	POP R0
POP ZH
POP ZL
RET

.endif