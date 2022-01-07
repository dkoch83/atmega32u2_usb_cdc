;/*
; * eeprom.inc
; *
; *  Created: 31.05.2014 18:27:40
; *   Author: Dominik
; */ 

.ifndef eeprom_include
.set eeprom_include = 1

; Copy this Section to Main.asm and uncomment
; ------ Start Section -----

;Uncommnet to DISABLE Functions
;.set NO_EE_READ_BYTE		= 1
;.set NO_EE_READ_BLOCK		= 1
;.set NO_EE_WRITE_BYTE		= 1
;.set NO_EE_WRITE_BLOCK		= 1
;.include "./drv/eeprom.inc"

; ------ End Section ------

.ifndef NO_EE_READ_BYTE
.message "EE_READ_BYTE"
EE_READ_BYTE:	; Pointer X (EPROM), R0 (OUT) 
	READ_BYTE_LOOP:
	SBIC EECR, EEPE
	RJMP READ_BYTE_LOOP

	.if (SRAM_SIZE > 255)
	OUT EEARH, XH		; Set EEPROM Adress
	.endif
	OUT EEARL, XL

	SBI EECR, EERE		; Enable EEPROM Read

	IN R0, EEDR		; COPY EEDR to R17
RET
.else 
.message "EE_READ_BYTE		DISABLED"
.endif


.ifndef NO_EE_READ_BLOCK
.message "EE_READ_BLOCK"
EE_READ_BLOCK:	; Pointer X (EEPROM) to Pointer Y (SRAM), R16 count
	PUSH XL					; Save Register
	.if (SRAM_SIZE > 255)
	PUSH XH
	.endif
	PUSH YL
	PUSH YH

	MOV R10, R16			; 

	READ_LOOP:
		SBIC EECR, EEPE
		RJMP READ_LOOP

		.if (SRAM_SIZE > 255)
		OUT EEARH, XH		; Set EEPROM Adress
		.endif
		OUT EEARL, XL

		SBI EECR, EERE		; Enable EEPROM Read

		IN R25, EEDR		; COPY EEDR to R17

		ST Y+,R25			; MOVE to SRAM Pointer
		ADIW X, 1			; ADD EEPROM Pointer + 1 

		DEC R10				; Count - 1
	BRNE READ_LOOP			; Loop wile count > 0

	; Restore Register
	POP YH
	POP YL
	.if (SRAM_SIZE > 255)
	POP XH
	.endif
	POP XL
RET
.else 
.message "EE_READ_BLOCK		DISABLED"
.endif

.ifndef NO_EE_WRITE_BLOCK
.message "EE_WRITE_BLOCK"
EE_WRITE_BLOCK:	; Pointer X (EEPROM) to Pointer Y (SRAM), R16 count
	; Save Pointer
	PUSH XL
	.if (SRAM_SIZE > 255)
	PUSH XH
	.endif
	PUSH YL
	PUSH YH

	MOV R10, R16

	WRITE_LOOP:
		SBIC EECR, EEPE
		RJMP WRITE_LOOP

		.if (SRAM_SIZE > 255)
		OUT EEARH, XH		; Set EEPROM Adress from X Pointer
		.endif
		OUT EEARL, XL

		LD R25, Y+			; Load Byte from Pointer (SRAM)
		OUT EEDR, R25		; Move R25 to EEDR

		;Set Progmode and Enable Master Program Enable
		LDI R25, (0<<EEPM1) | (0<<EEPM0) | (1<<EEMPE)
		OUT EECR, R25
		SBI EECR, EERE		; Program Enable

		ADIW X, 1			; ADD EEPROM Pointer + 1 

		DEC R10				; Count - 1
	BRNE WRITE_LOOP			; Loop wile count > 0

	; Restore Pointer
	POP YH
	POP YL
	.if (SRAM_SIZE > 255)
	POP XH
	.endif
	POP XL
RET
.else
.message "EE_WRITE_BLOCK		DISABLED"
.endif

.ifndef NO_EE_WRITE_BYTE
.message "EE_WRITE_BYTE"
EE_WRITE_BYTE: ; Pointer X (EEPROM), R16 (WRITE)
	; Wait for Programming
	SBIC EECR, EEPE
	RJMP EE_WRITE_BYTE
	
	;Set Programming mode
	LDI R25, (0<<EEPM1) | (0<<EEPM0)
	OUT EECR, R25

	; Set EEPROM Adress
	OUT EEARL, XL
	.if (SRAM_SIZE > 255)
	OUT EEARH, XH
	.endif

	; Set Data to Write
	OUT EEDR, R16

	;Set Progmode and Enable Master Program Enable
	LDI R25, (0<<EEPM1) | (0<<EEPM0) | (1<<EEMPE)
	OUT EECR, R25
	SBI EECR, EERE		; Program Enable
RET
.else 
.message "EE_WRITE_BYTE		DISABLED"
.endif


.endif ; eeprom_include