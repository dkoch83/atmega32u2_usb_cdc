/*
 * usb_buffer.inc
 *
 *  Created: 31.12.2021 11:22:57
 *   Author: Dkoch83
 */ 

.ifndef USB_BUFFER_INCLUDE
.set USB_BUFFER_INCLUDE = 1

 .DSEG
 	; RX Buffer
	USB_RX_BUFFER_P:
		USB_RX_BUFFER_P_L: .BYTE 1
		USB_RX_BUFFER_P_H: .BYTE 1

	; RX Buffer
	USB_TX_BUFFER_P:
		USB_TX_BUFFER_P_L: .BYTE 1
		USB_TX_BUFFER_P_H: .BYTE 1


/*
;################################################
;#			     BUFFER EXAMPLE                 #
;################################################
.DSEG
	.SET USB_RX_SIZE = 255
	USB_RX_BUFFER:
		USB_RX_BUFFER_MAX_SIZE:
			NEW_RX_BUFFER_MAX_SIZE_L: .BYTE 1
			NEW_RX_BUFFER_MAX_SIZE_H: .BYTE 1
		USB_RX_BUFFER_SIZE:
			NEW_RX_BUFFER_SIZE_L: .BYTE 1
			NEW_RX_BUFFER_SIZE_H: .BYTE 1
		USB_RX_BUFFER_POS:
			NEW_RX_BUFFER_POS_L: .BYTE 1
			NEW_RX_BUFFER_POS_H: .BYTE 1
		USB_RX_BUFFER_BUFFER: 
			.BYTE USB_RX_SIZE
			.BYTE 1 ; NULL TERMINATE
;#############################################
	.SET USB_TX_SIZE = 255
	USB_TX_BUFFER:
		USB_TX_BUFFER_MAX_SIZE:
			USB_TX_BUFFER_MAX_SIZE_L: .BYTE 1
			USB_TX_BUFFER_MAX_SIZE_H: .BYTE 1
		USB_TX_BUFFER_SIZE:
			USB_TX_BUFFER_SIZE_L: .BYTE 1
			USB_TX_BUFFER_SIZE_H: .BYTE 1
		USB_TX_BUFFER_POS:
			USB_TX_BUFFER_POS_L: .BYTE 1
			USB_TX_BUFFER_POS_H: .BYTE 1
		USB_TX_BUFFER_BUFFER: 
			.BYTE USB_TX_SIZE
			.BYTE 1 ; NULL TERMINATE	
;#############################################
;#                 OR SHORT                  #
;#############################################
	.SET USB_TX_SIZE = 255
	USB_TX_BUFFER: .BYTE (6 + USB_TX_SIZE + 1)

	.SET USB_RX_SIZE = 255
USB_RX_BUFFER: .BYTE (6 + USB_RX_SIZE + 1)
*/



.CSEG
	;##################################################
	;				  USB_INIT_BUFFER
	;##################################################
	.MACRO USB_INIT_BUFFER ;(Buffer_Struct, Buffer_Size)
		PUSH YL
		PUSH YH
		PUSH R16
		PUSH R17
		PUSH R18
			LDI YL, LOW(@0)
			LDI YH, HIGH(@0)

			LDI R16, LOW(@1)
			LDI R17, HIGH(@1)

			; Set BUFFER_MAX_SIZE
			STD Y+0, R16
			STD Y+1, R17

			; Clear BUFFER_SIZE
			CLR R18
			STD Y+2, R18
			STD Y+3, R18

			; Clear BUFFER_POS
			STD Y+4, R18
			STD Y+5, R18			

			;SET NULL Termination
			ADIW Y,(6+1)
			ADD YL, R16
			ADC YH, R17
			ST Y, R16
		POP R18
		POP R17
		POP R16
		POP YH
		POP YL
	.ENDMACRO
	;##################################################
	;				  USB_SET_RX_BUFFER
	;##################################################
	.MACRO USB_SET_RX_BUFFER ;(Buffer_Struct)
		PUSH XL
		PUSH XH
		PUSH YL
		PUSH YH
			LDI YL, LOW(USB_RX_BUFFER_P)
			LDI YH, HIGH(USB_RX_BUFFER_P)
			
			LDI XL, LOW(@0)
			LDI XH, HIGH(@0)

			STD Y+0, XL
			STD Y+1, XH
		POP YH
		POP YL
		POP XH
		POP XL
	.ENDMACRO
	;##################################################
	;				  USB_SET_TX_BUFFER
	;##################################################
	.MACRO USB_SET_TX_BUFFER ;(Buffer_Struct)
		PUSH XL
		PUSH XH
		PUSH YL
		PUSH YH
			LDI YL, LOW(USB_TX_BUFFER_P)
			LDI YH, HIGH(USB_TX_BUFFER_P)
			
			LDI XL, LOW(@0)
			LDI XH, HIGH(@0)

			STD Y+0, XL
			STD Y+1, XH
		POP YH
		POP YL
		POP XH
		POP XL
	.ENDMACRO
	;##################################################
	;				USB_FLUSH_TX_BUFFER
	;##################################################
	USB_FLUSH_TX_BUFFER:
	PUSH R16
	PUSH YL
	PUSH YH
	PUSH ZL
	PUSH ZH
		CLR R16

		;LOAD BUFFER POINTER
		LDI ZL, LOW(USB_TX_BUFFER_P)
		LDI ZH, HIGH(USB_TX_BUFFER_P)

		; Test for Point to Zero
		CP  ZL, R16
		CPC ZH, R16
		BREQ USB_FLUSH_TX_BUFFER_END

		;LOAD BUFFER Adress
		LDD YL, Z+0
		LDD YH, Z+1

		;CLEAR BUFFER SIZE
		STD Y+2, R16
		STD Y+3, R16

		;CLEAR BUFFER POS
		STD Y+4, R16
		STD Y+5, R16

		;FIRST BUFFER BYTE 0x00
		STD Y+6, R16

	USB_FLUSH_TX_BUFFER_END:
	POP ZH
	POP ZL
	POP YH
	POP YL
	POP R16
	RET
	;##################################################
	;				USB_FLUSH_RX_BUFFER
	;##################################################
	USB_FLUSH_RX_BUFFER:
	PUSH R16
	PUSH YL
	PUSH YH
	PUSH ZL
	PUSH ZH
		CLR R16

		;LOAD BUFFER POINTER
		LDI ZL, LOW(USB_RX_BUFFER_P)
		LDI ZH, HIGH(USB_RX_BUFFER_P)

		; Test for Point to Zero
		CP  ZL, R16
		CPC ZH, R16
		BREQ USB_FLUSH_RX_BUFFER_END

		;LOAD BUFFER Adress
		LDD YL, Z+0
		LDD YH, Z+1

		;CLEAR BUFFER SIZE
		STD Y+2, R16
		STD Y+3, R16

		;CLEAR BUFFER POS
		STD Y+4, R16
		STD Y+5, R16

		;FIRST BUFFER BYTE 0x00
		STD Y+6, R16

		/*
		CALL USB_CDC_CLR_RX_BUFFER_FULL
		CALL USB_CDC_CLR_NEW_LN_IN
		CALL USB_CDC_STATUS_SET_RX
		*/
	USB_FLUSH_RX_BUFFER_END:
	POP ZH
	POP ZL
	POP YH
	POP YL
	POP R16
	RET


	USB_RAM_STR_TO_BUFFER: ; Y = PUFFER_POINTER, Z = RAM_STR_POINTER
	PUSH XL
	PUSH XH
	PUSH YL
	PUSH YH
	PUSH ZL
	PUSH ZH
	PUSH R16
	PUSH R17
	PUSH R0

		LDD XL, Y+0
		LDD XH, Y+1
		MOVW Y, X

		; Load Max Size
		LDD R16, Y+0
		LDD R17, Y+1

		CLR R0
		; Clear Size
		STD Y+2, R0 
		STD Y+3, R1

		; Clear Pos
		STD Y+4, R0
		STD Y+5, R0

		; Get String Legth
		MOVW X, Z
		CALL STR_LEN16 ; XH:XL = Stinrg Pointer | Return = R1:R0

		; Wenn Max Size < Str_len dann Abbruch
		CP  R16, R0
		CPC R17, R1
		BRLO USB_RAM_STR_TO_BUFFER_END

		; Set Size
		STD Y+2, R0 
		STD Y+3, R1

		; Set Pointer to Buffer Begin
		ADIW Y, 6

		MOVW X, Z
		MOVW Z, R1:R0
		CALL COPY_RAM_TO_RAM ; X = From_Pointer, Y = To_Pointer, Z = Copy_Size

	USB_RAM_STR_TO_BUFFER_END:
	POP R0
	POP R17
	POP R16
	POP ZH
	POP ZL
	POP YH
	POP YL
	POP XH
	POP XL
	RET


.endif ;USB_BUFFER_INCLUDE