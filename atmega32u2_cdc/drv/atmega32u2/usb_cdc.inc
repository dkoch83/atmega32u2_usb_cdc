;/*	
; * usb_cdc.inc
; *
; *  Created: 23.02.2016 18:45:41
; *   Author: Dominik Koch
; */ 
.ifndef USB_CDC_INCLUDE
.set USB_CDC_INCLUDE = 1

.DSEG
.IF USB_CDC_ENABLE == 1
	CDC_LINE_CODING:
		dwDTERate:		.BYTE 4
		bCharFormat:	.BYTE 1
		bParityType:	.BYTE 1
		bDataBits:		.BYTE 1
	CDC_LINE_CODING_END:
	CDC_LINE_TMP:		.BYTE 1

	CDC_LINE_STATE:		.BYTE 1
	.SET CDC_DTR_bp = 0
	.SET CDC_DTR_bm = 0x01
	.SET CDC_RTS_bp = 1
	.SET CDC_RTS_bm = 0x02
	
	CDC_SERIAL_STATE_OLD: .BYTE 1
	CDC_SERIAL_STATE:     .BYTE 1
		.SET bOverRun_bp = 6
		.SET bOverRun_bm = 0b01000000

		.SET bParity_bp = 5
		.SET bParity_bm  = 0b00100000

		.SET bFraming_bp = 4
		.SET bFraming_bm = 0b00010000

		.SET bRingSignal_bp = 3 
		.SET bRingSignal_bm = 0b00001000

		.SET CDC_RI_bp = 3 
		.SET CDC_RI_bm = 0b00001000

		.SET bBreak_bp = 2
		.SET bBreak_bm = 0b00000100

		.SET CDC_BREAK_bp = 2
		.SET CDC_BREAK_bm = 0b00000100

		.SET bTxCarrier_bp = 1
		.SET bTxCarrier_bm = 0b00000010

		.SET CDC_DSR_bp = 1
		.SET CDC_DSR_bm = 0b00000010

		.SET bRxCarrier_bp = 0
		.SET bRxCarrier_bm = 0b00000001

		.SET CDC_DCD_bp = 0
		.SET CDC_DCD_bm = 0b00000001


	CDC_STATUS: .BYTE 1
		.SET CDC_TX_bp = 0
		.SET CDC_TX_bm = 0x01

		.SET CDC_RX_bp = 1
		.SET CDC_RX_bm = 0x02
		
		.SET CDC_ECHO_DISABLE_bp = 2
		.SET CDC_ECHO_DISABLE_bm = 0x04
		
		.SET CDC_BUFFER_LOCK_bp = 3
		.SET CDC_BUFFER_LOCK_bm = 0x08

		.SET CDC_NEW_LN_IN_bp = 4
		.SET CDC_NEW_LN_IN_bm = 0x10

		.SET CDC_RX_BUFFER_FULL_bp = 5
		.SET CDC_RX_BUFFER_FULL_bm = 0x20

		.SET CDC_TX_BUFFER_FULL_bp = 6
		.SET CDC_TX_BUFFER_FULL_bm = 0x40

		.SET CDC_CONNECT_bp = 7
		.SET CDC_CONNECT_bm = 0x80
.ENDIF
.CSEG

;##################################################
;					USB_CDC_INIT
;##################################################
USB_CDC_INIT:
	PUSH R16
	PUSH XL
	PUSH XH
		LDI R16, 0x40
		STS dwDTERate+0, R16
		LDI R16, 0x38
		STS dwDTERate+1, R16
		LDI R16, 0x00
		STS dwDTERate+2, R16
		LDI R16, 0x00
		STS dwDTERate+3, R16

		LDI R16, 0b00000001
		STS bCharFormat+0, R16

		LDI R16, 0b00000000
		STS bParityType+0, R16
	
		LDI R16, 8
		STS bDataBits+0, R16

		LDI R16, 0x00

		.IF USB_CDC_ECHO_OFF == 1
			SBR R16, CDC_ECHO_DISABLE_bm
		.ENDIF

		STS CDC_STATUS, R16
	POP XH
	POP XL
	POP R16
RET
;##################################################
;		USB_CDC_SEND_ENCAPSULATED_COMMAND
;##################################################
USB_CDC_SEND_ENCAPSULATED_COMMAND:
/*
	PUSH ZL
	PUSH ZH
	PUSH R16
	PUSH R17
		LDS R16, CDC_RESPONSE_AVAILABLE
		CPI R16, 0x01
		BREQ USB_CDC_SEND_ENCAPSULATED_COMMAND_RESPONSE_AVAILABLE
		RJMP USB_CDC_SEND_ENCAPSULATED_COMMAND_RESPONSE_AVAILABLE_END
		USB_CDC_SEND_ENCAPSULATED_COMMAND_RESPONSE_AVAILABLE:
			LDI R16, 0x00
			STS CDC_RESPONSE_AVAILABLE, R16

			LDI R16, 0b10100001
			STS EP0_BUFFER+0, R16
			LDI R16, 0x01 ; RESPONSE_AVAILABLE
			STS EP0_BUFFER+1, R16
			LDI R16, 0
			STS EP0_BUFFER+2, R16
			LDI R16, 0
			STS EP0_BUFFER+3, R16
			LDI R16, 0	; Interface
			STS EP0_BUFFER+4, R16
			LDI R16, 0
			STS EP0_BUFFER+5, R16
			LDI R16, 0
			STS EP0_BUFFER+6, R16
			LDI R16, 0
			STS EP0_BUFFER+7, R16

			LDI ZL, 0x08
			LDI ZH, 0x00
			CALL USB_EP0_SEND ; Z = Size	

			RJMP USB_CDC_SEND_ENCAPSULATED_COMMAND_END
		USB_CDC_SEND_ENCAPSULATED_COMMAND_RESPONSE_AVAILABLE_END:
		; Else Send ZLP
		LDI ZL, 0x08
		LDI ZH, 0x00
		CALL USB_EP0_SEND ; Z = Size	
USB_CDC_SEND_ENCAPSULATED_COMMAND_END:
	POP R17
	POP R16
	POP ZH
	POP ZL
*/
RET
;##################################################
;			USB_CDC_GET_ENCAPSULATED_RESPONSE
;##################################################
USB_CDC_GET_ENCAPSULATED_RESPONSE:
	; Send ZLP
	RCALL USB_EP0_SEND_ZLP
RET
;##################################################
;				USB_CDC_SET_LINE_CODING
;##################################################
USB_CDC_SET_LINE_CODING:
PUSH XL
PUSH XH
PUSH R16
		LDS R16, UEINTX
		CBR R16, (1<<RXOUTI)
		STS UEINTX, R16

		USB_CDC_SET_LINE_CODING_WAIT_OUT:
			LDS R16, UEINTX
			SBRS R16, RXOUTI
		RJMP USB_CDC_SET_LINE_CODING_WAIT_OUT
		USB_CDC_SET_LINE_CODING_WAIT_OUT_END:

		LDI XL, LOW(CDC_LINE_CODING)
		LDI XH, HIGH(CDC_LINE_CODING)
		RCALL USB_DATA_TO_RAM ; X = RAM_Pointer | Return = R0 = Byte Counter

		LDS R16, UEINTX
		CBR R16, (1<<RXOUTI)
		STS UEINTX, R16

		; Send ZLP
		RCALL USB_EP0_SEND_ZLP
POP R16
POP XH
POP XL
RET
;##################################################
;				USB_CDC_GET_LINE_CODING
;##################################################
USB_CDC_GET_LINE_CODING:
PUSH XL
PUSH XH
PUSH ZL
PUSH ZH
PUSH R16
PUSH R17
	LDI R17, 7
	LDI ZL, LOW(CDC_LINE_CODING)
	LDI ZH, HIGH(CDC_LINE_CODING)

	USB_CDC_GET_LINE_CODING_LOOP:
		LD  R16, Z+
		STS UEDATX, R16
		DEC R17
	BRNE USB_CDC_GET_LINE_CODING_LOOP

	LDI XH, HIGH(CDC_LINE_CODING_END - CDC_LINE_CODING)
	LDI XL, LOW(CDC_LINE_CODING_END - CDC_LINE_CODING)

	RCALL USB_EP0_SEND
POP R17
POP R16
POP ZH
POP ZL
POP XH
POP XL
RET
;##################################################
;		  USB_CDC_SET_CONTROL_LINE_STATE
;##################################################
USB_CDC_SET_CONTROL_LINE_STATE:
PUSH R16
PUSH R17
	LDS R16, wValueL ; Control Signal Bitmap
	STS CDC_LINE_STATE, R16
	; Send ZLP
	RCALL USB_EP0_SEND_ZLP
POP R17
POP R16
RET
;##################################################
;			USB_CDC_STATUS_CLREAR_TX
;##################################################
USB_CDC_STATUS_CLREAR_TX:
PUSH R16
	LDS R16, CDC_STATUS
	CBR R16, CDC_TX_bm
	STS CDC_STATUS, R16
USB_CDC_STATUS_CLREAR_TX_END:
POP R16
RET
;##################################################
;				USB_CDC_STATUS_CLEAR_RX
;##################################################
USB_CDC_STATUS_CLEAR_RX:
PUSH R16
PUSH ZL
PUSH ZH
	LDS R16, CDC_STATUS
	CBR R16, CDC_RX_bm
	STS CDC_STATUS, R16
USB_CDC_STATUS_CLEAR_RX_END:
POP ZH
POP ZL
POP R16
RET
;##################################################
;				USB_CDC_STATUS_SET_TX
;##################################################
USB_CDC_STATUS_SET_TX:
PUSH R16
	LDS R16, CDC_STATUS
	SBR R16, CDC_TX_bm
	STS CDC_STATUS, R16
USB_CDC_STATUS_SET_TX_END:
POP R16
RET
;##################################################
;				USB_CDC_STATUS_SET_RX
;##################################################
USB_CDC_STATUS_SET_RX:
PUSH R16
	LDS R16, CDC_STATUS
	SBR R16, CDC_RX_bm
	STS CDC_STATUS, R16
USB_CDC_STATUS_SET_RX_END:
POP R16
RET
;##################################################
;				USB_CDC_TX_WAIT
;##################################################
USB_CDC_TX_WAIT:
PUSH R16
		RCALL USB_CDC_STATUS_SET_TX
		USB_CDC_TX_WAIT_LOOP:
			LDS R16, CDC_STATUS	
			SBRC R16, CDC_TX_bp
			RJMP USB_CDC_TX_WAIT_LOOP
		USB_CDC_TX_WAIT_LOOP_END:
USB_CDC_TX_WAIT_END:
POP R16
RET
;##################################################
;					USB_CDC_RX_WAIT
;##################################################
USB_CDC_RX_WAIT:
PUSH R16
		RCALL USB_CDC_STATUS_SET_RX
		USB_CDC_RX_WAIT_LOOP:
			LDS R16, CDC_STATUS	
			SBRC R16, CDC_RX_bp
			RJMP USB_CDC_RX_WAIT_LOOP
		USB_CDC_RX_WAIT_LOOP_END:
USB_CDC_RX_WAIT_END:
POP R16
RET
;##################################################
;				USB_CDC_DISABLE_ECHO
;##################################################
USB_CDC_DISABLE_ECHO:
PUSH R16
	LDS R16, CDC_STATUS
	SBR R16, CDC_ECHO_DISABLE_bm
	STS CDC_STATUS, R16
USB_CDC_DISABLE_ECHO_END:
POP R16
RET
;##################################################
;				USB_CDC_ENABLE_ECHO
;##################################################
USB_CDC_ENABLE_ECHO:
PUSH R16
	LDS R16, CDC_STATUS
	CBR R16, CDC_ECHO_DISABLE_bm
	STS CDC_STATUS, R16
USB_CDC_ENABLE_ECHO_END:
POP R16
RET
;##################################################
;				USB_CDC_CLR_NEW_LN_IN
;##################################################
USB_CDC_CLR_NEW_LN_IN:
PUSH R16
	LDS R16, CDC_STATUS
	CBR R16, CDC_NEW_LN_IN_bm
	STS CDC_STATUS, R16
USB_CDC_CLR_NEW_LN_IN_END:
POP R16
RET
;##################################################
;				USB_CDC_SET_NEW_LN_IN
;##################################################
USB_CDC_SET_NEW_LN_IN:
PUSH R16
	LDS R16, CDC_STATUS
	SBR R16, CDC_NEW_LN_IN_bm
	STS CDC_STATUS, R16
USB_CDC_SET_NEW_LN_IN_END:
POP R16
RET
;##################################################
;				USB_CDC_CLR_RX_BUFFER_FULL
;##################################################
USB_CDC_CLR_RX_BUFFER_FULL:
PUSH R16
	LDS R16, CDC_STATUS
	CBR R16, CDC_RX_BUFFER_FULL_bm
	STS CDC_STATUS, R16
USB_CDC_CLR_RX_BUFFER_FULL_END:
POP R16
RET
;##################################################
;				USB_CDC_SET_RX_BUFFER_FULL
;##################################################
USB_CDC_SET_RX_BUFFER_FULL:
PUSH R16
	LDS R16, CDC_STATUS
	SBR R16, CDC_RX_BUFFER_FULL_bm
	STS CDC_STATUS, R16
USB_CDC_SET_RX_BUFFER_FULL_END:
POP R16
RET
;##################################################
;				USB_CDC_CLR_TX_BUFFER_FULL
;##################################################
USB_CDC_CLR_TX_BUFFER_FULL:
PUSH R16
	LDS R16, CDC_STATUS
	CBR R16, CDC_TX_BUFFER_FULL_bm
	STS CDC_STATUS, R16
USB_CDC_CLR_TX_BUFFER_FULL_END:
POP R16
RET
;##################################################
;				USB_CDC_SET_TX_BUFFER_FULL
;##################################################
USB_CDC_SET_TX_BUFFER_FULL:
PUSH R16
	LDS R16, CDC_STATUS
	SBR R16, CDC_TX_BUFFER_FULL_bm
	STS CDC_STATUS, R16
USB_CDC_SET_TX_BUFFER_FULL_END:
POP R16
RET


.include "./drv/atmega32u2/usb_cdc_f.inc"
.endif