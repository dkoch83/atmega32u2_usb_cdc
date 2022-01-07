;/*
; * usb.inc
; *
; *  Created: 31.05.2014 23:52:18
; *   Author: Dominik
; */ 

.ifndef atmega32u2_usb_include
.set atmega32u2_usb_include = 1



.DSEG
	USB_CONFIG:			.BYTE 1
	USB_INTERFACE:		.BYTE 1
	USB_INTERFACE_ALT:	.BYTE 1

	USB_EP_SETUP_BUFFER:
		bmRequestType:	.BYTE 1
		bRequest:		.BYTE 1
		wValueL:		.BYTE 1
		wValueH:		.BYTE 1
		wIndexL:		.BYTE 1
		wIndexH:		.BYTE 1
		wLengthL:		.BYTE 1
		wLengthH:		.BYTE 1
	USB_EP_SETUP_BUFFER_END:

	USB_INTERF_ALT_CONF:
		USB_INTERF00_ALT_CONF: .BYTE 1
		USB_INTERF01_ALT_CONF: .BYTE 1
		USB_INTERF02_ALT_CONF: .BYTE 1
	USB_INTERF_ALT_CONF_END:
.CSEG

.EQU EP_CONTROL		= 0b00000000
.EQU EP_ISOCHRONOUS	= 0b01000000
.EQU EP_BULK		= 0b10000000
.EQU EP_INTERRUPT	= 0b11000000

.EQU EP_IN			= 0b00000001
.EQU EP_OUT			= 0b00000000

; standard control endpoint request types
.EQU GET_STATUS			= 0x00
.EQU CLEAR_FEATURE		= 0x01
.EQU SET_FEATURE		= 0x03
.EQU SET_ADDRESS		= 0x05
.EQU GET_DESCRIPTOR		= 0x06
.EQU SET_DESCRIPTOR		= 0x07
.EQU GET_CONFIGURATION	= 0x08
.EQU SET_CONFIGURATION	= 0x09
.EQU GET_INTERFACE		= 0x0A
.EQU SET_INTERFACE		= 0x0B
.EQU SYNCH_FRAME		= 0x12

; HID (human interface device)
.EQU GET_REPORT			= 0x01
.EQU GET_IDLE			= 0x02
.EQU GET_PROTOCOL		= 0x03
.EQU SET_REPORT			= 0x09
.EQU SET_IDLE			= 0x0A
.EQU SET_PROTOCOL		= 0x0B

; CDC (communication class device)
.EQU SEND_ENCAPSULATED_COMMAND		= 0x00
.EQU GET_ENCAPSULATED_RESPONSE		= 0x01
.EQU SET_LINE_CODING				= 0x20
.EQU GET_LINE_CODING				= 0x21
.EQU SET_CONTROL_LINE_STATE			= 0x22
.EQU SEND_BREAK						= 0x23

; CDC (Class-Specific Notification Codes)
.EQU NETWORK_CONNECTION		= 0x00
.EQU RESPONSE_AVAILABLE		= 0x01
.EQU AUX_JACK_HOOK_STATE	= 0x08
.EQU RING_DETECT			= 0x09
.EQU SERIAL_STATE			= 0x20
.EQU CALL_STATE_CHANGE		= 0x28
.EQU LINE_STATE_CHANGE		= 0x23


; USB Mass Storage Class Bulk-Only (BBB) Transport
.EQU Get_Max_LUN		= 0xFE
.EQU GML				= 0xFE
.EQU BOMSR				= 0xFF

; Descriptors
.EQU DEVICE_DESCRIPTOR				= 0x01
.EQU CONFIG_DESCRIPTOR				= 0x02
.EQU STRING_DESCRIPTOR				= 0x03
.EQU INTERFACE_DESCRIPTOR			= 0x04
.EQU ENDPOINT_DESCRIPTOR			= 0x05
.EQU DEVICE_QUALIFIER_DESCRIPTOR	= 0x06
.EQU OTHER_SPEED_DESCRIPTOR			= 0x07
.EQU HID_DESCRIPTOR					= 0x21
.EQU REPORT_DESCRIPTOR				= 0x22
.EQU PHYSICAL_DESCRIPTOR			= 0x23
.EQU CS_INTERFACE					= 0x24
.EQU CS_ENDPOINT					= 0x25

; Request Type
.EQU STANDARDREQUEST	= 0b00000000
.EQU CLASSREQUEST		= 0b00100000
.EQU VENDORREQUEST		= 0b01000000

; Recipient
.EQU RequestType_DEVICE			= 0
.EQU RequestType_INTERFACE		= 1
.EQU RequestType_ENDPOINT		= 2
.EQU RequestType_OTHER			= 3

; bmAttributes
.EQU bmAttr_Control			= 0b00000000 
.EQU bmAttr_Isochronous		= 0b00000001
.EQU bmAttr_Bulk			= 0b00000010
.EQU bmAttr_Interrupt		= 0b00000011


;---------------------------------------------------------------------------------------
.MACRO usb_freeze_clock
	; Frezze USB Clock
	LDS R16, USBCON
	SBR R16, (1<<FRZLK)	
	STS USBCON, R16	
.ENDMACRO
;---------------------------------------------------------------------------------------
.MACRO usb_unfreeze_clock
	; Frezze USB Clock
	LDS R16, USBCON
	CBR R16, (1<<FRZLK)	
	STS USBCON, R16	
.ENDMACRO
;---------------------------------------------------------------------------------------
.MACRO usb_atach
	LDS R16, UDCON
	CBR R16, (1<<DETACH)	; Atach
	STS UDCON, R16
.ENDMACRO
;---------------------------------------------------------------------------------------
.MACRO usb_detach
	LDS R16, UDCON
	SBR R16, (1<<DETACH)	; detach
	STS UDCON, R16
.ENDMACRO
;---------------------------------------------------------------------------------------
.MACRO USB_SETUP_ACK
	LDS R16, UEINTX
	CBR R16, (1<<RXSTPI)
	STS UEINTX, R16
.ENDMACRO
;---------------------------------------------------------------------------------------
.MACRO USB_SETUP_STALL
	LDS R16, UECONX
	SBR R16, (1<<STALLRQ)
	STS UECONX, R16
.ENDMACRO
;---------------------------------------------------------------------------------------
.include "./drv/atmega32u2/usb_config.inc"
.include "./drv/atmega32u2/usb_buffer.inc"
.IF USB_CDC_ENABLE == 1
	.INCLUDE "./drv/atmega32u2/usb_cdc.inc"
.ENDIF
;---------------------------------------------------------------------------------------
USB_INIT:
PUSH R16
	.IF USB_CDC_ENABLE == 1
	RCALL USB_CDC_INIT
	.ENDIF

	LDS R16, REGCR
	CBR R16, (1<<REGDIS)	; Enable 3,3V Regulator
	STS REGCR, R16

	CLR R16
	SBR R16, (1<<USBE) 		; Enable USB
	STS USBCON, R16			; Enable USB
	STS USBCON, R16			; Unfreeze CLK

	CLR R16
	STS USB_CONFIG, R16		; Set USB Config to Zero
	STS USB_INTERFACE, R16	; Set Interface to Zero

	

	;UPRSME EORSME WAKEUPE EORSTE SOFE SUSPE
	.SET UDIENSET = (1<<EORSTE)
	.IF USB_ENABLE_SOFE == 1
		.SET UDIENSET = UDIENSET | (1<<SOFE)
	.ENDIF
	.IF USB_ENABLE_UPRSME == 1
		.SET UDIENSET = UDIENSET | (1<<UPRSME)
	.ENDIF
	.IF USB_ENABLE_WAKUPE == 1
		.SET UDIENSET = UDIENSET | (1<<WAKEUPE)
	.ENDIF
	.IF USB_ENABLE_SUSPE == 1
		.SET UDIENSET = UDIENSET | (1<<SUSPE)
	.ENDIF
	.IF USB_ENABLE_EORSME == 1
		.SET UDIENSET = UDIENSET | (1<<EORSME)
	.ENDIF

	;LDS R16, UDIEN
	LDI R16, UDIENSET

	;SBR	R16, (1<<EORSTE)	; Enable End Of Reset Interrupt
	STS UDIEN, R16

	LDS R16, UDCON
	CBR R16, (1<<DETACH)	; Atach
	STS UDCON, R16

	; Set Buffer Pointer to Zero
	LDI R16, 0x00
	STS USB_RX_BUFFER_P+0, R16
	STS USB_RX_BUFFER_P+1, R16

	STS USB_TX_BUFFER_P+0, R16
	STS USB_TX_BUFFER_P+1, R16

USB_INIT_END:
POP R16
RET
;---------------------------------------------------------------------------------------
USB_CLEAR_EPS:
PUSH R17
PUSH R16
	LDI R17, 1
	CLEAR_LOOP:
		STS UENUM, R17	; Set EP Nummer

		LDS R16, UECFG1X
		CBR R16, (1<<ALLOC) ; Dealloc
		STS UECFG1X, R16

		LDS R16, UECONX
		CBR R16, (1<<EPEN) ; Disable EP
		STS UECONX, R16

		INC R17
		CPI R17, 5
	BRNE CLEAR_LOOP
POP R16
POP R17
RET
;---------------------------------------------------------------------------------------
USB_CLEAR_EP: ; R16 EP_NUM
PUSH R16
	STS UENUM, R16	; Set EP Nummer

	LDS R16, UECFG1X
	CBR R16, (1<<ALLOC) ; Dealloc
	STS UECFG1X, R16

	LDS R16, UECONX
	CBR R16, (1<<EPEN) ; Disable EP
	STS UECONX, R16
POP R16
RET
;---------------------------------------------------------------------------------------
USB_INIT_EP: ;R16 EP_NUM, R17 EP_TYPE, R18 EP_SIZE, R19 EP_BANKS
PUSH R16
PUSH R17
PUSH R18
PUSH R19
PUSH R23
	; Set EP IN or OUT
	SBRC R16, 7			; 0x80 EP_IN  0x00 EP_OUT
	SBR  R17, 1<<EPDIR	; Set EP_IN
	CBR  R16, 0x80

	STS UENUM, R16		; Set EP Nummer

	; Enable EP
	LDS R23, UECONX
	SBR R23, (1<<EPEN)
	STS UECONX, R23	   

	; Config Endpoint  
	STS UECFG0X, R17

	; LOAD UECFG1X Register
	LDS R23, UECFG1X
	CBR R23, (1<<EPSIZE2) | (1<<EPSIZE1) | (1<<EPSIZE0)	; Clear EPSIZE default 8 Byte
	CBR R23, (1<<EPBK1) | (1<<EPBK0)					; Clear EPBK1:0 default One Bank

	; Set EP Size
	SET_EP_SIZE:
	CPI R18, 8	; Wenn Size 8
	BREQ EP_BANKS
	CPI R18, 16	; Wenn Size 16
	BREQ EP_S16
	CPI R18, 32 ; Wenn Size 32
	BREQ EP_S32

	RJMP EP_S64 ; Wenn Size 64
		
	EP_S16:
		SBR R23, (0<<EPSIZE2) | (0<<EPSIZE1) | (1<<EPSIZE0)
	RJMP EP_BANKS

	EP_S32:
		SBR R23, (0<<EPSIZE2) | (1<<EPSIZE1) | (0<<EPSIZE0)	
	RJMP EP_BANKS

	EP_S64:
		SBR R23, (0<<EPSIZE2) | (1<<EPSIZE1) | (1<<EPSIZE0)

	; Set EP Banks
	EP_BANKS:
		CPI R19, 0		; Wenn EP Bank 0 = One Bank
		BREQ EP_SIZE_ALLOC
		CPI R19, 1		; Wenn EP Bank 1 = One Bank
		BREQ EP_SIZE_ALLOC
		
		; Wenn EP Banks 2
		TWO_BANKS:
			SBR R23, (0<<EPBK1) | (1<<EPBK0)

	; Alloc EP
	EP_SIZE_ALLOC:
		STS UECFG1X, R23
		SBR R23, (1<<ALLOC)
		STS UECFG1X, R23

	; Config Interrupts
	TST R16						; Test for EP 0
	BREQ SETUP_EP				; If EP = 0 SETUP_EP

	SBRC R17, 0					; Test for IN OUP EP
	RJMP IN_EP
	
	OUT_EP:
		CLR R23
		SBR R23, (1<<RXOUTE)	; Enable Received OUT Data Interrupt
		STS UEIENX, R23	
	RJMP INIT_EP_END

	IN_EP:
		CLR R23
		SBR R23, (1<<NAKINE)	; Enable NAK IN Interrupt
		STS UEIENX, R23	
	RJMP INIT_EP_END

	SETUP_EP:
		CLR R23
		SBR R23, (1<<RXSTPE)	; Enable Recieved SETUP Interrupt
		STS UEIENX, R23	
INIT_EP_END:
POP R23
POP R19
POP R18
POP R17
POP R16
RET
;---------------------------------------------------------------------------------------
USB_PGM_SEND_DES: ;(Z) Pointer Address, (X) Send Bytes
PUSH R16
PUSH R17
PUSH ZL
PUSH ZH
PUSH YL
PUSH YH
PUSH XL
PUSH XH
	; Convert to Progmem Adress
	LSL ZL
	ROL ZH

	; Wenn X >= Y dann X = Y else X = X
	LDS YL, wLengthL
	LDS YH, wLengthH
	CP  XL, YL
	CPC XH, YH
	BRSH USB_PGM_SEND_DES_SIZE
	RJMP USB_PGM_SEND_DES_SIZE_END
	USB_PGM_SEND_DES_SIZE:
		MOVW X, Y
	USB_PGM_SEND_DES_SIZE_END:

	LOOP_SEND_PKG:
		LDI R16, EP0_SIZE
		LOOP_PGM_SEND_DES:
			LDS R17, UEINTX
			SBRC R17, RXOUTI			; Wenn KILLBK is clear Return
			RJMP USB_PGM_SEND_DES_AB
			
			LPM R17, Z+					; Read Byte from Z
			STS UEDATX, R17				; Store to USB FIFO

			SBIW X, 1					; Send Bytes - 1	
			BREQ USB_PGM_SEND_DES_LAST	; Wenn X = 0  

			DEC R16	
		BRNE LOOP_PGM_SEND_DES
		; Send PKG
		RCALL USB_EP0_SEND
	RJMP LOOP_SEND_PKG
	;#####################################################
	USB_PGM_SEND_DES_LAST:
		RCALL USB_EP0_SEND
		DEC R16
	BREQ USB_PGM_SEND_DES_ZLP

	RJMP USB_PGM_SEND_DES_ZLP_END
	USB_PGM_SEND_DES_ZLP:
		RCALL USB_EP0_SEND_ZLP
	USB_PGM_SEND_DES_ZLP_END:

	USB_DES_SEND_WAIT_RXOUTI:
		LDS R17, UEINTX
		SBRS R17, RXOUTI
	RJMP USB_DES_SEND_WAIT_RXOUTI

	USB_PGM_SEND_DES_AB:
		LDS R17, UEINTX
		CBR R17, (1<<RXOUTI)
		STS UEINTX, R17
	USB_PGM_SEND_DES_AB_END:

USB_PGM_SEND_DES_END:
POP XH
POP XL
POP YH
POP YL
POP ZH
POP ZL
POP R17
POP	R16
RET
;---------------------------------------------------------------------------------------
USB_EP0_SEND:
PUSH R16
	LDS R16, UEINTX
	CBR R16, (1<<TXINI | 1<<RXOUTI)
	STS UEINTX, R16
	USB_EP0_SEND_WAIT:
		LDS R16, UEINTX
		ANDI R16, (1<<TXINI) | (1<<RXOUTI)
		CPI R16, 0x00	
	BREQ USB_EP0_SEND_WAIT
POP R16
RET
;---------------------------------------------------------------------------------------
USB_EP0_SEND_ZLP:
PUSH R16
	; Reset EP0 Buffer
	LDS R16, UERST
	SBR R16, 0
	STS UERST, R16
	CBR R16, 0
	STS UERST, R16
	;Send ZLP
	LDS R16, UEINTX
	CBR R16, (1<<TXINI)
	STS UEINTX, R16
	USB_EP0_SEND_ZLP_WAIT:
		LDS R16, UEINTX
		SBRS R16, TXINI
	RJMP USB_EP0_SEND_ZLP_WAIT
POP R16
RET
;################################################################################
USB_DATA_TO_RAM: ; X = RAM_Pointer | Return = R0 = Byte Counter
PUSH R1
PUSH R16
PUSH R17
	LDS R0, UEBCLX
	MOV R1, R0

	CLR R17
	CP R0, R17
	BREQ USB_DATA_TO_RAM_END

	USB_DATA_TO_RAM_LOOP:
		LDS R16, UEDATX	
		ST X+, R16
		DEC R1
	BRNE USB_DATA_TO_RAM_LOOP
	USB_DATA_TO_RAM_END:
POP R17
POP R16
POP R1
RET
;################################################################################
USB_DATA_TO_BUFFER: ; X = RAM_Pointer | Return = R0 = (0 Buffer Free) / (1 Buffer Full)
PUSH R1
PUSH R18
PUSH YL
PUSH YH
PUSH XL
PUSH XH
PUSH ZL
PUSH ZH

	MOVW Y, X
	LDD XL, Y+0
	LDD XH, Y+1
	MOVW Y, X

	; Load Max Size
	LDD XL, Y+0
	LDD XH, Y+1

	; Load Buffer Size
	LDD ZL, Y+2
	LDD ZH, Y+3

	; Save OLD Size to POS (for echo)
	;STD Y+4, ZL
	;STD Y+5, ZH

	; Save Y to Stack
	PUSH YL
	PUSH YH

	; Set Buffer Pointer 
	ADIW Y, 6
	ADD YL, ZL
	ADC YH, ZH

	; Load paket length	
	LDS R1, UEBCLX
	CLR R0

	; Test for ZLP
	CP R1, R0
	BREQ USB_DATA_TO_BUFFER_END

	USB_DATA_TO_BUFFER_LOOP:
		; Compare Size Max with Size
		CP  ZL, XL
		CPC ZH, XH
		BRSH USB_DATA_BUFFER_FULL

		LDS R18, UEDATX	
		
		; Test for new Line (Enter)
		CPI R18, 0x0D
		BREQ USB_DATA_ENTER_CHAR

		ST Y+, R18
		ADIW Z, 1
		DEC R1
	BRNE USB_DATA_TO_BUFFER_LOOP
	USB_DATA_TO_BUFFER_END:

	; Zero Terminated String
	CLR R18
	ST Y+, R18

	RJMP USB_DATA_DUMMY_LOOP_END
	USB_DATA_BUFFER_FULL:
		INC R0	; INC R0 to 1 = Buffer is Full
		RJMP USB_DATA_DUMMY_LOOP
	USB_DATA_ENTER_CHAR:
		CALL USB_CDC_SET_NEW_LN_IN
		CLR R18
		ST Y+, R18
		;RJMP USB_DATA_DUMMY_LOOP
	USB_DATA_DUMMY_LOOP:
		LDS R18, UEDATX	
		DEC R1
	BRNE USB_DATA_DUMMY_LOOP
	USB_DATA_DUMMY_LOOP_END:

	; Load Y from Stack
	POP YH
	POP YL

	; Save new Buffer Size
	STD Y+2, ZL 
	STD Y+3, ZH
POP ZH
POP ZL
POP XH
POP XL
POP YH
POP YL
POP R18
POP R1
RET
;################################################################################
.include "./drv/atmega32u2/usb_cdc_des.inc"
.include "./drv/atmega32u2/usb_ep0_control.inc"
.IF USB_CDC_ENABLE == 1
	.INCLUDE "./include/cmd_string.inc"
	.INCLUDE "./drv/atmega32u2/usb_cdc_exec_funktions.inc"
.ENDIF
;################################################################################
.endif ;atmega32u2_usb_include