;/*
; * atmega32u2.asm
; *
; *  Created: 30.05.2014 02:30:03
; *   Author: Dominik
; */ 

.set F_MCU		= 16000000
.set ENABLE_USB = 1

.CSEG
.org $0000
JMP RESET			; External Pin, Power-on Reset, Brown-out Reset, Watchdog Reset, USB Reset and debugWIRE AVR Reset
JMP EX_INT0			; External Interrupt Request 0
JMP EX_INT1			; External Interrupt Request 1
JMP EX_INT2			; External Interrupt Request 2
JMP EX_INT3			; External Interrupt Request 3
JMP EX_INT4			; External Interrupt Request 4
JMP EX_INT5			; External Interrupt Request 5
JMP EX_INT6			; External Interrupt Request 6
JMP EX_INT7			; External Interrupt Request 7
JMP PC_INT0			; Pin Change Interrupt Request 0
JMP PC_INT1			; Pin Change Interrupt Request 1
JMP USB_GEN_INT		; General USB General Interrupt request
JMP USB_EP_INT		; Endpoint USB Endpoint Interrupt request
JMP WDT_INT			; Watchdog Time-out Interrupt
JMP TIMER1_CAPT		; Timer/Counter1 Capture Event
JMP TIMER1_COMPA	; Timer/Counter1 Compare Match A
JMP TIMER1_COMPB	; Timer/Counter1 Compare Match B
JMP TIMER1_COMPC	; Timer/Counter1 Compare Match C
JMP TIMER1_OVF		; Timer/Counter1 Overflow
JMP TIMER0_COMPA	; Timer/Counter0 Compare Match A
JMP TIMER0_COMPB	; Timer/Counter0 Compare match B
JMP TIMER0_OVF		; Timer/Counter0 Overflow
JMP SPI_STC			; SPI Serial Transfer Complete
JMP USART1_RX		; USART1 Rx Complete
JMP USART1_UDRE		; USART1 Data Register Empty
JMP USART1_TX		; USART1 Tx Complete
JMP ANALOG_COMP		; Analog Comparator
JMP EE_READY		; EEPROM Ready
JMP SPM_READY		; Store Program Memory Ready

.NOLIST
	.include "./drv/atmega32u2/clock.inc"
	.include "./drv/atmega32u2/spi.inc"
	.include "./drv/atmega32u2/usart_spi.inc"
	
	;Uncommnet to DISABLE Functions
	.set NO_PGM_READ_BYTE		= 1
	.set NO_PGM_READ_WORD		= 1
	.set NO_PGM_READ_BUFFER		= 1
	.set NO_PGM_READ_STR		= 1
	;.set NO_PGM_STRLEN			= 1
	.set NO_PGM_FIND_CHAR		= 1
	.include "./drv/pgm.inc"

	.include "./include/delay.inc"

.if ENABLE_USB == 1
	.include "./drv/atmega32u2/usb.inc"
	.include "./drv/atmega32u2/usb_cdc_string.inc"
.endif
.LIST

RESET:			; External Pin, Power-on Reset, Brown-out Reset, Watchdog Reset, USB Reset and debugWIRE AVR Reset
	;Clear Ram
	LDI	XL, LOW(SRAM_START)
	LDI	XH, HIGH(SRAM_START)

	LDI YL, LOW(SRAM_SIZE)
	LDI YH, HIGH(SRAM_SIZE)
	LDI		R16, 0x00
	RAMCLEAN:
		ST X+, R16
		SBIW Y, 1
	BRNE RAMCLEAN
	;Init stack pointer
	LDI		R16, LOW(RAMEND)	;
	LDI		R17, HIGH(RAMEND)	;
	OUT		SPL, R16			;
	OUT		SPH, R17			;
RJMP MAIN

EX_INT0:		; External Interrupt Request 0
RETI

EX_INT1:		; External Interrupt Request 1
RETI

EX_INT2:		; External Interrupt Request 2
RETI

EX_INT3:		; External Interrupt Request 3
RETI

EX_INT4:		; External Interrupt Request 4
RETI

EX_INT5:		; External Interrupt Request 5
RETI

EX_INT6:		; External Interrupt Request 6
RETI

EX_INT7:		; External Interrupt Request 7
RETI

PC_INT0:		; Pin Change Interrupt Request 0
RETI

PC_INT1:		; Pin Change Interrupt Request 1
RETI


.if USB_CDC_ENABLE == 1 
.INCLUDE "./drv/atmega32u2/usb_cdc_interrupts.inc"
.else 
	USB_GEN_INT:	; General USB General Interrupt request
	RETI

	USB_EP_INT:	; Endpoint USB Endpoint Interrupt request
	RETI
.endif

WDT_INT:		; Watchdog Time-out Interrupt
RETI

TIMER1_CAPT:	; Timer/Counter1 Capture Event
RETI

TIMER1_COMPA:	; Timer/Counter1 Compare Match A
RETI
 
TIMER1_COMPB:	; Timer/Counter1 Compare Match B
RETI

TIMER1_COMPC:	; Timer/Counter1 Compare Match C
RETI

TIMER1_OVF:	; Timer/Counter1 Overflow
RETI

TIMER0_COMPA:	; Timer/Counter0 Compare Match A
RETI

TIMER0_COMPB:	; Timer/Counter0 Compare match B
RETI

TIMER0_OVF:	; Timer/Counter0 Overflow
RETI

SPI_STC:		; SPI Serial Transfer Complete
RETI

USART1_RX:		; USART1 Rx Complete
RETI

USART1_UDRE:	; USART1 Data Register Empty
RETI

USART1_TX:		; USART1 Tx Complete
RETI

ANALOG_COMP:	; Analog Comparator
RETI

EE_READY:		; EEPROM Ready
RETI
  
SPM_READY:		; Store Program Memory Ready
RETI


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

.CSEG


; Main Init
MAIN:
	CLI
	SET_CLOCK_DIV CLK_DIV_1
	CALL INIT_PLL_EXT

	CALL USB_INIT

	USB_INIT_BUFFER USB_TX_BUFFER, 255
	USB_INIT_BUFFER USB_RX_BUFFER, 255

	USB_SET_TX_BUFFER USB_TX_BUFFER
	USB_SET_RX_BUFFER USB_RX_BUFFER

	SEI
	CALL USB_CDC_STATUS_SET_RX
MAIN_END:

; Main Loop
LOOP:
	LDS R16, CDC_STATUS
	SBRC R16, CDC_NEW_LN_IN_bp
	RJMP LOOP_RX_TX

	SBRC R16, CDC_RX_BUFFER_FULL_bp
	CALL USB_CDC_FLUSH_RX_BUFFER

	RJMP LOOP_RX_TX_END
	LOOP_RX_TX:
		;-----------------------------------------------
		; Echo Input
		LDS  R16, CDC_STATUS
		SBRS R16, CDC_ECHO_DISABLE_bp
		CALL USB_CDC_RX_ECHO
		;-----------------------------------------------

		LDI YL,  LOW(USB_RX_BUFFER_P)
		LDI YH, HIGH(USB_RX_BUFFER_P)
		LDD XL, Y+0
		LDD XH, Y+1
		MOVW Y, X
		ADIW Y, 6

		CALL STR_LEN8 ; XH:XL = Stinrg Pointer | Return = R0
		MOV R16, R0
		CPI R16, 0x00
		BREQ USB_CDC_CMD_FINISH_JMP

		RJMP USB_CDC_CMD_FINISH_JMP_END
		USB_CDC_CMD_FINISH_JMP:
		 JMP USB_CDC_CMD_FINISH
		USB_CDC_CMD_FINISH_JMP_END:

			;##################################################
			;					  CMD_ECHO
			;##################################################
			COMMAND_PGM_EXEC CMD_ECHO, CMD_ECHO_EXEC  ; (PGM_CMD_STRING_POINTER, FUNCTION_POINTER)
			SBRC R0, 0		
			RJMP USB_CDC_CMD_FINISH

			;##################################################
			;					  CMD_CDC ; Testing
			;##################################################
			COMMAND_PGM_EXEC CMD_CDC, CMD_CDC_EXEC  ; (PGM_CMD_STRING_POINTER, FUNCTION_POINTER)
			SBRC R0, 0		
			RJMP USB_CDC_CMD_FINISH

		USB_CDC_CMD_FINISH:
		LDS  R16, CDC_STATUS
		SBRC R16, CDC_NEW_LN_IN_bp
		CALL USB_CDC_FLUSH_RX_BUFFER
		CALL PRINT_NL
	LOOP_RX_TX_END:
RJMP LOOP
LOOP_END: