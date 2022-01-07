;/*
; * usb_config.inc
; *
; *  Created: 01.03.2016 21:10:48
; *   Author: Dominik Koch
; */ 

.SET USB_REMOTE_WAKEUP = 0

;Enable Interrupts
;---------------------
.SET USB_ENABLE_SOFE		= 0	;Start Of Frame Interrupt Enable
.SET USB_ENABLE_UPRSME		= 0 ;Upstream Resume Interrupt Enable
.SET USB_ENABLE_WAKUPE		= 0 ;Wake-up CPU Interrupt Enable
.SET USB_ENABLE_SUSPE		= 0 ;Suspend Interrupt Enable
.SET USB_ENABLE_EORSME		= 0 ;End Of Resume Interrupt Enable

.EQU EP0_SIZE	= 8
.EQU EP1_SIZE	= 32
.EQU EP2_SIZE	= 32
.EQU EP3_SIZE	= 16
.EQU EP4_SIZE	= 0

;################  USB CDC #################
.SET USB_CDC_ENABLE				= 1

.SET USB_CDC_ECHO_OFF			= 1


.SET USB_CDC_CONFIGURATION_NUM	= 0x01

.SET EP_CDC_C_SIZE				= 16
.SET EP_CDC_D_SIZE				= 32

.SET EP_CDC_D_OUT				= 0x01
.SET EP_CDC_D_IN				= 0x82	
.SET EP_CDC_C_IN				= 0x83

