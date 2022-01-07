;/*
; * usb_cdc_exec_funktions.inc
; *
; *  Created: 28.03.2016 18:11:39
; *   Author: Dominik Koch
;*/ 

.ifndef USB_CDC_EXEC_FUNKTIONS_INCLUDE
.set USB_CDC_EXEC_FUNKTIONS_INCLUDE = 1

;Text Mode
	.include "./drv/atmega32u2/usb_cdc_exec/echo.inc"
	.include "./drv/atmega32u2/usb_cdc_exec/cdc_test.inc"
.endif