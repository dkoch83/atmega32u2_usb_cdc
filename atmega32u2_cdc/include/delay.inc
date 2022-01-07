;/*
; * delay.inc
; *
; *  Created: 15.05.2014 22:57:38
; *   Author: Dominik
; */
.ifndef delay_include
.set delay_include = 1

;.set F_MCU		= 8000000

; delay_us us				max 16bit
; delay_ms ms				max 16bit


.SET clk_ns		= (F_MCU / 1000000000)
.SET clk_us		= (F_MCU / 1000000)
.SET clk_ms		= (F_MCU / 1000)
.SET clk_s		= (F_MCU / 1)

delay_us_1:
.SET clk_delay_us1 = ((clk_us)-14)
.IF ((clk_delay_us1 - 6) / 3) > 0
PUSH R16						; 2
		LDI R16, ((clk_delay_us1 - 6) / 3)	; 1
		delay_us_1_loop:
			DEC R16				; 1
		BRNE delay_us_1_loop	; 1/2
		.IF ((clk_delay_us1 - 6) % 3) < 3
			RJMP PC+1 
		.ELSEIF ((clk_delay_us1 - 6) % 3) < 2
			NOP
		.ENDIF
POP R16							; 2
.else 
	.IF clk_delay_us1 < 9
		JMP PC+1				; 3
		JMP PC+1				; 3
		JMP PC+1				; 3
	.ELSE .IF clk_delay_us1 < 8
		JMP PC+1				; 3
		JMP PC+1				; 3
		NOP						; 1
	.ELSE .IF clk_delay_us1 < 7
		JMP PC+1				; 3
		JMP PC+1				; 3
	.ELSE .IF clk_delay_us1 < 6
		JMP PC+1				; 3
		RJMP PC+1				; 2
	.ELSE .IF clk_delay_us1 < 5
		JMP PC+1				; 3
		NOP						; 1
	.ELSE .IF clk_delay_us1 < 4
		JMP PC+1				; 3
	.ELSE .IF clk_delay_us1 < 3
		RJMP PC+1				; 2
	.ELSE .IF clk_delay_us1 < 2
		NOP						; 1
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
.endif
delay_us_1_end:
RET


delay_ms_1:
PUSH R16
PUSH XL
PUSH XH
	.SET clk_delay_ms1 = ((clk_ms)-13-12-2) / 10000
	.IF clk_delay_ms1 > 0
		LDI R16, clk_delay_ms1				; 1
		delay_ms_1_LOOP_1:
			LDI XL,  LOW(9995 / 5)			; 1
			LDI XH, HIGH(9995 / 5)			; 1
			delay_ms_1_LOOP_10000:
				NOP							; 1
				SBIW X, 1					; 2
			BRNE delay_ms_1_LOOP_10000		; 1/2
			DEC R16							; 1
		BRNE delay_ms_1_LOOP_1				; 1/2
	.endif

	.SET clk_delay_ms1 = (((((clk_ms)-13-12-2) % 10000)-2) / 1000)
	.IF clk_delay_ms1 > 0
		LDI R16, clk_delay_ms1				; 1
		delay_ms_1_LOOP_2:
			LDI XL,  LOW(995 / 5)			; 1
			LDI XH, HIGH(995 / 5)			; 1
			delay_ms_1_LOOP_1000:
				NOP							; 1
				SBIW X, 1					; 2
			BRNE delay_ms_1_LOOP_1000		; 1/2
			DEC R16							; 1
		BRNE delay_ms_1_LOOP_2				; 1/2
	.endif

	.SET clk_delay_ms1 = (((((((clk_ms)-13-12-2) % 10000)-2) % 1000)-2) / 100)
	.IF clk_delay_ms1 > 0
		LDI R16, clk_delay_ms1				; 1
		delay_ms_1_LOOP_3:
			LDI XL,  LOW(95 / 5)			; 1
			LDI XH, HIGH(95 / 5)			; 1
			delay_ms_1_LOOP_100:
				NOP							; 1
				SBIW X, 1					; 2
			BRNE delay_ms_1_LOOP_100		; 1/2
			DEC R16							; 1
		BRNE delay_ms_1_LOOP_3				; 1/2
	.endif

	.SET clk_delay_ms1 = (((((((((clk_ms)-13-12-2) % 10000)-2) % 1000)-2) % 100)-2) / 10)
	.IF clk_delay_ms1 > 0
		LDI R16, clk_delay_ms1				; 1
		delay_ms_1_LOOP_4:
			LDI XL,  LOW(5 / 5)				; 1
			LDI XH, HIGH(5 / 5)				; 1
			delay_ms_1_LOOP_10:
				NOP							; 1
				SBIW X, 1					; 2
			BRNE delay_ms_1_LOOP_10			; 1/2
			DEC R16							; 1
		BRNE delay_ms_1_LOOP_4				; 1/2
	.endif

	.SET clk_delay_ms1 = ((((((((((clk_ms)-13-12-2) % 10000)-2) % 1000)-2) % 100)-2) % 10)-2)
	.IF clk_delay_ms1 < 10
		JMP PC+1				; 3
		JMP PC+1				; 3
		JMP PC+1				; 3
		NOP
	.ELSE .IF clk_delay_ms1 < 9
		JMP PC+1				; 3
		JMP PC+1				; 3
		JMP PC+1				; 3
	.ELSE .IF clk_delay_ms1 < 8
		JMP PC+1				; 3
		JMP PC+1				; 3
		NOP						; 1
	.ELSE .IF clk_delay_ms1 < 7
		JMP PC+1				; 3
		JMP PC+1				; 3
	.ELSE .IF clk_delay_ms1 < 6
		JMP PC+1				; 3
		RJMP PC+1				; 2
	.ELSE .IF clk_delay_ms1 < 5
		JMP PC+1				; 3
		NOP						; 1
	.ELSE .IF clk_delay_ms1 < 4
		JMP PC+1				; 3
	.ELSE .IF clk_delay_ms1 < 3
		RJMP PC+1				; 2
	.ELSE .IF clk_delay_ms1 < 2
		NOP						; 1
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
	.ENDIF
POP XH
POP XL
POP R16
delay_ms_1_end:
RET

 ; delay_us us		max 16bit
.macro delay_us ; 
PUSH R16
	LDI R16, (@0)
	delay_us_loop:
		CALL delay_us_1
		DEC R16
	BRNE delay_us_loop
POP R16
.endmacro

 ; delay_ms ms		max 16bit
.macro delay_ms
PUSH R16
	LDI R16, (@0)
	delay_ms_loop:
		CALL delay_ms_1
		DEC R16
	BRNE delay_ms_loop
POP R16
.endmacro

 ; delay_s s		max 8bit 
 .macro delay_s
	.set loop_count_s = (@0)
	.if (loop_count_s <= 255)
		push ZL										; save ZL
		ldi ZL, LOW(loop_count_s)
			DELAY_S_LOOP:
				delay_ms 1000
				dec ZL
				brne DELAY_S_LOOP
		pop ZL										; restore ZL
	.endif
 .endmacro

.endif ;delay_include