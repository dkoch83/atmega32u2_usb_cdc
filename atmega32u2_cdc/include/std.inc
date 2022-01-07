;/*
;* std.inc
;*
;*  Created: 26.08.2014 21:46:03
;*   Author: Dominik
;*/ 

.ifndef std_include
.set std_include = 1

.SET ATMEGA	 = 0
.SET ATTINY	 = 0
.SET ATXMEGA = 1

.SET TRUE=1
.SET FALSE=0

.MACRO LDIX
	LDI XH, HIGH(@0)
	LDI XL, LOW(@0)
.ENDMACRO

.MACRO LDIY
	LDI YH, HIGH(@0)
	LDI YL, LOW(@0)
.ENDMACRO

.MACRO LDIZ
	LDI ZH, HIGH(@0)
	LDI ZL, LOW(@0)
.ENDMACRO

.MACRO PUSHX
	PUSH XH
	PUSH XL
.ENDMACRO

.MACRO POPX
	POP XL
	POP XH
.ENDMACRO

.MACRO PUSHY
	PUSH YH
	PUSH YL
.ENDMACRO

.MACRO POPY
	POP YL
	POP YH
.ENDMACRO

.MACRO PUSHZ
	PUSH ZH
	PUSH ZL
.ENDMACRO

.MACRO POPZ
	POP ZL
	POP ZH
.ENDMACRO

.MACRO PUSH_ ; @0 = FROM, @1 = TO
	.IF @0 <= 0
		.IF @1 >= 0	
			PUSH R0
		.ENDIF
	.ENDIF
	.IF @0 <= 1
		.IF @1 >= 1	
			PUSH R1
		.ENDIF
	.ENDIF
	.IF @0 <= 2
		.IF @1 >= 2	
			PUSH R2
		.ENDIF
	.ENDIF
	.IF @0 <= 3
		.IF @1 >= 3	
			PUSH R3
		.ENDIF
	.ENDIF
	.IF @0 <= 4
		.IF @1 >= 4	
			PUSH R4
		.ENDIF
	.ENDIF
	.IF @0 <= 5
		.IF @1 >= 5
			PUSH R5
		.ENDIF
	.ENDIF
	.IF @0 <= 6
		.IF @1 >= 6
			PUSH R6
		.ENDIF
	.ENDIF
	.IF @0 <= 7
		.IF @1 >= 7
			PUSH R7
		.ENDIF
	.ENDIF
	.IF @0 <= 8
		.IF @1 >= 8
			PUSH R8
		.ENDIF
	.ENDIF
	.IF @0 <= 9
		.IF @1 >= 9
			PUSH R9
		.ENDIF
	.ENDIF
	.IF @0 <= 10
		.IF @1 >= 10
			PUSH R10
		.ENDIF
	.ENDIF
	.IF @0 <= 11
		.IF @1 >= 11
			PUSH R11
		.ENDIF
	.ENDIF
	.IF @0 <= 12
		.IF @1 >= 12
			PUSH R12
		.ENDIF
	.ENDIF
	.IF @0 <= 13
		.IF @1 >= 13
			PUSH R13
		.ENDIF
	.ENDIF
	.IF @0 <= 14
		.IF @1 >= 14
			PUSH R14
		.ENDIF
	.ENDIF
	.IF @0 <= 15
		.IF @1 >= 15
			PUSH R15
		.ENDIF
	.ENDIF
	.IF @0 <= 16
		.IF @1 >= 16
			PUSH R16
		.ENDIF
	.ENDIF
	.IF @0 <= 17
		.IF @1 >= 17
			PUSH R17
		.ENDIF
	.ENDIF
	.IF @0 <= 18
		.IF @1 >= 18
			PUSH R18
		.ENDIF
	.ENDIF
	.IF @0 <= 19
		.IF @1 >= 19
			PUSH R19
		.ENDIF
	.ENDIF
	.IF @0 <= 20
		.IF @1 >= 20
			PUSH R20
		.ENDIF
	.ENDIF
	.IF @0 <= 21
		.IF @1 >= 21
			PUSH R21
		.ENDIF
	.ENDIF
	.IF @0 <= 22
		.IF @1 >= 22
			PUSH R22
		.ENDIF
	.ENDIF
	.IF @0 <= 23
		.IF @1 >= 23
			PUSH R23
		.ENDIF
	.ENDIF
	.IF @0 <= 24
		.IF @1 >= 24
			PUSH R24
		.ENDIF
	.ENDIF
	.IF @0 <= 25
		.IF @1 >= 25
			PUSH R25
		.ENDIF
	.ENDIF
	.IF @0 <= 26
		.IF @1 >= 26
			PUSH R26
		.ENDIF
	.ENDIF
	.IF @0 <= 27
		.IF @1 >= 27
			PUSH R27
		.ENDIF
	.ENDIF
	.IF @0 <= 28
		.IF @1 >= 28
			PUSH R28
		.ENDIF
	.ENDIF
	.IF @0 <= 29
		.IF @1 >= 29
			PUSH R29
		.ENDIF
	.ENDIF
	.IF @0 <= 30
		.IF @1 >= 30
			PUSH R30
		.ENDIF
	.ENDIF
	.IF @0 <= 31
		.IF @1 >= 31
			PUSH R31
		.ENDIF
	.ENDIF
.ENDMACRO


.MACRO POP_  ; @0 = FROM, @1 = TO
	.IF @0 <= 31
		.IF @1 >= 31
			POP R31
		.ENDIF
	.ENDIF
	.IF @0 <= 30
		.IF @1 >= 30
			POP R30
		.ENDIF
	.ENDIF
	.IF @0 <= 29
		.IF @1 >= 29
			POP R29
		.ENDIF
	.ENDIF
	.IF @0 <= 28
		.IF @1 >= 28
			POP R28
		.ENDIF
	.ENDIF
	.IF @0 <= 27
		.IF @1 >= 27
			POP R27
		.ENDIF
	.ENDIF
	.IF @0 <= 26
		.IF @1 >= 26
			POP R26
		.ENDIF
	.ENDIF
	.IF @0 <= 25
		.IF @1 >= 25
			POP R25
		.ENDIF
	.ENDIF
	.IF @0 <= 24
		.IF @1 >= 24
			POP R24
		.ENDIF
	.ENDIF
	.IF @0 <= 23
		.IF @1 >= 23
			POP R23
		.ENDIF
	.ENDIF
	.IF @0 <= 22
		.IF @1 >= 22
			POP R22
		.ENDIF
	.ENDIF
	.IF @0 <= 21
		.IF @1 >= 21
			POP R21
		.ENDIF
	.ENDIF
	.IF @0 <= 20
		.IF @1 >= 20
			POP R20
		.ENDIF
	.ENDIF
	.IF @0 <= 19
		.IF @1 >= 19
			POP R19
		.ENDIF
	.ENDIF
	.IF @0 <= 18
		.IF @1 >= 18
			POP R18
		.ENDIF
	.ENDIF
	.IF @0 <= 17
		.IF @1 >= 17
			POP R17
		.ENDIF
	.ENDIF
	.IF @0 <= 16
		.IF @1 >= 16
			POP R16
		.ENDIF
	.ENDIF
	.IF @0 <= 15
		.IF @1 >= 15
			POP R15
		.ENDIF
	.ENDIF
	.IF @0 <= 14
		.IF @1 >= 14
			POP R14
		.ENDIF
	.ENDIF
	.IF @0 <= 13
		.IF @1 >= 13
			POP R13
		.ENDIF
	.ENDIF
	.IF @0 <= 12
		.IF @1 >= 12
			POP R12
		.ENDIF
	.ENDIF
	.IF @0 <= 11
		.IF @1 >= 11
			POP R11
		.ENDIF
	.ENDIF
	.IF @0 <= 10
		.IF @1 >= 10
			POP R10
		.ENDIF
	.ENDIF
	.IF @0 <= 9
		.IF @1 >= 9
			POP R9
		.ENDIF
	.ENDIF
	.IF @0 <= 8
		.IF @1 >= 8
			POP R8
		.ENDIF
	.ENDIF
	.IF @0 <= 7
		.IF @1 >= 7
			POP R7
		.ENDIF
	.ENDIF
	.IF @0 <= 6
		.IF @1 >= 6
			POP R6
		.ENDIF
	.ENDIF
	.IF @0 <= 5
		.IF @1 >= 5
			POP R5
		.ENDIF
	.ENDIF
	.IF @0 <= 4
		.IF @1 >= 4
			POP R4
		.ENDIF
	.ENDIF
	.IF @0 <= 3
		.IF @1 >= 3
			POP R3
		.ENDIF
	.ENDIF
	.IF @0 <= 2
		.IF @1 >= 2
			POP R2
		.ENDIF
	.ENDIF
	.IF @0 <= 1
		.IF @1 >= 1
			POP R1
		.ENDIF
	.ENDIF
	.IF @0 <= 0
		.IF @1 >= 0
			POP R0
		.ENDIF
	.ENDIF
.ENDMACRO

.MACRO CLR_ ; @0 = FROM, @1 = TO
	.IF @0 <= 0
		.IF @1 >= 0	
			CLR R0
		.ENDIF
	.ENDIF
	.IF @0 <= 1
		.IF @1 >= 1	
			CLR R1
		.ENDIF
	.ENDIF
	.IF @0 <= 2
		.IF @1 >= 2	
			CLR R2
		.ENDIF
	.ENDIF
	.IF @0 <= 3
		.IF @1 >= 3	
			CLR R3
		.ENDIF
	.ENDIF
	.IF @0 <= 4
		.IF @1 >= 4	
			CLR R4
		.ENDIF
	.ENDIF
	.IF @0 <= 5
		.IF @1 >= 5
			CLR R5
		.ENDIF
	.ENDIF
	.IF @0 <= 6
		.IF @1 >= 6
			CLR R6
		.ENDIF
	.ENDIF
	.IF @0 <= 7
		.IF @1 >= 7
			CLR R7
		.ENDIF
	.ENDIF
	.IF @0 <= 8
		.IF @1 >= 8
			CLR R8
		.ENDIF
	.ENDIF
	.IF @0 <= 9
		.IF @1 >= 9
			CLR R9
		.ENDIF
	.ENDIF
	.IF @0 <= 10
		.IF @1 >= 10
			CLR R10
		.ENDIF
	.ENDIF
	.IF @0 <= 11
		.IF @1 >= 11
			CLR R11
		.ENDIF
	.ENDIF
	.IF @0 <= 12
		.IF @1 >= 12
			CLR R12
		.ENDIF
	.ENDIF
	.IF @0 <= 13
		.IF @1 >= 13
			CLR R13
		.ENDIF
	.ENDIF
	.IF @0 <= 14
		.IF @1 >= 14
			CLR R14
		.ENDIF
	.ENDIF
	.IF @0 <= 15
		.IF @1 >= 15
			CLR R15
		.ENDIF
	.ENDIF
	.IF @0 <= 16
		.IF @1 >= 16
			CLR R16
		.ENDIF
	.ENDIF
	.IF @0 <= 17
		.IF @1 >= 17
			CLR R17
		.ENDIF
	.ENDIF
	.IF @0 <= 18
		.IF @1 >= 18
			CLR R18
		.ENDIF
	.ENDIF
	.IF @0 <= 19
		.IF @1 >= 19
			CLR R19
		.ENDIF
	.ENDIF
	.IF @0 <= 20
		.IF @1 >= 20
			CLR R20
		.ENDIF
	.ENDIF
	.IF @0 <= 21
		.IF @1 >= 21
			CLR R21
		.ENDIF
	.ENDIF
	.IF @0 <= 22
		.IF @1 >= 22
			CLR R22
		.ENDIF
	.ENDIF
	.IF @0 <= 23
		.IF @1 >= 23
			CLR R23
		.ENDIF
	.ENDIF
	.IF @0 <= 24
		.IF @1 >= 24
			CLR R24
		.ENDIF
	.ENDIF
	.IF @0 <= 25
		.IF @1 >= 25
			CLR R25
		.ENDIF
	.ENDIF
	.IF @0 <= 26
		.IF @1 >= 26
			CLR R26
		.ENDIF
	.ENDIF
	.IF @0 <= 27
		.IF @1 >= 27
			CLR R27
		.ENDIF
	.ENDIF
	.IF @0 <= 28
		.IF @1 >= 28
			CLR R28
		.ENDIF
	.ENDIF
	.IF @0 <= 29
		.IF @1 >= 29
			CLR R29
		.ENDIF
	.ENDIF
	.IF @0 <= 30
		.IF @1 >= 30
			CLR R30
		.ENDIF
	.ENDIF
	.IF @0 <= 31
		.IF @1 >= 31
			CLR R31
		.ENDIF
	.ENDIF
.ENDMACRO

.endif

