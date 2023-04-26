/*
 * math.inc
 *
 *  Created: 03.12.2018 15:51:43
 *   Author: Terran
 */ 
;=============================================================================
;=============================================================================
;=============================================================================
;=============================================================================
/**
Копирует число в памяти из Yh:Yl в Xh:Xl 
*/
MOV_M:
	MOV_CYC_00:
		LD	MacroR,Y+
		ST	X+, MacroR
		DEC	Counter
	BRNE MOV_CYC_00
 RET
;=============================================================================
;=============================================================================
;=============================================================================
;=============================================================================
//Очищает все регистры. Адрес в памяти в Х, количество регистров в Counter
 CLR_M:
	CLR		MacroR
	CLRA64_00:
		ST	X+, MacroR
		DEC	Counter
	BRNE CLRA64_00
 RET

COM_M:
	COM_32_00:
		LD	MacroR,X
		COM MacroR
		ST	X+, MacroR
		DEC	Counter
	BRNE COM_32_00
 RET

 //Прибавляет к числу разрядом Counter 1
 INC_M:
	PUSH	tmp3
	CLR		tmp3
	//Прибавляемая единица
	SEC
	INC_32_M_00:
		LD	MacroR,X
		//А вот тут мы складываем с 0 и с С. На первом шаге С будет, на последнем... Как знать
		ADC	tmp2,tmp3
		ST	X+, MacroR
		DEC	Counter
	BRNE INC_32_M_00
	POP		tmp3
 RET
;=============================================================================
;=============================================================================
;=============================================================================
;=============================================================================
 LPM_Z_To_X_Counter:
 		LPM_Z_To_X_Counter_00:
			LPM		tmp2,Z+
			ST		X+,tmp2
			DEC	Counter
		BRNE LPM_Z_To_X_Counter_00
 RET
;=============================================================================
;=============================================================================
;=============================================================================
;=============================================================================

//Загрузка двубайтного числа в 4 бита. Два параметра - адрес ячейки и число  @0=@1
.MACRO		LDI_16
	PUSH	Counter

	LDI		Counter,4
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	RCALL	CLR_M
	 
	LDI	MacroR,low(@1)
	STS	@0+3,MacroR	
	LDI	MacroR,high(@1)
	STS	@0+2,MacroR

	POP		Counter
.ENDMACRO

//Загрузка однобайтного числа в 4 бита. Два параметра - адрес ячейки и число @0=@1
.MACRO		LDI_8
	PUSH	XL     
	PUSH	XH  
	PUSH	Counter

	LDI		Counter,4
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	RCALL	CLR_M

	LDI	MacroR,@1
	STS	@0+3,MacroR

	POP		Counter
	POP		XH
	POP		XL
.ENDMACRO

//Загрузка числа в аккумулятор A = @0
.MACRO		_LDI_A  
	//Очищаем аккумулятор
	LDI		Counter,8
   	LDI 	XL,low(ACC_1)
	LDI 	XH,high(ACC_1)
	CALL	CLR_M
   	.IF	@0 <= 0xFF
		LDI	MacroR,@0
		STS	ACC_1+7,MacroR
	.ELSE
   		.IF	@0 <= 0xFFFF
			LDI	MacroR,low(@0)
			STS	ACC_1+7,MacroR	
			LDI	MacroR,high(@0)
			STS	ACC_1+6,MacroR
		.ELSE
      		.error "Аругмент не может быть больше 2х байт!"
   		.endif
   	.endif
.ENDMACRO
//Загрузка числа в аккумулятор A = @0
.MACRO		LDI_A  
	PUSH	XL     
	PUSH	XH  
	PUSH	Counter

	_LDI_A @0

	POP		Counter
	POP		XH
	POP		XL
.ENDMACRO

//Очищает 4-х байтное число. Один параметр - адрес @0=0
.MACRO		_CLR_32
	LDI		Counter,4
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	RCALL	CLR_M
.ENDMACRO
.MACRO		CLR_32
	PUSH	XL     
	PUSH	XH  
	PUSH	Counter

	_CLR_32 @0

	POP		Counter
	POP		XH
	POP		XL
.ENDMACRO
//Очищает 64-х разрядное число. Один параметр - адрес @0=0
.MACRO		_CLR_64
	LDI		Counter,8
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	RCALL	CLR_M
.ENDMACRO
.MACRO		CLR_64
	PUSH	XL     
	PUSH	XH  
	PUSH	Counter

	_CLR_64 @0

	POP		Counter
	POP		XH
	POP		XL
.ENDMACRO

//Прибавляет к 32х разрядному числу единицу. Один параметр - адрес @0++
.MACRO		_INC_32

	LDI		Counter,4
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	RCALL	INC_M

.ENDMACRO
.MACRO		INC_32
	PUSH	XL     
	PUSH	XH  
	PUSH	Counter

	_INC_32 @0

	POP		Counter
	POP		XH
	POP		XL
.ENDMACRO

//Прибавляет к 32х разрядному числу единицу. Один параметр - адрес @0++

//Перемещает из одной ячейки памяти в другую 32х разрядные числа. @0=@1
.MACRO		_MOV_32
	LDI		Counter,4
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
   	LDI 	YL,low(@1)
	LDI 	YH,high(@1)
	RCALL	MOV_M
.ENDMACRO
.MACRO		MOV_32
	PUSH	XL     
	PUSH	XH  
	PUSH	YL     
	PUSH	YH  
	PUSH	Counter

	_MOV_32 @0,@1

	POP		Counter
	POP		YH
	POP		YL
	POP		XH
	POP		XL
.ENDMACRO
//Перемещает из одной ячейки памяти в другую 64х разрядные числа. @0=@1
.MACRO		_MOV_64
	LDI		Counter,8
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
   	LDI 	YL,low(@1)
	LDI 	YH,high(@1)
	RCALL	MOV_M
.ENDMACRO
.MACRO		MOV_64
	PUSH	XL     
	PUSH	XH  
	PUSH	YL     
	PUSH	YH  
	PUSH	Counter

	_MOV_64 @0,@1

	POP		Counter
	POP		YH
	POP		YL
	POP		XH
	POP		XL
.ENDMACRO

//Деление 64х разрядного числа
.include "Math/DIV_64.inc"
