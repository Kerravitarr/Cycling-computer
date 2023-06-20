/*
 * math.inc
 *
 *  Created: 03.12.2018 15:51:43
 *   Author: Terran
 */ 
 /**
 Программы:
//Загрузка числа в 32-х разрядный регистр. Два параметра - адрес ячейки и число  @0=@1
.MACRO		LDI_32

//Загрузка числа в аккумулятор A = @0
.MACRO		_LDI_A  

//Очищает 4-х байтное число. Один параметр - адрес @0=0
.MACRO		_CLR_32

//Очищает 64-х разрядное число. Один параметр - адрес @0=0
.MACRO		_CLR_64

//Прибавляет к 32х разрядному числу единицу. Один параметр - адрес @0++
.MACRO		_INC_32

//Логический сдвиг вправо.
//Первый параметр - адрес.
//Второй параметр - на сколько двигать.
//@0 >> @1 
.MACRO		_LSR_32

//Логический сдвиг влево.
//Первый параметр - адрес.
//Второй параметр - на сколько двигать.
//@0 << @1 
.MACRO		_LSL_32

//Перемещает 16 битные числа
.MACRO		MOV_16

//Перемещает из одной ячейки памяти в другую 32х разрядные числа. @0=@1
.MACRO		_MOV_32

//Перемещает из одной ячейки памяти в другую 64х разрядные числа. @0=@1
.MACRO		_MOV_64

//Расширяет число со знаком из 16 разрядов в 32
//@0 указание на старшую часть 32 разрядного чилса.
//@0 = int_32t(@0)
.MACRO EXP_32_F_16

//Расширяет число со знаком из 32 разрядов в 64
//@0 указание на старшую часть 64 разрядного чилса.
//@0 = int_64t(@0)
.MACRO EXP_64_F_32

//Сравнивает два 4х разрядных числа. CP @0, @1
.MACRO		_CP_32

//Вычитает из одной ячейки памяти в другую 32х разрядные числа. @0-=@1
.MACRO		_SUB_32

//Инвертирует число
//@0 = -@0
.MACRO _NEG_32

//Тестирует число @0. Устанавливает флаг +- и 0
.MACRO		_TST_32

//Вычисляет СRC8. Macro = CRC8(Data: @0, Lenght: @1)
.MACRO		_CRC8

 */
/**
Копирует число в памяти из Yh:Yl в Xh:Xl 
Использует Counter как счётчик байт
*/
MOV_M:
	MOV_CYC_00:
		LD	MacroR,Y+
		ST	X+, MacroR
		DEC	Counter
	BRNE MOV_CYC_00
 RET

//Очищает все регистры. Адрес в памяти в Х, количество регистров в Counter
 CLR_M:
	CLR		MacroR
	CLRA64_00:
		ST	X+, MacroR
		DEC	Counter
	BRNE CLRA64_00
 RET
//Очищает все регистры. Адрес в памяти в Х, количество регистров в Counter
 SER_M:
	SER		MacroR
	 SERA64_00:
		ST	X+, MacroR
		DEC	Counter
	BRNE  SERA64_00
 RET

//Прибавляет к числу разрядом Counter
//Xh:Xl - число
//Counter - количество в числе байт
 INC_M:
	PUSH	tmp3
	CLR		tmp3
	//Так как у нас в памяти числа идут от старшего разряда к младшему...
	ADD Xl, Counter
	ADC	Xh,tmp3
	//Прибавляемая единица
	SEC
	INC_32_M_00:
		LD	MacroR,-X
		//А вот тут мы складываем с 0 и с С. На первом шаге С будет, на последнем... Как знать
		ADC	MacroR,tmp3
		ST	X, MacroR
		DEC	Counter
	BRNE INC_32_M_00
	POP		tmp3
 RET

 //Складывает два числа
 //Yh:Yl += Xh:Xl 
 //Указатели на ячейку следующую за младшим разрядом!!!!
 //MacroR - длина числа
 //Использует 
 ADD_M:
	PUSH	tmp3
	PUSH	tmp2
		CLR		tmp3
		CLC
		
		ADD_M_00:
			LD	tmp2,-Y
			LD	tmp3,-X
			ADC	tmp2,tmp3
			ST	Y,tmp2
			DEC	MacroR
		BRNE ADD_M_00

	POP		tmp2
	POP		tmp3
 RET	

 //Вычитает два числа
 //Yh:Yl += Xh:Xl 
 //MacroR - длина числа
 //Указатели на ячейку следующую за младшим разрядом!!!!
 //Использует 
SUB_M:
	PUSH	tmp3
	PUSH	tmp2
		CLR		tmp3
		CLC
		SUB_M_00:
			LD	tmp2,-Y
			LD	tmp3,-X
			SBC	tmp2,tmp3
			ST	Y,tmp2
			DEC	MacroR
		BRNE SUB_M_00

	POP		tmp2
	POP		tmp3
 RET	

 //Сдвигает огромные числа вправо на 1 бит
//Xl:Xh - где число лежит. Указатель на старший разряд!
//MacroR - сколько в нём байт
 LSR_M:
	PUSH	tmp2
	CLC //Мы в начало цифры добавляем 0. Мы же сдвигаем в право
	LD tmp2, X
	SBRC tmp2, 7 //Если тут не 0, то мы устанавливаем флаг переноса! 
		SEC	//Устанавливаем старший разряд в 1, так как число отрицательное!
	LSR_M_l:
		LD	tmp2,X
		ROR tmp2
		ST X+, tmp2
		DEC MacroR
	BRNE /*!=*/ LSR_M_l
	POP		tmp2
 RET

 //Сдвигает огромные числа влево на 1 бит
//Xl:Xh - где число лежит. Указатель на МЛАДШИЙ разряд!
//MacroR - сколько в нём байт
 LSL_M:
	PUSH	tmp2
	ADIW X, 1 //Ну нет функции с постдекрементом! Притеснение программистов!!!
	CLC
	LSL_M_l:
		LD	tmp2,-X
		ROL tmp2
		ST	X, tmp2
		DEC MacroR
	BRNE /*!=*/ LSL_M_l
	POP		tmp2
 RET

//Переводит число в дополнительный код.
//Xh:Xl - число, указатель на его старшый разряд
//MacroR - количество в числе байт
NEG_M:
	PUSH tmp2
	PUSH tmp3
	PUSH Counter

	MOV Counter, MacroR
	SER tmp2
	NEG_Ml:
		LD tmp3, X
		EOR tmp3, tmp2 //Инвертируем все биты
		ST X+, tmp3
		DEC MacroR
	BRNE /*!=*/ NEG_Ml
	//Теперь надо сдвинуть назад память
	//CLR MacroR - как удачно он стал 0!
	SUB Xl,Counter
	SBC Xh,MacroR	//Сдвигаем в конец повествования
	//И готово - + шаг!
	RCALL INC_M

	POP Counter
	POP tmp3
	POP tmp2
RET

//Тестирует число
//MacroR - количество байт
//Х - указатель на СТАРШИЙ разряд
TST_M:
	PUSH tmp2

	LD tmp2, X+
	DEC MacroR
	TST tmp2
	BREQ /*=*/ TST_M1
		//У нас страший разряд не 0. Случай чисто на изи
		ANDI tmp2, 1 << 7
		BREQ /*=*/TST_Mpositiv 
			//Старший разряд 1, число отрицательное
			CLZ //Сброс фалага нуля
			SEN //Флаг отрицательного
			SES //Тоже знак...
			RJMP TST_Mend
		TST_Mpositiv:
			//Старший разряд 0, но какие-то биты тут есть
			CLZ //Сброс фалага нуля
			CLN //Сброс флага отрицательного
			CLS //Тоже знак...
			RJMP TST_Mend
	TST_M1:
		LD tmp2, X+
		TST tmp2
		BREQ /*=*/ TST_M_next
			//Тут байт есть!!!
			RJMP TST_Mpositiv //НЕЛЬЗЯ ТАК ДЕЛАТЬ!!!!!
		TST_M_next:
		DEC MacroR
	BRNE /*!=*/ TST_M1
	//Ну всё проверили, ни чего нет!
	SEZ //А у нас-то 0!
	CLN //Сброс флага отрицательного
	CLS //Тоже знак...

	TST_Mend:
	POP tmp2
RET

//Функция вычисляет CRC
//Х - указатель на начало текста
//MacroR - количество байт
//Выход: MacroR - рассчитанное CRC
CRC_8_M:
	//Инициализация
	.equ CRC_INIT = 0x00
	//Полином: 0x31    x^8 + x^5 + x^4 + 1
	.equ CRC_POLY = 0x31
	
	PUSH tmp2
	PUSH tmp3
	PUSH tmp4
	PUSHW X
		//Это наш СРС
		LDI tmp2, CRC_INIT
		//Это наш полином
		LDI tmp4, CRC_POLY
		CRC_8_M_dataL:
			LD tmp3, X+
			EOR tmp2, tmp3 //crc ^= *DATA++;
			LDI tmp3, 8

			CRC_8_M_byteL:
				LSL tmp2 //crc << 1 и старший бит, то есть crc & 0x80, в С
				BRCC /*C == 0*/ CRC_8_M_crcNotPoli
					//C == 1 => crc & 0x80 == true
					EOR tmp2, tmp4 //(crc << 1) ^ CRC_POLY
				CRC_8_M_crcNotPoli:
				DEC tmp3
			BRNE /*!=*/ CRC_8_M_byteL

			DEC MacroR
		BRNE /*!=*/ CRC_8_M_dataL
		MOV MacroR, tmp2
	POPW X
	POP tmp4
	POP tmp3
	POP tmp2

RET


;=============================================================================
;=============================================================================
;=============================================================================
;=============================================================================

//Загрузка числа в 32-х разрядный регистр. Два параметра - адрес ячейки и число  @0=@1
.MACRO		_LDI_32
	//Очищаем
	LDI		Counter,4
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	CALL	CLR_M
	.IF	@1 < 0
		LDI	MacroR,low(@1)
		STS	@0+3,MacroR	
		LDI	MacroR,high(@1)
		STS	@0+2,MacroR
		.IF	(-@1) <= 0xFFFF
			LDI	MacroR,0xFF
			STS @0+1,MacroR	
			LDI	MacroR,0xFF
			STS @0+0,MacroR
		.ELSE
			LDI	MacroR,low(@1 / 0xFFFF)
			STS	@0+1,MacroR	
			LDI	MacroR,high(@1 / 0xFFFF)
			STS	@0+0,MacroR	
		.endif
	.ELSE
   		.IF	@1 <= 0xFF
			LDI	MacroR,@1
			STS	@0+3,MacroR
		.ELSE
   			.IF	@1 <= 0xFFFF
				LDI	MacroR,low(@1)
				STS	@0+3,MacroR	
				LDI	MacroR,high(@1)
				STS	@0+2,MacroR
			.ELSE
				.IF	@1 <= 0xFFFFFF
					LDI	MacroR,low(@1)
					STS	@0+3,MacroR	
					LDI	MacroR,high(@1)
					STS	@0+2,MacroR
					LDI	MacroR,low(@1 / 0xFFFF)
					STS	@0+1,MacroR	
				.ELSE
   					.IF	@1 <= 0xFFFFFFFF
						LDI	MacroR,low(@1)
						STS	@0+3,MacroR	
						LDI	MacroR,high(@1)
						STS	@0+2,MacroR
						LDI	MacroR,low(@1 / 0xFFFF)
						STS	@0+1,MacroR	
						LDI	MacroR,high(@1 / 0xFFFF)
						STS	@0+0,MacroR	
					.ELSE
      					.error "Аругмент не может быть больше 4ых байт!"
   					.endif
   				.endif
   			.endif
   		.endif
   	.endif
.ENDMACRO
.MACRO		LDI_32  
	PUSH	XL     
	PUSH	XH  
	PUSH	Counter

	_LDI_32 @0,@1

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
   			.IF	@0 <= 0xFFFFFF
				LDI	MacroR,low(@0)
				STS	ACC_1+7,MacroR	
				LDI	MacroR,high(@0)
				STS	ACC_1+6,MacroR
				LDI	MacroR,low(@0 / 0xFFFF)
				STS	ACC_1+5,MacroR	
			.ELSE
   				.IF	@0 <= 0xFFFFFFFF
					LDI	MacroR,low(@0)
					STS	ACC_1+7,MacroR	
					LDI	MacroR,high(@0)
					STS	ACC_1+6,MacroR
					LDI	MacroR,low(@0 / 0xFFFF)
					STS	ACC_1+5,MacroR	
					LDI	MacroR,high(@0 / 0xFFFF)
					STS	ACC_1+4,MacroR	
				.ELSE
      				.error "Аругмент не может быть больше 4ых байт!"
   				.endif
   			.endif
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
	CALL	CLR_M
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
.MACRO		_CLR_64
	LDI		Counter,8
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	CALL	CLR_M
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

//Усиаеавоиваеи все биты числа. Один параметр - адрес @0=0xFFFFF....
.MACRO		_SER_32
	LDI		Counter,4
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	CALL	SER_M
.ENDMACRO
.MACRO		SER_32
	PUSH	XL     
	PUSH	XH  
	PUSH	Counter

	_SER_32 @0

	POP		Counter
	POP		XH
	POP		XL
.ENDMACRO
.MACRO		_SER_64
	LDI		Counter,8
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	CALL	SER_M
.ENDMACRO
.MACRO		SER_64
	PUSH	XL     
	PUSH	XH  
	PUSH	Counter

	_SER_64 @0

	POP		Counter
	POP		XH
	POP		XL
.ENDMACRO

//Прибавляет к 32х разрядному числу единицу. Один параметр - адрес @0++
.MACRO		_INC_32

	LDI		Counter,4
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	CALL	INC_M

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

//Логический сдвиг вправо.
//Первый параметр - адрес.
//Второй параметр - на сколько двигать.
//@0 >> @1 
.MACRO		_LSR_32
	.IF @1 < 1 
		.error "Param @1 must be greater than 0"
	.endif
	.IF @1 == 1
		LDI		MacroR,4
   		LDI 	XL,low(@0)
		LDI 	XH,high(@0)
		RCALL	LSR_M
	.else
		LDI		Counter, @1
		loop:
			LDI		MacroR,4
   			LDI 	XL,low(@0)
			LDI 	XH,high(@0)
			CALL	LSR_M
			DEC Counter
		BRNE /*!=*/ loop
	.endif
.ENDMACRO
.MACRO		LSR_32
	PUSH	XL     
	PUSH	XH 
	
	.IF @1 == 1
		_LSR_32 @0
	.else
		PUSH	Counter 
		_LSR_32 @0
		POP		Counter
	.endif
	
	POP		XH
	POP		XL
.ENDMACRO
.MACRO		_LSR_64
	.IF @1 < 1 
		.error "Param @1 must be greater than 0"
	.endif
	.IF @1 == 1
		LDI		MacroR,8
   		LDI 	XL,low(@0)
		LDI 	XH,high(@0)
		RCALL	LSR_M
	.else
		LDI		Counter, @1
		loop:
			LDI		MacroR,8
   			LDI 	XL,low(@0)
			LDI 	XH,high(@0)
			CALL	LSR_M
			DEC Counter
		BRNE /*!=*/ loop
	.endif
.ENDMACRO
.MACRO		LSR_64
	PUSH	XL     
	PUSH	XH 
	
	.IF @1 == 1
		_LSR_64 @0
	.else
		PUSH	Counter 
		_LSR_64 @0
		POP		Counter
	.endif
	
	POP		XH
	POP		XL
.ENDMACRO



//Логический сдвиг влево.
//Первый параметр - адрес.
//Второй параметр - на сколько двигать.
//@0 << @1 
.MACRO		_LSL_32
	.IF @1 < 1 
		.error "Param @1 must be greater than 0"
	.endif
	.IF @1 == 1
		LDI		MacroR,4
		LDIW X, @0+3
		RCALL	LSL_M
	.else
		LDI		Counter, @1
		loop:
			LDI		MacroR,4
			LDIW X, @0+3
			CALL	LSL_M
			DEC Counter
		BRNE /*!=*/ loop
	.endif
.ENDMACRO
.MACRO		LSL_32
	PUSH	XL     
	PUSH	XH 
	
	.IF @1 == 1
		_LSL_32 @0
	.else
		PUSH	Counter 
		_LSL_32 @0
		POP		Counter
	.endif
	
	POP		XH
	POP		XL
.ENDMACRO
.MACRO		_LSL_64
	.IF @1 < 1 
		.error "Param @1 must be greater than 0"
	.endif
	.IF @1 == 1
		LDI		MacroR,8
		LDIW	X, @0+7
		RCALL	LSL_M
	.else
		LDI		Counter, @1
		loop:
			LDI		MacroR,8
			LDIW	X, @0+7
			CALL	LSL_M
			DEC Counter
		BRNE /*!=*/ loop
	.endif
.ENDMACRO
.MACRO		LSL_64
	PUSH	XL     
	PUSH	XH 
	
	.IF @1 == 1
		_LSL_64 @0
	.else
		PUSH	Counter 
		_LSL_64 @0
		POP		Counter
	.endif
	
	POP		XH
	POP		XL
.ENDMACRO

//Перемещает 16 битные числа
.MACRO		MOV_16
	LDS MacroR, @1
	STS @0, MacroR
	LDS MacroR, @1 + 1
	STS @0+1, MacroR
.ENDMACRO

//Перемещает из одной ячейки памяти в другую 32х разрядные числа. @0=@1
.MACRO		_MOV_32
	LDI		Counter,4
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
   	LDI 	YL,low(@1)
	LDI 	YH,high(@1)
	CALL	MOV_M
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
	CALL	MOV_M
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

//Расширяет число со знаком из 16 разрядов в 32
//@0 указание на старшую часть 32 разрядного чилса.
//@0 = int_32t(@0)
.MACRO EXP_32_F_16
	LDS MacroR, @0 + 2
	ANDI MacroR, 1 << 7
	BREQ /*=*/ pos
		SER  MacroR
		RJMP end
	pos:
		CLR  MacroR
	end:
	STS @0 + 1, MacroR
	STS @0 + 0, MacroR
.ENDMACRO
//Расширяет число со знаком из 32 разрядов в 64
//@0 указание на старшую часть 64 разрядного чилса.
//@0 = int_64t(@0)
.MACRO EXP_64_F_32
	LDS MacroR, @0 + 4
	ANDI MacroR, 1 << 7
	BREQ /*=*/ pos
		SER  MacroR
		RJMP end
	pos:
		CLR  MacroR
	end:
		STS @0 + 3, MacroR
		STS @0 + 2, MacroR
		STS @0 + 1, MacroR
		STS @0 + 0, MacroR
.ENDMACRO

//Сравнивает два 4х разрядных числа. CP @0, @1
.MACRO		_CP_32
	LDS	MacroR, @0 + 3
	LDS	tmp2,	@1 + 3
	CP MacroR, tmp2
	LDS	MacroR, @0 + 2
	LDS	tmp2,	@1 + 2
	CPC MacroR, tmp2
	LDS	MacroR, @0 + 1
	LDS	tmp2,	@1 + 1
	CPC MacroR, tmp2
	LDS	MacroR, @0 + 0
	LDS	tmp2,	@1 + 0
	CPC MacroR, tmp2
.ENDMACRO
.MACRO		CP_32
	PUSH	tmp2

	_CP_32 @0,@1

	POP		tmp2
.ENDMACRO

//Вычитает из одной ячейки памяти в другую 32х разрядные числа. @0-=@1
.MACRO		_SUB_32
	LDI		MacroR,4
	LDIW	Y, @0 + 4
	LDIW	X, @1 + 4
	CALL	SUB_M
.ENDMACRO
.MACRO		SUB_32
	PUSH	XL     
	PUSH	XH  
	PUSH	YL     
	PUSH	YH  

	_SUB_32 @0,@1

	POP		YH
	POP		YL
	POP		XH
	POP		XL
.ENDMACRO
.MACRO		_SUB_64
	LDI		MacroR,8
	LDIW	Y, @0 + 8
	LDIW	X, @1 + 8
	CALL	SUB_M
.ENDMACRO
.MACRO		SUB_64
	PUSH	XL     
	PUSH	XH  
	PUSH	YL     
	PUSH	YH  

	_SUB_64 @0,@1

	POP		YH
	POP		YL
	POP		XH
	POP		XL
.ENDMACRO

//Складывает две 32х разрядные ячейки. @0+=@1
.MACRO		_ADD_32
	LDI		MacroR,4
	LDIW	Y, @0 + 4
	LDIW	X, @1 + 4
	CALL	ADD_M
.ENDMACRO
.MACRO		ADD_32
	PUSH	XL     
	PUSH	XH  
	PUSH	YL     
	PUSH	YH  

	_ADD_32 @0,@1

	POP		YH
	POP		YL
	POP		XH
	POP		XL
.ENDMACRO
.MACRO		_ADD_64
	LDI		MacroR,8
	LDIW	Y, @0 + 8
	LDIW	X, @1 + 8
	CALL	ADD_M
.ENDMACRO
.MACRO		ADD_64
	PUSH	XL     
	PUSH	XH  
	PUSH	YL     
	PUSH	YH  

	_ADD_64 @0,@1

	POP		YH
	POP		YL
	POP		XH
	POP		XL
.ENDMACRO

//Инвертирует число
//@0 = -@0
.MACRO _NEG_32
	LDI		MacroR,4
	LDIW	X, @0
	CALL	NEG_M
.ENDMACRO

.MACRO NEG_32
	PUSH Xl
	PUSH Xh

	_NEG_32 @0

	POP Xh
	POP Xl
.ENDMACRO
.MACRO _NEG_64
	LDI		MacroR,8
	LDIW	X, @0
	CALL	NEG_M
.ENDMACRO

.MACRO NEG_64
	PUSH Xl
	PUSH Xh

	_NEG_64 @0

	POP Xh
	POP Xl
.ENDMACRO
//Тестирует число @0. Устанавливает флаг +- и 0
.MACRO		_TST_32
	LDI		Counter,4
   	LDIW 	X,@0
	CALL	TST_M
.ENDMACRO
.MACRO		TST_32
	PUSH	XL     
	PUSH	XH  
	PUSH	Counter

	_TST_32 @0

	POP		Counter
	POP		XH
	POP		XL
.ENDMACRO
//Тестирует число @0. Устанавливает флаг +- и 0
.MACRO		_TST_64
	LDI		Counter,8
   	LDIW 	X,@0
	CALL	TST_M
.ENDMACRO
.MACRO		TST_64
	PUSH	XL     
	PUSH	XH  
	PUSH	Counter

	_TST_64 @0

	POP		Counter
	POP		XH
	POP		XL
.ENDMACRO

.MACRO		_CRC8
	LDI		MacroR,@1
   	LDIW 	X,@0
	CALL	CRC_8_M
.ENDMACRO
.MACRO		CRC8
	PUSHW	X 

	_CRC8 @0, @1

	POPW	X
.ENDMACRO




//Деление 64х разрядного числа
.include "Math/DIV_64.inc"
//Умножение 64х разрядного числа
.include "Math/MUL_64.inc"
