/*
 * DIV_64.inc
 *
 Деление целочисленное! Беззнаковое!
 В зависимости от регистра MacroR выполняет разное деление:
 DIV_64_MODE_ACC_BY_32 : АСС(64 бит) = АСС(64 бит) / Xl(32)
 DIV_64_MODE_ACC_BY_64 : АСС(64 бит) = АСС(64 бит) / Xl(64)
 DIV_64_MODE_32_BY_ACC : АСС(64 бит) = Xl(32) / АСС(64 бит)
 DIV_64_MODE_64_BY_ACC : АСС(64 бит) = Xl(64) / АСС(64 бит)

 Если делитель = 0, то в ACC_1 будет 0xFFFFFFFF, а ACC_2 = 0

 Входные данные:
 MacroR - код операции
 Xh:Xl - адрес второго операнда

 Выходные данные:
 ACC_1 - частное
 ACC_2 - остаток

 *  Created: 24.04.2023 19:27:23
 *   Author: ????
 */ 
 .equ DIV_64_MODE_ACC_BY_32 = 0x0A
 .equ DIV_64_MODE_ACC_BY_64 = 0x0B
 .equ DIV_64_MODE_32_BY_ACC = 0x0C
 .equ DIV_64_MODE_64_BY_ACC = 0x0D

 //Спецфлаги, нужны для деления знаковых чисел
 .equ DIVS_FLAG_negA = 1 << 0
 .equ DIVS_FLAG_negR = 1 << 1

 //АСС(беззнак 64 бит) = АСС(беззнак 64 бит) / @0(беззнак 32)
 .MACRO		_DIV_64_A_DIV_R32
	LDI		MacroR,DIV_64_MODE_ACC_BY_32
	LDIW	X,@0
	CALL	DIV64_M
.ENDMACRO
.MACRO		DIV_64_A_DIV_R32
	PUSH	XL     
	PUSH	XH

	_DIV_64_A_DIV_R32 @0

	POP		XH
	POP		XL
.ENDMACRO

 //АСС(беззнак 64 бит) = АСС(беззнак 64 бит) / @0(беззнак 64)
 .MACRO		_DIV_64_A_DIV_R64
	LDI		MacroR,DIV_64_MODE_ACC_BY_64
	LDIW	X,@0
	RCALL	DIV64_M
.ENDMACRO
.MACRO		DIV_64_A_DIV_R64
	PUSH	XL     
	PUSH	XH

	_DIV_64_A_DIV_R64 @0

	POP		XH
	POP		XL
.ENDMACRO

 //АСС(беззнак 64 бит) = @0(беззнак 32) / АСС(беззнак 64 бит)
 .MACRO		_DIV_64_R32_DIV_A
	LDI		MacroR,DIV_64_MODE_32_BY_ACC
	LDIW	X,@0
	CALL	DIV64_M
.ENDMACRO
.MACRO		DIV_64_R32_DIV_A
	PUSH	XL     
	PUSH	XH

	_DIV_64_R32_DIV_A @0

	POP		XH
	POP		XL
.ENDMACRO

 //АСС(беззнак 64 бит) = @0(беззнак 64) / АСС(беззнак 64 бит)
 .MACRO		_DIV_64_R64_DIV_A
	LDI		MacroR,DIV_64_MODE_64_BY_ACC
	LDIW	X,@0
	CALL	DIV64_M
.ENDMACRO
.MACRO		DIV_64_R64_DIV_A
	PUSH	XL     
	PUSH	XH

	_DIV_64_R64_DIV_A @0

	POP		XH
	POP		XL
.ENDMACRO

 //АСС(знак 64 бит) = АСС(знак 64 бит) / @0(знак 32)
 .MACRO		_DIVS_64_A_DIV_R32
	LDI		MacroR,DIV_64_MODE_ACC_BY_32
	LDIW	X,@0
	CALL	DIVS_M
.ENDMACRO
.MACRO		DIVS_64_A_DIV_R32
	PUSH	XL     
	PUSH	XH

	_DIVS_64_A_DIV_R32 @0

	POP		XH
	POP		XL
.ENDMACRO

 //АСС(знак 64 бит) = АСС(знак 64 бит) / @0(знак 64)
 .MACRO		_DIVS_64_A_DIV_R64
	LDI		MacroR,DIV_64_MODE_ACC_BY_64
	LDIW	X,@0
	CALL	DIVS_M
.ENDMACRO
.MACRO		DIVS_64_A_DIV_R64
	PUSH	XL     
	PUSH	XH

	_DIVS_64_A_DIV_R64 @0

	POP		XH
	POP		XL
.ENDMACRO


DIVS_M:
	PUSH tmp2
	PUSH tmp3
	PUSH MacroR
	MOV tmp3, MacroR //Сохраняем флаг

	CLR tmp2
	LDS MacroR, ACC_1
	ANDI MacroR, 1 << 7
	BREQ /*=*/ DIVS_M_ACC_positive
		ORI tmp2, DIVS_FLAG_negA
		NEG_64 ACC_1
	DIVS_M_ACC_positive:

	LD MacroR, X
	ANDI MacroR, 1 << 7
	BREQ /*=*/ DIVS_M_R_positive
		ORI tmp2, DIVS_FLAG_negR
		PUSH Xl
		PUSH Xh
		
			CPI tmp3, DIV_64_MODE_ACC_BY_32
			BREQ /*=*/ DIVS_M_R32
			CPI tmp3, DIV_64_MODE_32_BY_ACC
			BREQ /*=*/ DIVS_M_R32
				LDI MacroR, 8
				CALL NEG_M
				RJMP DIVS_M_R_end
			DIVS_M_R32:
				LDI MacroR, 4
				CALL NEG_M
			DIVS_M_R_end:
			
		POP Xh
		POP Xl
	DIVS_M_R_positive:
	//Теперь у нас аккумулятор и число точно положительные, а в регистре tmp3 хранится знак числа
	POP MacroR //Достаём режим работы
	PUSH Xl
	PUSH Xh
	RCALL DIV64_M//И умножаем
	POP Xh
	POP Xl
	TST tmp2
	BREQ /*=*/ DIVS_M_end //Шик, ни чего делать и не нужно!
		//Как жаль, где-то негатив есть.
		CPI tmp2, DIVS_FLAG_negA | DIVS_FLAG_negR
		BREQ /*=*/ DIVS_M_neg2
			//У нас только одно число негативное
			//Значит результат тоже отрицательным будет :(
			NEG_64 ACC_1
			ANDI tmp2, DIVS_FLAG_negA
			BRNE /*!=*/ DIVS_M_end //Отрицательный - аккумулятор. Ни чего не меняем
				//Отрицательное число. Вот его и инвертнём дальше!
		DIVS_M_neg2:
			//У нас оба числа негативные. А - на - даст +!
			//Надо инвертнуть только число
			CPI tmp3, DIV_64_MODE_ACC_BY_32
			BREQ /*=*/ DIVS_M_R32
			CPI tmp3, DIV_64_MODE_32_BY_ACC
			BREQ /*=*/ DIVS_M_R32
				LDI MacroR, 8
				CALL NEG_M
				RJMP DIVS_M_end
			DIVS_M_R32_2:
				LDI MacroR, 4
				CALL NEG_M
	DIVS_M_end:
	POP tmp3
	POP tmp2
RET


 //Переносит из Yh:Yl в r0:r7. Причём в R0 - младший байт, в R7 - старший
 DIV64_M_Load:
	LD		R7,Y+
	LD		R6,Y+
	LD		R5,Y+
	LD		R4,Y+
	LD		R3,Y+
	LD		R2,Y+
	LD		R1,Y+
	LD		R0,Y
 RET
 //Переносит из r0:r7 в Yh:Yl. Причём в R0 - младший байт, в R7 - старший
 //Но сохраняет в обратном порядке!!!! Адрес надо указывать конца массива, а не начала!!!
 DIV64_M_Save:
	ST		Y, R0
	ST		-Y, R1
	ST		-Y, R2
	ST		-Y, R3
	ST		-Y, R4
	ST		-Y, R5
	ST		-Y, R6
	ST		-Y, R7
 RET


 //Делит два беззнаковых числа друг на друга
 //MacroR - режим, которым делим
 //X - указатель на один из делимых чисел
 DIV64_M:
	//Регистры доступа к памяти
	PUSH	YL
	PUSH	YH
	//Одно из чисел в регистрах
	PUSH	R0
	PUSH	R1
	PUSH	R2
	PUSH	R3
	PUSH	R4
	PUSH	R5
	PUSH	R6
	PUSH	R7
	//Счётчик
	PUSH	Counter
	PUSH	R8
	PUSH	R9
	PUSH	R10
	PUSH	R11
	PUSH	R12
	PUSH	R13
	PUSH	R14
	PUSH	R15

	MOV R15, MacroR

	//Засовываем делимое в ACC_2. Мы его будем делить, так что по итогу там останется остаток
	CPI		MacroR,DIV_64_MODE_ACC_BY_32
	BREQ /*=*/ DIV64_M_dividend_as_ACC
	CPI		MacroR,DIV_64_MODE_ACC_BY_64
	BREQ /*=*/ DIV64_M_dividend_as_ACC
		//Делимое - входной регистр
		CPI		MacroR,DIV_64_MODE_32_BY_ACC
		BREQ /*=*/ DIV64_M_dividend_as_32bit
			//Делимое - 64х разрядный
			LDI		Counter,8
			MOVW	YH:YL,XH:XL
   			LDI 	XL,low(ACC_2)
			LDI 	XH,high(ACC_2)
			CALL	MOV_M
			RJMP DIV64_M_dividend_as_32bit_end
		DIV64_M_dividend_as_32bit:
			//Делимое только 32х битное
			CLR_64	ACC_2
			LDI		Counter,4
			MOVW	YH:YL,XH:XL
   			LDI 	XL,low(ACC_2 + 4)
			LDI 	XH,high(ACC_2 + 4)
			CALL	MOV_M
			//Делимое - 32х разрядный
		DIV64_M_dividend_as_32bit_end:
		RJMP DIV64_M_dividend_end
	DIV64_M_dividend_as_ACC:
		//Делимое - аккумулятор
		//Тут главное не засрать XH:XL, а то у нас не будет адреса второго оператора!
		MOV_64 ACC_2, ACC_1
	DIV64_M_dividend_end:

	//Загружаем делимое
   	LDI 	YL,low(ACC_2)
	LDI 	YH,high(ACC_2)
	RCALL DIV64_M_Load

	//Считаем, сколько делимое имеет реально бит
	CLR Counter
	
	TST	R7
	BRNE DIV64_M_dividend_8b
	ADI Counter, 8	//Уже 8 старших можно сэкономить
	TST	R6
	BRNE DIV64_M_dividend_7b
	ADI Counter, 8	
	TST	R5
	BRNE DIV64_M_dividend_6b
	ADI Counter, 8	
	TST	R4
	BRNE DIV64_M_dividend_5b
	ADI Counter, 8	
	TST	R3
	BRNE DIV64_M_dividend_4b
	ADI Counter, 8	
	TST	R2
	BRNE DIV64_M_dividend_3b
	ADI Counter, 8	
	TST	R1
	BRNE DIV64_M_dividend_2b
	ADI Counter, 8	
	TST	R0
	BRNE DIV64_M_dividend_1b
	//Вот тут делимое = 0. А это что значит? Что мы закончили!. Результат = 0, остаток тоже 0
		_CLR_64	ACC_1
		_CLR_64	ACC_2
	RJMP DIV64_M_end

	DIV64_M_dividend_8b:
		MOV	R8,r7
		RJMP DIV64_M_dividend_bit_end
	DIV64_M_dividend_7b:
		MOV	R8,r6
		RJMP DIV64_M_dividend_bit_end
	DIV64_M_dividend_6b:
		MOV	R8,r5
		RJMP DIV64_M_dividend_bit_end
	DIV64_M_dividend_5b:
		MOV	R8,r4
		RJMP DIV64_M_dividend_bit_end
	DIV64_M_dividend_4b:
		MOV	R8,r3
		RJMP DIV64_M_dividend_bit_end
	DIV64_M_dividend_3b:
		MOV	R8,r2
		RJMP DIV64_M_dividend_bit_end
	DIV64_M_dividend_2b:
		MOV	R8,r1
		RJMP DIV64_M_dividend_bit_end
	DIV64_M_dividend_1b:
		MOV	R8,r0
		//RJMP DIV64_M_dividend_end
	DIV64_M_dividend_bit_end:
	//Запоминаем
	MOV R14, Counter
	
	//У нас, к сожалению, делимое не 0. Очень жаль.
	
	//Засовываем делитель в TMP_64. К сожалению, не придумал как обойтись без дополнительной памяти
	MOV MacroR, R15
	CPI		MacroR,DIV_64_MODE_32_BY_ACC
	BREQ /*=*/ DIV64_M_divider_as_ACC
	CPI		MacroR,DIV_64_MODE_64_BY_ACC
	BREQ /*=*/ DIV64_M_divider_as_ACC
		//Делитель - входной регистр
		CPI		MacroR,DIV_64_MODE_ACC_BY_32
		BREQ /*=*/ DIV64_M_divider_as_32bit
			//Делимое - 64х разрядный
			LDI		Counter,8
			MOVW	YH:YL,XH:XL
   			LDI 	XL,low(TMP_64)
			LDI 	XH,high(TMP_64)
			CALL	MOV_M
			RJMP DIV64_M_divider_as_32bit_end
		DIV64_M_divider_as_32bit:
			//Делимое только 32х битное
			CLR_64	TMP_64
			LDI		Counter,4
			MOVW	YH:YL,XH:XL
   			LDI 	XL,low(TMP_64 + 4)
			LDI 	XH,high(TMP_64 + 4)
			CALL	MOV_M
			//Делимое - 32х разрядный
		DIV64_M_divider_as_32bit_end:
		RJMP DIV64_M_divider_end
	DIV64_M_divider_as_ACC:
		//Делитель - аккумулятор
		MOV_64 TMP_64, ACC_1
	DIV64_M_divider_end:

	//Теперь посчитаем, сколько у нас там делитель занимает размера
   	LDI 	YL,low(TMP_64)
	LDI 	YH,high(TMP_64)
	RCALL DIV64_M_Load
	
	//Достаём
	MOV Counter, R14
	TST	R7
	BRNE DIV64_M_divider_8b
	SUBI Counter, 8	//Уже 8 есть, вдруг разница будет отрицательной? Тогда же можно не делить!
	TST	R6
	BRNE DIV64_M_divider_7b
	SUBI Counter, 8	
	TST	R5
	BRNE DIV64_M_divider_6b
	SUBI Counter, 8	
	TST	R4
	BRNE DIV64_M_divider_5b
	SUBI Counter, 8	
	TST	R3
	BRNE DIV64_M_divider_4b
	SUBI Counter, 8	
	TST	R2
	BRNE DIV64_M_divider_3b
	SUBI Counter, 8	
	TST	R1
	BRNE DIV64_M_divider_2b
	SUBI Counter, 8	
	TST	R0
	BRNE DIV64_M_divider_1b
	//Вот тут делитель = 0. А это что значит? Что мы закончили!. Результат = 0xFFFFFFFF, остаток 0
		LDI		Counter,8
   		LDI 	XL,low(ACC_1)
		LDI 	XH,high(ACC_1)
		SER		MacroR
		DIV64_M_CLRA64_00:
			ST	X+, MacroR
			DEC	Counter
		BRNE DIV64_M_CLRA64_00
		_CLR_64	ACC_2
	RJMP DIV64_M_end

	DIV64_M_divider_8b:
		MOV	R9,r7
		RJMP DIV64_M_divider_bit_end
	DIV64_M_divider_7b:
		MOV	R9,r6
		RJMP DIV64_M_divider_bit_end
	DIV64_M_divider_6b:
		MOV	R9,r5
		RJMP DIV64_M_divider_bit_end
	DIV64_M_divider_5b:
		MOV	R9,r4
		RJMP DIV64_M_divider_bit_end
	DIV64_M_divider_4b:
		MOV	R9,r3
		RJMP DIV64_M_divider_bit_end
	DIV64_M_divider_3b:
		MOV	R9,r2
		RJMP DIV64_M_divider_bit_end
	DIV64_M_divider_2b:
		MOV	R9,r1
		RJMP DIV64_M_divider_bit_end
	DIV64_M_divider_1b:
		MOV	R9,r0
		//RJMP DIV64_M_dividend_end
	DIV64_M_divider_bit_end:

	//Теперь подсчитаем, сколько правда разница в числах, в битах
	DIV64_M_LSR_dividend:
		CP	R8,R9
		BRLO /*<*/ DIV64_M_LSR_dividend_end
		BREQ /*=*/ DIV64_M_LSR_dividend_end
			LSR	R8
			DEC	Counter
		RJMP DIV64_M_LSR_dividend
	DIV64_M_LSR_dividend_end:
	//А теперь двигаем делитель. Если это вообще требуется
	//Мы знаем разницу в разрядах между делимым и делителем. А нам нужно, чтобы делитель голоовй упёрся в голову делимому. Поэтому и двигаем
	/*
	Дело в том, что сейчас у нас делимое уже в регистрах, так что грузить его ещё раз не нужно
   	LDI 	YL,low(TMP_64)
	LDI 	YH,high(TMP_64)
	RCALL DIV64_M_Load
	*/
	MOV	R8,Counter
	TST R8
	DIV64_M_LSR_dividend_bit:
		BRPL /*+*/DIV64_M_LSR_dividend_bit_end
			LSL	r0
			ROL r1
			ROL r2
			ROL r3
			ROL r4
			ROL r5
			ROL r6
			ROL r7
			INC	Counter
		RJMP DIV64_M_LSR_dividend_bit
	DIV64_M_LSR_dividend_bit_end:
	//Сохраняем новое делимое
	RCALL DIV64_M_Save
	//А теперь поехали делить! В столбик! Counter раз нужно поеделить и будет счастье.
	_CLR_64	ACC_1

	MOV	Counter,R8
	NEG Counter
	INC	Counter


	/*
	А теперь самое сложное - поделить...
	На первом шаге мы сдвигаем получившийся результат на один бит
	Ибо! Новый разряд надо куда-то писать

	Затем мы загружаем остаток от деления (На первом шаге это и будте всё делимое)
	И наш делитель, сдвинутый под голову остатка
	Ну и вычитаем.
	Если делитель больше - то ни чего не делаем
	А вот если делитель меньше, то есть разница оказалась положительной, запоминаем новый остаток и увеличиваем бит резульата на 1.
	
	А после делитель сдвинуть на один разряд. Мы же его недавно двигали вперёд? Двигали. Надо вернуть. 
	И так повторять пока делитель не сдвинется в самое начало себя. Тогда и конец математики
	*/
	DIV64_M_division:
		WDR	//Плохо это делать именно тут... Но иначе мы не сможем поделить, такая математика слишком сложна
		//Сдвигаем наш результат на ещё один разряд
   		LDI 	YL,low(ACC_1)
		LDI 	YH,high(ACC_1)
		RCALL DIV64_M_Load
		LSL	r0
		ROL r1
		ROL r2
		ROL r3
		ROL r4
		ROL r5
		ROL r6
		ROL r7
		RCALL DIV64_M_Save
		//Остаток от делимого
   		LDI 	YL,low(ACC_2)
		LDI 	YH,high(ACC_2)
		RCALL DIV64_M_Load
		//Делитель
   		LDI 	YL,low(TMP_64)
		LDI 	YH,high(TMP_64)
		LD		R15,Y+
		LD		R14,Y+
		LD		R13,Y+
		LD		R12,Y+
		LD		R11,Y+
		LD		R10,Y+
		LD		R9,Y+
		LD		R8,Y

		SUB r0,r8
		SBC r1,r9
		SBC r2,r10
		SBC r3,r11
		SBC r4,r12
		SBC r5,r13
		SBC r6,r14
		SBC r7,r15

		BRMI /*-*/ DIV64_M_division_litle
			//Вычли с плюсом, новый остаток
   			LDI 	YL,low(ACC_2+7)
			LDI 	YH,high(ACC_2+7)
			RCALL DIV64_M_Save
			//Единица
			LDS MacroR, ACC_1 + 7
			INC MacroR
			STS ACC_1 + 7, MacroR
		DIV64_M_division_litle:
		
   		LDI 	YL,low(TMP_64)
		LDI 	YH,high(TMP_64)
		RCALL DIV64_M_Load
		//Теперь делитель сдвигается обратно на единицу
		LSR	r7
		ROR r6
		ROR r5
		ROR r4
		ROR r3
		ROR r2
		ROR r1
		ROR r0
		RCALL DIV64_M_Save
		
		DEC Counter
	BRNE /*!=*/ DIV64_M_division


	DIV64_M_end:
	
	POP		R15
	POP		R14
	POP		R13
	POP		R12
	POP		R11
	POP		R10
	POP		R9
	POP		R8
	POP		Counter
	POP		R7
	POP		R6
	POP		R5
	POP		R4
	POP		R3
	POP		R2
	POP		R1
	POP		R0
	POP		YH
	POP		YL
 RET