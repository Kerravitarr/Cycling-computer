/*
 * MUL_64.inc
 *
 *  Created: 26.04.2023 18:55:38
 *   Author: ????
 */ 
 /*
 * MUL_64.inc
 *
 Умножает два беззнаковых числа, из которых одно - аккумулятор
 В зависимости от регистра MacroR выполняет разное умножение:
 MUL_64_MODE_32 : АСС(64 бит) = АСС(64 бит) * Xl(32)
 MUL_64_MODE_64 : АСС(64 бит) = АСС(64 бит) * Xl(64)

 Входные данные:
 MacroR - код операции
 Xh:Xl - адрес множителя

 Выходные данные:
 ACC_1 - произведение

 *  Created: 26.04.2023 7:20:28
 *   Author: Terran
 */ 
 .equ MUL_64_MODE_32 = 4-1
 .equ MUL_64_MODE_64 = 8-1
 
 //Флаги для знакового умножения
 .equ MUL_64_FLAG_negA = 1 << 0
 .equ MUL_64_FLAG_negR = 1 << 1
 
 //АСС(беззнака 64 бит) = АСС(беззнака 64 бит) * Xl(беззнака 32)
 .MACRO		_MUL64_A_R32
	LDI		MacroR,MUL_64_MODE_32
	LDIW 	X, @0
	CALL	MUL64_M
.ENDMACRO
.MACRO		MUL64_A_R32
	PUSH	XL     
	PUSH	XH

	_MUL64_A_R32 @0

	POP		XH
	POP		XL
.ENDMACRO
 //АСС(беззнака 64 бит) = АСС(беззнака 64 бит) * Xl(беззнака 64)
 .MACRO		_MUL64_A_R64
	LDI		MacroR,MUL_64_MODE_64
	LDIW 	X, @0
	CALL	MUL64_M
.ENDMACRO
.MACRO		MUL64_A_R64
	PUSH	XL     
	PUSH	XH

	_MUL64_A_R64 @0

	POP		XH
	POP		XL
.ENDMACRO

 //АСС(знаковые 64 бит) = АСС(знаковые 64 бит) * Xl(знаковые 32)
 .MACRO		_MULS64_A_R32
	LDI		MacroR,MUL_64_MODE_32
	LDIW 	X, @0
	CALL	MULS64_M
.ENDMACRO
.MACRO		MULS64_A_R32
	PUSH	XL     
	PUSH	XH

	_MULS64_A_R32 @0

	POP		XH
	POP		XL
.ENDMACRO
 //АСС(знаковые 64 бит) = АСС(знаковые 64 бит) * Xl(знаковые 64)
 .MACRO		_MULS64_A_R64
	LDI		MacroR,MUL_64_MODE_64
	LDIW 	X, @0
	CALL	MULS64_M
.ENDMACRO
.MACRO		MULS64_A_R64
	PUSH	XL     
	PUSH	XH

	_MULS64_A_R64 @0

	POP		XH
	POP		XL
.ENDMACRO

MULS64_M:
	PUSH tmp2
	PUSH tmp3
	PUSH MacroR
	MOV tmp3, MacroR //Сохраняем флаг

	CLR tmp2
	LDS MacroR, ACC_1
	ANDI MacroR, 1 << 7
	BREQ /*=*/ SMUL64_M_ACC_positive
		ORI tmp2, MUL_64_FLAG_negA
		NEG_64 ACC_1
	SMUL64_M_ACC_positive:

	LD MacroR, X
	ANDI MacroR, 1 << 7
	BREQ /*=*/ SMUL64_M_R_positive
		ORI tmp2, MUL_64_FLAG_negR
		PUSH Xl
		PUSH Xh
		
			CPI tmp3, MUL_64_MODE_32
			BREQ /*=*/ SMUL64_M_R32
				LDI MacroR, 8
				CALL NEG_M
				RJMP SMUL64_M_R_end
			SMUL64_M_R32:
				LDI MacroR, 4
				CALL NEG_M
			SMUL64_M_R_end:
			

		POP Xh
		POP Xl
	SMUL64_M_R_positive:
	//Теперь у нас аккумулятор и число точно положительные, а в регистре tmp3 хранится знак числа
	POP MacroR //Достаём режим работы
	PUSH Xl
	PUSH Xh
	RCALL MUL64_M//И умножаем
	POP Xh
	POP Xl
	TST tmp2
	BREQ /*=*/ MULS64_M_end //Шик, ни чего делать и не нужно!
		//Как жаль, где-то негатив есть.
		CPI tmp2, MUL_64_FLAG_negA | MUL_64_FLAG_negR
		BREQ /*=*/ MULS64_M_neg2
			//У нас только одно число негативное
			//Значит результат тоже отрицательным будет :(
			NEG_64 ACC_1
			ANDI tmp2, MUL_64_FLAG_negA
			BRNE /*!=*/ MULS64_M_end //Отрицательный - аккумулятор. Ни чего не меняем
				//Отрицательное число. Вот его и инвертнём дальше!
		MULS64_M_neg2:
			//У нас оба числа негативные. А - на - даст +!
			//Надо инвертнуть только число
			CPI tmp3, MUL_64_MODE_32
			BREQ /*=*/ SMUL64_M_R32_2
				LDI MacroR, 8
				CALL NEG_M
				RJMP MULS64_M_end
			SMUL64_M_R32_2:
				LDI MacroR, 4
				CALL NEG_M
	MULS64_M_end:
	POP tmp3
	POP tmp2
RET

//Xh:Xl - второй множитель, указатель на его старшый разряд
//MacroR - режим, в котором работает функция
MUL64_M:
		PUSH	r0
		PUSH	r1
		PUSH	r2
		PUSH	r3
		PUSH	r4
		PUSH	r5
		PUSH	YL
		PUSH	YH
		PUSH	Counter
		
		MOV		r4, MacroR //Запомнили длину второго множителя
		MOV_64	TMP_64, ACC_1	//Второе число
		CLR_64	ACC_1			//Произведение = 0
		CLR_64	ACC_2			//Вспомогательный регистр в 0 тоже
		LDI 	YL,low(TMP_64 + 7 + 1)
		LDI 	YH,high(TMP_64 + 7 + 1)	//Отсюда будем брать результат
								//А эти странные + 1, так как у нас есть команда с предекрементом, а нужен постдекремент. Увы и ах
		CLR		MacroR
		ADD		Xl,r4
		ADC		Xh,MacroR			//Сдвинули второй множитель в голову
		LDI 	ZL,low(ACC_1 + 7)
		LDI 	ZH,high(ACC_1 + 7)	//Сюда будем записывать результат

		//Х - множитель входной
		//Y - множитель ACC
		//Z - результат

		MUL64_M_R_for:
			WDR	//Плохо это делать именно тут... Но иначе мы не сможем поделить, такая математика слишком сложна
			LD	r5,X	//Загружаем байт первого множителя
			TST r5
			BRNE /*!=*/ MUL64_M_R_not0
				//У нас в r0 = 0, то есть я заранее готов дать ответ
				// чему будет равен результат
				ADIW Z,1
				RJMP MUL64_M_R_for_0
			MUL64_M_R_not0:
				LDI	Counter,8		//Сохранили длину аккумулятора
				MUL64_M_ACC_for:
					LD	r0,-Y	//Загружаем байт второго множителя
					TST r0
					BREQ /*=*/  MUL64_M_ACC_for_0
						//Оба операнда не 0, будем умножать, что поделать
						MOVW r3:r2, Zh:Zl
						MUL R0,r5		//Умножаем
						LD	MacroR,Z	//А теперь прибавляем полученное произведение к первым двум разрядам
						ADD	MacroR,r0
						ST	Z,MacroR
						LD	MacroR,-Z
						ADC MacroR,r1
						ST	Z,MacroR
						BRCC /*С = 0*/ MUL64_M_ACC_for_end_summ
							MUL64_M_ACC_for_summ:
								LD	MacroR,-Z
								SUBI MacroR,-1 //+1, это как раз перенос
								ST	Z,MacroR
							BRCC /*С = 0*/ MUL64_M_ACC_for_summ
						MUL64_M_ACC_for_end_summ:
						MOVW Zh:Zl, r3:r2
					MUL64_M_ACC_for_0:
					SBIW Z,1
					DEC	Counter
				BRNE /*!=*/ MUL64_M_ACC_for
				ADIW Y,8	//Сдвигаем голову второго множителя вновь в начало
				ADIW Z,8-1
			MUL64_M_R_for_0:
			SBIW X,1
			DEC	r4
		BRNE /*!=*/ MUL64_M_R_for

		
		POP	Counter
		POP		YH
		POP		YL
		POP		r5
		POP		r4
		POP		r3
		POP		r2
		POP		r1
		POP		r0
 RET