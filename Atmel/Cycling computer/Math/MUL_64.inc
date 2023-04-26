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

 
 //АСС(64 бит) = АСС(64 бит) * Xl(32)
 .MACRO		_MUL64_A_R32
	LDI		MacroR,MUL_64_MODE_32
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	RCALL	MUL64_M
.ENDMACRO
.MACRO		MUL64_A_R32
	PUSH	XL     
	PUSH	XH

	_MUL64_A_R32 @0

	POP		XH
	POP		XL
.ENDMACRO
 //АСС(64 бит) = АСС(64 бит) * Xl(64)
 .MACRO		_MUL64_A_R64
	LDI		MacroR,MUL_64_MODE_64
   	LDI 	XL,low(@0)
	LDI 	XH,high(@0)
	RCALL	MUL64_M
.ENDMACRO
.MACRO		MUL64_A_R64
	PUSH	XL     
	PUSH	XH

	_MUL64_A_R64 @0

	POP		XH
	POP		XL
.ENDMACRO

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