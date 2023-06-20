/*
 * DriverIO.inc
 *
 *  Created: 25.11.2018 11:40:51
 *   Author: Terran
 */ 
 
 //Очищает флаг регистра вывода. Эти флаги начинаются с O_
 .MACRO COPin
	.IF	@0 < 8
		LDS MacroR,WordOut
		ANDI MacroR, ~(1 << @0)
		STS WordOut,MacroR       
	.ELSE
		LDS MacroR,WordOut+1
		ANDI MacroR, ~(1 << (@0-8))
		STS WordOut+1,MacroR   
	.endif
.ENDMACRO
 //Тестирует флаг регистра вывода. Эти флаги начинаются с O_
 .MACRO TST_OPin
	.IF	@0 < 8
		LDS MacroR,WordOut
		ANDI MacroR, 1 << @0
	.ELSE
		LDS MacroR,WordOut+1
		ANDI MacroR, 1 << (@0-8)
	.endif
 .ENDMACRO
 //Сохраняет флаг регистра вывода. Эти флаги начинаются с O_
 .MACRO SOPin
	.IF	@0 < 8
		LDS MacroR,WordOut
		ORI MacroR, 1 << @0
		STS WordOut,MacroR       
	.ELSE
		LDS MacroR,WordOut+1
		ORI MacroR, 1 << (@0-8)
		STS WordOut+1,MacroR   
	.endif
 .ENDMACRO

 O:		
	PUSH 	ZL
	PUSH 	ZH
	PUSH 	tmp2
	PUSH 	Counter

	LDIW 	Z, WordOut + WordOutSize-1 //Берём с конца

			
	LDI 	Counter, WordOutSize
	MOV		Counter2,Counter
	LDI 	Counter, 8

	CLI

	LD	 	tmp2, Z
	cbi		PORTD,6
	O04:
			cbi		PORTB,5
			LSL		tmp2
			BRCS O01
				cbi		PORTD,4
				RJMP	O02
			O01:
				sbi		PORTD,4
			O02:
			sbi		PORTB,5

			DEC		Counter
		BRNE 	O04			;Прошли слово

		LD	 	tmp2, -Z
		DEC 	Counter2
		LDI 	Counter, 8
	BRNE 	O04			;Прошли фразу
	SEI
			
	cbi		PORTB,5		//Выключили пин 13
	sbi		PORTD,6

	pop 	Counter
	pop 	tmp2
	pop 	ZH
	pop 	ZL

RET

//Чтение данных с входной микрухи
I:
	//Читаем регистры
	LDIW	Z, WordIn//Берём память, куда будем писать

	LDI 	tmp2, WordInSize //Всего регистров

	CBI		PORTD,7	//запоминание состояний входа. Обновляем входящую микруху
	SBI		PORTD,7 //Микруха быстрее нас - успеет за миг
	I04:
		LDI 	Counter, 8	//Читаем 8 бит
		CLR 	tmp3		//От сюда читаем
		I_world_l:
			CBI		PORTB,4
			LSL		tmp3	//Подготавливаем под бит место
			SBIC	PIND, 5 //данные извне. Если тут не 0 (если 0 - пропускаем), то запоминаем!
				ORI		tmp3,1
			SBI		PORTB,4 //Строб - следующий, пожалуйста 
			DEC		Counter
		BRNE /*!=*/ 	I_world_l	;Прошли слово?
		ST	 	Z,tmp3
		ADIW	Z,2 //Сдвигаем по 2, так как у нас подряд идёт байт текущий и предыдущий
		DEC 	tmp2
	BRNE /*!=*/ 	I04	;Всё вычитали?
			
	//Обрабатываем что начитали

	LDS	Zl,WordIn		//Стало
	LDS	Zh,WordIn+1		//Было
	TST Zh
	BRNE /*!=*/ SignalIn_nend
		//У нас Zl - пустой. Это инициализация, нулевой запуск. Тут все кнопки нажаты разом
		RJMP SignalIn_end;
	SignalIn_nend:
	LDI Counter, 1
	SignalIn_test_w1:
		MOV tmp2, Zh
		EOR	tmp2, Zl //Сравниваем состояния
		AND tmp2, Counter //Выбираем текущий бит
		BREQ /*=*/ SignalIn_w1_not_dif
			//Есть разница!
			MOV tmp2, Zl
			AND tmp2, Counter //Выбираем текущий бит
			BRNE /*!=*/ SignalIn_w1_not_enable //Там мы смотрим по 0. 0 = включение кнопки
				//Эта разница ещё и на включение! Тогда выполняем функцию!
				CPI Counter, 1 << 0
				BRNE /*!=*/ SignalIn_w1_not_UP
					SetTask TS_Key_UP
					RJMP SignalIn_w1_not_enable
				SignalIn_w1_not_UP:
				CPI Counter, 1 << 1
				BRNE /*!=*/ SignalIn_w1_not_R
					SetTask TS_Key_R
					RJMP SignalIn_w1_not_enable
				SignalIn_w1_not_R:
				CPI Counter, 1 << 2
				BRNE /*!=*/ SignalIn_w1_not_D
					SetTask TS_Key_D
					RJMP SignalIn_w1_not_enable
				SignalIn_w1_not_D:
				CPI Counter, 1 << 3
				BRNE /*!=*/ SignalIn_w1_not_L
					SetTask TS_Key_L
					RJMP SignalIn_w1_not_enable
				SignalIn_w1_not_L:
				CPI Counter, 1 << 4
				BRNE /*!=*/ SignalIn_w1_not_enter
					//Нажали ентер!
					SetTask TS_Key_E
					RJMP SignalIn_w1_not_enable
				SignalIn_w1_not_enter:
			SignalIn_w1_not_enable:
		SignalIn_w1_not_dif:
		LSL Counter //Сдвигаем бит по всему байту
	BRNE /*!=*/ SignalIn_test_w1

	SignalIn_end:
	//Теперь обновляем все состояния и свободны
	LDIW Z, WordIn
	LDI Counter, WordInSize
	SignalIn_copy:
		LD tmp2, Z+
		ST Z+, tmp2
		DEC Counter
	BRNE /*!=*/ SignalIn_copy

RET


/**Сработал Геркон*/
ReedSwitch_E:
	_INC_32 Dist
	_INC_32 Odometr
	_MOV_32 B_32, TimeTrip
	_MOV_32 C_32, TimeTrip
	
	//Проверяем - мы двигаемся?
	TSTFlag isMove
	BRNE /*!=*/ ReedSwitch_E_move
		RJMP ReedSwitch_E_not_move
	ReedSwitch_E_move:
		//Тогда находим нашу мгновенную скорость
		//Мы знаем время оборота в мс или мс/оборот, а нужно в скорость перевести - оборотв/мин. Поэтому надо 60'000 / T 
		//Обороты в минуту, потому что целочисленное деление
		_CP_32 B_32, TimeVOld
		BRNE /*!=*/ ReedSwitch_E_nmax_V
			RJMP ReedSwitch_E_max_V
		ReedSwitch_E_nmax_V:
			//Скорость адекватная, по счастью
			_LDI_A 60000
			_SUB_32 B_32, TimeVOld
			_DIV_64_A_DIV_R32 B_32
			_MOV_32 Speed, ACC_32
			_CP_32 MaxSpeed, Speed
			BRSH /*>=*/ ReedSwitch_E_not_move
				_MOV_32 MaxSpeed, Speed
				_CP_32 MaxSpeedOdo, Speed //Может это максимальная скорость за всё время?! О_О
				BRSH /*>=*/ ReedSwitch_E_not_move
					_MOV_32 MaxSpeedOdo, Speed
				RJMP ReedSwitch_E_not_move
			RJMP ReedSwitch_E_not_move
		ReedSwitch_E_max_V:
			_LDI_32 Speed, 999
	ReedSwitch_E_not_move:
	SBIFlag isMove
	_MOV_32 TimeVOld, C_32
RET

TWI_IO_End:
		CLI
			CBIFlag TWI_Busy
		SEI

		LDS		tmp2, TWI_IO
		CPI		tmp2,DS1307Write
		BRNE	TIE_not_cw
			RCALL TWI_IO_End_Clock_Write
			RJMP TWI_IO_End_end
		TIE_not_cw:
		CPI		tmp2,DS1307Read
		BRNE	TIE_not_cr
			RCALL TWI_IO_End_Clock_Read
			RJMP TWI_IO_End_end
		TIE_not_cr:
		
		CPI		tmp2,BME250Write
		BRNE	TIE_not_bw
			RCALL TWI_IO_End_BME250_Write
			RJMP TWI_IO_End_end
		TIE_not_bw:
		CPI		tmp2,BME250Read
		BRNE	TIE_not_br
			RCALL TWI_IO_End_BME250_Read
			RJMP TWI_IO_End_end
		TIE_not_br:
		
		CPI		tmp2,EEPROMWriteAdr
		BRNE	TIE_not_ew
			RCALL TWI_IO_End_EEPROM_Write
			RJMP TWI_IO_End_end
		TIE_not_ew:
		CPI		tmp2,EEPROMReadAdr
		BRNE	TIE_not_er
			RCALL TWI_IO_End_EEPROM_Read
			RJMP TWI_IO_End_end
		TIE_not_er:

		TWI_IO_End_end:
RET

//Файлы драйверов
 .include "TWIDevice/Clock.inc"
 .include "TWIDevice/EEPROM.inc"
 .include "TWIDevice/Wether.inc"



/*Функция, вызывается, если у нас устройство не ответило*/
TWI_IO_Error:
		CLI
			CBIFlag TWI_Busy
		SEI
		LDS		tmp2, TWI_IO
		CPI		tmp2,EEPROMWriteAdr
		BREQ /*=*/ TIER_EE
		CPI		tmp2,EEPROMReadAdr
		BREQ /*=*/ TIER_EE
			RJMP TIER_not_EE
		TIER_EE:
			//Ошибка в EEPROM
			CALL TWI_IO_ERR_EEPROM
			RJMP TWI_IO_ERR_end
		TIER_not_EE:

		TWI_IO_ERR_end:
		/*
		LDI 	YL,low(TWI_IO)
		LDI 	YH,high(TWI_IO)
		INC		Yl
		LD		OSRG,Y
		LSR		OSRG	//Неважно нам что мы там хотели, принять или передать, факт - у нас ошибка
		
		/*LDI		tmp2,24 //Надо передать 20 символов
		STS		UART_O,tmp2

		LPM_Z_X	ScrinUARTErr,UART_O+1,24
		MOV		tmp2,OSRG
		SWAP	tmp2
		ANDI	tmp2,0x0F
		ANDI	OSRG,0x0F
		SUBI	tmp2,-0x30
		SUBI	OSRG,-0x30
		STS		UART_O+1+15,tmp2
		STS		UART_O+1+16,OSRG
		SetTask TS_StartUART*/
RET

UART_End:
	/*CLI
	LDS	OSRG,UART_I
	CPI	OSRG,'U' //Символ вверх
	BREQ UART_End_U
	CPI	OSRG,'D' //Символ вниз
	BREQ UART_End_D
	CPI	OSRG,'L' //Символ влево
	BREQ UART_End_L
	CPI	OSRG,'R' //Символ вправо
	BREQ UART_End_R
	CPI	OSRG,'E' //Ввод
	BREQ UART_End_E
	CPI	OSRG,'A' //Рестарт
	BREQ UART_End_A
	CPI	OSRG,'B' //Свет
	BREQ UART_End_B
	
	SetTask TS_UART_PrintExo
	RJMP UART_End_00

		UART_End_U:
				LDS		tmp3,WordIn+1				//Было
				EORI	tmp3,0b10000000
		RJMP UART_End_01
		UART_End_D:
				LDS		tmp3,WordIn+1				//Было
				EORI	tmp3,0b100000
		RJMP UART_End_01
		UART_End_L:
				LDS		tmp3,WordIn+1				//Было
				EORI	tmp3,0b10000
		RJMP UART_End_01
		UART_End_R:
				LDS		tmp3,WordIn+1				//Было
				EORI	tmp3,0b1000000
		RJMP UART_End_01
		UART_End_E:
			//А теперь надо нажать кнопку ентер
				LDS		tmp3,WordIn+1				//Было
				EORI	tmp3,0b1000
		
		UART_End_01:
			STS		WordIn,tmp3					//Стало
//			SetTask TS_SignalIn
			//SetTimerTask TS_UART_PrintStr,10
		RJMP UART_End_00

		UART_End_A:
			JMP Reset
		RJMP UART_End_00
		UART_End_B:
				MOV	OSRG,Flag_1
				EORI OSRG,0b100			//Изменяем состояние лампы
				MOV	Flag_1,OSRG
				ANDI	OSRG,0b100		//Флаг лампы
				LDS		OSRG, WordOut+1
				BRNE	UART_End_02
					//Выключаем лампу
						ANDI	OSRG,0b11101111
					RJMP UART_End_03
				UART_End_02:   //Включаем лампу
						ORI		OSRG,0b10000	
				UART_End_03:
				STS      WordOut+1,OSRG	
		RJMP UART_End_00

	UART_End_00:*/
RETI

UART_PrintExo:
	UARTPrint '>'
	LDS		MacroR,UART_I
	CALL UART_Print
	UARTPrint '\n'
RET


StartUART:
	LDI 	OSRG, (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(0<<TXCIE0)|(1<<UDRIE0)
	UOUT 	UCSR0B, OSRG
	CLI
		//Flag_1 = 0b UART TWI Lamp U Temp
		MOV		OSRG,Flag_1
		ORI		OSRG,0b10000		//Флаг Занятости UART
		MOV		Flag_1,OSRG
	SEI
RET

//Функция запуска I2C
StartTWI:
	LDI OSRG,(1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA)			/*разрешаем прерывание и формируем состояние СТАРТ, мастер - это мы!*/
	UOUT TWCR,OSRG
	CLI
		STI	TWI_IOc, 0
		SBIFlag TWI_Busy
	SEI
RET


//Печать символа. 
UART_Print:
	PUSH Zl
	PUSH Zh
	PUSH tmp2
	PUSH MacroR
	CLI
		LDS tmp2,UART_O_head
		INC tmp2
		CPI tmp2,UARTSize
		BRLO /*<*/ UART_Print_notEnd
			CLR tmp2
		UART_Print_notEnd:
		LDS Zl,UART_O_tail
		CP tmp2,Zl
		BREQ /*=*/ UART_Print_contAdd
			LDI 	ZL,low(UART_O_buf)
			LDI 	ZH,high(UART_O_buf)
			LDS		MacroR,UART_O_head
			ADD		ZL,MacroR
			SBCI	ZH, 0
			POP		MacroR
			ST		Z,MacroR
			STS UART_O_head,tmp2
			RJMP UART_Print_Add
		UART_Print_contAdd:
			POP MacroR
		UART_Print_Add:
		POP tmp2
		POP Zh
		POP Zl
		LDS MacroR,UCSR0B
		SBRC MacroR,UDRIE0
			RJMP UART_Print_UART_Start
		//UART не запущен
		LDI 	MacroR, (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(0<<TXCIE0)|(1<<UDRIE0)
		UOUT 	UCSR0B, MacroR
		UART_Print_UART_Start:
	SEI
RET