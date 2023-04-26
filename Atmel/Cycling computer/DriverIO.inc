/*
 * DriverIO.inc
 *
 *  Created: 25.11.2018 11:40:51
 *   Author: Terran
 */ 

 O:		
			PUSH 	ZL
			PUSH 	ZH
			PUSH 	word
			PUSH 	Counter

			LDI 	ZL,low(WordOut)
			LDI 	ZH,high(WordOut)
			ADIW	ZL,WordOutSize-1

			
			LDI 	Counter, WordOutSize
			MOV		Counter2,Counter
			LDI 	Counter, 8

			CLI

			LD	 	word, Z
			cbi		PORTD,6
			O04:
					cbi		PORTB,5
					LSL		word
					BRCS O01
						cbi		PORTD,4
						RJMP	O02
					O01:
						sbi		PORTD,4
					O02:
					sbi		PORTB,5

					DEC		Counter
				BRNE 	O04			;Прошли слово

				LD	 	word, -Z
				DEC 	Counter2
				LDI 	Counter, 8
			BRNE 	O04			;Прошли фразу
			SEI
			
			cbi		PORTB,5		//Выключили пин 13
			sbi		PORTD,6

			pop 	Counter
			pop 	word
			pop 	ZH
			pop 	ZL

RET

/**Функция появления прерывания со стороны ног входных сигналов*/
I:		
			PUSH 	ZL
			PUSH 	ZH
			PUSH 	word
			PUSH 	Counter
			PUSH 	OSRG

			LDI 	ZL,low(WordIn)
			LDI 	ZH,high(WordIn)
			DEC		Zl
			DEC		Zl

			
			LDI 	Counter, WordInSize
			MOV		Counter2,Counter
			LDI 	Counter, 8

			CLI

			ANDI 	word, 0
			cbi		PORTD,7
			sbi		PORTD,7
			Sbi		PORTB,1
			I04:
					cbi		PORTB,5
					IN		OSRG,PIND
					ANDI	OSRG,0b00100000
					BRNE I01
						RJMP	I02
					I01:
						ORI		word,1
					I02:

					ROR		word
					sbi		PORTB,5
					DEC		Counter
				BRNE 	I04			;Прошли слово
				ROR		word

				INC		ZL
				INC		ZL
				ST	 	Z,word
				DEC 	Counter2
				LDI 	Counter, 8
			BRNE 	I04			;Прошли фразу
			Cbi		PORTB,1
			
//			SetTask TS_SignalIn

			LDI 	Counter, 1
			OUT		EIFR,Counter
			SEI

			pop 	OSRG
			pop 	Counter
			pop 	word
			pop 	ZH
			pop 	ZL
			
RET

/**Сработал Геркон*/
ReedSwitch_E:
	_INC_32 Dist
	_INC_32 Odometr
	_MOV_32 B_32, TimeTrip
	
	//Проверяем - мы двигаемся?
	TSTFlag isMove
	BRNE /*!=*/ ReedSwitch_E_not_move
		//Тогда находим нашу мгновенную скорость
		//Мы знаем время оборота в мс или мс/оборот, а нужно в скорость перевести - оборотв/мин. Поэтому надо 60'000 / T 
		//Обороты в минуту, потому что целочисленное деление
		_LDI_A 60000
		_DIV_64_A_DIV_R32 B_32
		_MOV_32 Speed, ACC_1
	ReedSwitch_E_not_move:
	_MOV_32 TimeVOld, B_32
RET


SignalIn:	
			/*LDI 	ZL,low(WordIn)
			LDI 	ZH,high(WordIn)
			LD		tmp2,Z+					//Стало
			LD		tmp3,Z					//Было
			TST		tmp3
			BRNE	SI98
				JMP SI99			//Если у нас было 0, значит мы срочно дуем в конец, это инициализация!!!
			SI98:
			EOR		tmp2,tmp3
			SI00:
			SBRC	tmp2,0
				RJMP SI01
			SBRC	tmp2,1
				RJMP SI02
			SBRC	tmp2,2
				RJMP SI03
			SBRC	tmp2,3
				RJMP SI04
			SBRC	tmp2,4
				RJMP SI05
			SBRC	tmp2,5
				RJMP SI06
			SBRC	tmp2,6
				RJMP SI07
			SBRC	tmp2,7
				RJMP SI08
			RJMP SI99


			SI01:
					EORI tmp2,0b1
					RJMP SI00
			SI02:		//Геркон педалей
					EORI tmp2,0b10
					RJMP SI00
			SI03:		//Геркон колеса
					EORI tmp2,0b100
					PUSH tmp2
					CLR_32 B_2
					CP_F B_2,TimeV
					BRNE /*!=*/ /*SI03_01
						//Мы только стартанули
						LDI_8 B_2,1
						ADD_F TimeV,B_2
						LDI_8 Speed,1
						SetTimerTask TS_StartTime,1
						RJMP SI03_03
					SI03_01:
						//У нас новое касание
						MOV_32 B_2,Lenght
						DIV_F B_2,TimeV	//Скорость м/с
						//LPM_Z_X Const_3_6,C_2,4
						//MUL_F B_2,C_2	//Скорость км/ч
						MOV_32 Speed, B_2
						/*LPM_Z_X Alf,C_2,4
						MUL_F B_2,C_2
						MOV_32 D_2,Speed
						MUL_F D_2,C_2
						ADD_F Speed,B_2
						SUB_F Speed,D_2*//*
							//Текущая скорость

						ADD_F Dist,Lenght //Дистанция

						ADD_F TimeTrip,TimeV //Общее время

						MOV_32 AverageSpeed,Dist
						DIV_F AverageSpeed,TimeTrip //Средняя скорость м/с
						//LPM_Z_X Const_3_6,C_2,4
						//MUL_F AverageSpeed,C_2	//Средняя скорость км/ч

						CP_F MaxSpeed,Speed
						BRSH /*>=*/ /*SI03_02
							MOV_32 MaxSpeed,Speed
						SI03_02:

						CP_F MaxSpeedOdo,Speed
						BRSH /*>=*//* SI03_04
							MOV_32 MaxSpeedOdo,Speed
						SI03_04:

						MOV_32 B_2,Lenght
						ADD_F Odometr,B_2

						ADD_F TimeOdo,TimeV

						CLR_32 TimeV

						SI03_03:
						LDI 	ZL,low(WordIn)
						LDI 	ZH,high(WordIn)
						LD		tmp3,Z
						ORI tmp3,0b100
						ST		Z,tmp3
						POP tmp2
						SetTask TS_LedPower //Крутим колёса
					RJMP SI00
			SI04:					//Нажали ентер
					SetTimerTask TS_New_Init_Scrin,60000 //Обновление экрана через 60 секунд
					//Flag_1 = 0b Key1 Key2 Scr MENU UART TWI Lamp U Temp
					MOV	tmp3, Flag_1
					ANDI tmp3,0b100000
					BREQ SI0410
						JMP SI0411
					SI0410:				//Инициализируем меню
							//Сохраняем время, часы и минуты, дни недели ((одноразовый))

							//Делаем всё, чтобы меню не чувствовало себя ущербно
							LDI		word,-2

							LDI		OSRG,2				//Сохраняем новый пункт меню
							LDI 	YL,low(StringOutOne)	//Установили строку которую переносим
							LDI 	YH,high(StringOutOne)	//
							MOV		Al,Yl
							MOV		Ah,Yh
							MOV		ZL,Yl
							MOV		ZH,Yh
							CALL	MenuUpDown	
							LDI		OSRG,1				//Сохраняем новый пункт меню
							LDI 	YL,low(StringOutOne)	//Установили строку которую переносим
							LDI 	YH,high(StringOutOne)	//
							MOV		Al,Yl
							MOV		Ah,Yh
							LDI 	ZL,low(StringOutTwo)	//Установили строку в которую переносим
							LDI 	ZH,high(StringOutTwo)	//
							CALL	MenuUpDown				//Мы первый засунем в первый, а потом его же и обновим
							EORI Flag_1,0b100000//Устанавливаем флаг, что мы инициализиаровали меню
							//-----------------------------------//Очищем экран
							SetScrinKeyWord 0b1
							SetTask TS_O

							NewScrinUpdate

							RJMP	SI041

					SI0411:	//А если мы инициализировали меню, то переходим сюда!
							LDI 	YL,low(MenuPCBS)		//Установили строку которую переносим
							LDI 	YH,high(MenuPCBS)		//	
					
							SUBI	Yl,-2					//Берём дитя
							SBCI	Yh,-1
							LD		OSRG,Y
							TST		OSRG
							BREQ	SI041
								LDI 	YL,low(StringOutOne)	//Установили строку которую переносим
								LDI 	YH,high(StringOutOne)	//
								MOV		Al,Yl
								MOV		Ah,Yh
								MOV		ZL,Yl
								MOV		ZH,Yh
								CALL	MenuUpDown				//Мы первый засунем в первый, а потом его же и обновимъ
								LDI 	YL,low(MenuPCBS)		//Сохраняем новый пункт меню
								LDI 	YH,high(MenuPCBS)		//
								SUBI	Yl,-4
								SBCI	Yh,-1
								LD		OSRG,Y
								LDI 	YL,low(StringOutOne)	//Установили строку которую переносим
								LDI 	YH,high(StringOutOne)	//
								MOV		Al,Yl
								MOV		Ah,Yh
								LDI 	ZL,low(StringOutTwo)	//Установили строку в которую переносим
								LDI 	ZH,high(StringOutTwo)	//
					
								CALL	MenuUpDown				//Мы первый засунем в первый, а потом его же и обновим
								ORI tmp2,0b100000				//Нажимаем кнопку вниз
								//-----------------------------------//Очищем экран
								SetScrinKeyWord 0b1
								SetTask TS_O
								NewScrinUpdate
					SI041:		
					LDI 	ZL,low(WordIn)
					LDI 	ZH,high(WordIn)
					LD		tmp3,Z
					ORI tmp3,0b1000
					ST		Z,tmp3
					INC		Zl
					EORI tmp2,0b1000
					RJMP SI00
			SI05:		//Нажали кнопку влево
					//Flag_1 = 0b Key1 Key2 ~ MENU UART TWI Lamp U Temp
					MOV	tmp3, Flag_1
					ANDI tmp3,0b100000
					BRNE SI0510
						JMP SI051
					SI0510:	
						//Обработка нажатия
						LDS tmp3,StringOutTwo+(StringOutSize-1)-1
						DEC tmp3
						CPI tmp3,8
						BREQ /*=*//* SI051
							STS StringOutTwo+(StringOutSize-1)-1,tmp3
					SI051:
					LDI 	ZL,low(WordIn)
					LDI 	ZH,high(WordIn)
					LD		tmp3,Z
					ORI tmp3,0b10000
					ST		Z,tmp3
					INC		Zl
					EORI tmp2,0b10000
					RJMP SI00
			SI06:		//Нажали кнопку вниз
					//Flag_1 = 0b Key1 Key2 ~ MENU UART TWI Lamp U Temp
					PUSH tmp2
					MOV	tmp3, Flag_1
					ANDI tmp3,0b100000
					BRNE SI0610
						JMP SI061
					SI0610:
						LDS tmp3,StringOutTwo+(StringOutSize-1)-1
						SUBI tmp3,8

						CLR tmp2
						SBRC tmp3,1
							LDI tmp2,10
						SBRS tmp3,0
							LDI tmp2,100
						SBRC tmp3,2
							LDI tmp2,1

						CLR_32 B_2
						STS B_2+3,tmp2
						TST tmp2
						BRNE /*!=*//* SI0611
							LDI tmp2,0x03
							STS B_2+2,tmp2
							LDI tmp2,0xE8
							STS B_2+3,tmp2
						SI0611:
						SUB_F Lenght,B_2
					SI061:
					LDI 	ZL,low(WordIn)
					LDI 	ZH,high(WordIn)
					LD		tmp3,Z
					POP tmp2
					ORI		tmp3,0b100000
					ST		Z,tmp3
					INC		Zl
					EORI tmp2,0b100000
					RJMP SI00
			SI07:		//Нажали кнопку впрaво
					//Flag_1 = 0b Key1 Key2 ~ MENU UART TWI Lamp U Temp
					MOV	tmp3, Flag_1
					ANDI tmp3,0b100000
					BRNE SI0710
						JMP SI071
					SI0710:	
						//Обработка нажатия
						LDS tmp3,StringOutTwo+(StringOutSize-1)-1
						INC tmp3
						CPI tmp3,13
						BRNE /*!=*//* SI072
							STI Mode,Mode51
							LDI tmp3,3
						SI072:
						STS StringOutTwo+(StringOutSize-1)-1,tmp3
					SI071:
					LDI 	ZL,low(WordIn)
					LDI 	ZH,high(WordIn)
					LD		tmp3,Z
					ORI		tmp3,0b1000000
					ST		Z,tmp3
					INC		Zl
					EORI tmp2,0b1000000
					RJMP SI00
			SI08:		//Нажали кнопку вверх
					//Flag_1 = 0b Key1 Key2 ~ MENU UART TWI Lamp U Temp
					PUSH tmp2
					MOV	tmp3, Flag_1
					ANDI tmp3,0b100000
					BRNE SI0810
						JMP SI081
					SI0810:
						LDS tmp3,StringOutTwo+(StringOutSize-1)-1
						SUBI tmp3,8

						CLR tmp2
						SBRC tmp3,1
							LDI tmp2,10
						SBRS tmp3,0
							LDI tmp2,100
						SBRC tmp3,2
							LDI tmp2,1

						CLR_32 B_2
						STS B_2+3,tmp2
						TST tmp2
						BRNE /*!=*/ /*SI0811
							LDI tmp2,0x03
							STS B_2+2,tmp2
							LDI tmp2,0xE8
							STS B_2+3,tmp2
						SI0811:
						ADD_F Lenght,B_2
					SI081:
					LDI 	ZL,low(WordIn)
					LDI 	ZH,high(WordIn)
					LD		tmp3,Z
					ORI tmp3,0b10000000
					ST		Z,tmp3
					INC		Zl
					POP tmp2
					EORI tmp2,0b10000000
					RJMP SI00
			SI99:
			LDI 	ZL,low(WordIn)
			LDI 	ZH,high(WordIn)
			LD		tmp3,Z+
			ST		Z,tmp3
RET*/



MenuUpDown:
					
RET

TWI_IO_End:
		LDI 	YL,low(TWI_IO)
		LDI 	YH,high(TWI_IO)
		INC		Yl
		LD		OSRG,Y
		CPI		OSRG,(0b1101000<<1)|1
		BRNE	TIE01
			//RCALL TWI_IO_End_Clock_Read
			JMP TIE00
		TIE01:
		CPI		OSRG,(0b1010000<<1)|1
		BRNE	TIE02
			//Работаем с будильниками. Для этого у нас есть флаговый бит, это двоеточие в 00:00
			//RCALL TWI_IO_End_EEPROM_Read
			RJMP TIE00
		TIE02:
		CPI		OSRG,(0b1110110<<1)|1
		BRNE	TIE03
			//RCALL TWI_IO_End_Wether_Read
			RJMP TIE00
		TIE03:

		CPI		OSRG,(0b1101000<<1)|0
		BRNE	TOE01
			//CALL TWI_IO_End_Clock_Write
			RJMP TIE00
		TOE01:

		CPI		OSRG,EEPROMWriteAdr
		BRNE	TIE10
			//CALL TWI_IO_End_EEPROM_Write
			RJMP TIE00
		TIE10:

		CPI		OSRG,(0b1110110<<1)|0
		BRNE	TIE20
			//CALL TWI_IO_End_Wether_Write
			RJMP TIE00
		TIE20:

		TIE00:
		CLI
			//Flag_1 = 0b TWI Lamp U Temp
			MOV		OSRG,Flag_1
			ANDI	OSRG,0b11110111		//Флаг Занятости TWI
			MOV		Flag_1,OSRG
		SEI
RET
//Файлы драйверов
 //.include "TWIDevice/Clock.inc"
 //.include "TWIDevice/EEPROM.inc"
 //.include "TWIDevice/Wether.inc"

/*Функция, вызывается, если у нас устройство не ответило*/
TWI_IO_Error:
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

		//Flag_1 = 0b TWI Lamp U Temp
		ANDI	Flag_1,0b11110111		//Флаг Занятости TWI
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

StartTWI:
	LDI OSRG,(1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA)			/*разрешаем прерывание и формируем состояние СТАРТ, мастер - это мы!*/
	UOUT TWCR,OSRG
	CLI
		//Flag_1 = 0b TWI Lamp U Temp
		MOV		OSRG,Flag_1
		ORI		OSRG,0b1000		//Флаг Занятости TWI
		MOV		Flag_1,OSRG
	SEI
RET
/*
EEPROMRead:
			SBI	PORTB,0		//Охрана записи
			LDI 	ZL,0
			LDI 	ZH,0
			LDI 	tmp2,EEPROMStrSize-1
			STS 	Eread,tmp2

			OUT 	EEARL,ZL 		; Загружаем адрес нужной ячейки
			OUT 	EEARH,ZH  		; старший и младший байт адреса
			sbi  EECR,EERE          ; Запускаем чтение установкой флага на чтение
			
			in   OSRG, EEDR          ; Возвращаем результат чтения в нужный регистр
			LDI 	ZL,low(EEPROMStr)
			LDI 	ZH,high(EEPROMStr)
			ST		Z,OSRG
			SBI 	EECR,EERIE		; Включаем прерывание
RET*/
/*
EEPROMWrite:
			SBI	PORTB,0		//Охрана записи
			LDI 	ZL,0
			LDI 	ZH,0
			LDI 	tmp2,EEPROMStrSize-1
			STS 	Ewrite,tmp2

			OUT 	EEARL,ZL 		; Загружаем адрес нужной ячейки
			OUT 	EEARH,ZH  		; старший и младший байт адреса
			
			LDS	OSRG,EEPROMStr
			
			OUT 	EEDR,OSRG 		; и сами данные, которые нам нужно загрузить
 
			SBI 	EECR,EEMPE 		; взводим предохранитель
			SBI 	EECR,EEPE 		; записываем байт

			SBI 	EECR,EERIE		; Включаем прерывание
RET*/

EEPROM_End:
			/*MOV_32	MaxSpeedOdo,EEPROMStr
			//MOV_32	Lenght,EEPROMStr+4
			//			LPM_Z_X PKKLKLJLKHJKH,Lenght,4
			MOV_32	TimeOdo,EEPROMStr+8
			MOV_32	Odometr,EEPROMStr+12

			CLR_32 B_2
			CP_F   B_2,Lenght
			BRNE /*!=*//* EE_01
				//У нас длина колеса = 0. Надо запустить процесс инициализации
				STI Mode,Mode51
				EORI Flag_1,0b100000//Устанавливаем флаг, что мы инициализиаровали меню

				LDI 	ZL,low(StringOutTwo+(StringOutSize-1))	//Берём первую строку
				LDI 	ZH,high(StringOutTwo+(StringOutSize-1))	
				
				LDI		word,0xFF			//Первый символ превращается в чёрный кубик
				ST		Z,word
				LDI		word,9				//Мигает 8 символ
				ST		-Z,word
				LDI		word,'0'				//Оригинал символа далее
				ST		-Z,word
				LDI		word,2				//Мигает вторая строка
				STS		StringOutOne+(StringOutSize-1)-3,word

				SetTimerTask TS_ScreenBlink,250
			EE_01:*/
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