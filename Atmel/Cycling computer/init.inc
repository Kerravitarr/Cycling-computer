/*
 * IncFile1.inc
 *
 *  Created: 16.11.2018 7:10:16
 *   Author: Terran
 */ 

 .MACRO	INIT_RTOS
	ldi tmp2, 0x00
	out SREG, tmp2			; Инициализация SREG 

	CALL ClearTimers		; Очистить список таймеров РТОС
	CALL ClearSlowTimers	; Очистить список медленных таймеров
	CALL ClearTaskQueue		; Очистить очередь событий РТОС
	sei						; Разрешить обработку прерываний

	; Init Timer 2
	; Основной таймер для ядра РТОС

	.equ MainClock 		= 16000000				; CPU Clock
	.equ TimerDivider 	= MainClock/64/1000 	; 1 mS
			
	ldi tmp2,low(TimerDivider)
	UOUT OCR2A,tmp2				; Установить значение в регистр сравнения

	LDI tmp2,(1<<WGM01)|(0<<WGM00)	; Автосброс после достижения регистра сравнения
	UOUT TCCR2A,tmp2			
	LDI tmp2,0b0000100			; Freq = CK/64 - Установить режим и предделитель //64 было выше
	UOUT TCCR2B,tmp2			
	LDI tmp2,0b011
	UOUT TIMSK2,tmp2			; Разрешение прерываний

	CLR tmp2					; Установить начальное значение счётчиков
	UOUT TCNT2,tmp2				;

	;Регистры прерывания
	LDI OSRG,(1 << ISC01) |	(0 << ISC00) | (1 << ISC11) |	(0 << ISC10) 		;Прерывание 0 и 1 по нисходящему фронут
	UOUT EICRA,OSRG

	LDI OSRG,(1 << INT1) |	(1 << INT0)			;Разрешаем прерывание 0 и 1
	UOUT EIMSK,OSRG

	;Сторожевой таймер
	WDR
	LDI tmp2,(0 << WDIF) | (1 << WDIE) | (0 << WDE) | (1 << WDP3) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);; // Установите бит «Разрешить изменение бит» и «Включить режим сброса системы Watchdog»
	UOUT WDTCSR,tmp2
	LDI tmp2,(1 << WDIF) | (1 << WDIE) | (0 << WDE) | (1 << WDP3) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0); // Устанавливаем предварительную калибровку сторожевого таймера WDPS_1S 
	UOUT WDTCSR,tmp2
	WDR

	//I2C
	LDI tmp2,0x72			;скорость передачи (при 8 мГц получается 100 кГц), а у нас хз
	UOUT TWBR,tmp2
	LDI tmp2,0x00				
	UOUT TWSR,tmp2

	//Sleep
	LDI tmp2,(1 << SE)			;Мод Idle, спать можно
	UOUT SMCR,tmp2
			
	//UART
	//Скорость
	LDI 	tmp2, low(bauddivider)
	UOUT 	UBRR0L,tmp2
	LDI 	tmp2, high(bauddivider)
	UOUT 	UBRR0H,tmp2

	LDI 	tmp2,0
	UOUT 	UCSR0A, tmp2
 
	; Формат кадра - 8 бит, пишем в регистр UCSRC, за это отвечает бит селектор
	LDI 	tmp2, (0<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01)
	UOUT 	UCSR0C, tmp2

	; Прерывания разрешены, прием-передача разрешен.
	LDI 	tmp2, (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(0<<TXCIE0)|(0<<UDRIE0)
	UOUT 	UCSR0B, tmp2	

.ENDMACRO

Init_Scrin:	
			/*LDI 	ZL,low(WordOut+1)
			LDI 	ZH,high(WordOut+1)
			LD 		word,Z
			ANDI	word,0b1				;Узнаём состояние стробирующего бита и если он 1 - просто опускаем его в 0
			BREQ  IS01
				CLI
				LD 		word, Z
				EORI	word,0b1
				ST      Z,word
				SEI
				RJMP IS10
			IS01:
			LD 		word, Z
			ANDI		word,0b10				;Узнаём состояние командного бита. 0 - команда, 1 - слово
			BRNE  IS11
				LD 		word, -Z
				CPI		word, 0					//Если слово пустое, значит мы ещё ни разу ни чего не писали в него.
					BRNE IS04					//А значит надо инициализировать
					SetScrinKeyWord 0b00111100		//И записываем первое кодовое слово
					RJMP IS10
;----------------------------------------------------------------------
IS11:							JMP	IS03						;Островок передачи прыжка
;-----------------------------------------------------------------------
		IS04:	CPI		word, 0b00111100	;Кодовое слово 1 / 8 бит данных
				BRNE IS06
					SetScrinKeyWord 0b00001100
					RJMP IS10
		IS06:	CPI		word, 0b00001100	;Кодовое слово 2 / включеие экрана, 
				BRNE IS07
					SetScrinKeyWord 0b00000001
					RJMP IS10
		IS07:	CPI		word, 0b00000001	;Кодовое слово 3 / очистить и что то ещё с экраном
				BRNE IS08
					SetScrinKeyWord 0b00000110
					RJMP IS10
		IS08:
											//До этого мы инициалазировали дисплей
											//Далее мы заполним его память нашими секретными символами
											//Это очень важно, всего их три.
											//А номер нового символа будет храниться в строке выхода
											//Всё равно её пока ни кто не будет использовать

			LDI 	ZL,low(StringOutOne)
			LDI 	ZH,high(StringOutOne)
			LD		word,Z
			CPI		word, 0					//А мы вообще что ни будь делали?
			BRNE IS09
				SetScrinKeyWord 0b01001000	//Нет? Тогда срочно отправляем приказ писать символ!
				INC	word
				ST	Z,word
				RJMP IS10
		IS09:
			CPI		word, 2					//Пишем символ катушки
			BRNE IS16
				SetScrinKeyWord 0b01010000	//Нет? Тогда срочно отправляем приказ писать символ!
				INC	word
				ST	Z,word
				RJMP IS10
		IS16:
			CPI		word, 4					//Пишем символ радио
			BRNE IS17
				SetScrinKeyWord 0b01011000	//Нет? Тогда срочно отправляем приказ писать символ!
				INC	word
				ST	Z,word
				RJMP IS10
		IS17:
			CPI		word, 6					//Пишем символ точки
				BRNE IS18
					SetScrinKeyWord 0b01100000	//Нет? Тогда срочно отправляем приказ писать символ!
					INC	word
					ST	Z,word
					RJMP IS10
		IS18:
			CPI		word, 8					//Закончили запись?
			BRNE IS12
				RJMP	IS05				//Тогда инициализируем строку и ни в чём себе не отказываем
		IS12:

				
			INC	word
			ST	Z,word
			INC ZL


			CPI		word, 2					//Пишем символ градусов
			BRNE IS13
				LDI	YL,low(DegreesCelsiusSymbol*2)
				LDI	YH,high(DegreesCelsiusSymbol*2)
				INC	Zl
				ST	Z+,YL
				ST	Z,YH
			RJMP IS19
		IS13:
			CPI		word, 4					//Пишем символ катушки
			BRNE IS14
				//Включаем радио и катушку вместе
				LDI	YL,low(InductorSymbol*2)			//Если радио включено
				LDI	YH,high(InductorSymbol*2)
				INC	Zl
				ST	Z+,YL
				ST	Z,YH
			RJMP IS19
		IS14:
			CPI		word, 6					//Пишем символ Усилка
			BRNE IS15
				LDI	YL,low(SpeakerSymbol*2)			//Если усилок включён
				LDI	YH,high(SpeakerSymbol*2)

				INC	Zl
				ST	Z+,YL
				ST	Z,YH
				RJMP IS19
		IS15:
			CPI		word, 8					//Пишем символ лампы
			BRNE IS19
				LDI	YL,low(LampSymbol*2)
				LDI	YH,high(LampSymbol*2)
				INC	Zl
				ST	Z+,YL
				ST	Z,YH
		IS19:

			MOV 	ZL,YL
			MOV 	ZH,YH
			lpm 	word, Z
			SetScrinWord word
			RJMP IS10
			
IS03:
			LD 		word, -Z
			ROL		word
			ROL		word
			ROL		word
			ROL		word
			ANDI	word,0b111

			LDI 	YL,low(StringOutOne+1)
			LDI 	YH,high(StringOutOne+1)
			INC		YL

			LD		ZL,Y+
			LD		ZH,Y
			
			INC		word
			ADD		ZL,word
			BRCC	IS50
				INC Zh
			IS50:
			CPI		word,8					//Проверяем, а не последний ли символ мы пускаем в сеть?
			BRNE	IS20
				SetScrinKeyWord	ScrinKeyClear //Кодовое слово, мы закончили передавать этот символ
				RJMP IS10
			IS20:
			lpm 	word, Z
			SetScrinWord word
			RJMP IS10

IS05:			PUSH 	ZL
				PUSH 	ZH
				PUSH 	word
				PUSH 	Counter

				LDI 	YL,low(StringOutOne)
				LDI 	YH,high(StringOutOne)

				LDI 	ZL,low(ScrinStrOne*2)
				LDI 	ZH,high(ScrinStrOne*2)

				LDI 	Counter, StringOutSize
				LDI 	word, ' '
				IS99:						//Переписываем кодовое слово в память программы
					ST      Y+,word
					DEC		Counter
				BRNE 	IS99
				
				LDI 	YL,low(StringOutOne)
				LDI 	YH,high(StringOutOne)
				LDI 	word, 40
				MOV		Counter,word
				SUBI	Counter,40-35
				ST      Y+,Counter
				CLR		word
				ST      Y+,word
				ST      Y+,word
				
				IS00:						//Переписываем кодовое слово в память программы
					lpm 	word, Z+
					ST      Y+,word
					DEC		Counter
				BRNE 	IS00

				LDI 	YL,low(StringOutTwo)
				LDI 	YH,high(StringOutTwo)

				LDI 	ZL,low(ScrinStrTwo*2)
				LDI 	ZH,high(ScrinStrTwo*2)

				LDI 	Counter, StringOutSize
				LDI 	word, ' '
				IS98:						//Переписываем кодовое слово в память программы
					ST      Y+,word
					DEC		Counter
				BRNE 	IS98

				LDI 	YL,low(StringOutTwo)
				LDI 	YH,high(StringOutTwo)
				LDI 	word, 28
				MOV		Counter,word
				SUBI	Counter,28-14
				ST      Y+,Counter
				CLR		word
				ST      Y+,word
				ST      Y+,word
				
			
				IS000:						//Переписываем кодовое слово в память программы
					lpm 	word, Z+
					ST      Y+,word
					DEC		Counter
				BRNE 	IS000

				
				MOV	word,Flag_1
				//Flag_1 = 0b Lamp U Temp
				ANDI	word,0b1		//Флаг катушки
				BRNE	IS141
					//Если катушка выключена
					LDI 	YL,low(StringOutOne)
					LDI 	YH,high(StringOutOne)
					ADIW	Yl,40-6	//Тут символ катушки
					LDI		word,' '
					ST		Y,word
				IS141:

				MOV	word,Flag_1
				//Flag_1 = 0b Lamp U Temp
				ANDI	word,0b10		//Флаг напряжения
				BRNE	IS142
					//Если катушка выключена
					LDI 	YL,low(StringOutOne)
					LDI 	YH,high(StringOutOne)
					ADIW	Yl,40-4	//Тут символ катушки
					LDI		word,' '
					ST		Y,word
				IS142:



				pop 	Counter
				pop 	word
				pop 	ZH
				pop 	ZL

				SetTask TS_O				//А теперь можно начать писать символы на экран
				SetTimerTask TS_ScrinUpdate,10
				SetTimerTask TS_ClockUpdate,20	//Записываем время
				SetTimerTask TS_WetherInit	,30				//Запустили погоду
				SetTimerTask TS_EEPROMRead	,40				//Запустили всю инфу из памяти
				SetTimerTask TS_SCANMode	,50				//Запустили обновление экранчика
				SetTimerTask TS_SCANModeUp	,100				//Запустили обновление модов экранчика
				//SetTimerTask TS_SearchAllarm,40 --Этот лежит в часах, часы обновили - обновляем и будильники
				CALL NewSec					//Запустили часы

				RET

IS10:			SetTask TS_O
				SetTimerTask TS_Init_Scrin,1	*/			
RET

New_Init_Scrin:
			/*LDI 	ZL,low(WordOut)
			LDI 	ZH,high(WordOut)
			LDI		word,0
			ST		Z,word
			LDI 	ZL,low(StringOutOne)
			LDI 	ZH,high(StringOutOne)
			ST		Z,word
			SetTask	TS_Init_Scrin
			SetTimerTask TS_ScrinUpdate,0xFFFF //Мы откладываем обновление на максимум, чтобы оно нам не мешало
			//Flag_1 = 0b Key1 Key2 ~ MENU UART TWI Lamp U Temp
			ANDI	Flag_1,0b11011111//Сбрасываем флаг меню навсегда, дальше точно не оно*/

RET