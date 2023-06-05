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
	LDI OSRG,(0 << ISC01) |	(1 << ISC00) | (1 << ISC11) |	(0 << ISC10) 		;Прерывание 0 по любому изменению и 1 по нисходящему фронут
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


/**
	Специальный отладочный марос. Его основная задача - вывести адреса переменных в память
*/
//Чисто печатает адрес
.MACRO PRINT_ALL_ADR_FUN
	//Непосредственно печатает адрес
	//Адрес в регистре Z
	CLR_32 B_32
	STS B_32 + 3, Zh
	_LDI_A 16
	_DIV_64_R32_DIV_A B_32
	LDS tmp2, ACC_2 + 7
	CPI tmp2, 10
	BRSH /*>=*/ PRINT_ADR_hex_mode_S
		ADI tmp2, '0'
		RJMP PRINT_ADR_hex_mode_end
	PRINT_ADR_hex_mode_S:
		ADI tmp2, 'A' - 10	
	PRINT_ADR_hex_mode_end:
	STD Y + 1, tmp2
	LDS tmp2, ACC_1 + 7
	CPI tmp2, 10
	BRSH /*>=*/ PRINT_ADR_hex_mode_F
		ADI tmp2, '0'
		RJMP PRINT_ADR_hex_mode_endF
	PRINT_ADR_hex_mode_F:
		ADI tmp2, 'A' - 10	
	PRINT_ADR_hex_mode_endF:
	STD Y + 0, tmp2
		
	STS B_32 + 3, Zl
	_LDI_A 16
	_DIV_64_R32_DIV_A B_32
	LDS tmp2, ACC_2 + 7
	CPI tmp2, 10
	BRSH /*>=*/ PRINT_ADR_hex_mode_S2
		ADI tmp2, '0'
		RJMP PRINT_ADR_hex_mode_end2
	PRINT_ADR_hex_mode_S2:
		ADI tmp2, 'A' - 10	
	PRINT_ADR_hex_mode_end2:
	STD Y + 3, tmp2
	LDS tmp2, ACC_1 + 7
	CPI tmp2, 10
	BRSH /*>=*/ PRINT_ADR_hex_mode_F2
		ADI tmp2, '0'
		RJMP PRINT_ADR_hex_mode_endF2
	PRINT_ADR_hex_mode_F2:
		ADI tmp2, 'A' - 10	
	PRINT_ADR_hex_mode_endF2:
	STD Y + 2, tmp2

	ADIW Yl, 8
.ENDMACRO
.MACRO PRINT_ADR
	LDIW Z,@0
	RCALL PRINT_ADR_F
.ENDMACRO