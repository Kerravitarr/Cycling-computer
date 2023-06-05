;
; Cycling computer.asm
;
; Created: 12.05.2019 20:55:42
; Author : Terran
;


; Replace with your application code
;= Start macro.inc ========================================
.LISTMAC //После директивы LISTMAC компилятор будет показывать в листинге содержимое макроса. 
		//По умолчанию в листинге показывается только вызов макроса и передаваемые параметры.
.LIST

		//http://radioaktiv.ru/custom_character_generator_for_hd44780.html - рисовалка символов для экрана

;===========Константы==============================================================
	.equ 	F_CPU = 16000000 	
	.equ 	baudrate = 9600  
	.equ 	bauddivider = F_CPU/(16*baudrate)-1

	//Инструкций в секунду
	.set 	SEC_PER_INSTR = 1.0/F_CPU
	//Инструкций в милисекунду
	.set 	M_SEC_PER_INSTR = 1000.0/F_CPU
	//Инструкций в микросекунду
	.set 	MK_SEC_PER_INSTR = 1000000/F_CPU
	// Настраиваем частоту SPI (в Гц).
	// Например, для ATtiny @ 128 кГц: в таблице данных указано, что тактовый импульс высокого и низкого уровня
	// SPI должен быть> 2 циклов ЦП, поэтому возьмите 3 цикла, т.е. разделите цель
	// f_cpu на 6:
	// #define SPI_CLOCK (128000/6)
	//
	// Частота, достаточно медленная для ATtiny85 @ 1 МГц, является разумным значением по умолчанию:
	// Частота в Герцах
	.equ SPI_SPEED = 1000000/6
	.equ SPI_DELAY = F_CPU/SPI_SPEED
;===========Макросы==============================================================
			.include "macro.inc"
;= End 	macro.inc =======================================
;============SSEG=============================================================
			.DSEG
			.def Counter2 = r2				; Счетчик (преимущественно используется для организации циклов)			;
			.def Al		 = r4				; Промежуточный регистр А			;
			.def Ah		 = r5				; Промежуточный регистр А		;
			.def Bl		 = r6				; Промежуточный регистр B			;
			.def Bh		 = r7				; Промежуточный регистр B		;	
			.def r8_	 = r8				;		
			.def r9_	 = r9				;		
			.def r10_	 = r10				;		
			.def r11_	 = r11				;		
			.def r12_	 = r12				;		
			.def r13_	 = r13				;		
			.def r14_	 = r14				;		
			.def r15_	 = r15				;			
			.def Counter = r16				; Счетчик (преимущественно используется для организации циклов)			;
			.def OSRG	 = r17				;По идее спецрегистр для RTOS... Но по факту просто пока не нужен этот регистр, вот его ни кто и не трогает
			.def tmp2	 = r18				;
			.def tmp3	 = r19				;
			.def tmp4	 = r20				; Некоторые переменные общего назначения
			.def interrupt= r21				; Регистр полностью под властью прерываний
			.def r22_	 = r22				; Переменная для работы со словами
			.def MacroR	 = r23				; Регистр для макросов
			.def r24_	 = r24				;
			.def Flag_1	 = r25				; Флаги регистра
			//.def Xl	 = r26				
			//.def Xh	 = r27				
			//.def Yl	 = r28			
			//.def Yh	 = r29				
			//.def Zl	 = r30			
			//.def Zh	 = r31				
;============SSEG=============================================================
			.DSEG

Flag_2:		.byte 1							;Флаги из памяти
			.equ 	TWI_Busy		= 0				; Флаг занятости устройства
			.equ 	EEPROM_write	= 1				; Флаг записи в EEPROM
			.equ 	isMove			= 2				; Флаг движения. Если тут 1 - мы куда-то едем
			.equ 	LSD_SCAN		= 3				; Флаг сканирования. Меняет на экране строки отображения по кругу
			.equ 	F_LSD_SCAN		= 4				; Флаг сканирования строки, чтобы менялись циферки автоматом
			.equ 	F_LSD_isBlink	= 5				; При меню, показывает - мигать курсором или нет


			.equ 	TaskQueueSize	= 30			; Размер очереди событий
TaskQueue: 	.byte 	TaskQueueSize 					; Адрес очереди сотытий в SRAM

			.equ 	TimersPoolSize	= 20			; Количество таймеров
TimersPool:	.byte 	TimersPoolSize*3				; Адреса информации о таймерах

			.equ 	SlowTimersPoolSize	 = 4		; Количество таймеров
SlowTimersPool:	.byte 	SlowTimersPoolSize*2		; Адреса информации о таймерах

			.equ 	WordOutSize		 = 2			; Количество регистров вывода
WordOut:	.byte 	WordOutSize						; Адреса слов
			//.equ	LSD_D			 = 0-7			; Первый байт - D0-D7 вход экрана
			.equ	O_LSD_E			 = 8			; Вход Е экрана. При переходе Е с высокого лог. уровня на низкий данные, которые уже «висят» на выводах DB0..DB7, записываются в память
			//.equ	O_LSD_RW		 = -			; Вход RW экрана. R/W определяет направление работы выводов DB0..DB7 – если на R/W «0», то мы можем только писать в порт DB, а если R/W = «1», то можем прочитать с него 
			.equ	O_LSD_RS		 = 9			; Вход RS экрана. При высоком лог. уровне на R/S(Register Select) LCS воспринимает этот набор битов как данные(код символа), а при низком – как инструкцию и направляет их в соответствующий регистр.

			.equ	LSD_ROW_LENGHT = 20;			; Длина строки экрана
LSD_ROW_P:	.byte 	1								; Позиция передачи
LSD_ROW_1:	.byte 	LSD_ROW_LENGHT					; Строка экрана
LSD_ROW_2:	.byte 	LSD_ROW_LENGHT					; Строка экрана
LSD_ROW_3:	.byte 	LSD_ROW_LENGHT					; Строка экрана
LSD_ROW_4:	.byte 	LSD_ROW_LENGHT					; Строка экрана
					;А эти переменные указывают откуда можно писать дополнительные параметры на экран
			.equ	LSD_ROW_1_PAR = LSD_ROW_1 + LSD_ROW_LENGHT / 2
			.equ	LSD_ROW_2_PAR = LSD_ROW_2 + LSD_ROW_LENGHT / 2
			.equ	LSD_ROW_3_PAR = LSD_ROW_3 + LSD_ROW_LENGHT / 2
			.equ	LSD_ROW_4_PAR = LSD_ROW_4 + LSD_ROW_LENGHT / 2

			.equ 	WordInSize			= 1			; Количество регистров ввода
WordIn:		.byte 	WordInSize*2					; Адреса слов


			.equ 	TWISize			 = 20			; Количество символов которые мы ожидаем принять по I2C
TWI_IO:		.byte 	TWISize							;
TWI_IOc:	.byte 	1								;Число переданных символов
TWI_IOl:	.byte 	1								;Ожидается символов


			.equ 	UARTSize		 = 40			; Количество символов которые мы ожидаем выкинуть
UART_O_buf:	.byte 	UARTSize						; Cообщение по UART
UART_O_head:.byte 	1								; Голова буфера
UART_O_tail:.byte 	1								; Хвост буфера
UART_I:		.byte 	1								; Cообщение по UART,которое мы примем - всего один символ ожидается

			.equ 	MenuPCBSSize	 = 5			; Запас для меню, первые и вторые символы
MenuPCBS:	.byte 	MenuPCBSSize*2

			.equ 	EEPROMStrSize	= 1				;
EEPROMStr:	.byte 	EEPROMStrSize					;Память аппарата
EEPROMStr_c:.byte 1

//Блок памяти для длинной арифметики
ACC_2:		.byte	8								;Аккумулятор. Второй. Используется только в инструкции DIV. В него помещается остаток от деления, если вдруг нужены
													;Но есть у него и неписанная функция.
													;Для операции MUL он служит буфером, в котором будет мусор.
													;Дело в том, что при умножении 64*64 бита результат может быть 128 бит. Вот ещё 8 байт и будут эти. 
ACC_1:		.byte	8								;Аккумулятор. Первый
.equ ACC_32			= ACC_1 + 4						;Указатель на 32х разрядный аккумулятор, его кусочек
B_32:		.byte	4								;Регистр B
C_32:		.byte	4								;Регистр C
.equ BC_64			= B_32							;Регистровая пара BC
D_32:		.byte	4								;Регистр D
E_32:		.byte	4								;Регистр E
.equ DE_64			= D_32							;Регистровая пара DE
H_32:		.byte	4								;Регистр H
L_32:		.byte	4								;Регистр L
.equ HL_64			= H_32							;Регистровая пара HL
TMP_64:		.byte	8								;Специальный временный регистр, используется функцией DIV и MUL, использовать самостоятельно не рекомендуется



WheelLength:.byte 	4								; Длина окружности колеса , мм
TimeOdo:	.byte 	4								; Всё время всех поездок, в мс
Odometr:	.byte 	4								; Весь пройденный путь всех поездок в оборотах колеса
MaxSpeedOdo:.byte 	4								; Максимальная скорость за всё время оборотв/минуту
TimeTrip:	.byte 	4								; Время текущей поездки, мс
TimeVOld:	.byte 	4								; Время от предыдущего сигнала колеса, мс ((Нужно для определения мгновенной скорости))
Dist:		.byte 	4								; Пройденный путь в оборотах колеса
Speed:		.byte 	4								; Скорость оборотв/минуту
AverageSpeed:.byte 	4								; Средняя скорость оборотв/минуту
MaxSpeed:	.byte 	4								; Максимальная скорость оборотв/минуту
Kaden:		.byte 	4								; Каденс
KadenMax:	.byte 	4								; Каденс максимальный
KadenAverage:.byte 	4								; Каденс средний
LeftTimeDist:.byte 	4								; Оставшееся время (мс) или расстояние (мм)
Degrees:	.byte 	1								; Угол подъёма, причём +-!
Height:		.byte	2								;Высота, м
Clock:		.byte 	8								;Тут у нас храняться часы. HH:MM:SS
AccLevel:	.byte	1								;Уровень заряда батарее, число от 0 до 100
Temperature:.byte	2								;Температура in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23
Pressure:	.byte	2								;Давление в мм рт столба
Humidity:	.byte	1								;Влажность в %

//Параметры датчика влажности
BME_dig_T1u16: .byte	2							;
BME_dig_T2s16: .byte	2							;
BME_dig_T3s16: .byte	2							;
BME_dig_P1u16: .byte	2							;
BME_dig_P2s16: .byte	2							;
BME_dig_P3s16: .byte	2							;
BME_dig_P4s16: .byte	2							;
BME_dig_P5s16: .byte	2							;
BME_dig_P6s16: .byte	2							;
BME_dig_P7s16: .byte	2							;
BME_dig_P8s16: .byte	2							;
BME_dig_P9s16: .byte	2							;
BME_dig_H1u8:  .byte	1							;
BME_dig_H2s16: .byte	2							;
BME_dig_H3u8:  .byte	1							;
BME_dig_H4s16: .byte	2							;
BME_dig_H5s16: .byte	2							;
BME_dig_H6s8:  .byte	1							;
T_fine:		   .byte	4							;Вспомогательный параметр, на основе которого вводится коррекция данных от температуры


LSD_mode:	.byte	1								;В каком режиме находится экран
			.equ LSD_M_KEY_W_1			= 0			;Кодовое слово
			.equ LSD_M_KEY_W_2			= 1			;Кодовое слово
			.equ LSD_M_KEY_W_3			= 2			;Кодовое слово
			.equ LSD_M_KEY_W_4			= 3			;Кодовое слово
			.equ LSD_M_INIT				= 4			;Инициализируем дисплей
			.equ LSD_M_CLEAR			= 5			;Очищаем его
			.equ LSD_M_START			= 6			;Команда запуска дисплея
			.equ LSD_M_KEY_WS1			= 15		;Команда записи на экран символа 1
			.equ LSD_M_KEY_WS2			= 16		;Команда записи на экран символа 2
			.equ LSD_M_KEY_WS3			= 17		;Команда записи на экран символа 3
			.equ LSD_M_KEY_WS4			= 18		;Команда записи на экран символа 4
			.equ LSD_M_KEY_WS5			= 19		;Команда записи на экран символа 5
			.equ LSD_M_KEY_WS6			= 20		;Команда записи на экран символа 6
			.equ LSD_M_KEY_WS7			= 21		;Команда записи на экран символа 7
			.equ LSD_M_KEY_WS8			= 22		;Команда записи на экран символа 8

			.equ LSD_M_S1R				= 7			;Передача первой строки
			.equ LSD_M_S2R				= 8			;Передача второй строки
			.equ LSD_M_S3R				= 9			;Передача третьей строки
			.equ LSD_M_S4R				= 10		;Передача четвёртой строки
			.equ LSD_M_U1RF3			= 11		;Обновляем скорость первой строки, когда до этого обновляли третью
			.equ LSD_M_U2RF3			= 12		;Обновляем скорость второй строки, когда до этого обновляли третью
			.equ LSD_M_U1RF4			= 13		;Обновляем скорость первой строки, когда до этого обновляли четвёртую
			.equ LSD_M_U2RF4			= 14		;Обновляем скорость второй строки, когда до этого обновляли четвёртую

			.equ LSD_M_READY			= 255		;Теперь Дисплей готов к работе

LCD_cursor_pos:.byte 1								;Позиция курсора на дисплее 
LCD_display_mode:.byte 1							;Режим отображения экрана
			.equ DM_DEFAULT				= 0			;Базовый: 
			.equ DM_AVR_V				= 0				//средяня V
			.equ DM_nowT				= 1				//время
			.equ DM_T					= 2				//температура
			.equ DM_P					= 3				//Давление
			.equ DM_H					= 4				//Влажность
			.equ DM_maxV				= 5				//Максимальная скорость
			.equ DM_odo					= 6				//Весь пройденый путь за всё время
			.equ DM_SCAN				= 7			;Сколько у нас всего строк показывает это число
			.equ DM_MENU				= 8			;Пункты меню:
			.equ DM_MENU_RESTART		= 8			;	- обновить поездку
			.equ DM_MENU_TIME_TRIP		= 9			;	- Настроить время поездки
			.equ DM_MENU_WheelLength	= 10		;	- Длина колеса
			.equ DM_MENU_TST			= 11		;	- Показать все символы экрана
			.equ DM_MENU_EXIT			= 12		;	- Выход га верхний уровень

			
			.equ EEPROMWriteAdr			= (0b1010000<<1)|0	; //Адрес и бит квитирования для записи в EEPROM	0xA0
			.equ EEPROMReadAdr			= (0b1010000<<1)|1	; //Адрес и бит квитирования для чтения в EEPROM	0xA1
			.equ BME250Write			= (0x76<<1)|0		; //Адрес и бит квитирования для записи для датчика BME250
			.equ BME250Read				= (0x76<<1)|1		; //Адрес и бит квитирования для чтения для датчика BME250
			.equ DS1307Write			= (0b1101000<<1)|0	;Модуль часов
			.equ DS1307Read				= (0b1101000<<1)|1	;

;===========CSEG==============================================================
			; Собственно код начинается отсюда
			.include "vectors.inc"
;=============================================================================
; ОС реального времени
			.include "rtOS.inc"
; Interrupts procs
			.include "Interrupts.inc"
; Математика длинных чисел. 4х байтных
			.include "math.inc"
; Тут обработка всей инфы ввода-вывода. Передача данных, приём и т.д.
			.include "DriverIO.inc"
; Клавиатура, всё, что с ней связано
			.include "Keyboard.inc"
; Драйвер LCD экрана
			.include "LCD_driver.inc"
; Повторяющиеся события
			.include "TimeRepeat.inc"
; Init procs
			.include "init.inc"
;=============================================================================
; Main code
;=============================================================================
Reset:		OUTI 	SPL,low(RAMEND) 		; Первым делом инициализируем стек
			OUTI	SPH,High(RAMEND)								

; Очистка памяти
RAM_Flush:	LDI		ZL,Low(SRAM_START)
			LDI		ZH,High(SRAM_START)
			CLR		R16
Flush:		ST 		Z+,R16
			CPI		ZH,High(RAMEND)
			BRNE	Flush

			CPI		ZL,Low(RAMEND)
			BRNE	Flush

			LDI	ZL, 30		; Адрес самого старшего регистра	
			CLR	ZH			; А тут у нас будет ноль
			DEC	ZL			; Уменьшая адрес
			ST	Z, ZH		; Записываем в регистр 0
			BRNE	PC-2	; Пока не перебрали все не успокоились
			
			//Отладочный скрипт!!!
			/*LDIW Y, Flag_2

			PRINT_ADR Flag_2
			PRINT_ADR WordIn
				PRINT_ADR ACC_1
				PRINT_ADR ACC_2
			PRINT_ADR ACC_32
			PRINT_ADR B_32
				PRINT_ADR C_32
				PRINT_ADR BC_64
			PRINT_ADR D_32
			PRINT_ADR E_32
				PRINT_ADR DE_64
				PRINT_ADR WheelLength
			PRINT_ADR TimeOdo
			PRINT_ADR Odometr
				PRINT_ADR MaxSpeedOdo
				PRINT_ADR TimeTrip
			PRINT_ADR TimeVOld
			PRINT_ADR Dist
				PRINT_ADR Speed
				PRINT_ADR AverageSpeed
			PRINT_ADR MaxSpeed
			PRINT_ADR Kaden
				PRINT_ADR KadenMax
				PRINT_ADR KadenAverage
			PRINT_ADR LeftTimeDist
			PRINT_ADR Degrees
				PRINT_ADR Height
				PRINT_ADR Clock
			PRINT_ADR AccLevel
			PRINT_ADR Temperature
				PRINT_ADR Pressure
				PRINT_ADR Humidity
			
			JMP Reset
			PRINT_ADR_F: 
				PRINT_ALL_ADR_FUN
			RET/**/

;==================================================================================
; Init RTOS
;==================================================================================
			INIT_RTOS
;==================================================================================
; Инициализация периферии
;----------------------------------------------------------------------------------	
			SBI	DDRD,4						;Установили 4 ногу на выход		данные на выход
			CBI	DDRD,5						;Установили 5 ногу на вход		данные извне
			SBI	DDRD,6						;Установили 6 ногу на выход		обновление выхода
			SBI	DDRD,7						;Установили 7 ногу на выход		запоминание состояний входа
			SBI	DDRB,0						;Установили 8 ногу на выход		режим сна
			SBI	DDRB,5						;Установили 13 ногу на выход	строб для микросхемы выхода
			SBI	DDRB,4						;Установили 12 ногу на выход	строб для микросхемы входа
			CBI	DDRD,3						;Установили 3 ногу на вход		геркон

			

			//Инициализируем переменные в памяти
			RCALL InitMemory
			//Запускаем таймер на каждую мс
			SetTask TS_Evry_1_ms_Task
			//Запускаем таймер на каждые 40 мс
			SetTask TS_Evry_40_ms_Task
			//Запускаем таймер на каждую секунду
			SetTask TS_Evry_1_s_Task
			//Запустить медленный таймер. Каждые 60 секунд, или 1 минуту один тик. Всего на 65535 минут таймер
			SetTimerTask TS_SlowTimerService,60000 
			//Запускаем обновление первой строки дисплея
			SetTimerTask TS_Update_1_row_LCD,100	
			//Запустить обновление часов
			SetTask TS_ClockUpdate
			//Записать часы в их место
			STI Clock+0, '1'
			STI Clock+1, '2'
			STI Clock+2, ':'
			STI Clock+3, '0'
			STI Clock+4, '0'
			STI Clock+5, ':'
			STI Clock+6, '0'
			STI Clock+7, '0'
			//Запустить обновление погоды
			SetTask TS_WetherInit

			_LDI_32 WheelLength, 2000

Background: SetTask TS_Init_Scrin		//Инициализируем дисплей
			SetTimerTask TS_LCD_update,1000	//Через секунду начнём обновлять данные
			;CALL LedPower				//Запустили мигалку
			;CALL I						//Запомнили что у нас сейчас творится на переферии	

			OUTI 	SPL,low(RAMEND) 		; Ещё раз инициализируем стек, если мы сюда перешли как к задаче, то мы удаляем нахрен все переходы и начинаем с начала
			OUTI	SPH,High(RAMEND)	
;==============================================================================
;Main Code Here

Main:		
			SEI								; Разрешаем прерывания.
			wdr								; Reset Watch DOG (Если не "погладить" "собаку". то она устроит конец света в виде reset для процессора)
			CALL 	ProcessTaskQueue		; Обработка очереди процессов
			RCALL 	Idle					; Простой Ядра	
			LDS		tmp2,TaskQueue			// Смотрим на первую задачу, есть она вообще?	
			CPI		tmp2,0xFF
			BRNE	Main //Задачи ещё есть?
				//Пусто, можно и вздремнуть
				SLEEP
				WDRM: rjmp 	Main					; Основной цикл микроядра РТОС	
;=============================================================================
;Taskss
;=============================================================================
Idle:		
	LDI_A 65531
	LDI_32 B_32, 25
	MUL64_A_R32 B_32
RET

//Инициализирует всю память
InitMemory:
	_CLR_32 TimeTrip
	_CLR_32 TimeVOld
RET

.CSEG 

//ConstTemp1:				.DB 0xC4, 0xF2, 0x32, 0x03  
//ConstTemp2:				.DB 0x24, 0x68, 0x89, 0x5e, 0x26, 0xea, 0xff, 0xff


DegreesCelsiusSymbol:	.DB	0b00000001, 0b00101100, 0b01010010, 0b01110000, 0b10010000, 0b10110010, 0b11001100, 0b11100000
InductorSymbol:			.DB	0b00000000, 0b00100000, 0b01001010, 0b01110101, 0b10010001, 0b10110001, 0b11010001, 0b11100000
//RadioSymbol:			.DB	0b00001000, 0b00100100, 0b01000010, 0b01111111, 0b10010011, 0b10110001, 0b11011111, 0b11100000
LampSymbol:				.DB	0b00000000, 0b00100000, 0b01000000, 0b01100100, 0b10000000, 0b10100000, 0b11000000, 0b11100000
SpeakerSymbol:			.DB	0b00000010, 0b00100110, 0b01011010, 0b01110010, 0b10011010, 0b10100110, 0b11000010, 0b11100000

//Символы больших цифр для экрана:
LCD_BIG_NUMBER_0:		.DB 0b11111,0b11111,0b00011,0b00011,0b00011,0b00011,0b00011,0b00011	// ┒
LCD_BIG_NUMBER_1:		.DB 0b00011,0b00011,0b00011,0b00011,0b00011,0b00011,0b11111,0b11111 // _|
LCD_BIG_NUMBER_2:		.DB 0b11111,0b11111,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000 // ▔
LCD_BIG_NUMBER_3:		.DB 0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b11111,0b11111 //_
LCD_BIG_NUMBER_4:		.DB 0b11111,0b11111,0b00011,0b00011,0b00011,0b00011,0b11111,0b11111	//ɔ
LCD_BIG_NUMBER_5:		.DB 0b11111,0b11111,0b11000,0b11000,0b11000,0b11000,0b11111,0b11111	//с
LCD_BIG_NUMBER_6:		.DB 0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11111,0b11111	// |_
LCD_BIG_NUMBER_7:		.DB 0b11111,0b11111,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000 // Г

//Данные для подписей строк. Обязательно кончаются 0х00!!!!
LCD_MAX_V:				.DB "MAX:",0x00
LCD_ODO:				.DB "ODO",0x00



/*
	\0x01 - градусы цельсия
	\0x02 - катушка
	\0x03 - Динамик
	\0x04 - точка

*/
Week:		.DB 0xA8,0xBD,"B",0xBF,"Cp",0xAB,0xBF,0xA8,0xBF,"C",0xB2, "Bc","Of"
Month:		.DB 0xB1,0xBD,0xB3,0xAA, "e",0xB3,"MapA",0xBE,"p","Ma",0xB9,0xA5,0xC6,0xBD,0xA5,0xC6,0xBB,"A",0xB3,0xB_2,"Ce",0xBD,"O",0xBA,0xBF,"Ho",0xC7,0xE0,"e",0xBA

//			/*N*/		L	F	C	D	U	Lenght	Perrent Child Down	Up	длина строки,родитель, сын, брат вниз, брат вверх, ...
//Menu1:		/*1*/	.DB 8,	0,	6,	2,	5,		"I Allarm"


/*
Menu:		.dw Menu1					; [01] 
			.dw Menu2					; [02] 
			.dw Menu3					; [03] 
			.dw Menu4					; [04] 
			.dw Menu5					; [05] 
			.dw Menu1_1					; [06] 
			.dw Menu1_2					; [07] 
			.dw Menu2_1					; [08] 
			.dw Menu2_2					; [09] 
			.dw Menu3_1					; [10] 
			.dw Menu3_2					; [11] 
			.dw Menu5_1					; [12]
			.dw Menu1_1_1				; [13]
			.dw Menu1_1_2				; [14]
			.dw Menu1_1_3				; [15]
			.dw Menu1_1_4				; [16]
			.dw Menu1_1_5				; [17]
			.dw Menu1_1_4_1				; [18]
			.dw Menu4_1					; [19]
			.dw Menu4_2					; [20]
			.dw Menu4_3					; [21]
			.dw Menu4_1_1				; [22]
			.dw Menu4_1_2				; [23]
			.dw Menu4_1_3				; [24]
			.dw Menu4_1_2_1				; [25]
			.dw Menu4_2_1				; [26]
			.dw Menu4_2_2				; [27]
			.dw Menu4_2_3				; [28]
			.dw Menu4_2_2_1				; [29]
			*/

 .include "RTos/TaskManager.inc"
