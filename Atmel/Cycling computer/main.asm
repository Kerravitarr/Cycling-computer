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
			.def OSRG	 = r17
			.def tmp2	 = r18				;
			.def tmp3	 = r19				;
			.def tmp4	 = r20				; Некоторые переменные общего назначения
			.def interrupt= r21				; Регистр полностью под властью прерываний
			.def word	 = r22				; Переменная для работы со словами
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


			.equ 	TaskQueueSize	= 15			; Размер очереди событий
TaskQueue: 	.byte 	TaskQueueSize 					; Адрес очереди сотытий в SRAM

			.equ 	TimersPoolSize	= 15			; Количество таймеров
TimersPool:	.byte 	TimersPoolSize*3				; Адреса информации о таймерах

			.equ 	SlowTimersPoolSize	 = 4		; Количество таймеров
SlowTimersPool:	.byte 	SlowTimersPoolSize*2		; Адреса информации о таймерах

			.equ 	WordOutSize		 = 2			; Количество регистров вывода
WordOut:	.byte 	WordOutSize						; Адреса слов

			.equ 	StringOutSize	 = 43			; Память экрана

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
ACC_1:		.byte	8								;Аккумулятор. Первый
ACC_2:		.byte	8								;Аккумулятор. Второй
B_32:		.byte	4								;Регистр B
C_32:		.byte	4								;Регистр C
.equ BC_64			= B_32							;Регистровая пара BC
D_32:		.byte	4								;Регистр D
E_32:		.byte	4								;Регистр E
.equ DE_64			= D_32							;Регистровая пара DE
H_32:		.byte	4								;Регистр H
L_32:		.byte	4								;Регистр L
.equ HL_64			= H_32							;Регистровая пара HL

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
Temperature:.byte	1								;Температура
Pressure:	.byte	1								;Давление
Humidity:	.byte	1								;Влажность




Mode:		.byte 1									;Режим работы экрана
			.equ Mode1					= 0			; Загрузить в первую строчку строку 1, во вторую - 0
			.equ Mode2					= 1			; Загрузить в первую строчку строку 1, во вторую - 2
			.equ Mode3					= 2			; Загрузить в первую строчку строку 1, во вторую - 3
			.equ Mode4					= 3			; Загрузить в первую строчку строку 1, во вторую - 4
			.equ Mode5					= 4			; Загрузить в первую строчку строку 1, во вторую - 5
			.equ Mode6					= 5			; Загрузить в первую строчку строку 1, во вторую - 6
			.equ Mode7					= 6			; Загрузить в первую строчку строку 1, во вторую - 7
			.equ Mode8					= 7			; Загрузить в первую строчку строку 1, во вторую - 8
			.equ Mode50					= 8			; Загрузить в первую строчку строку 50, в третью - 51 - настройка спидометра 
			.equ Mode51					= 9			; Загрузить в первую строчку строку 50, в третью - 51 - настройка спидометра 

			
			.equ EEPROMWriteAdr			= (0b1010000<<1)|0	; //Адрес и бит квитирования для записи в EEPROM	0xA0
			.equ EEPROMReadAdr			= (0b1010000<<1)|1	; //Адрес и бит квитирования для чтения в EEPROM	0xA1
			.equ BME250Write			= (0b1110110<<1)|0	; //Адрес и бит квитирования для записи для датчика BME250
			.equ BME250Read				= (0b1110110<<1)|1	; //Адрес и бит квитирования для чтения для датчика BME250

			.equ ScrinKeyClear			= 0b1				;Команда очистки экрана

;===========CSEG==============================================================
			; Собственно код начинается отсюда
			.include "vectors.inc"
;=============================================================================
; Interrupts procs
			.include "Interrupts.inc"
; Математика длинных чисел. 4х байтных
			.include "math.inc"
; Тут обработка всей инфы ввода-вывода. Передача данных, приём и т.д.
			.include "DriverIO.inc"
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


;==================================================================================
; Init RTOS
;==================================================================================
			INIT_RTOS
;==================================================================================
; Инициализация периферии
;----------------------------------------------------------------------------------	
			SBI	DDRD,4						;Установили 4 ногу на выход		данные на выход
			CBI	DDRD,6						;Установили 5 ногу на вход		данные извне
			SBI	DDRD,6						;Установили 6 ногу на выход		обновление выхода
			SBI	DDRD,7						;Установили 7 ногу на выход		запоминание состояний входа
			SBI	DDRB,0						;Установили 8 ногу на выход		режим сна
			SBI	DDRB,1						;Установили 9 ногу на выход		запрет обновления регистра сравнения
			SBI	DDRB,5						;Установили 13 ногу на выход	строб

			//Инициализируем переменные в памяти
			RCALL InitMemory
			//Запускаем таймер на каждую мс
			SetTask TS_Evry_1_ms_Task
			//Запускаем таймер на каждые 40 мс
			SetTask TS_Evry_40_ms_Task
			//Запустить медленный таймер. Каждые 60 секунд, или 1 минуту один тик. Всего на 65535 минут таймер
			SetTimerTask TS_SlowTimerService,60000 

Background: ;RCALL Init_Scrin			//Инициализируем дисплей
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
	/*CLR_32 C_2
	LDI tmp2,0x03
	STS C_2+2,tmp2
	LDI tmp2,0xE8
	STS C_2+3,tmp2
	MOV_32 D_2,C_2
	MOV_32 B_2,C_2
	FloatToInt B_2
	IntToBCD B_2
	BTC_F B_2
	ADD_F D_2,C_2
	MOV_32 B_2,D_2
	FloatToInt B_2
	IntToBCD B_2
	BTC_F B_2
	SUB_F D_2,C_2
	MOV_32 B_2,D_2
	FloatToInt B_2
	IntToBCD B_2
	BTC_F B_2*/
RET

//Инициализирует всю память
InitMemory:
	_CLR_32 TimeTrip
	_CLR_32 TimeVOld
RET


WetherUpdate:
	/*//Flag_1 = 0b Key1 Key2 ~ ~ ~ TWI Lamp U Temp
	MOV		OSRG,Flag_1
	ANDI	OSRG,0b1000		//Флаг Занятости TWI
	BRNE	WU00
		TWI_IO_Init BME250Write,1
		LDI		OSRG,0xF7 //Первый регистр чтения
		ST		Z,OSRG
		SetTask TS_StartTWI
		//SetTimerTask TS_WetherUpdate,60000				//Запустили погодуБ раз в минуту обновление, выше крыши
		RET
	WU00:
	//Линия занята, обратитесь позже*
	SetTimerTask TS_WetherUpdate,1*/
RET

WetherInit:
	//Flag_1 = 0b Key1 Key2 ~ ~ ~ TWI Lamp U Temp
	/*MOV		OSRG,Flag_1
	ANDI	OSRG,0b1000		//Флаг Занятости TWI
	BRNE	WI00
		LDI 	ZL,low(TWI_IO+1)
		LDI 	ZH,high(TWI_IO+1)
		LD		OSRG,Z
		SBIW	Z,1
		CPI		OSRG,(0b1110110<<1)|1 //Если мы отправляли кодовое слово
		BRNE	WI01
			LDI		OSRG,3	//Регистр старта и два кодовых слова закидываем туда
			ST		Z+,OSRG
			LDI		OSRG,(0b1110110<<1)|0			/*В регистр данных TWDR загружаем адрес, а бит квитирования устанавливаем нулевым*//*
			ST		Z+,OSRG
			LDI		OSRG,0xF4		//Первый регистр - BME280_CTRL_ctrl_hum
			ST		Z+,OSRG
			LDI		OSRG,0x27		//регистр - BME280_CTRL_MEAS_REG
			ST		Z+,OSRG
			LDI		OSRG,0x00		//регистр - BME280_CONFIG_REG
			ST		Z+,OSRG
			SetTask TS_StartTWI
			SetTimerTask TS_WetherUpdate,1000				//Запустили погоду, раз в минуту обновление, выше крыши
			RET
		WI01:
		CPI		OSRG,(0b1110110<<1)|0 //Если мы отправляли кодовое слово
		BRNE	WI02
			RET
		WI02:

		LDI		OSRG,2	//Регистр старта и два кодовых слова закидываем туда
		ST		Z+,OSRG
		LDI		OSRG,(0b1110110<<1)|0			/*В регистр данных TWDR загружаем адрес, а бит квитирования устанавливаем нулевым*//*
		ST		Z+,OSRG
		LDI		OSRG,0xF2		//Первый регистр - BME280_CTRL_ctrl_hum
		ST		Z+,OSRG
		LDI		OSRG,0x01		//Первый регистр - BME280_CTRL_ctrl_hum
		ST		Z+,OSRG
		SetTask TS_StartTWI
	WI00:
	//Линия занята, обратитесь позже
	SetTimerTask TS_WetherInit,1*/
RET

ClockUpdate:
			/*//Flag_1 = 0b TWI Lamp U Temp
			MOV		OSRG,Flag_1
			ANDI	OSRG,0b1000		//Флаг Занятости TWI
			BRNE	CU00
				LDI 	ZL,low(TWI_IO)
				LDI 	ZH,high(TWI_IO)
				LDI		OSRG,1
				ST		Z+,OSRG
				LDI		OSRG,(0b1101000<<1)|0			/*В регистр данных TWDR загружаем адрес, а бит квитирования устанавливаем нулевым*//*
				ST		Z+,OSRG
				LDI		OSRG,0
				ST		Z,OSRG
				SetTask TS_StartTWI
				SetSlowTimerTask TS_ClockUpdate,60//Один раз в час мы сверяем часы с базовой станцией
				RET
			CU00:
			//Линия занята, обратитесь позже
			SetTimerTask TS_ClockUpdate,1*/
RET

;=============================================================================
; RTOS Here
;=============================================================================
.include "rtOS.inc"
;------------------------------------------------------------------------------

.CSEG 

//ConstTemp1:				.DB 0xC4, 0xF2, 0x32, 0x03  
//ConstTemp2:				.DB 0x24, 0x68, 0x89, 0x5e, 0x26, 0xea, 0xff, 0xff


DegreesCelsiusSymbol:	.DB	0b00000001, 0b00101100, 0b01010010, 0b01110000, 0b10010000, 0b10110010, 0b11001100, 0b11100000
InductorSymbol:			.DB	0b00000000, 0b00100000, 0b01001010, 0b01110101, 0b10010001, 0b10110001, 0b11010001, 0b11100000
//RadioSymbol:			.DB	0b00001000, 0b00100100, 0b01000010, 0b01111111, 0b10010011, 0b10110001, 0b11011111, 0b11100000
LampSymbol:				.DB	0b00000000, 0b00100000, 0b01000000, 0b01100100, 0b10000000, 0b10100000, 0b11000000, 0b11100000
SpeakerSymbol:			.DB	0b00000010, 0b00100110, 0b01011010, 0b01110010, 0b10011010, 0b10100110, 0b11000010, 0b11100000

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
