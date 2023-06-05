/*
 * LSD_driver.inc
 *	Тут всё, что касается экрана.
	Самое главное - не забыть вызвать инициализацию экрана прежде чем всё остальное делать :)
 *  Created: 28.04.2023 22:02:58
 *   Author: ????
 */ 
 //Сохраняет кодовое слово на экранею. Один параметр - слово
.MACRO	SetScrinKeyWord
		LDI		MacroR,@0
		CALL SetScrinKeyWord_M
.ENDMACRO
//Сохраняет символ на экране. Один параметр - символ
.MACRO	SetScrinWord
		MOV		MacroR,@0
		CALL SetScrinWord_M
.ENDMACRO

// commands
.equ LCD_CLEARDISPLAY	= 0x01
.equ LCD_RETURNHOME		= 0x02
.equ LCD_ENTRYMODESET	= 0x04
.equ LCD_DISPLAYCONTROL = 0x08
.equ LCD_CURSORSHIFT	= 0x10
.equ LCD_FUNCTIONSET	= 0x20
.equ LCD_SETCGRAMADDR	= 0x40
.equ LCD_SETDDRAMADDR	= 0x80

// flags for display entry mode
.equ LCD_ENTRYRIGHT		= 0x00
.equ LCD_ENTRYLEFT		= 0x02
.equ LCD_ENTRYSHIFTINCREMENT = 0x01
.equ LCD_ENTRYSHIFTDECREMENT = 0x00

// flags for display on/off control
.equ LCD_DISPLAYON		= 0x04
.equ LCD_DISPLAYOFF		= 0x00
.equ LCD_CURSORON		= 0x02
.equ LCD_CURSOROFF		= 0x00
.equ LCD_BLINKON		= 0x01
.equ LCD_BLINKOFF		= 0x00

// flags for display/cursor shift
.equ LCD_DISPLAYMOVE	= 0x08
.equ LCD_CURSORMOVE		= 0x00
.equ LCD_MOVERIGHT		= 0x04
.equ LCD_MOVELEFT		= 0x00

// flags for function set
.equ LCD_8BITMODE		= 0x10
.equ LCD_4BITMODE		= 0x00
.equ LCD_2LINE			= 0x08
.equ LCD_1LINE			= 0x00
.equ LCD_5x10DOTS		= 0x04
.equ LCD_5x8DOTS		= 0x00

//Адреса строк в памяти экрана
.equ LCD_ROW1_ADR		= 0x80
.equ LCD_ROW2_ADR		= 0xC0
.equ LCD_ROW3_ADR		= 0x94
.equ LCD_ROW4_ADR		= 0xD4

.equ LCD_KEY_WORD		= LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS
 
Init_Scrin:	
	TST_OPin O_LSD_E
	BREQ /*=*/ Init_Scrin_no_strib
		//У нас строб
		COPin O_LSD_E
		SetTask TS_O
		SetTask TS_Init_Scrin
		RET
	Init_Scrin_no_strib:
	LDS tmp2, LSD_mode
	CPI tmp2, LSD_M_KEY_W_1
	BRNE /*!=*/ LCD_init_notKW1
		SetScrinKeyWord LCD_KEY_WORD
		STI LSD_mode, LSD_M_KEY_W_2
		RJMP Init_Scrin_end
	LCD_init_notKW1:
	CPI tmp2, LSD_M_KEY_W_2
	BRNE /*!=*/ LCD_init_notKW2
		SetScrinKeyWord LCD_KEY_WORD
		STI LSD_mode, LSD_M_KEY_W_3
		RJMP Init_Scrin_end
	LCD_init_notKW2:
	CPI tmp2, LSD_M_KEY_W_3
	BRNE /*!=*/ LCD_init_notKW3
		SetScrinKeyWord LCD_KEY_WORD
		STI LSD_mode, LSD_M_KEY_W_4
		RJMP Init_Scrin_end
	LCD_init_notKW3:
	CPI tmp2, LSD_M_KEY_W_4
	BRNE /*!=*/ LCD_init_notKW4
		SetScrinKeyWord LCD_KEY_WORD
		STI LSD_mode, LSD_M_INIT
		RJMP Init_Scrin_end
	LCD_init_notKW4:
	CPI tmp2, LSD_M_INIT
	BRNE /*!=*/ LCD_init_notI
		SetScrinKeyWord LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF
		STI LSD_mode, LSD_M_CLEAR
		RJMP Init_Scrin_end
	LCD_init_notI:
	CPI tmp2, LSD_M_CLEAR
	BRNE /*!=*/ LCD_init_notCLR
		SetScrinKeyWord LCD_CLEARDISPLAY
		STI LSD_mode, LSD_M_START
		RJMP Init_Scrin_end
	LCD_init_notCLR:
	CPI tmp2, LSD_M_START
	BRNE /*!=*/ LCD_init_not_start
	LCD_init_start:
		SetScrinKeyWord LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT
		STI LSD_mode, LSD_M_KEY_WS1
		STI LSD_ROW_P, 0
		RJMP Init_Scrin_end
	LCD_init_not_start:
	CPI tmp2, LSD_M_KEY_WS1
	BRNE /*!=*/ LCD_init_not_key_ws1
		LDI tmp3, (0 << 3)
		LDIW Z, LCD_BIG_NUMBER_0*2
		RCALL LCD_INIT_WRITE_CHAR
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_init_key_ws1_not_end
			//Закончили с этим режимом
			STI LSD_mode, LSD_M_KEY_WS2
			RJMP Init_Scrin_end
		LCD_init_key_ws1_not_end:
		RJMP Init_Scrin_end
	LCD_init_not_key_ws1:
	CPI tmp2, LSD_M_KEY_WS2
	BRNE /*!=*/ LCD_init_not_key_ws2
		LDI tmp3, (1 << 3)
		LDIW Z, LCD_BIG_NUMBER_1*2
		RCALL LCD_INIT_WRITE_CHAR
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_init_key_ws2_not_end
			//Закончили с этим режимом
			STI LSD_mode, LSD_M_KEY_WS3
			RJMP Init_Scrin_end
		LCD_init_key_ws2_not_end:
		RJMP Init_Scrin_end
	LCD_init_not_key_ws2:
	CPI tmp2, LSD_M_KEY_WS3
	BRNE /*!=*/ LCD_init_not_key_ws3
		LDI tmp3, (2 << 3)
		LDIW Z, LCD_BIG_NUMBER_2*2
		RCALL LCD_INIT_WRITE_CHAR
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_init_key_ws3_not_end
			//Закончили с этим режимом
			STI LSD_mode, LSD_M_KEY_WS4
			RJMP Init_Scrin_end
		LCD_init_key_ws3_not_end:
		RJMP Init_Scrin_end
	LCD_init_not_key_ws3:
	CPI tmp2, LSD_M_KEY_WS4
	BRNE /*!=*/ LCD_init_not_key_ws4
		LDI tmp3, (3 << 3)
		LDIW Z, LCD_BIG_NUMBER_3*2
		RCALL LCD_INIT_WRITE_CHAR
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_init_key_ws4_not_end
			//Закончили с этим режимом
			STI LSD_mode, LSD_M_KEY_WS5
			RJMP Init_Scrin_end
		LCD_init_key_ws4_not_end:
		RJMP Init_Scrin_end
	LCD_init_not_key_ws4:
	CPI tmp2, LSD_M_KEY_WS5
	BRNE /*!=*/ LCD_init_not_key_ws5
		LDI tmp3, (4 << 3)
		LDIW Z, LCD_BIG_NUMBER_4*2
		RCALL LCD_INIT_WRITE_CHAR
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_init_key_ws5_not_end
			//Закончили с этим режимом
			STI LSD_mode, LSD_M_KEY_WS6
			RJMP Init_Scrin_end
		LCD_init_key_ws5_not_end:
		RJMP Init_Scrin_end
	LCD_init_not_key_ws5:
	CPI tmp2, LSD_M_KEY_WS6
	BRNE /*!=*/ LCD_init_not_key_ws6
		LDI tmp3, (5 << 3)
		LDIW Z, LCD_BIG_NUMBER_5*2
		RCALL LCD_INIT_WRITE_CHAR
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_init_key_ws6_not_end
			//Закончили с этим режимом
			STI LSD_mode, LSD_M_KEY_WS7
			RJMP Init_Scrin_end
		LCD_init_key_ws6_not_end:
		RJMP Init_Scrin_end
	LCD_init_not_key_ws6:
	CPI tmp2, LSD_M_KEY_WS7
	BRNE /*!=*/ LCD_init_not_key_ws7
		LDI tmp3, (6 << 3)
		LDIW Z, LCD_BIG_NUMBER_6*2
		RCALL LCD_INIT_WRITE_CHAR
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_init_key_ws7_not_end
			//Закончили с этим режимом
			STI LSD_mode, LSD_M_KEY_WS8
			RJMP Init_Scrin_end
		LCD_init_key_ws7_not_end:
		RJMP Init_Scrin_end
	LCD_init_not_key_ws7:
	CPI tmp2, LSD_M_KEY_WS8
	BREQ /*=*/ LCD_init_key_ws8 
		RJMP LCD_init_not_key_ws8
	LCD_init_key_ws8:
		LDI tmp3, (7 << 3)
		LDIW Z, LCD_BIG_NUMBER_7*2
		RCALL LCD_INIT_WRITE_CHAR
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_init_key_ws8_not_end
			//Закончили с этим режимом
			STI LSD_mode, LSD_M_READY
			//Очищаем экран
			LDI tmp2, ' '
			LDI Counter, LSD_ROW_LENGHT
			LDIW Z, LSD_ROW_1
			LCD_init_ready_clean_rw1:
				ST Z+, tmp2
				DEC Counter
			BRNE /*!=*/ LCD_init_ready_clean_rw1
			LDI Counter, LSD_ROW_LENGHT
			LDIW Z, LSD_ROW_2
			LCD_init_ready_clean_rw2:
				ST Z+, tmp2
				DEC Counter
			BRNE /*!=*/ LCD_init_ready_clean_rw2
			LDI Counter, LSD_ROW_LENGHT
			LDIW Z, LSD_ROW_3
			LCD_init_ready_clean_rw3:
				ST Z+, tmp2
				DEC Counter
			BRNE /*!=*/ LCD_init_ready_clean_rw3
			LDI Counter, LSD_ROW_LENGHT
			LDIW Z, LSD_ROW_4
			LCD_init_ready_clean_rw4:
				ST Z+, tmp2
				DEC Counter
			BRNE /*!=*/ LCD_init_ready_clean_rw4
			//Загружаем символы скорости. Они генятся при движении, так что надо загрузить заранее
			//Цифра 0
			STI LSD_ROW_1 + 0, 0x7
			STI LSD_ROW_1 + 1, 0x0
			STI LSD_ROW_2 + 0, 0x6
			STI LSD_ROW_2 + 1, 0x1
			//Цифра 0
			STI LSD_ROW_1 + 2, 0x7
			STI LSD_ROW_1 + 3, 0x0
			STI LSD_ROW_2 + 2, 0x6
			STI LSD_ROW_2 + 3, 0x1
			
			STI LSD_ROW_2 + 4, '0'
			RJMP Init_Scrin_end_ready
		LCD_init_key_ws8_not_end:
		RJMP Init_Scrin_end
	LCD_init_not_key_ws8:


	Init_Scrin_end: 
		SetTimerTask TS_Init_Scrin,5
	Init_Scrin_end_ready:
RET

//Пишет символы в память экрана:
//tmp3 - номер символа в сдвинутом виде. Точнее это выглядит как (location << 3), где location = [0,7]
//Z - регистр с указанием на место
LCD_INIT_WRITE_CHAR:
	LDS tmp2, LSD_ROW_P
	TST tmp2
	BRNE /*!=*/ LCD_INIT_WRITE_CHAR_write
		//Надо указать куда пишем
		//SetScrinKeyWord LCD_SETCGRAMADDR | (location << 3)
		MOV		MacroR,tmp3
		ORI		MacroR, LCD_SETCGRAMADDR
		CALL SetScrinKeyWord_M
		STI LSD_ROW_P, 1
		RJMP Init_Scrin_end
	LCD_INIT_WRITE_CHAR_write:
	DEC tmp2
	CLR tmp3
	//Смещаем
	ADD ZL,tmp2
	ADC Zh,tmp3
	LPM tmp3, Z
	SetScrinWord tmp3
	INC tmp2
	CPI tmp2, 8
	BRNE /*!=*/LCD_INIT_WRITE_CHAR_next
		//Передали все 8 байт
		STI LSD_ROW_P, 0
		RET
	LCD_INIT_WRITE_CHAR_next:
		INC tmp2
		STS LSD_ROW_P, tmp2
RET

//Обновление данных на экране
LCD_update:
	TST_OPin O_LSD_E
	BREQ /*=*/ LCD_update_no_strib
		//У нас строб
		COPin O_LSD_E
		SetTask TS_O
		SetTask TS_LCD_update
		RET
	LCD_update_no_strib:

	LDS tmp2, LSD_mode
	CPI tmp2, LSD_M_READY
	BRNE /*!=*/ LCD_update_not_ready
		LDS tmp2, LSD_ROW_P
		CPI	tmp2, LSD_M_S1R
		BRNE /*!=*/ LCD_update_ready_oldMode_not1
			//Старый мод был LSD_M_S1R, поэтому теперь LSD_M_S2R
			STI LSD_mode, LSD_M_S2R
			//Устанавливаем курсор
			SetScrinKeyWord LCD_SETDDRAMADDR | LCD_ROW2_ADR
			RJMP LCD_update_ready_oldMode_end
		LCD_update_ready_oldMode_not1:
		CPI	tmp2, LSD_M_S2R
		BRNE /*!=*/ LCD_update_ready_oldMode_not2
			STI LSD_mode, LSD_M_S3R
			SetScrinKeyWord LCD_SETDDRAMADDR | LCD_ROW3_ADR
			RJMP LCD_update_ready_oldMode_end
		LCD_update_ready_oldMode_not2:
		CPI	tmp2, LSD_M_S3R
		BRNE /*!=*/ LCD_update_ready_oldMode_not3
			STI LSD_mode, LSD_M_S4R
			SetScrinKeyWord LCD_SETDDRAMADDR | LCD_ROW4_ADR
			RJMP LCD_update_ready_oldMode_end
		LCD_update_ready_oldMode_not3:
			STI LSD_mode, LSD_M_S1R
			SetScrinKeyWord LCD_SETDDRAMADDR | LCD_ROW1_ADR
		LCD_update_ready_oldMode_end:
		//Начинаем печать строки
		STI LSD_ROW_P, 0
		//Даём время подумать
		SetTimerTask TS_LCD_update,1
		RET
	LCD_update_not_ready:
	CPI tmp2, LSD_M_S1R
	BRNE /*!=*/ LCD_update_not_1r
		//Обновляем первую строку
		LDS tmp2, LSD_ROW_P	//Берём смещение
		CPI tmp2, LSD_ROW_LENGHT
		BRSH /*>=*/ LCD_update_1r_r2Update
			LDIW Z, LSD_ROW_1
			RCALL LCD_update_ROW
			RET
		LCD_update_1r_r2Update:
		RCALL LCD_update_ROW2 //А теперь обновляем вторую строку
		LDS tmp2, LSD_ROW_P	//Берём смещение
		TST tmp2
		BREQ /*=*/ LCD_update_1r_end
			//Мы ещё не всё.
			SetTimerTask TS_LCD_update,1
			RET
		LCD_update_1r_end:
			//Мы обновили вторую строку. Больше нечего делать. Освобождаем экран
			STI LSD_ROW_P, LSD_M_S1R
			STI LSD_mode, LSD_M_READY
			SetTimerTask TS_LCD_update,40
			RET
	LCD_update_not_1r:
	CPI tmp2, LSD_M_S2R
	BRNE /*!=*/ LCD_update_not_2r
		LDS tmp2, LSD_ROW_P
		CPI tmp2, LSD_ROW_LENGHT
		BRSH /*>=*/ LCD_update_2r_r1Update
			LDIW Z, LSD_ROW_2
			RCALL LCD_update_ROW
			RET
		LCD_update_2r_r1Update:
		RCALL LCD_update_ROW1
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BREQ /*=*/ LCD_update_2r_end
			SetTimerTask TS_LCD_update,1
			RET
		LCD_update_2r_end:
			STI LSD_ROW_P, LSD_M_S2R
			STI LSD_mode, LSD_M_READY
			SetTimerTask TS_LCD_update,40
			RET
	LCD_update_not_2r:
	CPI tmp2, LSD_M_S3R
	BRNE /*!=*/ LCD_update_not_3r
		LDS tmp2, LSD_ROW_P
		CPI tmp2, LSD_ROW_LENGHT
		BRSH /*>=*/ LCD_update_3r_r1Update
			LDIW Z, LSD_ROW_3
			RCALL LCD_update_ROW
			RET
		LCD_update_3r_r1Update:
			//Нам нужно обновить и первую и вторую строку... Поэтому тут всё сложнее
			STI LSD_mode, LSD_M_U1RF3
			LDI tmp2, LSD_M_U1RF3
	LCD_update_not_3r:
	CPI tmp2, LSD_M_U1RF3
	BRNE /*!=*/ LCD_update_not_U1RF3
		RCALL LCD_update_ROW1
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_update_U1RF3_next
			//Меняем мод
			STI LSD_ROW_P, LSD_ROW_LENGHT
			STI LSD_mode, LSD_M_U2RF3
		LCD_update_U1RF3_next:
		SetTimerTask TS_LCD_update,1
		RET
	LCD_update_not_U1RF3:
	CPI tmp2, LSD_M_U2RF3
	BRNE /*!=*/ LCD_update_not_U2RF3
		RCALL LCD_update_ROW2
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_update_U2RF3_next
			//Меняем мод
			STI LSD_ROW_P, LSD_M_S3R
			STI LSD_mode, LSD_M_READY
			SetTimerTask TS_LCD_update,40
			RET
		LCD_update_U2RF3_next:
			SetTimerTask TS_LCD_update,1
			RET
	LCD_update_not_U2RF3:
	CPI tmp2, LSD_M_S4R
	BRNE /*!=*/ LCD_update_not_4r
		LDS tmp2, LSD_ROW_P
		CPI tmp2, LSD_ROW_LENGHT
		BRSH /*>=*/ LCD_update_4r_r1Update
			LDIW Z, LSD_ROW_4
			RCALL LCD_update_ROW
			RET
		LCD_update_4r_r1Update:
			//Нам нужно обновить и первую и вторую строку... Поэтому тут всё сложнее
			STI LSD_mode, LSD_M_U1RF4
			LDI tmp2, LSD_M_U1RF3
	LCD_update_not_4r:
	CPI tmp2, LSD_M_U1RF4
	BRNE /*!=*/ LCD_update_not_U1RF4
		RCALL LCD_update_ROW1
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_update_U1RF4_next
			//Меняем мод
			STI LSD_ROW_P, LSD_ROW_LENGHT
			STI LSD_mode, LSD_M_U2RF4
		LCD_update_U1RF4_next:
		SetTimerTask TS_LCD_update,1
		RET
	LCD_update_not_U1RF4:
	CPI tmp2, LSD_M_U2RF4
	BRNE /*!=*/ LCD_update_not_U2RF4
		RCALL LCD_update_ROW2
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_update_U2RF4_next
			//Меняем мод
			STI LSD_ROW_P, LSD_M_S4R
			STI LSD_mode, LSD_M_READY
			SetTimerTask TS_LCD_update,40
			RET
		LCD_update_U2RF4_next:
			SetTimerTask TS_LCD_update,1
			RET
	LCD_update_not_U2RF4:
	//Занято, надо подождать чутка
	SetTimerTask TS_LCD_update,10
RET

//Обновляет строку, адрес которой загружен в регистр Z
LCD_update_ROW:
	LDS tmp2, LSD_ROW_P	//Берём смещение
	CLR Al
	//Смещаем
	ADD ZL,tmp2
	ADC Zh,Al
	LD	tmp3, Z
	SetScrinWord tmp3
	//Передать следующий символ
	INC tmp2
	STS LSD_ROW_P, tmp2
	SetTimerTask TS_LCD_update,1
RET

//Спецфункция, обновляет первые 4 символа первой строки
LCD_update_ROW1:
	LDS tmp2, LSD_ROW_P	//Берём смещение
	CPI tmp2, LSD_ROW_LENGHT
	BRNE /*!=*/ LCD_update_ROW1_print
		//Нужно установить курсор
		SetScrinKeyWord LCD_SETDDRAMADDR | LCD_ROW1_ADR
		STI LSD_ROW_P, LSD_ROW_LENGHT + 1
		RET
	LCD_update_ROW1_print:
		//Нужно записать символы строки
		CLR Al
		//Берём адрес и смещение
		SUBI tmp2, LSD_ROW_LENGHT + 1
		LDIW Z,LSD_ROW_1
		//Смещаем
		ADD ZL,tmp2
		ADC Zh,Al
		LD	tmp3, Z
		SetScrinWord tmp3
		INC tmp2
		CPI tmp2, 4
		BREQ /*=*/ LCD_update_ROW1_end
			//Не все символы записали
			ADI tmp2, LSD_ROW_LENGHT + 1
			STS LSD_ROW_P, tmp2
			RET
		LCD_update_ROW1_end:
		//Работа закончена, показываем это записью сюда 0
			STI LSD_ROW_P, 0
RET
//Спецфункция, обновляет первые 5 символов второй строки
LCD_update_ROW2:
	LDS tmp2, LSD_ROW_P	//Берём смещение
	CPI tmp2, LSD_ROW_LENGHT
	BRNE /*!=*/ LCD_update_ROW2_print
		//Нужно установить курсор
		SetScrinKeyWord LCD_SETDDRAMADDR | LCD_ROW2_ADR
		STI LSD_ROW_P, LSD_ROW_LENGHT + 1
		RET
	LCD_update_ROW2_print:
		//Нужно записать символы строки
		CLR Al
		//Берём адрес и смещение
		SUBI tmp2, LSD_ROW_LENGHT + 1
		LDIW Z,LSD_ROW_2
		//Смещаем
		ADD ZL,tmp2
		ADC Zh,Al
		LD	tmp3, Z
		SetScrinWord tmp3
		INC tmp2
		CPI tmp2, 5
		BREQ /*=*/ LCD_update_ROW2_end
			//Не все символы записали
			ADI tmp2, LSD_ROW_LENGHT + 1
			STS LSD_ROW_P, tmp2
			RET
		LCD_update_ROW2_end:
		//Работа закончена, показываем это записью сюда 0
			STI LSD_ROW_P, 0
RET


 //Сохраняет кодовое слово на экранею. Один параметр - OSRG - слово
SetScrinKeyWord_M:
	CLI
		STS		WordOut,MacroR					;Записали кодовое слово в регистр 1
		LDS		MacroR, WordOut+1
		ANDI	MacroR,~((1 << (O_LSD_RS - 8)) /*| (1 << (O_LSD_RW - 8))*/)
		ORI		MacroR,(1 << (O_LSD_E - 8)) 
		STS     WordOut+1,MacroR					;Установили строб и показали что передаём команду
	SEI
	//А теперь обновляем выходы
	SetTask TS_O
RET
 //Сохраняет символ на экранею. Один параметр - OSRG - символ
SetScrinWord_M:
	CLI
		STS      WordOut,MacroR					;Записали слово в регистр 1
		LDS	 	MacroR, WordOut+1
		//ANDI	MacroR,~((1 << (O_LSD_RW - 8)))
		ORI		MacroR, (1 << (O_LSD_E - 8)) | (1 << (O_LSD_RS - 8))
		STS     WordOut+1,MacroR					;Установили строб и показали что передаём символ
	SEI
	//А теперь обновляем выходы
	SetTask TS_O
RET

//========================================
//А теперь фукнции печати в дисплей

//Копирует строки из программной памяти в строку
//Х - куда вставляем
//@0 откуда берём
.MACRO _LCD_COP_FROM_MEM
	LDIW Z, @0 * 2
	RCALL LCD_COP_FROM_MEM_M
.ENDMACRO
.MACRO LCD_COP_FROM_MEM
	PUSH Zl
	PUSH Zh
		_LCD_COP_FROM_MEM @0
	POP Zh
	POP Zl
.ENDMACRO

LCD_COP_FROM_MEM_M:
	LPM MacroR, Z+
	TST MacroR
	BREQ /*=*/ LCD_COP_FROM_MEM_M_RET
	ST X+, MacroR
	RJMP LCD_COP_FROM_MEM_M

	LCD_COP_FROM_MEM_M_RET:
RET

//Печатает в строку нужные данные. Рабочий режим - строки со средней скорость, температурой и т.д.
//MacroR - номер печатоемой строки
//Х - адрес строки, куда печатаем
LCD_PRINT_workMode:
	CPI MacroR, DM_AVR_V
	BRNE /*!=*/ LCD_PRINT_workMode_not1
		RCALL LCD_PRINT_AVR_V
		RET
	LCD_PRINT_workMode_not1:
	CPI MacroR, DM_nowT
	BRNE /*!=*/ LCD_PRINT_workMode_not2
		RCALL LCD_PRINT_TIME
		RET
	LCD_PRINT_workMode_not2:
	CPI MacroR, DM_T
	BRNE /*!=*/ LCD_PRINT_workMode_not3
		RCALL LCD_PRINT_T
		RET
	LCD_PRINT_workMode_not3:
	CPI MacroR, DM_P
	BRNE /*!=*/ LCD_PRINT_workMode_not4
		RCALL LCD_PRINT_P
		RET
	LCD_PRINT_workMode_not4:
	CPI MacroR, DM_H
	BRNE /*!=*/ LCD_PRINT_workMode_not5
		RCALL LCD_PRINT_H
		RET
	LCD_PRINT_workMode_not5:
	CPI MacroR, DM_maxV
	BRNE /*!=*/ LCD_PRINT_workMode_not6
		RCALL LCD_PRINT_MAX_V
		RET
	LCD_PRINT_workMode_not6:
	CPI MacroR, DM_odo
	BRNE /*!=*/ LCD_PRINT_workMode_not7
		RCALL LCD_PRINT_ODO
		RET
	LCD_PRINT_workMode_not7:
RET

//Печатает в строку нужные данные. Режим меню - всякие там буковки
//MacroR - номер печатоемой строки
//Х - адрес строки, куда печатаем
LCD_PRINT_menu:
	
RET

 //Печатает в строку среднюю скорость.
 //Адрес строки в X
 LCD_PRINT_AVR_V:
	LDI tmp2, 'A'
	ST	X+, tmp2
	LDI tmp2, 'V'
	ST	X+, tmp2
	LDI tmp2, 'R'
	ST	X+, tmp2
	LDI tmp2, ' '
	ST	X+, tmp2
	PUSH Xl
	PUSH Xh

		//Теперь у нас есть средняя скорость в оборотах/минуту. А нам что нужно? км/ч!
		_CLR_64 ACC_1
		_MOV_32 ACC_32, WheelLength
		_MUL64_A_R32 AverageSpeed	//С * V = мм/минуту
		_MOV_64 BC_64, ACC_1
		_LDI_A 60
		_MUL64_A_R64 BC_64 //A = C * V * 60  = мм/ч
		_MOV_64 BC_64, ACC_1
		_LDI_A 100000
		_DIV_64_R64_DIV_A BC_64 //A = C * V / 100000 = 10 * км/мин
		_MOV_32 B_32, ACC_32
		_LDI_A 10
		_DIV_64_R32_DIV_A B_32 //Мы делим AverageSpeed на 10. Получаем в итоге скорость + её десятую часть.
								//ACC_32 теперь содержит скорость
								//ACC_2 содержит число после запятой
	POP Xh
	POP Xl

	ADIW X, 4 //Мы теперь будем дописывать с конца
	
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	//Теперь единицы
	MOV_32 B_32, ACC_32
	LDI_A 10
	DIV_64_R32_DIV_A B_32 
	LDI tmp2, '.'
	ST	-X, tmp2
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	//Теперь десятки
	LDS tmp2, ACC_1 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
 RET
 
 //Печатает в строку текущее время
 //Адрес строки в X
 LCD_PRINT_TIME:
	RCALL LCD_PRINT_CLR_ROW
	LDI tmp2, 'T'
	ST	X+, tmp2
	LDI tmp2, ' '
	ST	X+, tmp2
 
	LDIW Y, Clock
	LDI Counter, 8
	LCD_PRINT_TIME_l:
		LD tmp2, Y+
		ST X+, tmp2
		DEC Counter
	BRNE /*!=*/ LCD_PRINT_TIME_l
 RET

 //Печатает в строку температуру
 //Адрес строки в X
 LCD_PRINT_T:
	RCALL LCD_PRINT_CLR_ROW
	LDI tmp2, 'T'
	ST	X+, tmp2
	LDI tmp2, ':'
	ST	X+, tmp2
	ADIW X,6 //Дальше мы число пишем сзади. При этом! У нас функции с предекрементом!

	CLR_64 ACC_1
	LDS tmp2, Temperature + 0
	STS ACC_32 + 2, tmp2
	LDS tmp2, Temperature + 1
	STS ACC_32 + 3, tmp2

	LDI_32 B_32, 10
	DIV_64_A_DIV_R32 B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	DIV_64_A_DIV_R32 B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	LDI tmp2, '.'
	ST	-X, tmp2
	DIV_64_A_DIV_R32 B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	DIV_64_A_DIV_R32 B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	ADIW X,6 //Перескакиваем через число
	LDI tmp2, 'C'
	ST	X+, tmp2

 RET
 //Печатает в строку Давление
 //Адрес строки в X
 LCD_PRINT_P:
	RCALL LCD_PRINT_CLR_ROW
	LDI tmp2, 'P'
	ST	X+, tmp2
	LDI tmp2, ':'
	ST	X+, tmp2
	ADIW X,4 //Дальше мы число пишем сзади. При этом! У нас функции с предекрементом!

	CLR_64 ACC_1
	LDS tmp2, Pressure + 0
	STS ACC_32 + 2, tmp2
	LDS tmp2, Pressure + 1
	STS ACC_32 + 3, tmp2

	LDI_32 B_32, 10
	DIV_64_A_DIV_R32 B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	DIV_64_A_DIV_R32 B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	DIV_64_A_DIV_R32 B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	ADIW X,4 //Перескакиваем через число
	LDI tmp2, 'm'
	ST	X+, tmp2
	ST	X+, tmp2

 RET
 //Печатает в строку Влажность
 //Адрес строки в X
 LCD_PRINT_H:
	RCALL LCD_PRINT_CLR_ROW
	LDI tmp2, 'H'
	ST	X+, tmp2
	LDI tmp2, ':'
	ST	X+, tmp2
	ADIW X,3 //Дальше мы число пишем сзади. При этом! У нас функции с предекрементом!

	CLR_64 ACC_1
	LDS tmp2, Humidity + 0
	STS ACC_32 + 3, tmp2

	LDI_32 B_32, 10
	DIV_64_A_DIV_R32 B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	DIV_64_A_DIV_R32 B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	ADIW X,3 //Перескакиваем через число
	LDI tmp2, ' '
	ST	X+, tmp2
	LDI tmp2, '%'
	ST	X+, tmp2
 RET
 //Печатает в строку одометр
 //Адрес строки в X
 LCD_PRINT_ODO:
	PUSH tmp2
	PUSH tmp3
	PUSH Counter
		RCALL LCD_PRINT_CLR_ROW
		_LCD_COP_FROM_MEM LCD_ODO
		ADIW X,7 //Дальше мы число из 4х символов пишем сзади функцией с преддекрементом
		PUSH Xl
		PUSH Xh

			MOV_32 ACC_32, Odometr
			EXP_64_F_32 ACC_1
			MUL64_A_R32 WheelLength //Умножаем на длину колеса, чтобы получить расстояние в мм
			LDI_32 B_32, 1000 
			DIV_64_A_DIV_R32 B_32//Переводим в метры
			LDI_32 B_32, 10
			LDI_32 C_32, 100000 //Максимально 5 знаков

			CLR tmp3
			LCD_PRINT_ODO_del:
				CP_32 ACC_32, C_32
				BRLO /*<*/ LCD_PRINT_ODO_print
				//Пока у нас не меньше, мы делим на 10 и считаем сколько сократили знаков
				INC tmp3
				DIV_64_A_DIV_R32 B_32
				RJMP LCD_PRINT_ODO_del
			LCD_PRINT_ODO_print:
			LDI MacroR, 3
			SUB MacroR, tmp3 //Counter = 3 - Counter. Нас интересуют только младшие 3 цифры, для разделения на м и км
			MOV tmp3, MacroR
			PUSH tmp3
				LDI Counter,5
				LCD_PRINT_ODO_loop_m:
					DIV_64_A_DIV_R32 B_32
					LDS tmp2, ACC_2 + 7
					ADI tmp2, '0'
					ST	-X, tmp2
					DEC tmp3
					BRNE /*!=*/ LCD_PRINT_ODO_l_npd
						//Когда счётчик обнулится - надо запятую поставить
						CPI Counter, 1
						BREQ /*=*/ LCD_PRINT_ODO_l_npd //НО! Чтобы нам не поставить запятую перед пустотой, мы делаем ещё одну проверку
						LDI tmp2, '.'
						ST	-X, tmp2
					LCD_PRINT_ODO_l_npd:
					DEC Counter
				BRNE /*!=*/ LCD_PRINT_ODO_loop_m
			POP tmp3
		POP Xh
		POP Xl
	POP Counter
	POP tmp3
	POP tmp2
 RET

 //Печатает в строку максимальную скорость
 //Адрес строки в X
 LCD_PRINT_MAX_V:
	RCALL LCD_PRINT_CLR_ROW
	_LCD_COP_FROM_MEM LCD_MAX_V
	PUSH Xl
	PUSH Xh

		//Теперь у нас есть средняя скорость в оборотах/минуту. А нам что нужно? км/ч!
		_CLR_64 ACC_1
		_MOV_32 ACC_32, WheelLength
		_MUL64_A_R32 MaxSpeed	//С * V = мм/минуту
		_MOV_64 BC_64, ACC_1
		_LDI_A 60
		_MUL64_A_R64 BC_64 //A = C * V * 60  = мм/ч
		_MOV_64 BC_64, ACC_1
		_LDI_A 100000
		_DIV_64_R64_DIV_A BC_64 //A = C * V / 100000 = 10 * км/мин
		_MOV_32 B_32, ACC_32
		_LDI_A 10
		_DIV_64_R32_DIV_A B_32 //Мы делим MaxSpeed на 10. Получаем в итоге скорость + её десятую часть.
								//ACC_32 теперь содержит скорость
								//ACC_2 содержит число после запятой
	POP Xh
	POP Xl

	ADIW X, 4 //Мы теперь будем дописывать с конца
	
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	//Теперь единицы
	MOV_32 B_32, ACC_32
	LDI_A 10
	DIV_64_R32_DIV_A B_32 
	LDI tmp2, '.'
	ST	-X, tmp2
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
	//Теперь десятки
	LDS tmp2, ACC_1 + 7
	ADI tmp2, '0'
	ST	-X, tmp2
 RET

 
 //Печатает в строку все символы
 //Адрес строки в X
 LCD_PRINT_SYM:
	RCALL LCD_PRINT_CLR_ROW
	LDI tmp2, 'T'
	ST	X+, tmp2
	LDI tmp2, 'S'
	ST	X+, tmp2
	LDI tmp2, 'T'
	ST	X+, tmp2
	ADIW X, 1//пробел

	LDS tmp3, Clock + 4 //Берём минуты
	SUBI tmp3, '0'//в число
	LDI tmp2, 60
	MUL tmp3, tmp2 //Умножаем минуты на 60
	MOV tmp3, r0 //Минуты на 60
	LDS tmp2, Clock + 6 //Десятки секунд в виде символа
	SUBI tmp2, '0' - 1//tmp2 - '0' + 1
	
	LCD_PRINT_loop:
		DEC tmp2
		BREQ /*=*/ LCD_PRINT_endLoop
		ADI tmp3, 10 //Прибавляем ещё 10 секунд
		RJMP LCD_PRINT_loop
	LCD_PRINT_endLoop:
	LDS tmp2, Clock + 7 //Cекунды в виде символа
	SUBI tmp2, '0'
	ADD tmp3, tmp2 //Теперь в tmp3 - символ
	MOV tmp2, tmp3
	LSR tmp2
	LSR tmp2
	LSR tmp2
	LSR tmp2
	CPI tmp2, 9 + 1
	BRSH /*>=*/ LCD_PRINT_1sAsS
		//Первый символ - число
		ADI tmp2, '0'
		RJMP LCD_PRINT_1send
	LCD_PRINT_1sAsS:
		ADI tmp2, 'A' - 10 // tmp2 + 'A' - 10
	LCD_PRINT_1send:
	ST	X+, tmp2
	MOV tmp2, tmp3
	ANDI tmp2, 0x0F
	CPI tmp2, 9 + 1
	BRSH /*>=*/ LCD_PRINT_2sAsS
		//Первый символ - число
		ADI tmp2, '0'
		RJMP LCD_PRINT_2send
	LCD_PRINT_2sAsS:
		ADI tmp2, 'A' - 10
	LCD_PRINT_2send:
	ST	X+, tmp2

	LDI tmp2, ' '
	ST	X+, tmp2
	ST	X+, tmp3
 RET

 //В Х адрес строки. Очищает строку, заполняя её проблемами
 LCD_PRINT_CLR_ROW:
	PUSH Xl
	PUSH Xh
	PUSH Counter
	PUSH tmp3
		LDI Counter, LSD_ROW_LENGHT / 2 //Половина строки отдана под печать данных!
		LDI tmp3, ' '
		LCD_PRINT_CLR_ROW_l:
			ST X+, tmp3
			DEC Counter
		BRNE /*!=*/ LCD_PRINT_CLR_ROW_l
	POP tmp3
	POP Counter
	POP Xh
	POP Xl
 RET