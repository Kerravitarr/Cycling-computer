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
	LDS word, LSD_mode
	CPI word, LSD_M_KEY_W_1
	BRNE /*!=*/ LCD_init_notKW1
		SetScrinKeyWord LCD_KEY_WORD
		STI LSD_mode, LSD_M_KEY_W_2
		RJMP Init_Scrin_end
	LCD_init_notKW1:
	CPI word, LSD_M_KEY_W_2
	BRNE /*!=*/ LCD_init_notKW2
		SetScrinKeyWord LCD_KEY_WORD
		STI LSD_mode, LSD_M_KEY_W_3
		RJMP Init_Scrin_end
	LCD_init_notKW2:
	CPI word, LSD_M_KEY_W_3
	BRNE /*!=*/ LCD_init_notKW3
		SetScrinKeyWord LCD_KEY_WORD
		STI LSD_mode, LSD_M_KEY_W_4
		RJMP Init_Scrin_end
	LCD_init_notKW3:
	CPI word, LSD_M_KEY_W_4
	BRNE /*!=*/ LCD_init_notKW4
		SetScrinKeyWord LCD_KEY_WORD
		STI LSD_mode, LSD_M_INIT
		RJMP Init_Scrin_end
	LCD_init_notKW4:
	CPI word, LSD_M_INIT
	BRNE /*!=*/ LCD_init_notI
		SetScrinKeyWord LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF
		STI LSD_mode, LSD_M_CLEAR
		RJMP Init_Scrin_end
	LCD_init_notI:
	CPI word, LSD_M_CLEAR
	BRNE /*!=*/ LCD_init_notCLR
		SetScrinKeyWord LCD_CLEARDISPLAY
		STI LSD_mode, LSD_M_START
		RJMP Init_Scrin_end
	LCD_init_notCLR:
	CPI word, LSD_M_START
	BRNE /*!=*/ LCD_init_not_start
	LCD_init_start:
		SetScrinKeyWord LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT
		STI LSD_mode, LSD_M_KEY_WS1
		STI LSD_ROW_P, 0
		RJMP Init_Scrin_end
	LCD_init_not_start:
	CPI word, LSD_M_KEY_WS1
	BRNE /*!=*/ LCD_init_not_key_ws1
		LDI tmp3, (0 << 3)
		LDIW Z, LCD_BIG_NUMBER_1*2
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
	CPI word, LSD_M_KEY_WS2
	BRNE /*!=*/ LCD_init_not_key_ws2
		LDI tmp3, (1 << 3)
		LDIW Z, LCD_BIG_NUMBER_2*2
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
	CPI word, LSD_M_KEY_WS3
	BRNE /*!=*/ LCD_init_not_key_ws3
		LDI tmp3, (2 << 3)
		LDIW Z, LCD_BIG_NUMBER_3*2
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
	CPI word, LSD_M_KEY_WS4
	BRNE /*!=*/ LCD_init_not_key_ws4
		LDI tmp3, (3 << 3)
		LDIW Z, LCD_BIG_NUMBER_4*2
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
	CPI word, LSD_M_KEY_WS5
	BRNE /*!=*/ LCD_init_not_key_ws5
		LDI tmp3, (4 << 3)
		LDIW Z, LCD_BIG_NUMBER_5*2
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
	CPI word, LSD_M_KEY_WS6
	BRNE /*!=*/ LCD_init_not_key_ws6
		LDI tmp3, (5 << 3)
		LDIW Z, LCD_BIG_NUMBER_6*2
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
	CPI word, LSD_M_KEY_WS7
	BRNE /*!=*/ LCD_init_not_key_ws7
		LDI tmp3, (6 << 3)
		LDIW Z, LCD_BIG_NUMBER_7*2
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
	CPI word, LSD_M_KEY_WS8
	BRNE /*!=*/ LCD_init_not_key_ws8
		LDI tmp3, (7 << 3)
		LDIW Z, LCD_BIG_NUMBER_8*2
		RCALL LCD_INIT_WRITE_CHAR
		LDS tmp2, LSD_ROW_P
		TST tmp2
		BRNE /*!=*/ LCD_init_key_ws8_not_end
			//Закончили с этим режимом
			STI LSD_mode, LSD_M_READY
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

	LDS word, LSD_mode
	CPI word, LSD_M_READY
	BRNE /*!=*/ LCD_update_not_ready
		LDS word, LSD_ROW_P
		CPI	word, LSD_M_S1R
		BRNE /*!=*/ LCD_update_ready_oldMode_not1
			//Старый мод был LSD_M_S1R, поэтому теперь LSD_M_S2R
			STI LSD_mode, LSD_M_S2R
			//Устанавливаем курсор
			SetScrinKeyWord LCD_SETDDRAMADDR | LCD_ROW2_ADR
			RJMP LCD_update_ready_oldMode_end
		LCD_update_ready_oldMode_not1:
		CPI	word, LSD_M_S2R
		BRNE /*!=*/ LCD_update_ready_oldMode_not2
			STI LSD_mode, LSD_M_S3R
			SetScrinKeyWord LCD_SETDDRAMADDR | LCD_ROW3_ADR
			RJMP LCD_update_ready_oldMode_end
		LCD_update_ready_oldMode_not2:
		CPI	word, LSD_M_S3R
		BRNE /*!=*/ LCD_update_ready_oldMode_not3
			STI LSD_mode, LSD_M_S4R
			SetScrinKeyWord LCD_SETDDRAMADDR | LCD_ROW4_ADR
			RJMP LCD_update_ready_oldMode_end
		LCD_update_ready_oldMode_not3:
			STI LSD_mode, LSD_M_S1R
			SetScrinKeyWord LCD_SETDDRAMADDR | LCD_ROW1_ADR
		LCD_update_ready_oldMode_end:
		//Даём время подумать
		SetTimerTask TS_LCD_update,1
		RET
	LCD_update_not_ready:
	CPI word, LSD_M_S1R
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
	CPI word, LSD_M_S2R
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
	CPI word, LSD_M_S3R
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
			LDI word, LSD_M_U1RF3
	LCD_update_not_3r:
	CPI word, LSD_M_U1RF3
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
	CPI word, LSD_M_U2RF3
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
	CPI word, LSD_M_S4R
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
			LDI word, LSD_M_U1RF3
	LCD_update_not_4r:
	CPI word, LSD_M_U1RF4
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
	CPI word, LSD_M_U2RF4
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
	CLR tmp3
	//Смещаем
	ADD ZL,tmp2
	ADC Zh,tmp3
	LD	word, Z
	SetScrinWord word
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
		CLR tmp3
		//Берём адрес и смещение
		SUBI tmp2, LSD_ROW_LENGHT + 1
		LDIW Z,LSD_ROW_1
		//Смещаем
		ADD ZL,tmp2
		ADC Zh,tmp3
		LD	word, Z
		SetScrinWord word
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
		CLR tmp3
		//Берём адрес и смещение
		SUBI tmp2, LSD_ROW_LENGHT + 1
		LDIW Z,LSD_ROW_2
		//Смещаем
		ADD ZL,tmp2
		ADC Zh,tmp3
		LD	word, Z
		SetScrinWord word
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