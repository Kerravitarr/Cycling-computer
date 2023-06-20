/*
 * TimeRepeat.inc
 *
 *  Created: 25.11.2018 11:43:22
 *   Author: Terran
 */ 
 
 //Таймер на 1 мс - отрабатывает каждую милисекунду
 Evry_1_ms_Task:
	//Проверяем - мы двигаемся?
	TSTFlag isMove
	BREQ /*=*/ Evry_1_ms_Task_not_move
		_INC_32 TimeOdo
		_INC_32 TimeTrip
	Evry_1_ms_Task_not_move:

	SetTimerTask TS_Evry_1_ms_Task,1
 RET

 //Таймер на 40 мс - 25 раз в секунду срабатывает
 Evry_40_ms_Task:
	//Проверяем - мы двигаемся?
	TSTFlag isMove
	BRNE /*!=*/ Evry_40_ms_Task_move
		RJMP Evry_40_ms_Task_not_move
	Evry_40_ms_Task_move:
		//Мы едем. Значит пересчитываем все значения

		//Максимальная текущая скорость в ACC
		_MOV_32 B_32, TimeTrip
		_CP_32 B_32, TimeVOld
		BRNE /*!=*/ Evry_40_ms_Task_not_zeroDiv
			//У нас косяк. Эти два числа не могут быть равными. Чтобы не поплыть как ржавй крейсер - сбрасываем
			RJMP Evry_40_ms_Task_not_move
		Evry_40_ms_Task_not_zeroDiv:
		_SUB_32 B_32, TimeVOld
		_LDI_A 60000
		_DIV_64_A_DIV_R32 B_32

		_CP_32 ACC_32, Speed
		BRSH /*>=*/ Evry_40_ms_Task_move_smal_v
			//У нас текущая скорость ниже возможной
			_MOV_32 Speed, ACC_32
		Evry_40_ms_Task_move_smal_v:
		
		_LDI_32 B_32, 10
		_CP_32 ACC_32, B_32
		BRSH /*>=*/ Evry_40_ms_Task_move_next
			//Наша скорость меньше 10 оборотов в минуту. Мы считаем это остановкой!
			CBIFlag isMove
			_LDI_32 Speed, 0
			_MOV_32 B_32, TimeTrip
			_SUB_32 B_32, TimeVOld
			_MOV_32 TimeTrip, TimeVOld
			_SUB_32 TimeOdo, B_32
		Evry_40_ms_Task_move_next:

		//Теперь у нас есть скорость в оборотах/минуту. А нам что нужно? км/ч!
		_CLR_64 ACC_1
		_MOV_32 ACC_32, WheelLength
		_MUL64_A_R32 Speed	//С * V = мм/минуту
		_MOV_64 BC_64, ACC_1
		_LDI_A 60
		_MUL64_A_R64 BC_64 //A = C * V * 60  = мм/ч
		_MOV_64 BC_64, ACC_1
		_LDI_A 100000
		_DIV_64_R64_DIV_A BC_64 //A = C * V / 100000 = 10 * км/мин
		_MOV_32 B_32, ACC_32
		_LDI_A 10
		_DIV_64_R32_DIV_A B_32 //Мы делим Speed на 10. Получаем в итоге скорость + её десятую часть.
								//ACC_32 теперь содержит скорость
								//ACC_2 содержит число после запятой
		LDS tmp2, ACC_2 + 7
		ADI tmp2, '0'
		STS LSD_ROW_2 + 4, tmp2
		//Теперь единицы
		_MOV_32 B_32, ACC_32
		_LDI_A 10
		_DIV_64_R32_DIV_A B_32 
		LDS tmp2, ACC_2 + 7
		CPI tmp2, 0
		BRNE /*!=*/ Evry_40_ms_Task_move_num2_not0
			//Цифра 0
			STI LSD_ROW_1 + 2, 0x7
			STI LSD_ROW_1 + 3, 0x0
			STI LSD_ROW_2 + 2, 0x6
			STI LSD_ROW_2 + 3, 0x1
			RJMP Evry_40_ms_Task_move_num2_end
		Evry_40_ms_Task_move_num2_not0:
		CPI tmp2, 1
		BRNE /*!=*/ Evry_40_ms_Task_move_num2_not1
			STI LSD_ROW_1 + 2, 0x0
			STI LSD_ROW_1 + 3, ' '
			STI LSD_ROW_2 + 2, 0x1
			STI LSD_ROW_2 + 3, 0x3
			RJMP Evry_40_ms_Task_move_num2_end
		Evry_40_ms_Task_move_num2_not1:
		CPI tmp2, 2
		BRNE /*!=*/ Evry_40_ms_Task_move_num2_not2
			STI LSD_ROW_1 + 2, 0x2
			STI LSD_ROW_1 + 3, 0x4
			STI LSD_ROW_2 + 2, 0x5
			STI LSD_ROW_2 + 3, 0x3
			RJMP Evry_40_ms_Task_move_num2_end
		Evry_40_ms_Task_move_num2_not2:
		CPI tmp2, 3
		BRNE /*!=*/ Evry_40_ms_Task_move_num2_not3
			STI LSD_ROW_1 + 2, 0x2
			STI LSD_ROW_1 + 3, 0x4
			STI LSD_ROW_2 + 2, 0x3
			STI LSD_ROW_2 + 3, 0x4
			RJMP Evry_40_ms_Task_move_num2_end
		Evry_40_ms_Task_move_num2_not3:
		CPI tmp2, 4
		BRNE /*!=*/ Evry_40_ms_Task_move_num2_not4
			STI LSD_ROW_1 + 2, 0x6
			STI LSD_ROW_1 + 3, 0x1
			STI LSD_ROW_2 + 2, ' '
			STI LSD_ROW_2 + 3, 0x0
			RJMP Evry_40_ms_Task_move_num2_end
		Evry_40_ms_Task_move_num2_not4:
		CPI tmp2, 5
		BRNE /*!=*/ Evry_40_ms_Task_move_num2_not5
			STI LSD_ROW_1 + 2, 0x5
			STI LSD_ROW_1 + 3, 0x2
			STI LSD_ROW_2 + 2, 0x3
			STI LSD_ROW_2 + 3, 0x4
			RJMP Evry_40_ms_Task_move_num2_end
		Evry_40_ms_Task_move_num2_not5:
		CPI tmp2, 6
		BRNE /*!=*/ Evry_40_ms_Task_move_num2_not6
			STI LSD_ROW_1 + 2, 0x5
			STI LSD_ROW_1 + 3, 0x2
			STI LSD_ROW_2 + 2, 0x5
			STI LSD_ROW_2 + 3, 0x4
			RJMP Evry_40_ms_Task_move_num2_end
		Evry_40_ms_Task_move_num2_not6:
		CPI tmp2, 7
		BRNE /*!=*/ Evry_40_ms_Task_move_num2_not7
			STI LSD_ROW_1 + 2, 0x2
			STI LSD_ROW_1 + 3, 0x0
			STI LSD_ROW_2 + 2, ' '
			STI LSD_ROW_2 + 3, 0x7
			RJMP Evry_40_ms_Task_move_num2_end
		Evry_40_ms_Task_move_num2_not7:
		CPI tmp2, 8
		BRNE /*!=*/ Evry_40_ms_Task_move_num2_not8
			STI LSD_ROW_1 + 2, 0x5
			STI LSD_ROW_1 + 3, 0x4
			STI LSD_ROW_2 + 2, 0x6
			STI LSD_ROW_2 + 3, 0x1
			RJMP Evry_40_ms_Task_move_num2_end
		Evry_40_ms_Task_move_num2_not8:
		CPI tmp2, 9
		BRNE /*!=*/ Evry_40_ms_Task_move_num2_not9
			STI LSD_ROW_1 + 2, 0x5
			STI LSD_ROW_1 + 3, 0x4
			STI LSD_ROW_2 + 2, 0x3
			STI LSD_ROW_2 + 3, 0x1
			RJMP Evry_40_ms_Task_move_num2_end
		Evry_40_ms_Task_move_num2_not9:

		Evry_40_ms_Task_move_num2_end:

		//Теперь десятки
		LDS tmp2, ACC_1 + 7
		CPI tmp2, 0
		BRNE /*!=*/ Evry_40_ms_Task_move_num1_not0
			//Цифра 0
			STI LSD_ROW_1 + 0, 0x7
			STI LSD_ROW_1 + 1, 0x0
			STI LSD_ROW_2 + 0, 0x6
			STI LSD_ROW_2 + 1, 0x1
			RJMP Evry_40_ms_Task_move_num1_end
		Evry_40_ms_Task_move_num1_not0:
		CPI tmp2, 1
		BRNE /*!=*/ Evry_40_ms_Task_move_num1_not1
			STI LSD_ROW_1 + 0, 0x0
			STI LSD_ROW_1 + 1, ' '
			STI LSD_ROW_2 + 0, 0x1
			STI LSD_ROW_2 + 1, 0x3
			RJMP Evry_40_ms_Task_move_num1_end
		Evry_40_ms_Task_move_num1_not1:
		CPI tmp2, 2
		BRNE /*!=*/ Evry_40_ms_Task_move_num1_not2
			STI LSD_ROW_1 + 0, 0x2
			STI LSD_ROW_1 + 1, 0x4
			STI LSD_ROW_2 + 0, 0x5
			STI LSD_ROW_2 + 1, 0x3
			RJMP Evry_40_ms_Task_move_num1_end
		Evry_40_ms_Task_move_num1_not2:
		CPI tmp2, 3
		BRNE /*!=*/ Evry_40_ms_Task_move_num1_not3
			STI LSD_ROW_1 + 0, 0x2
			STI LSD_ROW_1 + 1, 0x4
			STI LSD_ROW_2 + 0, 0x3
			STI LSD_ROW_2 + 1, 0x4
			RJMP Evry_40_ms_Task_move_num1_end
		Evry_40_ms_Task_move_num1_not3:
		CPI tmp2, 4
		BRNE /*!=*/ Evry_40_ms_Task_move_num1_not4
			STI LSD_ROW_1 + 0, 0x6
			STI LSD_ROW_1 + 1, 0x1
			STI LSD_ROW_2 + 0, ' '
			STI LSD_ROW_2 + 1, 0x0
			RJMP Evry_40_ms_Task_move_num1_end
		Evry_40_ms_Task_move_num1_not4:
		CPI tmp2, 5
		BRNE /*!=*/ Evry_40_ms_Task_move_num1_not5
			STI LSD_ROW_1 + 0, 0x5
			STI LSD_ROW_1 + 1, 0x2
			STI LSD_ROW_2 + 0, 0x3
			STI LSD_ROW_2 + 1, 0x4
			RJMP Evry_40_ms_Task_move_num1_end
		Evry_40_ms_Task_move_num1_not5:
		CPI tmp2, 6
		BRNE /*!=*/ Evry_40_ms_Task_move_num1_not6
			STI LSD_ROW_1 + 0, 0x5
			STI LSD_ROW_1 + 1, 0x2
			STI LSD_ROW_2 + 0, 0x5
			STI LSD_ROW_2 + 1, 0x4
			RJMP Evry_40_ms_Task_move_num1_end
		Evry_40_ms_Task_move_num1_not6:
		CPI tmp2, 7
		BRNE /*!=*/ Evry_40_ms_Task_move_num1_not7
			STI LSD_ROW_1 + 0, 0x2
			STI LSD_ROW_1 + 1, 0x0
			STI LSD_ROW_2 + 0, ' '
			STI LSD_ROW_2 + 1, 0x7
			RJMP Evry_40_ms_Task_move_num1_end
		Evry_40_ms_Task_move_num1_not7:
		CPI tmp2, 8
		BRNE /*!=*/ Evry_40_ms_Task_move_num1_not8
			STI LSD_ROW_1 + 0, 0x5
			STI LSD_ROW_1 + 1, 0x4
			STI LSD_ROW_2 + 0, 0x6
			STI LSD_ROW_2 + 1, 0x1
			RJMP Evry_40_ms_Task_move_num1_end
		Evry_40_ms_Task_move_num1_not8:
		CPI tmp2, 9
		BRNE /*!=*/ Evry_40_ms_Task_move_num1_not9
			STI LSD_ROW_1 + 0, 0x5
			STI LSD_ROW_1 + 1, 0x4
			STI LSD_ROW_2 + 0, 0x3
			STI LSD_ROW_2 + 1, 0x1
			RJMP Evry_40_ms_Task_move_num1_end
		Evry_40_ms_Task_move_num1_not9:

		Evry_40_ms_Task_move_num1_end:

	Evry_40_ms_Task_not_move:

	SetTimerTask TS_Evry_40_ms_Task,40
 RET
 
 //Таймер на каждую секунду. Тут выполняются всякие не интересные операции
 Evry_1_s_Task:

	//Обновляем часики
	LDS tmp2, Clock + 7
	INC tmp2
	CPI tmp2, '9' + 1
	BREQ /*=*/ Evry_1_s_Task_sec_up
		//Просто обновляем секунды
		STS Clock + 7, tmp2
		RJMP Evry_1_s_Task_clock_end
	Evry_1_s_Task_sec_up:
		//Нужно с десятками что-то сделать, у нас переполнение единиц
		STI Clock + 7, '0'
		LDS tmp2, Clock + 6
		INC tmp2
		CPI tmp2, '5' + 1
		BREQ /*=*/ Evry_1_s_Task_10sec_up
			STS Clock + 6, tmp2
			RJMP Evry_1_s_Task_clock_end
		Evry_1_s_Task_10sec_up:
			STI Clock + 6, '0'
			LDS tmp2, Clock + 4
			INC tmp2
			CPI tmp2, '9' + 1
			BREQ /*=*/ Evry_1_s_Task_min_up
				STS Clock + 4, tmp2
				RJMP Evry_1_s_Task_clock_end
			Evry_1_s_Task_min_up:
				STI Clock + 4, '0'
				LDS tmp2, Clock + 3
				INC tmp2
				CPI tmp2, '5' + 1
				BREQ /*=*/ Evry_1_s_Task_10min_up
					STS Clock + 3, tmp2
					RJMP Evry_1_s_Task_clock_end
				Evry_1_s_Task_10min_up:
					STI Clock + 3, '0'
					LDS tmp2, Clock + 1
					INC tmp2
					CPI tmp2, '9' + 1
					BREQ /*=*/ Evry_1_s_Task_h_up
						STS Clock + 1, tmp2
						RJMP Evry_1_s_Task_clock_end
					Evry_1_s_Task_h_up:
						STI Clock + 1, '0'
						LDS tmp2, Clock + 0
						INC tmp2
						CPI tmp2, '2' + 1
						BREQ /*=*/ Evry_1_s_Task_1h_up
							STS Clock + 0, tmp2
							RJMP Evry_1_s_Task_clock_end
						Evry_1_s_Task_1h_up:
						//У нас часы закончились. Дальше 00
						STI Clock + 0, '0'
	Evry_1_s_Task_clock_end:

	TSTFlag F_LSD_SCAN
	BREQ /*=*/ Evry_1_s_Task_notSCAN
		//Режим сканирования
		STI LSD_ROW_1_PAR -1, 'S'
		STI LSD_ROW_2_PAR -1, 'C'
		STI LSD_ROW_3_PAR -1, 'A'
		STI LSD_ROW_4_PAR -1, 'N'

		LDS tmp2, Clock + 7 //Берём секунды
		CPI tmp2, '0' //Следующий режим каждые 10 секунд
		BRNE /*!=*/ Evry_1_s_Task_notSCANend
			LDS tmp2, LCD_display_mode
			INC tmp2
			CPI tmp2, DM_SCAN
			BRNE /*!=*/ Evry_1_s_Task_nextSCAN
				//Но и мы обнуляем строчку, у нас круг
				STI LCD_display_mode, DM_DEFAULT
				RJMP Evry_1_s_Task_notSCANend
			Evry_1_s_Task_nextSCAN:
				STS LCD_display_mode, tmp2
		RJMP Evry_1_s_Task_notSCANend
	Evry_1_s_Task_notSCAN:
		STI LSD_ROW_1_PAR -1, ' '
		STI LSD_ROW_2_PAR -1, ' '
		STI LSD_ROW_3_PAR -1, ' '
		STI LSD_ROW_4_PAR -1, ' '
	Evry_1_s_Task_notSCANend:

	LDS tmp2, LCD_display_mode
	CPI tmp2, DM_SCAN
	BRLO /*<*/ Evry_1_s_Task_notBlink
		//У нас меню - нужен мигающий курсор
		LDS tmp2, Clock + 7 //Берём секунды
		ANDI tmp2, 1 << 1//Берём младший разряд числа в двоичном виде - будем каждые две секунды мигать
		BREQ /*=*/ Evry_1_s_Task_blinkA
			SBIFlag F_LSD_isBlink
			RJMP Evry_1_s_Task_blinkE
		Evry_1_s_Task_blinkA:
			CBIFlag F_LSD_isBlink
		Evry_1_s_Task_blinkE:
	Evry_1_s_Task_notBlink:
	
	SetTimerTask TS_Evry_1_s_Task,1000
 RET
 
 //Функция обновления значений на первой строке дисплея
 Update_1_row_LCD:
	
	//Средняя скорость = DIST * 60000 / TIME (V = S/T)
	_LDI_A 60000
	_MOV_32 B_32, Dist
	_MUL64_A_R32 B_32
	_MOV_32 B_32, TimeTrip
	_DIV_64_A_DIV_R32 B_32
	_MOV_32 AverageSpeed, ACC_32

	//Проверим как там со скростью дела
	_LDI_A 0
	_CP_32 Speed, ACC_32
	BRNE /*!=*/ Update_1_row_LCD_speed_not_0
		STI LSD_ROW_1 + 4, 'X'	//Когда скорость 0, мы сюда крест ставим
		RJMP Update_1_row_LCD_speed_end
	Update_1_row_LCD_speed_not_0:
		_MOV_32 B_32, Speed
		_SUB_32 B_32, AverageSpeed
		LDS tmp2, B_32
		SBRS tmp2,7
		RJMP Update_1_row_LCD_speed_posotiv
			//Старший бит = 1, значит у нас Speed < AverageSpeed
			_LDI_32 C_32, -10 //На сколько маленькое число игнорируем
			_CP_32 B_32,C_32
			BRLO /*<*/ Update_1_row_LCD_speed_sllow
				STI LSD_ROW_1 + 4, '='	//Когда скорость не больше чем на 10 отличается, ставим =
				RJMP Update_1_row_LCD_speed_end
			Update_1_row_LCD_speed_sllow:
				STI LSD_ROW_1 + 4, '-'	//Когда скорость меньше
				RJMP Update_1_row_LCD_speed_end
		Update_1_row_LCD_speed_posotiv:
			_LDI_32 C_32, 10 //На сколько маленькое число игнорируем
			_CP_32 B_32,C_32
			BRLO /*<*/ Update_1_row_LCD_speed_eq
				STI LSD_ROW_1 + 4, '+'	//Когда скорость больше
				RJMP Update_1_row_LCD_speed_end
			Update_1_row_LCD_speed_eq:
				STI LSD_ROW_1 + 4, '='	//Когда скорость не больше чем на 10 отличается, ставим =
				RJMP Update_1_row_LCD_speed_end
	Update_1_row_LCD_speed_end:
	
	LDIW Z, LSD_ROW_1_PAR
	LDS tmp2, LCD_display_mode
	MOV MacroR, tmp2
	CPI tmp2, DM_SCAN
	BRSH /*>=*/ Update_1_row_LCD_inMenu
		//У нас одна из строк
		CALL LCD_PRINT_workMode
		RJMP Update_1_row_LCD_inMenuend
	Update_1_row_LCD_inMenu:
		//У нас меню
		CALL LCD_PRINT_menu
	Update_1_row_LCD_inMenuend:
	SetTimerTask TS_Update_2_row_LCD,250
 RET
 //Функция обновления значений на второй строке дисплея
 Update_2_row_LCD:
 
	LDIW Z, LSD_ROW_2_PAR
	LDS tmp2, LCD_display_mode
	MOV MacroR, tmp2
	INC MacroR
	CPI tmp2, DM_SCAN
	BRSH /*>=*/ Update_2_row_LCD_inMenu
		CPI MacroR, DM_SCAN
		BRLO /*<*/ Update_2_row_LCD_wm_not_cyc
			SUBI MacroR, DM_SCAN//У нас зали за край, снова первую строку печатаем
		Update_2_row_LCD_wm_not_cyc:
		CALL LCD_PRINT_workMode
		RJMP Update_2_row_LCD_inMenuend
	Update_2_row_LCD_inMenu:
		//У нас меню
		CPI MacroR, DM_MENU_END
		BRLO /*<*/ Update_2_row_LCD_m_not_cyc
			SUBI MacroR, DM_MENU_END-DM_MENU//У нас зали за край, снова первую строку печатаем
		Update_2_row_LCD_m_not_cyc:
		CALL LCD_PRINT_menu
	Update_2_row_LCD_inMenuend:

	SetTimerTask TS_Update_3_row_LCD,250
 RET
 //Функция обновления значений на третьей строке дисплея
 Update_3_row_LCD:
	//Теперь поставим время
	_LDI_A 1000	//Так как время в мс, то делим эту тысячу
	_MOV_32 B_32, TimeTrip
	_DIV_64_R32_DIV_A B_32
	_MOV_32 B_32, ACC_32
	_LDI_A 10
	_DIV_64_R32_DIV_A B_32 //Мы делим TimeTrip на 10. Получаем в итоге десятки секунд + сами секунды
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_3 + 7, tmp2
	_MOV_32 B_32, ACC_32
	_LDI_A 6
	_DIV_64_R32_DIV_A B_32 //Теперь минуты и десятки секунд
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_3 + 6, tmp2
	STI LSD_ROW_3 + 5, ':'
	_MOV_32 B_32, ACC_32
	_LDI_A 10
	_DIV_64_R32_DIV_A B_32 //Теперь десятки минут и минуты
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_3 + 4, tmp2
	_MOV_32 B_32, ACC_32
	_LDI_A 6
	_DIV_64_R32_DIV_A B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_3 + 3, tmp2
	STI LSD_ROW_3 + 2, ':'
	_MOV_32 B_32, ACC_32
	_LDI_A 10
	_DIV_64_R32_DIV_A B_32 //Теперь десятки минут и минуты
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_3 + 1, tmp2
	LDS tmp2, ACC_1 + 7
	ADI tmp2, '0'
	STS LSD_ROW_3 + 0, tmp2
	
	
	LDIW Z, LSD_ROW_3_PAR
	LDS tmp2, LCD_display_mode
	MOV MacroR, tmp2
	ADI MacroR,2
	CPI tmp2, DM_SCAN
	BRSH /*>=*/ Update_3_row_LCD_inMenu
		CPI MacroR, DM_SCAN
		BRLO /*<*/ Update_3_row_LCD_wm_not_cyc
			SUBI MacroR, DM_SCAN//У нас зали за край, снова первую строку печатаем
		Update_3_row_LCD_wm_not_cyc:
		CALL LCD_PRINT_workMode
		RJMP Update_3_row_LCD_inMenuend
	Update_3_row_LCD_inMenu:
		//У нас меню
		CPI MacroR, DM_MENU_END
		BRLO /*<*/ Update_3_row_LCD_m_not_cyc
			SUBI MacroR, DM_MENU_END-DM_MENU//У нас зали за край, снова первую строку печатаем
		Update_3_row_LCD_m_not_cyc:
		CALL LCD_PRINT_menu
	Update_3_row_LCD_inMenuend:

	SetTimerTask TS_Update_4_row_LCD,250
 RET
 //Функция обновления значений на четвёртой строке дисплея
 Update_4_row_LCD:

 
	//Теперь одометр
	_CLR_64 ACC_1
	_MOV_32 ACC_32, Dist
	_MUL64_A_R32 WheelLength	//Количество оборотов на длину колеса - расстояние, в мм
	_LDI_32 B_32, 1000
	_DIV_64_A_DIV_R32 B_32	//Теперь путь в метрах
	_MOV_32 B_32, ACC_32

	_LDI_A 10
	_DIV_64_R32_DIV_A B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_4 + 6, tmp2
	_MOV_32 B_32, ACC_32

	_LDI_A 10
	_DIV_64_R32_DIV_A B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_4 + 5, tmp2
	_MOV_32 B_32, ACC_32
	
	_LDI_A 10
	_DIV_64_R32_DIV_A B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_4 + 4, tmp2
	STI LSD_ROW_4 + 3, '.'
	_MOV_32 B_32, ACC_32

	_LDI_A 10
	_DIV_64_R32_DIV_A B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_4 + 2, tmp2
	_MOV_32 B_32, ACC_32

	_LDI_A 10
	_DIV_64_R32_DIV_A B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_4 + 1, tmp2
	_MOV_32 B_32, ACC_32

	_LDI_A 10
	_DIV_64_R32_DIV_A B_32
	LDS tmp2, ACC_2 + 7
	ADI tmp2, '0'
	STS LSD_ROW_4 + 0, tmp2
	_MOV_32 B_32, ACC_32
		
	LDIW Z, LSD_ROW_4_PAR
	LDS tmp2, LCD_display_mode
	MOV MacroR, tmp2
	ADI MacroR,3
	CPI tmp2, DM_SCAN
	BRSH /*>=*/ Update_4_row_LCD_inMenu
		CPI MacroR, DM_SCAN
		BRLO /*<*/ Update_4_row_LCD_wm_not_cyc
			SUBI MacroR, DM_SCAN//У нас зали за край, снова первую строку печатаем
		Update_4_row_LCD_wm_not_cyc:
		CALL LCD_PRINT_workMode
		RJMP Update_4_row_LCD_inMenuend
	Update_4_row_LCD_inMenu:
		//У нас меню
		CPI MacroR, DM_MENU_END
		BRLO /*<*/ Update_4_row_LCD_m_not_cyc
			SUBI MacroR, DM_MENU_END-DM_MENU//У нас зали за край, снова первую строку печатаем
		Update_4_row_LCD_m_not_cyc:
		CALL LCD_PRINT_menu
	Update_4_row_LCD_inMenuend:

	SetTimerTask TS_Update_1_row_LCD,250
 RET


//Функция обновления часов. 
ClockUpdate:
	TSTFlag TWI_Busy
	BRNE	CU_busy
		STI TWI_IO+0, DS1307Write
		STI TWI_IO+1, 0	
		STI TWI_IOl, 2 ;Записываем 2 регистра - адрес и чтение будем осуществлять с 0
		CALL StartTWI
		SetSlowTimerTask TS_ClockUpdate,60//Один раз в час мы сверяем часы с базовой станцией
		RET
	CU_busy:
	//Линия занята, обратитесь позже
	SetTimerTask TS_ClockUpdate,1
RET

//Функция обновления погоды
WetherUpdate:
	TSTFlag TWI_Busy
	BRNE	WU00
		STI TWI_IO+0, BME250Write
		STI TWI_IO+1, BME_ADR_status //Читать будем от статуса и далее
		STI TWI_IOl, 2 ;Записываем 2 регистра - адрес и чтение будем осуществлять с 0
		CALL StartTWI
		SetTimerTask TS_WetherUpdate,10000//Один раз в 10 секунд обновляться - за глаза и за уши!
		RET
	WU00:
	//Линия занята, обратитесь позже*
	SetTimerTask TS_WetherUpdate,1
RET