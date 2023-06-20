/*
 * Keyboard.inc
 * Файл посвящён клавиатуре
 *  Created: 03.06.2023 20:30:12
 *   Author: ????
 */ 

 //Нажали кнопку ентера
 Key_E:
	LDS tmp2, LCD_display_mode
	CPI tmp2, DM_SCAN
	BRSH /*>=*/ Key_E_menu
		//У нас обычный режим просмотра данных, переходим в меню!
		CBIFlag F_LSD_SCAN
		STI LCD_display_mode, DM_MENU
		STI LCD_cursor_pos,0
		RJMP Key_E_ret
	Key_E_menu:
		//Мы в режиме менюшки
		CPI tmp2, DM_MENU_RESTART
		BRNE /*!=*/ Key_E_not_restart
			SetTask TS_RestartTrip
			STI LCD_display_mode, DM_DEFAULT
			RJMP Key_E_ret
		Key_E_not_restart:
		CPI tmp2,DM_MENU_TIME_TRIP
		BRNE /*!=*/ Key_E_not_TT
			TSTFlag F_isTLeft
			BREQ /*=*/ Key_E_TTfalse
				CBIFlag F_isTLeft
				RJMP Key_E_ret
			Key_E_TTfalse:
				SBIFlag F_isTLeft
				RJMP Key_E_ret
		Key_E_not_TT:
		CPI tmp2, DM_MENU_EXIT
		BRNE /*!=*/ Key_E_not_exit
			STI LCD_display_mode, DM_DEFAULT
			RJMP Key_E_ret
		Key_E_not_exit:
		CPI tmp2, DM_MENU_EEPROM_CL
		BRNE /*!=*/ Key_E_not_eeclr
			SetTask TS_EEPROM_CLR
			STI LCD_display_mode, DM_DEFAULT
			RJMP Key_E_ret
		Key_E_not_eeclr:

	Key_E_ret:
 RET
 
 //Нажали кнопку вверх
 Key_UP:
	LDS tmp2, LCD_display_mode
	CPI tmp2, DM_SCAN
	BRSH /*>=*/ Key_UP_menu
		//У нас обычный режим просмотра данных
		DEC tmp2
		BRPL /*+*/ Key_UP_notscan
			//Включаем режим сканирования
			SBIFlag F_LSD_SCAN
			//Но и мы обнуляем строчку, у нас круг
			STI LCD_display_mode, DM_SCAN - 1
			RET
		Key_UP_notscan:
			CBIFlag F_LSD_SCAN
			STS LCD_display_mode, tmp2
			RET
	Key_UP_menu:
		LDS tmp3, LCD_cursor_pos
		TST tmp3
		BRNE /*!=*/ Key_UPcmd
			//У нас курсор в 0, мы просто двигаем меню
			DEC tmp2
			CPI tmp2, DM_SCAN
			BREQ /*=*/ Key_UP_cyc
				STS LCD_display_mode, tmp2
				RET
			Key_UP_cyc:
				//Mы обнуляем строчку, у нас круг
				STI LCD_display_mode, DM_MENU_END - 1
				RET
		Key_UPcmd:

		CPI tmp2, DM_MENU_TIME_TRIP
		BREQ /*=*/ Key_UP_TT
			RJMP Key_UP_not_TT
		Key_UP_TT:
			TSTFlag F_isTLeft
			BREQ /*=*/ Key_UP_TT_not_t
				//Оставшееся время
				CPI tmp3, 2
				BRNE /*!=*/ Key_UP_TT_t_not2
					_LDI_32 B_32, 60*60*10 //+= 10 часов
					RJMP  Key_UP_TT_t_add
				Key_UP_TT_t_not2:
				CPI tmp3, 3
				BRNE /*!=*/ Key_UP_TT_t_not3
					_LDI_32 B_32, 60*60*1 //+= 1 час
					RJMP  Key_UP_TT_t_add
				Key_UP_TT_t_not3:
				CPI tmp3, 5
				BRNE /*!=*/ Key_UP_TT_t_not5
					_LDI_32 B_32, 60*10	//+= 10 мин
					RJMP  Key_UP_TT_t_add
				Key_UP_TT_t_not5:
					_LDI_32 B_32, 60 * 1

				Key_UP_TT_t_add:
				_ADD_32	LeftTimeDist, B_32 //+= add c
				RET 
			Key_UP_TT_not_t:
				//Оставшаяся дистанция, в оборотах колеса!!!
				CPI tmp3, 3
				BRNE /*!=*/ Key_UP_TT_not_t_not3
					_LDI_32 ACC_32, 100*1000*1000 //+= 100 км
					RJMP  Key_UP_TT_not_t_add
				Key_UP_TT_not_t_not3:
				CPI tmp3, 4
				BRNE /*!=*/ Key_UP_TT_not_t_not4
					_LDI_32 ACC_32, 10*1000*1000	//+= 10 км
					RJMP  Key_UP_TT_not_t_add
				Key_UP_TT_not_t_not4:
					_LDI_32 ACC_32, 1*1000*1000
				Key_UP_TT_not_t_add:
				//Дистанция в мм, а нужно в оборотах!
				EXP_64_F_32 ACC_1
				_DIV_64_A_DIV_R32 WheelLength
				_ADD_32 LeftTimeDist, ACC_32
				RET 
		Key_UP_not_TT:
		
		CPI tmp2, DM_MENU_WheelLength
		BRNE /*!=*/ Key_UP_not_WL
			//Длина колеса
			CPI tmp3, 3
			BRNE /*!=*/ Key_UP_WLnot3
				_LDI_32 B_32, 1000 //+= 1000мм
				RJMP Key_UP_WLadd
			Key_UP_WLnot3:
			CPI tmp3, 4
			BRNE /*!=*/ Key_UP_WLnot4
				_LDI_32 B_32, 100 //+= 100мм
				RJMP Key_UP_WLadd
			Key_UP_WLnot4:
			CPI tmp3, 5
			BRNE /*!=*/ Key_UP_WLnot5
				_LDI_32 B_32, 10 //+= 10мм
				RJMP Key_UP_WLadd
			Key_UP_WLnot5:
				_LDI_32 B_32, 1 //+= 1мм
			Key_UP_WLadd:
			_ADD_32 WheelLength, B_32
			RET
		Key_UP_not_WL:
 RET
 
 //Нажали кнопку вниз
 Key_D:
	LDS tmp2, LCD_display_mode
	CPI tmp2, DM_SCAN
	BRSH /*>=*/ Key_D_menu
		//У нас обычный режим просмотра данных
		INC tmp2
		CPI tmp2, DM_SCAN
		BRNE /*!=*/ Key_D_notscan
			//Включаем режим сканирования
			SBIFlag F_LSD_SCAN
			//Но и мы обнуляем строчку, у нас круг
			STI LCD_display_mode, DM_DEFAULT
			RET
		Key_D_notscan:
			CBIFlag F_LSD_SCAN
			STS LCD_display_mode, tmp2
			RET
	Key_D_menu:
		LDS tmp3, LCD_cursor_pos
		TST tmp3
		BRNE /*!=*/ Key_Dcmd
			//У нас курсор в 0, мы просто двигаем меню
			INC tmp2
			CPI tmp2, DM_MENU_END
			BRNE /*!=*/ Key_D_notcyc
				//Mы обнуляем строчку, у нас круг
				STI LCD_display_mode, DM_MENU
				RET
			Key_D_notcyc:
				STS LCD_display_mode, tmp2
				RET
		Key_Dcmd:	

		CPI tmp2, DM_MENU_TIME_TRIP
		BREQ /*=*/ Key_D_TT
			RJMP Key_D_not_TT
		Key_D_TT:
			TSTFlag F_isTLeft
			BREQ /*=*/ Key_D_TT_t
				RJMP Key_D_TT_not_t
			Key_D_TT_t:
				//Оставшееся время
				CPI tmp3, 2
				BRNE /*!=*/ Key_D_TT_t_not2
					_LDI_32 B_32, 60*60*10 //-= 10 часов
					RJMP  Key_D_TT_t_sub
				Key_D_TT_t_not2:
				CPI tmp3, 3
				BRNE /*!=*/ Key_D_TT_t_not3
					_LDI_32 B_32, 60*60*1 //-= 1 час
					RJMP  Key_D_TT_t_sub
				Key_D_TT_t_not3:
				CPI tmp3, 5
				BRNE /*!=*/ Key_D_TT_t_not5
					_LDI_32 B_32, 60*10	//-= 10 мин
					RJMP  Key_D_TT_t_sub
				Key_D_TT_t_not5:
					_LDI_32 B_32, 60 * 1

				Key_D_TT_t_sub:
				_SUB_32	LeftTimeDist, B_32 //-= B
				_TST_32 LeftTimeDist
				BRPL /*+*/ Key_D_TT_t_ret
					//Не может быть отрицательным!
					_CLR_32 LeftTimeDist
				Key_D_TT_t_ret:
				RET 
			Key_D_TT_not_t:
				//Оставшаяся дистанция, в оборотах колеса!!!
				CPI tmp3, 3
				BRNE /*!=*/ Key_D_TT_not_t_not3
					_LDI_32 ACC_32, 100*1000*1000 //+= 100 км
					RJMP  Key_D_TT_not_t_sub
				Key_D_TT_not_t_not3:
				CPI tmp3, 4
				BRNE /*!=*/ Key_D_TT_not_t_not4
					_LDI_32 ACC_32, 10*1000*1000	//+= 10 км
					RJMP  Key_D_TT_not_t_sub
				Key_D_TT_not_t_not4:
					_LDI_32 ACC_32, 1*1000*1000
				Key_D_TT_not_t_sub:
				//Дистанция в мм, а нужно в оборотах!
				EXP_64_F_32 ACC_1
				_DIV_64_A_DIV_R32 WheelLength
				_SUB_32 LeftTimeDist, ACC_32  //-= B
				_TST_32 LeftTimeDist
				BRPL /*+*/ Key_D_TT_not_t_ret
					//Не может быть отрицательным!
					_CLR_32 LeftTimeDist
				Key_D_TT_not_t_ret:
				RET 
		Key_D_not_TT:
		CPI tmp2, DM_MENU_WheelLength
		BRNE /*!=*/ Key_D_not_WL
			//Длина колеса
			CPI tmp3, 3
			BRNE /*!=*/ Key_D_WLnot3
				_LDI_32 B_32, 1000 //-= 1000мм
				RJMP Key_D_WLsub
			Key_D_WLnot3:
			CPI tmp3, 4
			BRNE /*!=*/ Key_D_WLnot4
				_LDI_32 B_32, 100 //-= 100мм
				RJMP Key_D_WLsub
			Key_D_WLnot4:
			CPI tmp3, 5
			BRNE /*!=*/ Key_D_WLnot5
				_LDI_32 B_32, 10 //-= 10мм
				RJMP Key_D_WLsub
			Key_D_WLnot5:
				_LDI_32 B_32, 1 //-= 1мм
			Key_D_WLsub:
			_SUB_32 WheelLength, B_32
			_TST_32 WheelLength
			BRPL /*+*/ Key_D_WL_ret
				//Не может быть отрицательным!
				_CLR_32 WheelLength
			Key_D_WL_ret:
			RET
		Key_D_not_WL:
 RET

 //Нажали кнопку вправо
 Key_R:
	LDS tmp3,LCD_cursor_pos
	CPI tmp3, LSD_MENU_ROW_LENGHT - 1
	BRNE /*!=*/ Key_R_can_r
		//Мы уже правее некуда, выходим
		RET
	Key_R_can_r:

	LDS tmp2, LCD_display_mode
	CPI tmp2, DM_MENU_TIME_TRIP
	BRNE /*!=*/ Key_R_not_TT
		TSTFlag F_isTLeft
		BREQ /*=*/ Key_R_TT_not_t
			//Оставшееся время
			CPI tmp3, 0
			BRNE /*!=*/ Key_R_TT_t_not_0
				STI LCD_cursor_pos, 2
				RET
			Key_R_TT_t_not_0:
			CPI tmp3, 2
			BRNE /*!=*/ Key_R_TT_t_not_2
				STI LCD_cursor_pos, 3
				RET
			Key_R_TT_t_not_2:
			CPI tmp3, 3
			BRNE /*!=*/ Key_R_TT_t_not_3
				STI LCD_cursor_pos, 5
				RET
			Key_R_TT_t_not_3:
				STI LCD_cursor_pos, 6
				RET
		Key_R_TT_not_t:
			//Оставшиеся км
			CPI tmp3, 0
			BRNE /*!=*/ Key_R_TT_not_t_not_0
				//Курсор на нуле
				STI LCD_cursor_pos, 3
				RET
			Key_R_TT_not_t_not_0:
			CPI tmp3, 3
			BRNE /*!=*/ Key_R_TT_not_t_not_3
				STI LCD_cursor_pos, 4
				RET
			Key_R_TT_not_t_not_3:
			//В остальных случаях притыкаем его к концу
			STI LCD_cursor_pos, 5
			RET
	Key_R_not_TT:

	CPI tmp2, DM_MENU_WheelLength
	BRNE /*!=*/ Key_R_not_WL
		//Длина колеса
		CPI tmp3, 0
		BRNE /*!=*/ Key_R_WL_not0
			STI LCD_cursor_pos, 3
			RET
		Key_R_WL_not0:
		CPI tmp3, 3
		BRNE /*!=*/ Key_R_WL_not3
			STI LCD_cursor_pos, 4
			RET
		Key_R_WL_not3:
		CPI tmp3, 4
		BRNE /*!=*/ Key_R_WL_not4
			STI LCD_cursor_pos, 5
			RET
		Key_R_WL_not4:
		STI LCD_cursor_pos, 6
		RET
	Key_R_not_WL:
	
 RET

 //Нажали кнопку влево
 Key_L:
	LDS tmp3,LCD_cursor_pos
	LDS tmp2, LCD_display_mode
	CPI tmp2, DM_MENU_TIME_TRIP
	BRNE /*!=*/ Key_L_not_TT
		TSTFlag F_isTLeft
		BREQ /*=*/ Key_L_TT_not_t
			//Оставшееся время
			CPI tmp3, 6
			BRNE /*!=*/ Key_L_TT_t_not_6
				STI LCD_cursor_pos, 5
				RET
			Key_L_TT_t_not_6:
			CPI tmp3, 5
			BRNE /*!=*/ Key_L_TT_t_not_5
				STI LCD_cursor_pos, 3
				RET
			Key_L_TT_t_not_5:
			CPI tmp3, 3
			BRNE /*!=*/ Key_L_TT_t_not_3
				STI LCD_cursor_pos, 2
				RET
			Key_L_TT_t_not_3:
				STI LCD_cursor_pos, 0
				RET
		Key_L_TT_not_t:
			//Оставшиеся км
			CPI tmp3, 5
			BRNE /*!=*/ Key_L_TT_not_t_not_5
				STI LCD_cursor_pos, 4
				RET
			Key_L_TT_not_t_not_5:
			CPI tmp3, 4
			BRNE /*!=*/ Key_L_TT_not_t_not_4
				STI LCD_cursor_pos, 3
				RET
			Key_L_TT_not_t_not_4:
				STI LCD_cursor_pos, 0
				RET
	Key_L_not_TT:

	CPI tmp2, DM_MENU_WheelLength
	BRNE /*!=*/ Key_L_not_WL
		//Длина колеса
		CPI tmp3, 6
		BRNE /*!=*/ Key_L_WL_not6
			STI LCD_cursor_pos, 5
			RET
		Key_L_WL_not6:
		CPI tmp3, 5
		BRNE /*!=*/ Key_L_WL_not5
			STI LCD_cursor_pos, 4
			RET
		Key_L_WL_not5:
		CPI tmp3, 4
		BRNE /*!=*/ Key_L_WL_not4
			STI LCD_cursor_pos, 3
			RET
		Key_L_WL_not4:
		//Возвращаем курсор на старт и сохраняем значение в память
		STI LCD_cursor_pos, 0
		SetTask TS_EEPROM_SET
		RET
	Key_L_not_WL:
 RET