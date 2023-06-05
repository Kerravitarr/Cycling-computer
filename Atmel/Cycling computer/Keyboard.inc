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

	Key_E_menu:
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
	
 RET

 //Нажали кнопку вправо
 Key_R:
	
 RET

 //Нажали кнопку влево
 Key_L:
	
 RET