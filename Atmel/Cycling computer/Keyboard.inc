/*
 * Keyboard.inc
 * ���� �������� ����������
 *  Created: 03.06.2023 20:30:12
 *   Author: ????
 */ 

 //������ ������ ������
 Key_E:
	LDS tmp2, LCD_display_mode
	CPI tmp2, DM_SCAN
	BRSH /*>=*/ Key_E_menu
		//� ��� ������� ����� ��������� ������, ��������� � ����!
		CBIFlag F_LSD_SCAN
		STI LCD_display_mode, DM_MENU
		STI LCD_cursor_pos,0

	Key_E_menu:
 RET
 
 //������ ������ �����
 Key_UP:
	LDS tmp2, LCD_display_mode
	CPI tmp2, DM_SCAN
	BRSH /*>=*/ Key_UP_menu
		//� ��� ������� ����� ��������� ������
		DEC tmp2
		BRPL /*+*/ Key_UP_notscan
			//�������� ����� ������������
			SBIFlag F_LSD_SCAN
			//�� � �� �������� �������, � ��� ����
			STI LCD_display_mode, DM_SCAN - 1
			RET
		Key_UP_notscan:
			CBIFlag F_LSD_SCAN
			STS LCD_display_mode, tmp2
			RET
	Key_UP_menu:
 RET
 
 //������ ������ ����
 Key_D:
	LDS tmp2, LCD_display_mode
	CPI tmp2, DM_SCAN
	BRSH /*>=*/ Key_D_menu
		//� ��� ������� ����� ��������� ������
		INC tmp2
		CPI tmp2, DM_SCAN
		BRNE /*!=*/ Key_D_notscan
			//�������� ����� ������������
			SBIFlag F_LSD_SCAN
			//�� � �� �������� �������, � ��� ����
			STI LCD_display_mode, DM_DEFAULT
			RET
		Key_D_notscan:
			CBIFlag F_LSD_SCAN
			STS LCD_display_mode, tmp2
			RET
	Key_D_menu:
	
 RET

 //������ ������ ������
 Key_R:
	
 RET

 //������ ������ �����
 Key_L:
	
 RET