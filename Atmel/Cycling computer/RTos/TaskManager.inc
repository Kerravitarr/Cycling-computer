/*
 * TaskManager.inc
 *
 *  Created: 23.04.2019 18:51:56
 *   Author: Terran
 */ 

//ƒиспетчер задач. ќпредел¤ет команды, которые можно выполн¤ть
TaskProcs: 	.dw Idle					; јдрес команды, где находитс¤ функци¤
			.equ TS_Idle 			= 0	; »ндекс этой команды в этом массиве
			.dw	TWI_IO_End
			.equ TS_TWI_IO_End		= 1	
			.dw	TWI_IO_Error
			.equ TS_TWI_IO_Error	= 2
			.dw SlowTimerService		
			.equ TS_SlowTimerService= 3
			.dw	UART_End				
			.equ TS_UART_End		= 4
			.dw Evry_40_ms_Task
			.equ TS_Evry_40_ms_Task = 5
			.dw Evry_1_ms_Task
			.equ TS_Evry_1_ms_Task	= 6
			.dw I
			.equ TS_I				= 7
			.dw ReedSwitch_E
			.equ TS_ReedSwitch_E	= 8
			.dw ClockUpdate
			.equ TS_ClockUpdate		= 9
			.dw O
			.equ TS_O				= 10
			.dw Init_Scrin
			.equ TS_Init_Scrin		= 11
			.dw LCD_update
			.equ TS_LCD_update		= 12
			.dw Evry_1_s_Task
			.equ TS_Evry_1_s_Task	= 13
			.dw Update_1_row_LCD
			.equ TS_Update_1_row_LCD= 14
			.dw Update_2_row_LCD
			.equ TS_Update_2_row_LCD= 15
			.dw Update_3_row_LCD
			.equ TS_Update_3_row_LCD= 16
			.dw Update_4_row_LCD
			.equ TS_Update_4_row_LCD= 17
			.dw WetherInit
			.equ TS_WetherInit		= 18
			.dw WetherUpdate
			.equ TS_WetherUpdate	= 19
			.dw BME250_read_P
			.equ TS_BME250_read_P	= 20
			.dw BME250_read_H
			.equ TS_BME250_read_H	= 21
			
			.dw Key_E
			.equ TS_Key_E			= 22
			.dw Key_UP
			.equ TS_Key_UP			= 23
			.dw Key_R
			.equ TS_Key_R			= 24
			.dw Key_L
			.equ TS_Key_L			= 25
			.dw Key_D
			.equ TS_Key_D			= 26

			.dw RestartTrip
			.equ TS_RestartTrip		= 27
			.dw EEPROM_CLR
			.equ TS_EEPROM_CLR		= 28
			.dw EEPROM_Read
			.equ TS_EEPROM_Read		= 29
			.dw EEPROM_SET
			.equ TS_EEPROM_SET		= 30
			.dw EEPROM_STEP
			.equ TS_EEPROM_STEP		= 31

			
			
			
			