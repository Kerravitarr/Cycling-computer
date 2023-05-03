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
			.dw	StartTWI				
			.equ TS_StartTWI		= 3
			.dw SlowTimerService		
			.equ TS_SlowTimerService= 4
			.dw	UART_End				
			.equ TS_UART_End		= 5
			.dw Evry_40_ms_Task
			.equ TS_Evry_40_ms_Task = 6
			.dw Evry_1_ms_Task
			.equ TS_Evry_1_ms_Task = 7
			.dw I
			.equ TS_I				= 8
			.dw ReedSwitch_E
			.equ TS_ReedSwitch_E	= 9
			.dw O
			.equ TS_O				= 10
			.dw Init_Scrin
			.equ TS_Init_Scrin		= 11
			.dw LCD_update
			.equ TS_LCD_update		= 12
			.dw Evry_1_s_Task
			.equ TS_Evry_1_s_Task	= 13

			

			
			
			
			