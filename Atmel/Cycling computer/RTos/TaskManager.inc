/*
 * TaskManager.inc
 *
 *  Created: 23.04.2019 18:51:56
 *   Author: Terran
 */ 

 
TaskProcs: 	.dw Idle					; [00] 
			.equ TS_Idle 			= 0		; 

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
			
			
			