/*
 * IncFile1.inc
 *
 *  Created: 16.11.2018 7:08:42
 *   Author: Terran
 */ 

 //Очищает флаг из памяти
 .MACRO CBIFlag
	.IF	@0 < 8
		LDS MacroR,Flag_2
		ANDI MacroR, ~(1 << @0)
		STS Flag_2,MacroR       
	.ELSE
		LDS MacroR,Flag_2+1
		ANDI MacroR, ~(1 << @0)
		STS Flag_2+1,MacroR   
	.endif
 .ENDMACRO
 //Тестирует флаг в памяти
  .MACRO TSTFlag
	LDS MacroR,Flag_2
	ANDI MacroR,(1 << @0)
 .ENDMACRO
 //Сохраняет флаг в памяти
 .MACRO SBIFlag
	LDS MacroR,Flag
	ORI MacroR, (1 << @0)
	STS Flag,MacroR
 .ENDMACRO

 .MACRO 	OUTI				; Ёто описание макроса. ¬езде где outi встретитс¤ в коде, то замен¤етс¤ на 
	LDI 	R16,@1 			; этот кусок кода, причем @0,@1 это параметры, они замен¤тс¤ введенными параметрами
	OUT 	@0,R16 			; макроса. ?анный макрос тупо копирует введенное число сначала в регистр R16, а из него 
.ENDMACRO

.MACRO 	EORI
	PUSH	r16
	LDI 	r16,@1
	EOR		@0,r16
	POP		r16
.ENDMACRO

.MACRO    UOUT        
   	.IF	@0 < 0x40
      	OUT	@0,@1         
	.ELSE
      	STS	@0,@1
   	.endif
.ENDMACRO
//Печатает символ в UART, символ - константа
 .MACRO UARTPrint
	LDI MacroR,@0
	CALL UART_Print
 .ENDM
 
//Печатает символ в UART, вход - регистр
 .MACRO UARTRPrint
	MOV MacroR,@0
	CALL UART_Print
 .ENDM

.MACRO		STI
	LDI		OSRG,@1
	STS		@0,OSRG
.ENDMACRO

//Прибавляет константу
.MACRO		ADI
	SUBI	@0,-@1
.ENDMACRO


.MACRO		ThosIsOtladka
		LDI		tmp2,1
		STS		UART_O,tmp2
		LDI		tmp2,@0
		STS		UART_O+1,tmp2
		SetTask TS_StartUART
.ENDMACRO



/************************************************************************************************/

;==========================================================================
;==========================================================================
.MACRO SetTask
		ldi OSRG, @0			; Обращение к процедуре через событийный диспетчер
		call SendTask				; 
.ENDMACRO
;==========================================================================
//Запустить задачу TS_@0 на @1 милисекунд (от 0 до 65535 мс(65 сек))
.MACRO SetTimerTask
		LDI 	OSRG, @0
		LDI 	XL, Low(@1)			;
		LDI 	XH, High(@1)			; ЗАдержка в милисекундах
		CALL 	SetTimer
.ENDM

//Запустить задачу TS_@0 на @1 минут (от 0 до 255 мин. ((4 часа)))
.MACRO SetSlowTimerTask
		LDI 	OSRG, @0
		LDI 	XL, Low(@1)			;Задержка в минутах
		CALL 	SetSlowTimer
.ENDM

;=======================================================================================
;FLASH
;======================================================================================

.MACRO	SetScrinKeyWord
		LDI		OSRG,@0
		CALL SetScrinKeyWord_M
.ENDMACRO


.MACRO	SetScrinWord
		MOV		OSRG,@0
		CALL SetScrinWord_M
.ENDMACRO

.MACRO	NewScrinUpdate
		LDI		OSRG,0
		STS		StringOutTwo+1,OSRG //Всё, больше грузить не надо, все флаги на ноль, курсор экрана в 0 позицию
		STS		StringOutOne+1,OSRG
		STS		StringOutThree+1,OSRG
		STS		StringOutFour+1,OSRG
		//Flag_2 = 0b ~ ~ ~ ~ Scr left(T=1) Scr Sleep
			LDS OSRG,Flag_2
		ANDI OSRG,0b11110101
			STS Flag_2,OSRG
//		SetScrinKeyWord ScrinKeyClear
//		SetTask TS_O
		SetTimerTask TS_ScrinUpdate,10
.ENDMACRO