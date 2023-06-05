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
		ANDI MacroR, ~(1 << (@0 - 8))
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
	.IF	@0 < 8
		LDS MacroR,Flag_2
	ORI MacroR, (1 << @0)
		STS Flag_2,MacroR       
	.ELSE
		LDS MacroR,Flag_2+1
		ORI MacroR, (1 << (@0 - 8))
		STS Flag_2+1,MacroR   
	.endif
 .ENDMACRO

 .MACRO 	OUTI				; Ёто описание макроса. ¬езде где outi встретитс¤ в коде, то замен¤етс¤ на 
	LDI 	MacroR,@1 			; этот кусок кода, причем @0,@1 это параметры, они замен¤тс¤ введенными параметрами
	OUT 	@0,MacroR 			; макроса. ?анный макрос тупо копирует введенное число сначала в регистр R16, а из него 
.ENDMACRO

.MACRO 	EORI
	LDI 	MacroR,@1
	EOR		@0,MacroR
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
 //Сохраняет число в память @0 = @1
.MACRO		STI
	LDI		MacroR,@1
	STS		@0,MacroR
.ENDMACRO

//Прибавляет константу
.MACRO		ADI
	SUBI	@0,-(@1)
.ENDMACRO

//Загружает в регистровую пару длинное число. @0=@1
.MACRO LDIW
	LDI    @0L, LOW(@1)
	LDI    @0H, HIGH(@1)
.ENDMACRO