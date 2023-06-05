/*
 * Clock.inc
 *
 *  Created: 23.04.2019 18:42:42
 *   Author: Terran
 */ 
 TWI_IO_End_Clock_Write:
	STI	TWI_IO, DS1307Read
	STI	TWI_IOl, 7
	RCALL StartTWI
RET

 TWI_IO_End_Clock_Read:
	LDS	tmp2, TWI_IO + 1	;Десятки секунд и секунды
	MOV tmp3, tmp2
	ANDI tmp3, 0x0F
	ADI tmp3, '0'
	STS Clock + 7, tmp3
	LSR tmp2
	LSR tmp2
	LSR tmp2
	LSR tmp2
	ANDI tmp2, 0x0F
	ADI tmp2, '0'
	STS Clock + 6, tmp2
	STI Clock + 5, ':'
	LDS	tmp2, TWI_IO + 2	;Десятки минут и минуты
	MOV tmp3, tmp2
	ANDI tmp3, 0x0F
	ADI tmp3, '0'
	STS Clock + 4, tmp3
	LSR tmp2
	LSR tmp2
	LSR tmp2
	LSR tmp2
	ANDI tmp2, 0x0F
	ADI tmp2, '0'
	STS Clock + 3, tmp2
	STI Clock + 2, ':'
	LDS	tmp2, TWI_IO + 3	;Десятки часов и часовое месево
	MOV tmp3, tmp2
	ANDI tmp3, 0x0F
	ADI tmp3, '0'
	STS Clock + 1, tmp3
	LSR tmp2
	LSR tmp2
	LSR tmp2
	LSR tmp2
	ANDI tmp2, 0x03
	ADI tmp2, '0'
	STS Clock + 0, tmp2
RET