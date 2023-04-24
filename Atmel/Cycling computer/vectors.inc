/*
 * vectors.inc
 *
 *  Created: 16.11.2018 7:11:51
 *   Author: Terran
 */ 
.CSEG
.ORG 	0x0000          ; RESET - сброс			; Проц стартует с нуля, но дальше идут вектора 
JMP 	Reset			; прерываний, поэтому отсяюда сразу же прыгаем
						; на начало программы. На метку Reset
						; Тут распологается таблица векторов важнейшая вещь. Если про нее забыть, то
						; можно огрести такие неразрешимые глюки, что просто капец. Так что не забываем.
						; я его тупо скопировал из файла m8def.inc и заменил все .equ на .org, а потом
						; дописал везде команду RETI. На случай если прерывание вдруг вылезет, чтобы 
						; вернулось туда же откуда прервалось. 
.ORG	0x0002			;INT0 - внешнее прерывание 0
RJMP	DataIN
.ORG	0x0004			;INT1 - внешнее прерывание 1
RJMP	ReedSwitch 
.ORG	0x0006			;PCINT0 - прерывание по изменению состояния нулевой группы выводов
RETI
.ORG	0x0008			;PCINT1 - прерывание по изменению состояния первой группы выводов
RETI
.ORG	0x000A			;PCINT2 - прерывание по изменению состояния второй группы выводов
RETI
.ORG	0x000C			;WDT - прерывание от сторожевого таймера
RJMP 	WDT			//Попробуем просто продолжить с начала
.ORG	0x000E			;TIMER2 COMPA - прерывание от таймера/счетчика T2 при совпадении с A
RJMP	OutComp2Int
.ORG	0x0010			;TIMER2 COMPB - прерывание от таймера/счетчика T2 при совпадении с B
RETI
.ORG	OVF2addr		; Переполнение таймера/счетчика Т2
RETI
.ORG	ICP1addr		; Захват таймера/счетчика Т1
RETI
.ORG	OC1Aaddr		; Совпадение «А» таймера/счетчика Т1
RETI
.ORG	OC1Baddr		; Timer/Counter1 Compare Match B
						;Совпадение «В» таймера/счетчика Т1
RETI
.ORG	OVF1addr		; Timer/Counter1 Overflow
						;Переполнение таймера/счетчика Т1
RETI
.ORG	OVF0addr		; Timer/Counter0 Overflow
						;Переполнение таймера/счетчика Т0
RETI
.ORG	SPIaddr			; Serial Transfer Complete
RETI					;Передача по SPI завершена

.ORG	URXCaddr		; USART, Rx Complete
RJMP	UARTCaddr		;USART, прием завершен

.ORG	UDREaddr		; USART Data Register Empty
RJMP	UARTEaddr		;Регистр данных USART пуст

.ORG	UTXCaddr		; USART, Tx Complete
						;USART, передача завершена
RETI
.ORG	ADCCaddr		; ADC Conversion Complete
						;Преобразование АЦП завершено
RETI

.ORG	ERDYaddr		; EEPROM Ready
RJMP EEPROMrdy			;EEPROM, готово

.ORG	ACIaddr			; Analog Comparator
						;Аналоговый компаратор
RETI
.ORG	TWIaddr			; 2-wire Serial Interface
RJMP	TWI_Event		;Прерывание от модуля TWI

.ORG	SPMRaddr		; Store Program Memory Ready
						;Готовность SPM
RETI

.ORG	INT_VECTORS_SIZE		; Конец таблицы прерываний