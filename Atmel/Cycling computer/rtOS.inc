/*
 * rtOS.inc
 *
 *  Created: 16.11.2018 17:40:44
 *   Author: Terran
 */ 
 
;==========================================================================
.MACRO SetTask
		ldi MacroR, @0			; Обращение к процедуре через событийный диспетчер
		call SendTask			; 
.ENDMACRO
;==========================================================================
//Запустить задачу TS_@0 на @1 милисекунд (от 0 до 65535 мс(65 сек))
.MACRO SetTimerTask
		LDI 	MacroR, @0
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

 ;------------------------------------------------------------------------------
ProcessTaskQueue:
	ldi 	ZL, low(TaskQueue)
	ldi 	ZH, high(TaskQueue)

	ld 		tmp2, Z		; For Event
	cpi 	tmp2, $FF	; No Event or Addr out of Range
	BREQ /*=*/ 	PTQL02	; У нас нет  задач в очереди
	
	clr 	ZH
	lsl 	tmp2
	mov 	ZL, tmp2

	subi 	ZL, low(-TaskProcs*2)
	sbci 	ZH, high(-TaskProcs*2) ; Add
	
	lpm					; mov r0 <- CODE[Z]
	mov 	tmp2, r0
	ld 		r0, Z+		; inc Z
	lpm	
	mov 	ZL, tmp2	; Get Addr
	mov 	ZH, r0
	
	push 	ZL
	push 	ZH

		; Сдвижка очереди на один пункт к началу
		ldi 	Counter, TaskQueueSize-1
		ldi 	ZL, low(TaskQueue)
		ldi 	ZH, high(TaskQueue)
	
		cli
			PTQL01:
				ldd 	tmp2, Z+1 		;Взяли следующий пункт
				st 		Z+, tmp2		;Сохранили в текущий
				dec 	Counter		
			brne 	PTQL01		; Loop
			ldi 	tmp2, $FF	;Добавили в конец пустоту
			st 		Z+, tmp2		
		sei

	pop 	ZH
	pop 	ZL

	ijmp 			; Прыгаем на выполнение задачи
PTQL02:	ret	

;-------------------------------------------------------------------------
; OSRG - Event
SendTask:
	push 	ZL
	push 	ZH
	push 	Counter
	push 	tmp2

	ldi 	ZL,low(TaskQueue)		;Загружаем очредеь событий
	ldi 	ZH,high(TaskQueue)

	ldi 	Counter, TaskQueueSize ;Длину этой очереди

	SEQL01: 
		ld 		tmp2, Z+	;Переходим к событию
		cpi 	tmp2, $FF	;Если оно FF - то тут уже пустота, сюда и сохраним
		BRNE /*!=*/ SEQL02
			st 		-Z, MacroR	; Положить в очередь
			RJMP SEQL03
		SEQL02:
		breq 	SEQL02

		dec 	Counter		; А если нет, то ещё один шажок
	BRNE /*!=*/	SEQL01		; Если очередь переполнена, то выходим

	SEQL03:					; EXIT
	pop 	tmp2
	pop 	Counter
	pop 	ZH
	pop 	ZL
RET

;------------------------------------------------------------------------	
; OSRG - Timer Event
; X - Counter
SetTimer:
	push 	ZL
	push 	ZH
	push 	tmp2
	push 	Counter

	ldi 	ZL,low(TimersPool)
	ldi 	ZH,high(TimersPool)

	ldi 	Counter, TimersPoolSize
	
	STL01: 	
		ld 		tmp2, Z		; Value / Counter
		cp 		tmp2, MacroR		; Поиск в очереди
		BRNE /*!=*/ STL02
			//Такая задача уже есть, обновляем её и выходим
			std 	Z+1, XL		; Critical Section
			std 	Z+2, XH		; Update Counter
			rjmp	STL06		; Exit
		STL02:
	
		subi 	ZL, Low(-3)	; Skip Counter
		sbci 	ZH, High(-3)	; Z+=3

		dec 	Counter		;
	BRNE /*!=*/ STL01 //Всё ищем и ищем
	//Не нашли, а жаль. Значит добавляем в очередь
	ldi 	ZL, low(TimersPool)
	ldi 	ZH, high(TimersPool)

	ldi 	Counter, TimersPoolSize
	
	STL04:	
		ld 		tmp2, Z		; Value / Counter
		cpi 	tmp2, $FF		; Search for Empty Timer
		BRNE /*!=*/ STL05
			st 		Z, MacroR		; Set Event 
			std 	Z+1, XL
			std 	Z+2, XH
			RJMP STL06
		STL05:	
		subi 	ZL, Low(-3)	; Skip Counter
		sbci 	ZH, High(-3)	; Z+=3
		dec 	Counter		;
	BRNE /*!=*/ STL04

	STL06:
	pop 	Counter
	pop 	tmp2
	pop 	ZH
	pop 	ZL
RET	
;------------------------------------------------------------------------	
; OSRG - Timer Event
; X - Counter
SetSlowTimer:
		push 	ZL
		push 	ZH
		push 	Tmp2
		push 	Counter

		ldi 	ZL,low(SlowTimersPool)
		ldi 	ZH,high(SlowTimersPool)

		ldi 	Counter, SlowTimersPoolSize
	
SSTL01: ld 		Tmp2, Z		; Value / Counter
		cp 		Tmp2, OSRG		; Search for Event
		breq 	SSTL02
	
		subi 	ZL, Low(-2)	; Skip Counter
		sbci 	ZH, High(-2); Z+=2

		dec 	Counter		;
		breq 	SSTL03		; Loop
		rjmp 	SSTL01
	
SSTL02:	;cli
		std 	Z+1, XL		; Critical Section
		;sei				; leave Critical Section
		rjmp	SSTL06		; Exit
SSTL03:

		ldi 	ZL, low(SlowTimersPool)
		ldi 	ZH, high(SlowTimersPool)

		ldi 	Counter, SlowTimersPoolSize
	
SSTL04:	ld 		Tmp2, Z		; Value / Counter
		cpi 	Tmp2, $FF		; Search for Empty Timer
		breq 	SSTL05
	
		subi 	ZL, Low(-2)	; Skip Counter
		sbci 	ZH, High(-2)	; Z+=2

		dec 	Counter		;
		breq 	SSTL06		; No Empty Timer
		rjmp 	SSTL04
	
SSTL05:	cli
		st 		Z, OSRG		; Set Event 
		std	 	Z+1, XL
		sei

SSTL06:
		pop 	Counter
		pop 	Tmp2
		pop 	ZH
		pop 	ZL
RET	

ClearTaskQueue:
		push 	ZL
		push 	ZH

		ldi 	ZL, low(TaskQueue)
		ldi 	ZH, high(TaskQueue)

		ldi 	OSRG, $FF		
		ldi 	Counter, TaskQueueSize

CEQL01: st 		Z+, OSRG		;
		dec 	Counter		;
		brne 	CEQL01		; Loop

		pop 	ZH
		pop 	ZL
		ret
	
ClearTimers:
		push 	ZL
		push 	ZH

		ldi 	ZL, low(TimersPool)
		ldi 	ZH, high(TimersPool)

		ldi 	Counter, TimersPoolSize
		ldi 	OSRG, $FF		; Empty 
		ldi 	Tmp2, $00

CTL01:	st 		Z+, OSRG		; Event
		st 		Z+, Tmp2		; Counter Lo
		st 		Z+, Tmp2		; Counter Hi

		dec 	Counter		;
		brne 	CTL01		; Loop
	
		pop 	ZH
		pop 	ZL
		ret	

ClearSlowTimers:
		push 	ZL
		push 	ZH

		ldi 	ZL, low(SlowTimersPool)
		ldi 	ZH, high(SlowTimersPool)

		ldi 	Counter, SlowTimersPoolSize
		ldi 	OSRG, $FF		; Empty 
		ldi 	Tmp2, $00

CSTL01:	st 		Z+, OSRG		; Event
		st 		Z+, Tmp2		; Counter Lo

		dec 	Counter		;
		brne 	CSTL01		; Loop
	
		pop 	ZH
		pop 	ZL
		ret	


SlowTimerService:
			PUSH 	OSRG
			in 		OSRG,SREG			; Save Sreg
			PUSH 	OSRG				; Сохранение регистра OSRG и регистра состояния SREG

			PUSH 	ZL	
			PUSH 	ZH					; сохранение Регистра Z
			PUSH 	Counter				; сохранение Регистра Counter
	
			LDI 	ZL,low(SlowTimersPool)	; Загрузка с регистр Z адреса RAM, 
			LDI 	ZH,high(SlowTimersPool); по которому находится информация о таймерах

			LDI 	Counter,SlowTimersPoolSize ; максимальное количество таймеров
	
Comp1SL01:	ld 		OSRG,Z				; OSRG = [Z] ; Получить номер события
			cpi 	OSRG,$FF			; Проверить на "NOP"
			breq 	Comp1SL03			; Если NOP то переход к следующей позиции

			clt							; Флаг T используется для сохранения информации об окончании счёта
			ldd 	OSRG,Z+1			; 
			subi 	OSRG,Low(1) 		; Уменьшение части счётчика на 1
			std 	Z+1,OSRG	
			breq 	Comp1SL02			; Если 0 то флаг T не устанавливаем
			set							; 

Comp1SL02:								; 

			brne 	Comp1SL03			; Счёт не окончен
			brts 	Comp1SL03			; Счёт не окончен (по T)	
	
			ld 		OSRG,Z				; Получить номер события
			CALL 	SendTask			; послать в системную очередь задач
	
			LDI 	OSRG,$FF			; = NOP (задача выполнена, таймер самоудаляется)
			st 		Z, OSRG				; Clear Event

Comp1SL03:	subi 	ZL,Low(-2)			; Skip Counter
			sbci 	ZH,High(-2)			; Z+=3 - переход к следующему таймеру
			dec 	Counter				; счетчик таймеров
			brne 	Comp1SL01			; Loop	

			pop 	Counter				; восстанавливаем переменные
			pop 	ZH
			pop 	ZL

			pop 	OSRG				; Восстанавливаем регистры
			out 	SREG,OSRG			; 
			pop 	OSRG
			SetTimerTask TS_SlowTimerService,60000 //Каждые 60 секунд, или 1 минуту один тик
RET