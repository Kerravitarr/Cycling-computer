/*
 * EEROM.inc
 *
 *  Created: 23.04.2019 18:44:48
 *   Author: Terran
 */ 
 /*
 Организация EEPROM
 Страница						| Назначение
	[0:EEPROM_SIZE_PAGE-1]		| Заголовок.
								| В нём будет EEPROM_SIZE_PAGE байт подряд
								| Например [0,1,...,N] означает, что мы последней записывали страницу с индексом EEPROM_SIZE_PAGE+1
								| А вот если будет [10,1,2,...], где 10 - 1 == EEPROM_SIZE_PAGE, то последней мы записывали страницу с индексом 1
								| Нарушение этой логики - каждое следующее число больше предыдущего на 1 и не больше одного на EEPROM_SIZE_PAGE 
								| позволяет проверить, что данные в EEPROM верные
	-							| Первый блок данных [EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA * 1: EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA * 1 + EEPROM_SIZE_PAGE)
	-							| Второй блок данных [EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA * 2: EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA * 2 + EEPROM_SIZE_PAGE)
	-							| Третий блок данных [EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA * 3: EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA * 3 + EEPROM_SIZE_PAGE)
			......
	[EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA * EEPROM_SIZE_PAGE: EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA * EEPROM_SIZE_PAGE + EEPROM_SIZE_PAGE) | Последний блок данных

 */


 //Сколько максимум размер страницы у микрухи
 .equ EEPROM_MAX_PAGE_SIZE = 16
 //Размер памяти в байтах
 .equ EEPROM_SIZE = 4096
 //Сколько страниц всего
 .equ EEPROM_COUNT_PAGE = EEPROM_SIZE / EEPROM_MAX_PAGE_SIZE
 //Сколько страниц на данные используется
 .equ EEPROM_PAGE_PER_DATA = 1;

 //У микрухи 32 байта на страницу. Но не всегда мы можем передать столько!
 //Поэтому EEPROM_SIZE_PAGE определяет сколько байт из страницы мы можем использовать
.IF TWISize - 1 - 2 < EEPROM_MAX_PAGE_SIZE
	.equ EEPROM_SIZE_PAGE = TWISize - 1 - 2
.ELSE
	.equ EEPROM_SIZE_PAGE = EEPROM_MAX_PAGE_SIZE
.ENDIF
//Проверяем, чтобы влезло!
 .IF (EEPROM_MAX_PAGE_SIZE * EEPROM_PAGE_PER_DATA) > EEPROM_COUNT_PAGE
	.error "EEPROM IS SMALLER THEN NEED"
.ENDIF

//Закончили запись данных памяти. Надо поглядеть что мы там записали и что будем дальше делать
TWI_IO_End_EEPROM_Write:
	LDS tmp2, EEPROM_mode
	CPI tmp2, EEM_CLRH
	BRNE /*!=*/ TIE_BME250w_notCLRH
		//Очистили заголовок. Теперь очищаем тело через 10 мс, дабы память успела среагировать!
		STI EEPROM_mode, EEM_CLRB1
		SetTimerTask TS_EEPROM_CLR,10
		RET
	TIE_BME250w_notCLRH:
	CPI tmp2, EEM_CLRB1
	BREQ /*=*/ TIE_BME250w_RB1
	CPI tmp2, EEM_UPDATEH
	BREQ /*=*/ TIE_BME250w_RB1
		RJMP TIE_BME250w_notRB1
	TIE_BME250w_RB1:
		//Очистили данные, мы завершили работу
		//Или обновили заголовок, мы завершили работу
		STI EEPROM_mode, EEM_READY
		//И перечитываем данные, как только микруха додумается, что всё с ней в порядке
		SetTimerTask TS_EEPROM_Read,10
		RET
	TIE_BME250w_notRB1:
	CPI tmp2, EEM_READH
	BREQ /*=*/ TIE_BME250w_RD //Записали адрес заголовка, теперь прочтём сам заголовок
	CPI tmp2, EEM_READP1
	BREQ /*=*/ TIE_BME250w_RD //Записали адрес данных, теперь прочтём сами данные
	CPI tmp2, EEM_SREADH
	BREQ /*=*/ TIE_BME250w_RD //Записали адрес заголовка, теперь прочтём его, чтобы узнать куда там писать данные
	CPI tmp2, EEM_RSETH
	BREQ /*=*/ TIE_BME250w_RD //Записали адрес заголовка, чтобы обновить его данные
		RJMP TIE_BME250w_notRD
	TIE_BME250w_RD:
		STI TWI_IO+0, EEPROMReadAdr
		STI TWI_IOl, EEPROM_SIZE_PAGE
		CALL StartTWI
		RET
	TIE_BME250w_notRD:
	CPI tmp2, EEM_SETP1
	BRNE /*!=*/ TIE_BME250w_notSP1
		//Мы сохранили данные в P1, во всяком случае надеемся на это. Теперь нужно будет обновить номер страницы. В будущем
		STI EEPROM_mode, EEM_SETH
		//И перечитываем данные, как только микруха додумается, что всё с ней в порядке
		SetTimerTask TS_EEPROM_STEP,10
		RET
	TIE_BME250w_notSP1:
RET

//Прочитали страницу из памяти EEPROM
TWI_IO_End_EEPROM_Read:
	LDS tmp2, EEPROM_mode
	CPI tmp2, EEM_READH
	BREQ /*=*/ TWI_IO_End_ER_readh
	CPI tmp2, EEM_SREADH
	BREQ /*=*/ TWI_IO_End_ER_readh
	CPI tmp2, EEM_RSETH
	BREQ /*=*/ TWI_IO_End_ER_readh
		RJMP TWI_IO_End_ER_nreadh
	TWI_IO_End_ER_readh:
		//Читаем заголовок
		LDIW Z, TWI_IO+1
		LD tmp2, Z+ //Стартовый адрес
		INC tmp2 //Следующий адрес должен быть на 1 больше
		LDI Counter, EEPROM_SIZE_PAGE - 1 //Сколько у нас всего ячеек. -1, так как первую уже прошли
		CLR Counter2 //А это счётчик ячейки, в которой переход
		TWI_IO_End_ER_loop:
			LD tmp3, Z+ //Стартовый адрес
			CP tmp2, tmp3
			BREQ /*=*/ TWI_IO_End_ER_next
				//Мы нашли конец предыдущего блока.
				//Нужно проверить, что разрыв ровно в EEPROM_SIZE_PAGE!
				SUB tmp2, tmp3
				CPI tmp2, EEPROM_SIZE_PAGE
				BREQ /*=*/ TWI_IO_End_ER_next2 //Прыгаем в конец текущего цикла. Нужно дотестировать память до конца, дабы убедиться, что всё нормально!
					//Плохо, разница не та, что ожидали. Это конец
					RCALL EEPROM_BREAK_MEM
					RET
				TWI_IO_End_ER_loop2:
					LD tmp3, Z+ //Стартовый адрес
					CP tmp2, tmp3
					BREQ /*=*/ TWI_IO_End_ER_next2
						//Ну вот и всё. Два расхождения. Память битая. Ошибка.
						RCALL EEPROM_BREAK_MEM
						RET
					TWI_IO_End_ER_next2:
					MOV tmp2, tmp3
					INC tmp2 //Следующий адрес должен быть на 1 больше
					DEC Counter
				BRNE /*!=*/ TWI_IO_End_ER_loop2
				//Шик. Внутренний цикл не нашёл ошибок. Идём дальше!
				RJMP TWI_IO_End_ER_readNext		
			TWI_IO_End_ER_next:
			MOV tmp2, tmp3
			INC tmp2 //Следующий адрес должен быть на 1 больше
			INC Counter2 //Переход не в этой ячейке
			DEC Counter
		BRNE /*!=*/ TWI_IO_End_ER_loop
		//Все числа идут один за одним... Шикарно.
		TWI_IO_End_ER_readNext: 
		//В Counter2 номер строки, которую читаем. Или на 1 меньше, чем строка, в которую пишем
		LDS tmp2, EEPROM_mode
		CPI tmp2, EEM_READH
		BRNE /*!=*/ TWI_IO_End_ER_readhNR
			STI EEPROM_mode, EEM_READP1
			//Мы просто читаем данные, нужно прочитать по адресу из Counter2
			INC Counter2 //У нас в Counter2 номер страницы... с 1 по EEPROM_SIZE_PAGE+1
			//Теперь нужно номер страницы умножить на EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA
			LDI tmp2, EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA
			MUL Counter2,tmp2

			STI TWI_IO+0, EEPROMWriteAdr
			STS TWI_IO+1, Mh	//Адрес чтения
			STS TWI_IO+2, Ml
			STI TWI_IOl, 3 ;Устанавливаем чтение с нулевого адреса
			CALL StartTWI
			RET
		TWI_IO_End_ER_readhNR:
		CPI tmp2, EEM_SREADH
		BRNE /*!=*/ TWI_IO_End_ER_readhNSR
			STI EEPROM_mode, EEM_SETP1
			//Мы прочитали заголовк для того, чтобы записать данные в следующую ячейку!
			MOV Counter, Counter2
			INC Counter
			CPI Counter, EEPROM_SIZE_PAGE
			BRLO /*<*/ TWI_IO_End_ER_readhNRNotLoop
				//У нас Counter стал больше EEPROM_SIZE_PAGE. Надо его обнулить
				CLR Counter
			TWI_IO_End_ER_readhNRNotLoop:
			INC Counter //У нас в Counter номер страницы... с 1 по EEPROM_SIZE_PAGE+1
			//Теперь нужно номер страницы умножить на EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA
			LDI tmp2, EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA
			MUL Counter,tmp2
			STI TWI_IO+0, EEPROMWriteAdr
			STS TWI_IO+1, Mh	//Адрес чтения
			STS TWI_IO+2, Ml
			//Первая страница:
				//2б - WheelLength
				//2б - MaxSpeedOdo
				//4б - TimeOdo
				//4б - Odometr
			LDS tmp3, WheelLength + 2
			STS TWI_IO + 3, tmp3
			LDS tmp3, WheelLength + 3
			STS TWI_IO + 4, tmp3
			LDS tmp3, MaxSpeedOdo + 2
			STS TWI_IO + 5, tmp3
			LDS tmp3, MaxSpeedOdo + 3
			STS TWI_IO + 6, tmp3
			_MOV_32 TWI_IO+7, TimeOdo
			_MOV_32 TWI_IO+7+4, Odometr
			//Вычисляем CRC этих данных
			_CRC8 TWI_IO+3, EEPROM_SIZE_PAGE-1
			STS TWI_IO+2 + EEPROM_SIZE_PAGE, MacroR
			STI TWI_IOl, EEPROM_SIZE_PAGE + 1 + 2 ;Записали всю страницу да ещё адрес микрухи да ещё два байта на адрес памяти
			CALL StartTWI
			RET
		TWI_IO_End_ER_readhNSR:
			STI EEPROM_mode, EEM_UPDATEH
			//Мы данные обновили, теперь нам нужно обновить заголовок
			//Текущий адреас ячейки в Counter2
			LDIW Z, TWI_IO+1
			ADD Zl,Counter2
			ADC Zh, Const0 //Z+=Counter2
			LD tmp3,Z
			INC tmp3; //У нас следующее число
			MOV Counter, Counter2
			INC Counter
			CPI Counter, EEPROM_SIZE_PAGE
			BRLO /*<*/ TWI_IO_End_ER_readhNSRNotLoop
				//У нас Counter стал больше EEPROM_SIZE_PAGE. Надо его обнулить
				CLR Counter
			TWI_IO_End_ER_readhNSRNotLoop:
			STI TWI_IO+0, EEPROMWriteAdr
			STI TWI_IO+1, 0
			STS TWI_IO+2, Counter //Адрес
			STS TWI_IO+3, tmp3 //Новое число
			STI TWI_IOl, 4 ;Записать всего 4 байта
			CALL StartTWI
			RET
	TWI_IO_End_ER_nreadh:
	CPI tmp2, EEM_READP1
	BRNE /*!=*/ TWI_IO_End_ER_nreadp1
		//Читаем первую страницу данных
		//Вычисляем CRC по этим данным
		_CRC8 TWI_IO+1, EEPROM_SIZE_PAGE
		TST MacroR
		BREQ /*=*/ TWI_IO_End_ER_RP1_OK
			//СRC не сошёлся... Приехали...
			RCALL EEPROM_BREAK_MEM
			RET
		TWI_IO_End_ER_RP1_OK:
		//Прочитали данные и они даже верные!
		//Первая страница:
			//2б - WheelLength
			//2б - MaxSpeedOdo
			//4б - TimeOdo
			//4б - Odometr
			_CLR_32 WheelLength
			LDS tmp3, TWI_IO+1
			STS WheelLength + 2, tmp3
			LDS tmp3, TWI_IO+2
			STS WheelLength + 3, tmp3
			_CLR_32 MaxSpeedOdo
			LDS tmp3, TWI_IO+3
			STS MaxSpeedOdo + 2, tmp3
			LDS tmp3, TWI_IO+4
			STS MaxSpeedOdo + 3, tmp3
			_MOV_32 TimeOdo, TWI_IO+5
			_MOV_32 Odometr, TWI_IO+5+4
		STI EEPROM_mode, EEM_READY
	TWI_IO_End_ER_nreadp1:
RET

//Произошла ошибка при работе с EEPROM
TWI_IO_ERR_EEPROM:
	RCALL EEPROM_BREAK_MEM
RET

//Функцию нужно вызвать, если память повреждена. Тут может потом что ни будь ещё добавится
EEPROM_BREAK_MEM:
	STI EEPROM_mode, EEM_READY
	
	_CLR_32 WheelLength
	_CLR_32 MaxSpeedOdo
	_CLR_32 TimeOdo
	_CLR_32 Odometr
RET

//Читает данные из EEPROM. Точнее, эта часть только активирует чтение шапочки
EEPROM_Read:
	TSTFlag TWI_Busy
	BRNE /*!=*/ EEPROM_Read_retry
		LDS tmp2, EEPROM_mode
		CPI tmp2, EEM_READY
		BRNE /*!=*/ EEPROM_ReadNready
			//Память готова к чтению!
			STI EEPROM_mode, EEM_READH
			STI TWI_IO+0, EEPROMWriteAdr
			STI TWI_IO+1, 0 //Адрес записи с 0 начинаем
			STI TWI_IO+2, 0
			STI TWI_IOl, 3 ;Устанавливаем чтение с нулевого адреса
			CALL StartTWI
			RET
		EEPROM_ReadNready:
	EEPROM_Read_retry:
	//Линия занята, обратитесь позже*
	SetTimerTask TS_EEPROM_Read,1
RET
//Сохраняет данные в память
EEPROM_SET:
	TSTFlag TWI_Busy
	BRNE /*!=*/ EEPROM_set_retry
		LDS tmp2, EEPROM_mode
		CPI tmp2, EEM_READY
		BRNE /*!=*/ EEPROM_setNready
			//Память готова к записи!
			//Однако сначала надо узнать куда именно мы пишем
			STI EEPROM_mode, EEM_SREADH
			STI TWI_IO+0, EEPROMWriteAdr
			STI TWI_IO+1, 0 //Адрес записи с 0 начинаем
			STI TWI_IO+2, 0
			STI TWI_IOl, 3 ;Устанавливаем чтение с нулевого адреса
			CALL StartTWI
			RET
		EEPROM_setNready:
	EEPROM_set_retry:
	//Линия занята, обратитесь позже*
	SetTimerTask TS_EEPROM_SET,1
RET

//Шаг работы EEPROM, особая функция, которую нельзя просто так вызвать извне.
//Служит для работы внутреннего механизма
EEPROM_STEP:
	TSTFlag TWI_Busy
	BRNE /*!=*/ EEPROM_step_retry
		LDS tmp2, EEPROM_mode
		CPI tmp2, EEM_SETH
		BRNE /*!=*/ EEPROM_setNSH
			//Нужно сохранить заголовок
			//А для этого его нужно... Правильно - прочитать.. Опять...
			STI EEPROM_mode, EEM_RSETH
			STI TWI_IO+0, EEPROMWriteAdr
			STI TWI_IO+1, 0 //Адрес записи с 0 начинаем
			STI TWI_IO+2, 0
			STI TWI_IOl, 3 ;Устанавливаем чтение с нулевого адреса
			CALL StartTWI
			RET
		EEPROM_setNSH:
	EEPROM_step_retry:
	//Линия занята, обратитесь позже*
	SetTimerTask TS_EEPROM_STEP,1
RET

//Вспомогательная функция. Очищает память EEPROM, чтобы в неё можно было вновь записать данные
//Не всю память очищает, конечно. Только шапочку, которая как раз является ссылкой на данные
EEPROM_CLR:
	TSTFlag TWI_Busy
	BRNE /*!=*/EEPROM_CLR_retry
		LDS tmp2, EEPROM_mode
		CPI tmp2, EEM_READY
		BRNE /*!=*/ EEPROM_CLR_notReady //Пока занята EEPROM - ждём
			//Первым делом заполним заголовок числами от 0 до скольки получится
			STI EEPROM_mode, EEM_CLRH
			STI TWI_IO+0, EEPROMWriteAdr
			STI TWI_IO+1, 0 //Адрес записи с 0 начинаем
			STI TWI_IO+2, 0 
			LDIW Z, TWI_IO + 3
			CLR Counter
			EEPROM_CLR_loop:
				ST Z+, Counter	//Все данные по возрастающей
				INC Counter
				CPI Counter, EEPROM_SIZE_PAGE //Считаем, что у нас всегда EEPROM_SIZE_PAGE > 0!!!!!
			BRNE /*!=*/ EEPROM_CLR_loop
			STI TWI_IOl, EEPROM_SIZE_PAGE + 1 + 2 ;Записали всю страницу да ещё адрес микрухи да ещё два байта на адрес памяти
			CALL StartTWI
			RET
		EEPROM_CLR_notReady:
		CPI tmp2, EEM_CLRB1
		BRNE /*!=*/ EEPROM_CLR_notB //Хотя если занята она нами - чего ждать?! Действовать надо!
			STI EEPROM_mode, EEM_CLRB1 //Тут можно указать, что мы теперь будем следующую старницу очищать... Или не будем
			STI TWI_IO+0, EEPROMWriteAdr
			 //Теперь запишем пустые данные в самую последнюю ячейку памяти
			STI TWI_IO+1, high(EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA * EEPROM_SIZE_PAGE)
			STI TWI_IO+2, low(EEPROM_MAX_PAGE_SIZE*EEPROM_PAGE_PER_DATA * EEPROM_SIZE_PAGE)
			LDIW Z, TWI_IO + 3
			CLR Counter
			EEPROM_CLR_RB1loop:
				ST Z+, Const0	//Все данные в нули
				INC Counter
				CPI Counter, EEPROM_SIZE_PAGE-1 //Считаем, что у нас всегда EEPROM_SIZE_PAGE > 1!!!!!
			BRNE /*!=*/ EEPROM_CLR_RB1loop
			//Вычисляем CRC этого результата
			_CRC8 TWI_IO+3, EEPROM_SIZE_PAGE-1
			ST Z+, MacroR
			STI TWI_IOl, EEPROM_SIZE_PAGE + 1 + 2 ;Записали всю страницу да ещё адрес микрухи да ещё два байта на адрес памяти
			CALL StartTWI
			RET
		EEPROM_CLR_notB:

	EEPROM_CLR_retry:
	//Линия занята, обратитесь позже*
	SetTimerTask TS_EEPROM_CLR,1
RET