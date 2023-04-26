/*
 * TimeRepeat.inc
 *
 *  Created: 25.11.2018 11:43:22
 *   Author: Terran
 */ 
 
 //Таймер на 1 мс - отрабатывает каждую милисекунду
 Evry_1_ms_Task:
	//Проверяем - мы двигаемся?
	TSTFlag isMove
	BRNE /*!=*/ Evry_1_ms_Task_not_move
		_INC_32 TimeOdo
		_INC_32 TimeTrip
	Evry_1_ms_Task_not_move:

	SetTimerTask TS_Evry_1_ms_Task,1
 RET

 //Таймер на 40 мс - 25 раз в секунду срабатывает
 Evry_40_ms_Task:
	//Проверяем - мы двигаемся?
	TSTFlag isMove
	BRNE /*!=*/ Evry_40_ms_Task_not_move
		//Мы едем. Значит пересчитываем все значения
		//Средняя скорость = DIST * 60000 / TIME (V = S/T)
		_LDI_A 60000
		_MOV_32 B_32, Dist
		_MUL64_A_R32 B_32
		_MOV_32 B_32, TimeTrip
		_DIV_64_A_DIV_R32 B_32
		_MOV_32 AverageSpeed, ACC_1

		//Максимальная текущая скорость
		_MOV_32 B_32, TimeTrip
		_LDI_A 60000
		_DIV_64_A_DIV_R32 B_32

	Evry_40_ms_Task_not_move:

	SetTimerTask TS_Evry_40_ms_Task,40
 RET




//Flag_1 = 0b Key1 Key2 MENU UART TWI Lamp U Temp
//Flag_2 = 0b ~ ~ ~ ~ ~ ~ Scr Sleep
//Устаревшие кусочки
/*
 ScrinUpdate:		
		LDI 	ZL,low(WordOut+1)
		LDI 	ZH,high(WordOut+1)
		LD 		word,Z
		ANDI	word,0b00000001				;Узнаём состояние стробирующего бита и если он 1 - просто опускаем его в 0
		BREQ  SU01
			CLI
				LD 		word, Z
				EORI	word,0b00000001
				ST      Z,word
			SEI
			rjmp SU02
		SU01:

		LD 		word, Z
		ANDI		word,0b0000010				;Узнаём состояние командного бита. 0 - команда, 1 - слово
		BRNE  SU03
				LD 		word, -Z			;Если у нас передавалась команда
				CPI		word, 0b00000001	;Кодовое слово 3 / очистить и что то ещё с экраном
				BRNE SU03
					SetScrinKeyWord 0b00000110
					rjmp SU02
SU03:								
				//Flag_2 = 0b ~ ~ ~ ~ Scr left(T=1) Scr Sleep
				LDS	tmp3, Flag_2
				ANDI tmp3,0x0A
				BRNE /*!=*//* SU060
					//Первый мод
					LDI 	YL,low(StringOutOne)
					LDI 	YH,high(StringOutOne)
					rjmp	SU07
				SU060:
				CPI tmp3,0x02
				BRNE /*!=*//* SU061
					//Второй мод
					LDI 	YL,low(StringOutTwo)
					LDI 	YH,high(StringOutTwo)
					rjmp	SU07
				SU061:
				CPI tmp3,0x08
				BRNE /*!=*/ /*SU062
					//Третий мод
					LDI 	YL,low(StringOutThree)
					LDI 	YH,high(StringOutThree)
					rjmp	SU07
				SU062:
					LDI 	YL,low(StringOutFour)
					LDI 	YH,high(StringOutFour)
				SU07:
							
				MOV 	ZL,Yl
				MOV 	ZH,Yh
				LD 		Counter,Z+											;Взяли длину строки
				LD 		tmp2,Z+												;Взяли число записаннных символов на экран
				LD 		tmp3,Z												;Взяли переменную, следяющую за местом начала "окна"

				CPI		tmp2,20
				BREQ	SU08
						MOV		tmp4,tmp2
						ADD		tmp4,tmp3
						CP		tmp4,Counter
						BRGE	SU04
							ADIW	Zl,1
							ADD		Zl,tmp4
							LD 		word,Z
							rjmp	SU05
						SU04:				//если больше или равно
							CPI		Counter,21		//А если у нас строка короткая, то мы закидываем дальше только пробелы
							BRLO	SU041
								ADIW	Zl,1
								SUB		tmp4,Counter
								ADD		Zl,tmp4
								LD 		word,Z
								rjmp SU042
							SU041:
								LDI		word,' '
							SU042:
						SU05:

						INC		tmp2
						MOV 	ZL,Yl
						MOV 	ZH,Yh
						ADIW	Zl,1
						ST		Z,tmp2

						SetScrinWord word
						rjmp SU02
				SU08:
				MOV 	ZL,Yl
				MOV 	ZH,Yh
				ADIW	Zl,1

				MOV		tmp4,Counter
				SUBI	tmp4,-20
				CP		tmp3,tmp4
				BREQ	SU09				//если равно
				BRLT	SU09				//и если меньше
					SUB	tmp3,Counter		//Тогда надо сбросить строку в 0
				SU09:

				CPI		Counter,21
				BRLO	SU10
					INC`	tmp3
				SU10:
				ADIW	Zl,1
				ST		Z,tmp3

				
				//Flag_2 = 0b ~ ~ ~ ~ Scr left(T=1) Scr Sleep
				LDS	tmp3, Flag_2
				MOV tmp2,tmp3
				ANDI tmp3,0x0A
				CPI tmp3,0x0A
				BRNE /*!=*/ /*SU120
					//Первый мод
					ANDI	tmp2,0xF5
					SetScrinKeyWord 0x80
					rjmp	SU13
				SU120:
				CPI tmp3,0x00
				BRNE /*!=*//* SU121
					//Второй мод
					ORI	tmp2,0x02
					SetScrinKeyWord 0xC0
					rjmp	SU13
				SU121:
				CPI tmp3,0x02
				BRNE /*!=*//* SU122
					//Третий мод
					ORI	tmp2,0x08
					ANDI tmp2,0xFD
					SetScrinKeyWord 0x94 //За третью строчку
					rjmp	SU13
				SU122:
					ORI tmp2,0x0A
					SetScrinKeyWord 0xD4 //За четвёртую строку
				SU13:
				STS Flag_2,tmp2
				rjmp	SU02
				//Flag_2 = 0b ~ ~ ~ ~ Scr left(T=1) Scr Sleep
				/*LDS	tmp3, Flag_2
				ANDI tmp3,0x02
				BREQ SU13_0
					RJMP SU13
				SU13_0:
					//Была первая строка, надо проверить - нужна ли вторая
					LDS		Counter,StringOutTwo+1
					//Если Counter = 0, то надо загрузить вторую строчку - хочешь не хочешь.
					TST	Counter
					BRNE SU15
							LDS OSRG,Flag_2
						ORI OSRG,0x02
							STS Flag_2,OSRG
						SetScrinKeyWord 0b11000000
						rjmp	SU02
					SU15:
					//Мы вторую строчку уже загружали. А надо ли её по новой загружать?
					LDS		Counter,StringOutTwo
					CPI		Counter,17
					BRLO	SU16
						//Если больше, то надо грузить вторую строчку
							LDS OSRG,Flag_2
						ORI OSRG,0x02
							STS Flag_2,OSRG
						SetScrinKeyWord 0b11000000
						rjmp	SU02
					SU16:
						//Если меньше, то вторую строчку грузить не надо
						LDS		Counter,StringOutOne
						CPI		Counter,17
						BRSH SU18_0
							RJMP SU18
						SU18_0:
							//Если больше, то надо грузить первую строчку ещё раз
							SetScrinKeyWord ScrinKeyClear
							SetTimerTask TS_ScrinUpdate,500
							LDI		tmp4,0
							STS		StringOutOne+1,tmp4
							rjmp	SU14
				SU13:
					//Была вторая строка, надо проверить - нужна ли первая
					LDS		Counter,StringOutOne
					CPI		Counter,17
					BRLO	SU17
						//Если больше, то надо грузить первую строчку
							LDS OSRG,Flag_2
						ANDI OSRG,0b11111101
							STS Flag_2,OSRG
						SetScrinKeyWord ScrinKeyClear
						SetTimerTask TS_ScrinUpdate,500
						LDI		tmp4,0
						STS		StringOutOne+1,tmp4
						rjmp	SU14
					SU17:
					LDS		Counter,StringOutTwo
					CPI		Counter,17
					BRLO	SU18
						//Если больше, то надо грузить вторую строчку ещё раз
							LDS OSRG,Flag_2
						ORI OSRG,0x20
							STS Flag_2,OSRG
						SetScrinKeyWord 0b11000000
						LDI		tmp4,0
						STS		StringOutTwo+1,tmp4
						rjmp	SU02
				
				SU18:
				LDI		tmp4,0
				STS		StringOutTwo+1,tmp4 //Всё, больше грузить не надо, все флаги на ноль, курсор экрана в 0 позицию
				STS		StringOutOne+1,tmp4
					LDS OSRG,Flag_2
				ANDI OSRG,0b11111101
					STS Flag_2,OSRG
				SetScrinKeyWord ScrinKeyClear
				
				SU14:
				SetTask TS_O
				rjmp SU12*/
				/*
SU02:			;Выход
				SetTask TS_O														
				SetTimerTask TS_ScrinUpdate,1
SU12:
RET
;------------------------------------------------------------------------------
;------------------------------------------------------------------------------
;------------------------------------------------------------------------------
LedPower:	
		LDS 		word,WordOut+1

		CLI
			LDS 		word, WordOut+1
			LDS 		tmp2, WordOut+1
			ANDI	tmp2,0b00001100
			CPI		tmp2,0b00001100
			BRNE /*!=*/ /*LP_00
				ANDI	word,0b11110011
			LP_00:
			ADI		word,0b00000100
			STS      WordOut+1,word
		SEI

		SetTask TS_O

		//SetTimerTask TS_LedPower,50
RET

;------------------------------------------------------------------------------
;------------------------------------------------------------------------------
;------------------------------------------------------------------------------
NewSec:		
				SetTimerTask TS_NewSec,1000
				LDI		tmp2,0xA0
				LDI		tmp3,0x06
				LDI 	ZL,low(Clock)
				LDI 	ZH,high(Clock)
				LDI		Counter,3
				
				LD	word,Z
				INC	word
				ST	Z,word
				NS_03:
					LD	word,Z
					ADD word,tmp3
					BRHC NS_01
						ST	Z,word
					NS_01:
					SUB word,tmp3
					ADD word,tmp2
					BRCC NS_02
						ST	Z,word
						LDD	word,Z+1
						INC	word
						STD	Z+1,word
					NS_02:
					ADIW Zl,1
					//Теперь разговор уже идёт о следующей цифре
					DEC Counter
				BRNE /*!=*//* NS_03
				//Надо проверить часы, они как раз в памяти

			NS00:

RET


SetScrinWord_M:
	CLI
		STS      WordOut,OSRG					;Записали слово в регистр 1

		LDS	 	OSRG, WordOut+1
		ANDI	OSRG,0b11111100
		ORI		OSRG,0b11
		STS      WordOut+1,OSRG					;Установили строб и показали что передаём команду
	SEI
RET
SetScrinKeyWord_M:
	CLI

		STS      WordOut,OSRG					;Записали кодовое слово в регистр 1

		LDS	 	OSRG, WordOut+1
		ANDI	OSRG,0b11111100
		ORI		OSRG,0b1
		STS      WordOut+1,OSRG					;Установили строб и показали что передаём команду

	SEI
RET
;------------------------------------------------------------------------------
;------------------------------------------------------------------------------
;------------------------------------------------------------------------------

StartTime:
	CLR_32 B_2
	CP_F B_2,Speed
	BREQ /*=*/ /*ST_00
		LDI_8 B_2,1
		ADD_F TimeV,B_2
		SetTimerTask TS_StartTime,1
	ST_00:
RET



SCANMode:
	NewScrinUpdate
	LDI word,20
	STS StringOutOne,word
	STS StringOutTwo,word
	STS StringOutThree,word
	STS StringOutFour,word
	CLR word
	STS StringOutOne+1,word
	STS StringOutOne+2,word
	STS StringOutTwo+1,word
	STS StringOutTwo+2,word
	STS StringOutThree+1,word
	STS StringOutThree+2,word
	STS StringOutFour+1,word
	STS StringOutFour+2,word
	RCALL SCANM_One
	SetTimerTask TS_SCANMode,250


	LDS	word,Mode
	CPI	word,Mode1
	BREQ /*=*//* SCANM_1
	CPI	word,Mode2
	BREQ /*=*//* SCANM_2
	CPI	word,Mode3
	BRNE /*!=*//* SCANM0_3
		JMP SCANM_3
	SCANM0_3:
	CPI	word,Mode4
	BRNE /*!=*//* SCANM0_4
		JMP SCANM_4
	SCANM0_4:
	CPI	word,Mode5
	BRNE /*!=*/ /*SCANM0_5
		JMP SCANM_5
	SCANM0_5:
	CPI	word,Mode6
	BRNE /*!=*//* SCANM0_6
		JMP SCANM_6
	SCANM0_6:
	CPI	word,Mode7
	BRNE /*!=*/ /*SCANM0_7
		JMP SCANM_7
	SCANM0_7:
	CPI	word,Mode8
	BRNE /*!=*//* SCANM0_8
		JMP SCANM_8
	SCANM0_8:

	
	CPI	word,Mode50
	BRNE /*!=*//* SCANM0_50
		JMP SCANM_50
	SCANM0_50:
	CPI	word,Mode51
	BRNE /*!=*//* SCANM0_51
		JMP SCANM_51
	SCANM0_51:

//RET
	SCANM_1:
		LPM_Z_X ScrinStrMode1,StringOutOne+3,20
		LPM_Z_X ScrinStrMode0,StringOutTwo,32
															RCALL SCANM_One
//RET
	SCANM_2:
		//Вторая строка
		LPM_Z_X ScrinStrMode2,StringOutTwo+3,20
		MOV_32 B_2,Dist
		LDI_16 C_2,1000
		DIV_F	B_2,C_2
		LDI_16 C_2,10000 //10км
		CP_F B_2, C_2
		BRSH /*>=*/ /*SCANM_2_04
			FloatToInt B_2
			IntToBCD B_2
			LDS word,B_2
			ADI word,0x30
			STS StringOutTwo+1+3,word
			LDS word,B_2+1
			ADI word,0x30
			STS StringOutTwo+1+3+1,word
			LDS word,B_2+2
			ADI word,0x30
			STS StringOutTwo+1+3+2,word
			LDS word,B_2+3
			ADI word,0x30
			STS StringOutTwo+1+3+3,word
		SCANM_2_04:

		MOV_32 B_2,Odometr
		LDI_16 C_2,1000
		DIV_F	B_2,C_2 //мм в м
		DIV_F	B_2,C_2 //м в км
		FloatToInt B_2
		IntToBCD B_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F B_2,C_2
		LDS word,B_2
		ADI word,0x30
		STS B_2,word
		MOV_32 StringOutTwo+10+3,B_2
//RET
		SCANM_3:
		LPM_Z_X ScrinStrMode3,StringOutThree+3,20
		MOV_32 B_2,AverageSpeed
		LPM_Z_X Const_3_6,C_2,4
		MUL_F B_2,C_2	//Скорость км/ч
		LDI_8 C_2,10
		MUL_F B_2,C_2
		FloatToInt B_2
		IntToBCD B_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F B_2,C_2
		LDS		word,B_2+1
		STS		StringOutThree+3+3,word
		LDS		word,B_2+2
		STS		StringOutThree+4+3,word
		LDS		word,B_2+3
		STS		StringOutThree+6+3,word
		
		MOV_32 B_2,MaxSpeed
		LPM_Z_X Const_3_6,C_2,4
		MUL_F B_2,C_2	//Скорость км/ч
		LDI_8 C_2,10
		MUL_F B_2,C_2
		FloatToInt B_2
		IntToBCD B_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F B_2,C_2
		LDS		word,B_2+1
		STS		StringOutThree+12+3,word
		LDS		word,B_2+2
		STS		StringOutThree+13+3,word
		LDS		word,B_2+3
		STS		StringOutThree+15+3,word
//RET
		SCANM_4:
		LPM_Z_X ScrinStrMode4,StringOutFour+3,20
		MOV_32 B_2,TimeTrip
		LDI_16 C_2,1000
		DIV_F	B_2,C_2 //время в секундах
		MOV_32 D_2,B_2
		LDI_16 C_2,3600 //секунд в часе
		DIV_F	B_2,C_2 //Количество часов
		FloatToInt B_2
		MOV_32 E_2,B_2
		IntToBCD B_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F B_2,C_2
		LDS		word,B_2+2
		STS		StringOutFour+1+3,word
		LDS		word,B_2+3
		STS		StringOutFour+2+3,word
		LDI_16 C_2,3600
		MUL_F E_2,C_2 //Колличество секунд которые забирают часы
		SUB_F D_2,E_2 //Сколько секунд приходится на минуты и секунды
		MOV_32 B_2,D_2
		LDI_8 C_2,60
		DIV_F	B_2,C_2 //Количество минут
		FloatToInt B_2
		MOV_32 E_2,B_2
		IntToBCD B_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F B_2,C_2
		LDS		word,B_2+2
		STS		StringOutFour+4+3,word
		LDS		word,B_2+3
		STS		StringOutFour+5+3,word
		LDI_8 C_2,60
		MUL_F E_2,C_2 //Колличество секунд которые забирают минуты
		SUB_F D_2,E_2
		FloatToInt D_2
		IntToBCD D_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F D_2,C_2
		LDS		word,D_2+2
		STS		StringOutFour+7+3,word
		LDS		word,D_2+3
		STS		StringOutFour+8+3,word

		MOV_32 B_2,TimeOdo
		RCALL SCANM_Two_Time
RET
		SCANM_5:
		LPM_Z_X ScrinStrMode5,StringOutTwo+3,20
		
		MOV_32 B_2,KadenAverage
		FloatToInt B_2
		IntToBCD B_2
		BTC_F B_2
		LDS		word,B_2+1
		STS		StringOutTwo+4+3,word
		LDS		word,B_2+2
		STS		StringOutTwo+5+3,word
		LDS		word,B_2+3
		STS		StringOutTwo+6+3,word
		
		MOV_32 B_2,KadenMax
		FloatToInt B_2
		IntToBCD B_2
		BTC_F B_2
		LDS		word,B_2+1
		STS		StringOutTwo+13+3,word
		LDS		word,B_2+2
		STS		StringOutTwo+14+3,word
		LDS		word,B_2+3
		STS		StringOutTwo+15+3,word
RET
		SCANM_6:
		LPM_Z_X ScrinStrMode6,StringOutTwo+3,20
		//Flag_2 = 0b ~ ~ ~ ~ ~ left(T=1) Scr Sleep
		LDS word,Flag_2
		SBRS word,2
			RJMP  SCANM_6_Dist
		//У нас записано оставшееся время, надо его записать и подписать оставшуюся дистанцию
		LDI word,'>'
		STS		StringOutTwo+8+3,word

		MOV_32 B_2,LeftTimeDist
		RCALL SCANM_Two_Time
		

		MOV_32 B_2,LeftTimeDist
		LDI_16 C_2,1000
		DIV_F	B_2,C_2 //время в секундах
		MOV_32  C_2,AverageSpeed //Среняя скорость м/с >> перемножаем и узнаём колличество метров
		MUL_F B_2,C_2 //Оставшееся расстояние, в метрах
		RCALL SCANM_Two_Dist
RET
		SCANM_6_Dist:
		//У нас записана оставшаяся дистанция ((а может уже ни чего нет)), значит надо или найти время или что то ещё
		CLR_32 B_2
		CP_F B_2,LeftTimeDist
		BREQ /*=*//* SCANM_6_00
		//Теперь то факт, что то да осталось
		LDI word,'<'
		STS		StringOutTwo+8+3,word

		MOV_32 B_2,LeftTimeDist
		LDI_16 C_2,1000
		DIV_F	B_2,C_2 //расстояние в метрах
		RCALL SCANM_Two_Dist

		MOV_32 B_2,LeftTimeDist//расстояние в мм
		MOV_32  C_2,AverageSpeed //Среняя скорость м/с
		DIV_F  B_2,C_2 //Время в мс
		RCALL SCANM_Two_Time

		SCANM_6_00:
RET
		SCANM_7:
		LPM_Z_X ScrinStrMode7,StringOutTwo+3,20
		SetTask TS_WetherUpdate
RET
		SCANM_8:
		LPM_Z_X ScrinStrMode8,StringOutTwo+3,20
		MOV_32 B_2,Degrees
		
RET
		SCANM_50:
		LDI word,0
		STS StringOutOne,word
		STS StringOutThree,word
		STS StringOutOne+1,word
		STS StringOutOne+2,word
		LPM_Z_X ScrinStrMode50,StringOutTwo+3,20
		LPM_Z_X ScrinStrMode51,StringOutFour+3,20
		MOV_32 B_2, Lenght
		FloatToInt B_2
		IntToBCD B_2
		BTC_F B_2
		MOV_32 StringOutTwo+3+8,B_2
RET
		SCANM_51:
		LDI word,0
		STS StringOutOne,word
		STS StringOutThree,word
		STS StringOutOne+1,word
		STS StringOutOne+2,word
		LPM_Z_X ScrinStrMode51,StringOutTwo+3,20
		LPM_Z_X ScrinStrMode50,StringOutFour+3,20
		MOV_32 B_2, Lenght
		FloatToInt B_2
		IntToBCD B_2
		BTC_F B_2
		MOV_32 StringOutFour+3+8,B_2

		LDS tmp2,Clock
		ANDI tmp2,0x0F
		STS B_2+3,tmp2
		LDS tmp2,Clock
		SWAP tmp2
		ANDI tmp2,0x0F
		STS B_2+2,tmp2
		
		LDS tmp2,Clock+1
		ANDI tmp2,0x0F
		STS B_2+1,tmp2
		LDS tmp2,Clock+1
		SWAP tmp2
		ANDI tmp2,0x0F
		STS B_2,tmp2
		BTC_F B_2

		LDS tmp2,B_2
		STS StringOutTwo+8,tmp2
		LDS tmp2,B_2+1
		STS StringOutTwo+9,tmp2
		LDS tmp2,B_2+2
		STS StringOutTwo+11,tmp2
		LDS tmp2,B_2+3
		STS StringOutTwo+12,tmp2
		//Теперь часы и дни
		LDS tmp2,Clock+2
		ANDI tmp2,0x0F
		STS B_2+3,tmp2
		LDS tmp2,Clock+2
		SWAP tmp2
		ANDI tmp2,0x0F
		STS B_2+2,tmp2

		
		LDS tmp2,Clock+4
		ANDI tmp2,0x0F
		STS B_2+1,tmp2
		LDS tmp2,Clock+4
		SWAP tmp2
		ANDI tmp2,0x0F
		STS B_2,tmp2
		BTC_F B_2

		LDS tmp2,B_2
		STS StringOutTwo+14,tmp2
		LDS tmp2,B_2+1
		STS StringOutTwo+15,tmp2
		LDS tmp2,B_2+2
		STS StringOutTwo+5,tmp2
		LDS tmp2,B_2+3
		STS StringOutTwo+6,tmp2

		
		//Теперь месяц и год
		LDS tmp2,Clock+5
		ANDI tmp2,0x0F
		STS B_2+3,tmp2
		LDS tmp2,Clock+5
		SWAP tmp2
		ANDI tmp2,0x0F
		STS B_2+2,tmp2

		
		LDS tmp2,Clock+6
		ANDI tmp2,0x0F
		STS B_2+1,tmp2
		LDS tmp2,Clock+6
		SWAP tmp2
		ANDI tmp2,0x0F
		STS B_2,tmp2
		BTC_F B_2

		LDS tmp2,B_2
		STS StringOutTwo+20,tmp2
		LDS tmp2,B_2+1
		STS StringOutTwo+21,tmp2
		LDS tmp2,B_2+2
		STS StringOutTwo+17,tmp2
		LDS tmp2,B_2+3
		STS StringOutTwo+18,tmp2
RET
SCANM_One:
		MOV_32 B_2,Speed
		LPM_Z_X Const_3_6,C_2,4
		MUL_F B_2,C_2	//Скорость км/ч
		LDI_8 C_2,10
		MUL_F B_2,C_2
		FloatToInt B_2
		IntToBCD B_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F B_2,C_2
		LDS		word,B_2+1
		STS		StringOutOne+3+3,word
		LDS		word,B_2+2
		STS		StringOutOne+4+3,word
		LDS		word,B_2+3
		STS		StringOutOne+6+3,word
		
		//Повортники.
		LDS 	word, WordOut+1
		ANDI	word,0b00110000
		CPI		word,0b00110000
		BRNE /*!=*//* SCANM_2_01
			LDI		word,'<'
			STS		StringOutOne+3,word
		SCANM_2_01:
		CPI		word,0b00100000
		BRNE /*!=*//* SCANM_2_02
			LDI		word,'>'
			STS		StringOutOne+15+3,word
		SCANM_2_02:
		CPI		word,0b00010000
		BRNE /*!=*//* SCANM_2_03
			LDI		word,'!'
			STS		StringOutOne+7+3,word
			STS		StringOutOne+8+3,word
		SCANM_2_03:
		MOV_32 B_2,Kaden
		FloatToInt B_2
		IntToBCD B_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F B_2,C_2
		LDS		word,B_2+1
		STS		StringOutOne+12+3,word
		LDS		word,B_2+2
		STS		StringOutOne+13+3,word
		LDS		word,B_2+3
		STS		StringOutOne+14+3,word
RET
SCANM_Two_Dist:
		LDI_16 C_2,1000
		CP_F B_2,C_2
		BRLO /*<*/ /*SCANM_6_Time_01
			LDI word,'k'
			STS		StringOutTwo+5+3,word
			LDI word,'m'
			STS		StringOutTwo+6+3,word
			DIV_F	B_2,C_2 //Метры в километры
		SCANM_6_Time_01:
		FloatToInt B_2
		IntToBCD B_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F B_2,C_2
		LDS		word,B_2+1
		STS		StringOutTwo+2+3,word
		LDS		word,B_2+2
		STS		StringOutTwo+3+3,word
		LDS		word,B_2+3
		STS		StringOutTwo+4+3,word
RET
SCANM_Two_Time:
		LDI_16 C_2,1000
		DIV_F	B_2,C_2 //время в секундах
		MOV_32 D_2,B_2
		LDI_16 C_2,3600 //секунд в часе
		DIV_F	B_2,C_2 //Количество часов
		FloatToInt B_2
		MOV_32 E_2,B_2
		IntToBCD B_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F B_2,C_2
		LDS		word,B_2+2
		STS		StringOutTwo+11+3,word
		LDS		word,B_2+3
		STS		StringOutTwo+12+3,word
		LDI_16 C_2,3600
		MUL_F E_2,C_2 //Колличество секунд которые забирают часы
		SUB_F D_2,E_2 //Сколько секунд приходится на минуты и секунды
		MOV_32 B_2,D_2
		LDI_8 C_2,60
		DIV_F	B_2,C_2 //Количество минут
		FloatToInt B_2
		MOV_32 E_2,B_2
		IntToBCD B_2
		LPM_Z_X Const_0_3_3_3,C_2,4
		ADD_F B_2,C_2
		LDS		word,B_2+2
		STS		StringOutTwo+14+3,word
		LDS		word,B_2+3
		STS		StringOutTwo+15+3,word



SCANModeUp:
	LDS	word,Mode
	CPI	word,Mode1
	BREQ /*=*/ /*SCANM_U_1
	CPI	word,Mode2
	BREQ /*=*/ /*SCANM_U_2
	CPI	word,Mode3
	BREQ /*=*/ /*SCANM_U_3
	CPI	word,Mode4
	BREQ /*=*/ /*SCANM_U_4
	CPI	word,Mode5
	BREQ /*=*/ /*SCANM_U_5
	CPI	word,Mode6
	BREQ /*=*/ /*SCANM_U_6
	CPI	word,Mode7
	BREQ /*=*/ /*SCANM_U_7
	CPI	word,Mode8
	BREQ /*=*/ /*SCANM_U_1
	RJMP SCANM_U_00


	SCANM_U_1:
		LDI word,Mode2
		RJMP SCANM_U_00
	SCANM_U_2:
		LDI word,Mode3
		RJMP SCANM_U_00
	SCANM_U_3:
		LDI word,Mode4
		RJMP SCANM_U_00
	SCANM_U_4:
		LDI word,Mode5
		RJMP SCANM_U_00
	SCANM_U_5:
		LDI word,Mode6
		RJMP SCANM_U_00
	SCANM_U_6:
		LDI word,Mode7
		RJMP SCANM_U_00
	SCANM_U_7:
		LDI word,Mode8
		RJMP SCANM_U_00



	SCANM_U_00:
		STS Mode,word
		SetTimerTask TS_SCANModeUp,999
		NewScrinUpdate
		SetTimerTask TS_SCANMode,500 //Чтобы успел обновиться
RET



ScreenBlink:
			LDS		OSRG,StringOutOne+(StringOutSize-1)-3
			CPI OSRG,4
			BRNE /*!=*/ /*SB01
				LDI 	YL,low(StringOutFour)
				LDI 	YH,high(StringOutFour)
				rjmp	SB04
			SB01:
			CPI OSRG,3
			BRNE /*!=*//* SB02
				LDI 	YL,low(StringOutThree)
				LDI 	YH,high(StringOutThree)
				rjmp	SB04
			SB02:
			CPI OSRG,2
			BRNE /*!=*/ /*SB03
				LDI 	YL,low(StringOutTwo)
				LDI 	YH,high(StringOutTwo)
				rjmp	SB04
			SB03:
				LDI 	YL,low(StringOutOne)
				LDI 	YH,high(StringOutOne)
			SB04:

			MOV 	ZL,YL	//Берём первую строку
			MOV 	ZH,YH	
			ADIW	Zl,(StringOutSize-1)				//Z+=StringOutOneSize-1
														//Смотрим последний символ, этой строки

			LD		word,Z
			CPI		word,0x20
			BREQ	SCFAB03 //Перейти, если там не квадрат, значит вообще тут нам делать нечего
					LD		tmp2,Z		//Этим мы блинкуем
					LD		Counter,-Z //Сразу за знаком идут номер мигающего символа
					LD		word,-Z    //И его оригинал

					ADIW	Yl,3 //Y+=3, пропускаем начальные символы
					DEC		Counter //Опять массивы с нуля
					ADD		YL,Counter
					BRCC	SCFAB05
						INC Yh
					SCFAB05:
					LD		OSRG,Y
					CP		OSRG,tmp2
					BREQ SCFAB0302 //Если равен, то идём по ссылке
						MOV		word,tmp2
					SCFAB0302:
					ST	Y,word
					SetTimerTask TS_ScreenBlink,250
					RJMP SCFAB04
			SCFAB03:
				//Кончаем блинковать!
				LD		word,Z
				CPI		word,0xFF		//Если последний символ - чёрный квадрат, значит мы мигали этой строкой. А вдруг мы забыли снять чёрный квадрат ?!
				BRNE	SCFAB04 //Перейти, если там не квадрат, значит вообще тут нам делать нечего, теперь уж точно
					LDI		word,' '
					ST		Z,word

					LD		Counter,-Z //Сразу за знаком идут номер мигающего символа
					LD		word,-Z    //И его оригинал

					ADIW	Yl,3 //Y+=3, пропускаем начальные символы
					DEC		Counter //Опять массивы с нуля
					ADD		YL,Counter
					BRCC	SCFAB0301
						INC Yh
					SCFAB0301:
					ST	Y,word
			SCFAB04:
RET*/