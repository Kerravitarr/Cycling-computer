/*
 * TWIDriver.inc
 *
 *  Created: 08.02.2019 14:31:58
 *   Author: Terran
 */ 
 

 //Регистр настроек ctrl_hum:
.equ BME_ADR_ctrl_hum = 0xF2
	//Режим передискретизации для измерения влажности
	.equ BME_orsr_h_OFF = 0b000 //	измерения отключены
	.equ BME_orsr_h_1	= 0b001 //	передискретизация x 1
	.equ BME_orsr_h_2	= 0b010 //	передискретизация x 2
	.equ BME_orsr_h_4	= 0b011 //	передискретизация x 4
	.equ BME_orsr_h_8	= 0b100 //	передискретизация x 8
	.equ BME_orsr_h_16	= 0b101 // другие	передискретизация x 16
//Регистр status: Данный регистр служит для считывания статуса измерений.
.equ BME_ADR_status = 0xF3
	.equ BME_bit_measuring = 3		//Автоматически устанавливается в "1" при выполнении преобразования
	.equ BME_bit_im_update = 0		//Автоматически устанавливается в "1" при копировании данных NVM в регистры изображений
 //Регистр настроек ctrl_meas:
.equ BME_ADR_ctrl_meas = 0xF4
	//Режим передискретизации для измерения температуры
	.equ BME_osrs_t_OFF = 0b00000000	//измерения отключены (значение по умолчанию 0x80000)
	.equ BME_osrs_t_1	= 0b00100000	//передискретизация x 1
	.equ BME_osrs_t_2	= 0b01000000	//передискретизация x 2
	.equ BME_osrs_t_4	= 0b01100000	//передискретизация x 4
	.equ BME_osrs_t_8	= 0b10000000	//передискретизация x 8
	.equ BME_osrs_t_16	= 0b10100000 //, другие	передискретизация x 16
	//Режим передискретизации для измерения давления
	.equ BME_osrs_p_OFF = 0b00000	//измерения отключены (значение по умолчанию 0x80000)
	.equ BME_osrs_p_1	= 0b00100	//передискретизация x 1
	.equ BME_osrs_p_2	= 0b01000	//передискретизация x 2
	.equ BME_osrs_p_4	= 0b01100	//передискретизация x 4
	.equ BME_osrs_p_8	= 0b10000	//передискретизация x 8
	.equ BME_osrs_p_16	= 0b10100	//, другие	передискретизация x 16
	//Выбор режима работы датчика
	.equ BME_mode_S	= 0b00	//Sleep mode 
	.equ BME_mode_F	= 0b01	//или 0b10	Forced mode
	.equ BME_mode_N	= 0b11	//Normal mode
//Регистр настроек config:
.equ BME_ADR_config = 0xF5
	//Time standby. Контроль интерактивной задержки в режиме Normal mode
	.equ BME_t_sb_05	= 0b00000000 	//0.5 ms
	.equ BME_t_sb_62	= 0b00100000 	//62.5 ms
	.equ BME_t_sb_125	= 0b01000000 	//125 ms
	.equ BME_t_sb_250	= 0b01100000 	//250 ms
	.equ BME_t_sb_500	= 0b10000000 	//500 ms
	.equ BME_t_sb_1000	= 0b10100000 	//1000 ms
	.equ BME_t_sb_10	= 0b11000000 	//10  ms
	.equ BME_t_sb_20	= 0b11100000 	//20  ms
	//Контроль IIR фильтра
	.equ BME_filter_OFF	= 0b00000	//Фильтр выключен
	.equ BME_filter_2	= 0b00100	//2
	.equ BME_filter_4	= 0b01000	//4
	.equ BME_filter_8	= 0b01100	//8
	.equ BME_filter_16	= 0b10000	//16
	//Включение интерфейса SPI
	.equ BME_spi3w_en_OFF	= 0b0	//Выкл
	.equ BME_spi3w_en_ON	= 0b1	//Вкл

//Регистр "reset" служит для сброса значений регистров датчика, если в него записать значение 0xB6.
.equ BME_ADR_reset = 0xE0 
	.equ BME_reset_cmd = 0xB6
//В регистре "id" хранится уникальное значение для каждого датчика - chip_id.
.equ BME_ADR_id = 0xD0 

//Калибровочные константы
//dig_Tx - температура, dig_Px - давление, dig_H - влажность.
.equ BME_ADR_dig_T1 = 0x88 //0x88 / 0x89 - [7:0] / [15:8]	= unsigned short
.equ BME_ADR_dig_T2 = 0x8A //0x8A / 0x8B - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_T3 = 0x8C //0x8C / 0x8D - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_P1 = 0x8E //0x8E / 0x8F - [7:0] / [15:8]	= unsigned short
.equ BME_ADR_dig_P2 = 0x90 //0x90 / 0x91 - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_P3 = 0x92 //0x92 / 0x93 - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_P4 = 0x94 //0x94 / 0x95 - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_P5 = 0x96 //0x96 / 0x97 - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_P6 = 0x98 //0x98 / 0x99 - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_P7 = 0x9A //0x9A / 0x9B - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_P8 = 0x9C //0x9C / 0x9D - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_P9 = 0x9E //0x9E / 0x9F - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_H1 = 0xA1 //0xA1 - [7:0]					= unsigned char

.equ BME_ADR_dig_H2 = 0xE1 // 0xE1 / 0xE2 - [7:0] / [15:8]	= signed short
.equ BME_ADR_dig_H3 = 0xE3 // 0xE3 - [7:0]					= unsigned char
.equ BME_ADR_dig_H4 = 0xE4 // 0xE4 / 0xE5[3:0] - [11:4] / [3:0] = signed short
.equ BME_ADR_dig_H5 = 0xE5 // 0xE5[7:4] / 0xE6 - [3:0] / [11:4] = signed short
.equ BME_ADR_dig_H6 = 0xE7 // 0xE7 - [7:0]					= signed char

//Спец флаги для настройки
.equ BME_MODE_reset = 0
.equ BME_MODE_setctrl_hum = 1
.equ BME_MODE_ctrl_meas = 2
.equ BME_MODE_config = 3
.equ BME_MODE_rearPar1 = 4
.equ BME_MODE_rearPar2 = 5
.equ BME_MODE_rearPar3 = 6

//Загружает 16 бит без знака в 32 бита со знаком
//@0 = @1
.MACRO		_MOV_16u_to_32
	LDS MacroR, @1 + 1
	STS @0+3, MacroR
	LDS MacroR, @1 + 0
	STS @0+2, MacroR
	CLR MacroR
	STS @0+1, MacroR
	STS @0+0, MacroR
.ENDMACRO

//Загружает 16 бит со знаком в 32 бита со знаком
//@0 = @1
.MACRO	_MOV_16s_to_32
	LDS MacroR, @1 + 1
	STS @0+3, MacroR
	LDS MacroR, @1 + 0
	STS @0+2, MacroR
	
	LDS MacroR, @1
	ANDI MacroR, 1 << 7
	BREQ /*=*/ positiv
		SER MacroR
		RJMP end
	positiv:
		CLR MacroR
	end:
	STS @0+1, MacroR
	STS @0+0, MacroR
.ENDMACRO

//Закончили запись данных датчику
TWI_IO_End_BME250_Write:
	LDS tmp2, TWI_IO + 1
	CPI tmp2, BME_ADR_dig_T1
	BRNE /*!=*/ TIE_BME250w_notRp1
		//Мы только что установили внутренний счётчик датчика на адрес настроечных регистров. Будем их читать
		STI TWI_IO+0, BME250Read
		STI TWI_IOl, 14
		CALL StartTWI
		RET
	TIE_BME250w_notRp1:
	CPI tmp2, BME_ADR_dig_P5
	BRNE /*!=*/ TIE_BME250w_notRp2
		//Мы только что установили внутренний счётчик датчика на адрес настроечных регистров. Будем их читать
		STI TWI_IO+0, BME250Read
		STI TWI_IOl, 12 + 1 //Тут мы специально вычитываем ещё один лишний байт. Это нужно чтобы отличать чтение регистров и данных!
		CALL StartTWI
		RET
	TIE_BME250w_notRp2:
	CPI tmp2, BME_ADR_dig_H2
	BRNE /*!=*/ TIE_BME250w_notRp3
		//Мы только что установили внутренний счётчик датчика на адрес настроечных регистров. Будем их читать
		STI TWI_IO+0, BME250Read
		STI TWI_IOl, 7
		CALL StartTWI
		RET
	TIE_BME250w_notRp3:
	CPI tmp2, BME_ADR_status
	BRNE /*!=*/ TIE_BME250w_notReadData
		STI TWI_IO+0, BME250Read
		STI TWI_IOl, 12 //Статус, BME_ADR_ctrl_meas, BME_ADR_config, + 8 показаний
		CALL StartTWI
		RET
	TIE_BME250w_notReadData:
RET
//Закончили чтение данных от датчика
TWI_IO_End_BME250_Read:
	LDS tmp2, TWI_IOc
	CPI tmp2, 14
	BRNE /*!=*/ TIE_BME250r_notRp1
		LDIW Z, TWI_IO+1
		LDIW Y, BME_dig_T1u16 //В памяти МК они идут подряд
		LDI Counter, 7 //[BME_dig_T1u16 ; BME_dig_P4s16]
		TIE_BME250rrp1_l:
			LD tmp2, Z+
			STD Y+1,tmp2
			LD tmp2, Z+
			ST Y, tmp2
			ADIW Y, 2
			DEC Counter
		BRNE /*!=*/ TIE_BME250rrp1_l
	TIE_BME250r_notRp1:
	CPI tmp2, 12 + 1
	BRNE /*!=*/ TIE_BME250r_notRp2
		LDIW Z, TWI_IO+1
		LDIW Y, BME_dig_P5s16
		LDI Counter, 5 //[BME_dig_P5s16;BME_dig_P9s16]
		TIE_BME250rrp2_l:
			LD tmp2, Z+
			STD Y+1,tmp2
			LD tmp2, Z+
			ST Y, tmp2
			ADIW Y, 2
			DEC Counter
		BRNE /*!=*/ TIE_BME250rrp2_l
		LDS tmp3, TWI_IO+12
		STS BME_dig_H1u8, tmp3
	TIE_BME250r_notRp2:
	CPI tmp2, 7
	BREQ /*=*/ TIE_BME250r_Rp3
		RJMP TIE_BME250r_notRp3
	TIE_BME250r_Rp3:
		LDS tmp2, TWI_IO+1
		STS BME_dig_H2s16 + 1, tmp2
		LDS tmp2, TWI_IO+2
		STS BME_dig_H2s16 + 0, tmp2

		LDS tmp3, TWI_IO+3
		STS BME_dig_H3u8, tmp3

		LDS tmp3, TWI_IO+5
		ANDI tmp3, 0x0F
		LDS tmp2, TWI_IO+4
		LSL tmp2
		LSL tmp2
		LSL tmp2
		LSL tmp2
		ANDI tmp2, 0xF0
		OR tmp3, tmp2
		STS BME_dig_H4s16+1,tmp3
		LDS tmp2, TWI_IO+4
		LSR tmp2
		LSR tmp2
		LSR tmp2
		LSR tmp2
		ANDI tmp2, 0x0F
		STS BME_dig_H4s16,tmp2
		
		LDS tmp2, TWI_IO+5
		LSR tmp2
		LSR tmp2
		LSR tmp2
		LSR tmp2
		ANDI tmp2, 0x0F
		LDS tmp3, TWI_IO+6
		LSL tmp3
		LSL tmp3
		LSL tmp3
		LSL tmp3
		ANDI tmp3, 0xF0
		OR tmp3, tmp2
		STS BME_dig_H5s16 + 1,tmp3
		LDS tmp2, TWI_IO+6
		LSR tmp2
		LSR tmp2
		LSR tmp2
		LSR tmp2
		ANDI tmp2, 0x0F
		STS BME_dig_H5s16,tmp2

		LDS tmp3, TWI_IO+7
		STS BME_dig_H6s8, tmp3
		RET
	TIE_BME250r_notRp3:
	CPI tmp2, 12
	BREQ /*=*/ TIE_BME250r_ReadData
		RJMP TIE_BME250r_notReadData
	TIE_BME250r_ReadData:
		LDS tmp2, TWI_IO+1 //Читаем статус. Как было сказано выше - тут должны быть 0
		TST tmp2
		BREQ /*=*/ TIE_BME250r_RDstOk
			//Статус не тот! Попробуем позже
			SetTimerTask TS_WetherUpdate,1
			RET
		TIE_BME250r_RDstOk:
		_CLR_32 B_32
		LDS tmp2, TWI_IO+10
		STS B_32 + 3, tmp2
		LDS tmp2, TWI_IO+9
		STS B_32 + 2, tmp2
		LDS tmp2, TWI_IO+8
		STS B_32 + 1, tmp2
		_LSR_32 B_32, 4 //Так как число состоит из 2 байт и 4 бит!


		_LSR_32 B_32, 3
		_MOV_32 D_32, B_32
		_LSR_32 B_32, 1
		_MOV_16s_to_32 E_32, BME_dig_T1u16
		_LSL_32 E_32, 1
		_SUB_32 D_32, E_32
		_MOV_16s_to_32 ACC_32, BME_dig_T2s16
		EXP_64_F_32 ACC_1
		_MULS64_A_R32 D_32
		_MOV_32 D_32, ACC_32
		_LSR_32 D_32, 11
		
		_MOV_16s_to_32 C_32, BME_dig_T1u16
		_SUB_32 B_32, C_32
		_MOV_32 ACC_32, B_32
		EXP_64_F_32 ACC_1
		_MULS64_A_R32 B_32
		_LSR_64 ACC_1, 12
		_MOV_16s_to_32 C_32, BME_dig_T3s16
		_MULS64_A_R32 C_32
		_LSR_64 ACC_1, 14
		_MOV_32 B_32, ACC_32
		_ADD_32 D_32, B_32
		_MOV_32 T_fine, D_32

		_LDI_A 5
		_MULS64_A_R32 D_32
		_MOV_32 D_32, ACC_32
		_LDI_A 128
		_ADD_32 D_32, ACC_32
		_LSR_64 D_32, 8
		LDS tmp2, D_32 + 3
		STS Temperature + 1, tmp2
		LDS tmp2, D_32 + 2
		STS Temperature + 0, tmp2

		//Тут мы занимаемся подлогом. У нас чертовски много вычилсений. И чтобы ни что не поехало, 
		//Нам надо успеть за 1 мс. Мне стрёмно. ТМП мы посчитали
		//Так что пропустим всех и второй этап выполним ниже
		SBIFlag TWI_Busy
		SetTask TS_BME250_read_P
	TIE_BME250r_notReadData:
RET

//Запускает настройку датчика температуры
WetherInit:
	TSTFlag TWI_Busy
	BREQ /*=*/ WetherInit_not_busy
		RJMP WI00
	WetherInit_not_busy:
		//Мы его используем как флаг
		LDS tmp2, BME_dig_H6s8
		CPI tmp2, BME_MODE_reset
		BRNE /*!=*/ WetherInit_nreset
			STI TWI_IO+0, BME250Write
			STI TWI_IO+1, BME_ADR_reset
			STI TWI_IO+2, BME_reset_cmd
			STI TWI_IOl, 3 ;Записываем 3 регистра - адрес, адрес куда пишем, и значение 
			CALL StartTWI
			STI BME_dig_H6s8, BME_MODE_setctrl_hum
			SetTimerTask TS_WetherInit,10
			RET
		WetherInit_nreset:
		CPI tmp2, BME_MODE_setctrl_hum
		BRNE /*!=*/ WetherInit_nm0
			STI TWI_IO+0, BME250Write
			STI TWI_IO+1, BME_ADR_ctrl_hum
			STI TWI_IO+2, BME_orsr_h_16
			STI TWI_IOl, 3 ;Записываем 3 регистра - адрес, адрес куда пишем, и значение 
			CALL StartTWI
			STI BME_dig_H6s8, BME_MODE_ctrl_meas
			SetTimerTask TS_WetherInit,10
			RET
		WetherInit_nm0:
		CPI tmp2, BME_MODE_ctrl_meas
		BRNE /*!=*/ WetherInit_ncm
			STI TWI_IO+0, BME250Write
			STI TWI_IO+1, BME_ADR_ctrl_meas
			STI TWI_IO+2, BME_osrs_t_16 | BME_osrs_p_16 | BME_mode_N
			STI TWI_IOl, 3
			CALL StartTWI
			STI BME_dig_H6s8, BME_MODE_config
			SetTimerTask TS_WetherInit,10
			RET
		WetherInit_ncm:
		CPI tmp2, BME_MODE_config
		BRNE /*!=*/ WetherInit_nc
			STI TWI_IO+0, BME250Write
			STI TWI_IO+1, BME_ADR_config
			STI TWI_IO+2, BME_t_sb_1000 | BME_filter_16 | BME_spi3w_en_OFF
			STI TWI_IOl, 3
			CALL StartTWI
			STI BME_dig_H6s8, BME_MODE_rearPar1
			SetTimerTask TS_WetherInit,10
			RET
		WetherInit_nc:
		CPI tmp2, BME_MODE_rearPar1
		BRNE /*!=*/ WetherInit_nrp1
			STI TWI_IO+0, BME250Write
			STI TWI_IO+1, BME_ADR_dig_T1	//Будем читать с первого регистра первого блока данных
			STI TWI_IOl, 2
			CALL StartTWI
			STI BME_dig_H6s8, BME_MODE_rearPar2
			SetTimerTask TS_WetherInit,10
			RET
		WetherInit_nrp1:
		CPI tmp2, BME_MODE_rearPar2
		BRNE /*!=*/ WetherInit_nrp2
			STI TWI_IO+0, BME250Write
			STI TWI_IO+1, BME_ADR_dig_P5	//У нас буфер в 20 символов, так что за раз мы можем прочитать только ограниченное количество данных!
			STI TWI_IOl, 2
			CALL StartTWI
			STI BME_dig_H6s8, BME_MODE_rearPar3
			SetTimerTask TS_WetherInit,10
			RET
		WetherInit_nrp2:
		CPI tmp2, BME_MODE_rearPar3
		BRNE /*!=*/ WetherInit_nrp3
			STI TWI_IO+0, BME250Write
			STI TWI_IO+1, BME_ADR_dig_H2	//Будем читать с первого регистра второго блока данных
			STI TWI_IOl, 2
			CALL StartTWI
			SetTimerTask TS_WetherUpdate,10
			RET
		WetherInit_nrp3:

		RET
	WI00:
	//Линия занята, обратитесь позже
	SetTimerTask TS_WetherInit,1
RET

//Считаем давление.
//И мы до сих пор держим в своих руках I2C!
BME250_read_P:
	_LDI_32 C_32, 128000
	_MOV_32 B_32, T_Fine
	_SUB_32 B_32, C_32
	//B_32 = t_fine – 128000;
	_MOV_32 ACC_32,B_32
	EXP_64_F_32 ACC_1
	_MULS64_A_R32 B_32
	_MOV_16s_to_32 C_32, BME_dig_P6s16
	_MULS64_A_R32 C_32
	_MOV_64 DE_64, ACC_1
	//DE_64 = B_32 * B_32 * P6;
	
	LDS tmp2 , B_32
	ANDI tmp2, 1 << 7
	BREQ /*=*/ BME250_read_P_pos1
		//У нас B_32 - отрицательное!
		_SER_64 ACC_1
		RJMP BME250_read_P_end1
	BME250_read_P_pos1:
		_CLR_64 ACC_1
	BME250_read_P_end1:
	_MOV_32 ACC_32,B_32
	EXP_64_F_32 ACC_1
	_MOV_16s_to_32 C_32, BME_dig_P5s16
	_MULS64_A_R32 C_32
	//ACC_1 = B_32 * P5
	_LSL_64 ACC_1, 17
	//ACC_1 = ((B_32 * P5)<<17)
	_ADD_64 ACC_1, DE_64
	//ACC_1 = ACC_1 + DE_64;
	_CLR_64 DE_64
	_MOV_16s_to_32 E_32, BME_dig_P4s16
	_LSL_64 DE_64, 35
	//DE_64 = (P4)<<35
	_ADD_64 ACC_1, DE_64
	_MOV_64 DE_64, ACC_1
	//DE_64 = ACC_1 + DE_64;

	_CLR_64 ACC_1
	_MOV_32 ACC_32,B_32
	EXP_64_F_32 ACC_1
	_MULS64_A_R32 B_32
	_MOV_16s_to_32 C_32, BME_dig_P3s16
	_MULS64_A_R32 C_32
	_LSR_64 ACC_1, 8
	_MOV_64 HL_64, ACC_1
	//HL_64 = (B_32 * B_32 * P3)>>8
	
	_CLR_64 ACC_1
	_MOV_32 ACC_32,B_32
	EXP_64_F_32 ACC_1
	_MOV_16s_to_32 C_32, BME_dig_P2s16
	_MULS64_A_R32 C_32
	_LSL_64 ACC_1, 12
	//ACC = ((B_32 * P2)<<12)
	_ADD_64 HL_64, ACC_1 
	// HL_64 = (B_32 * B_32 * P3)>>8 + ((B_32 * P2)<<12)
	_LDI_32 ACC_1, 1 << (47 - 4 * 8) //Загружаем в ACC 1 < 47
	_ADD_64 ACC_1, HL_64
	_MOV_16u_to_32 C_32, BME_dig_P1u16
	_MULS64_A_R32 C_32
	_LSR_64 ACC_1, 33
	_MOV_64 HL_64, ACC_1
	//HL_64 = (((1<<47)+HL_64)* P1)>>33
	
	_TST_64 ACC_1
	BRNE /*!=*/ BME250_read_P_not0
		//Тут 0, дальше делать нечего
		CLR tmp2
		STS Pressure + 1, tmp2
		STS Pressure + 0, tmp2
		RJMP BME250_read_P_ret
	BME250_read_P_not0:
	
	_CLR_32 B_32
	LDS tmp2, TWI_IO+7
	STS B_32 + 3, tmp2
	LDS tmp2, TWI_IO+6
	STS B_32 + 2, tmp2
	LDS tmp2, TWI_IO+5
	STS B_32 + 1, tmp2
	_LSR_32 B_32, 4 //Так как число состоит из 2 байт и 4 бит!
	_LDI_32 C_32, 1048576
	_SUB_32 C_32, B_32
	//C_32 = 1048576 - adc_P;
	EXP_64_F_32 BC_64
	_LSL_64 BC_64, 31
	_SUB_64 BC_64, DE_64
	_LDI_A 3125
	_MULS64_A_R64 BC_64
	_DIVS_64_A_DIV_R64 HL_64
	_MOV_64 HL_64, ACC_1
	//HL_64 = (((C_32<<31)-DE_64)*3125)/HL_64;
	_MOV_64  DE_64, HL_64
	_LSR_64 DE_64, 13
	_MOV_64 ACC_1, DE_64
	_MULS64_A_R64 DE_64
	_MOV_16s_to_32 E_32, BME_dig_P9s16
	_MULS64_A_R32 E_32
	_LSR_64 ACC_1, 25
	_MOV_64 DE_64, ACC_1
	//DE_64 = (P9 * (HL_64>>13) * (HL_64>>13)) >> 25;
	_MOV_16s_to_32 C_32, BME_dig_P8s16
	_MOV_64 ACC_1, HL_64
	_MULS64_A_R32 C_32
	_LSR_64 ACC_1, 19
	_MOV_64 BC_64, ACC_1
	//BC_64 = (P8 * HL_64) >> 19;
	_ADD_64 HL_64, DE_64
	_ADD_64 HL_64, BC_64
	_LSR_64 HL_64, 8
	//HL_64 = ((HL_64 + DE_64 + BC_64) >> 8)
	_MOV_16s_to_32 C_32, BME_dig_P7s16
	EXP_64_F_32 BC_64
	_LSL_64 BC_64, 4
	_ADD_64 HL_64, BC_64
	//HL_64 = HL_64 + ((P7)<<4);
	_LDI_A 760
	_MULS64_A_R64 HL_64
	_LDI_32 B_32, (256 * 101325)
	_DIVS_64_A_DIV_R32 B_32
	//ACC = HL_64 * 760 / (256 * 101325) мм ртс
	
	LDS tmp2, ACC_32 + 3
	STS Pressure + 1, tmp2
	LDS tmp2, ACC_32 + 2
	STS Pressure + 0, tmp2
	
	BME250_read_P_ret:
	//Тут мы занимаемся подлогом. У нас чертовски много вычилсений. И чтобы ни что не поехало, 
	//Нам надо успеть за 1 мс. Мне стрёмно. ТМП мы посчитали
	//Так что пропустим всех и второй этап выполним ниже
	SetTask TS_BME250_read_H
RET
//Считаем давление.
//И мы до сих пор держим в своих руках I2C!
//Главное не забыть отпустить!
BME250_read_H:
	_LDI_32 C_32, 76800
	_MOV_32 B_32, T_Fine
	_SUB_32 B_32, C_32
	//B_32 = t_fine – 76800;
	
	_CLR_32 C_32
	LDS tmp2, TWI_IO+12
	STS C_32 + 3, tmp2
	LDS tmp2, TWI_IO+11
	STS C_32 + 2, tmp2
	_LSL_32 C_32,14
	//C_32 = (adc_H << 14)
	_MOV_16s_to_32 D_32, BME_dig_H4s16
	_LSL_32 D_32,20
	//D_32 = (H4 << 20)
	_MOV_16s_to_32 ACC_32, BME_dig_H5s16
	EXP_64_F_32 ACC_1
	_MULS64_A_R32 B_32
	//ACC_32 = (H5 * v_x1_u32r)
	_SUB_32 C_32,D_32
	_SUB_32 C_32,ACC_32
	_LDI_32 D_32, 16384
	_ADD_32 C_32, D_32
	//C_32 = (C_32 – D_32 – ACC_32 + 16384)
	_LSR_32 C_32,15
	//C_32 = C_32 >> 15
	_CLR_64 ACC_1
	LDS tmp2, BME_dig_H3u8
	STS ACC_1 + 7, tmp2
	_MULS64_A_R32 B_32
	_LSR_32 ACC_32,11
	//ACC_32 = (v_x1_u32r * H3) >> 11
	_LDI_32 D_32, 32768
	_ADD_32 ACC_32, D_32
	_MOV_32 E_32, ACC_32
	//E_32 = ACC_32 + 32768
	LDS tmp2, BME_dig_H6s8
	TST tmp2
	BRGE /*num>=*/ BME250_read_H_poositiv
		_SER_64 ACC_1
		RJMP BME250_read_H_e
	BME250_read_H_poositiv:
		_CLR_64 ACC_1
	BME250_read_H_e:
	STS ACC_1 + 7, tmp2
	_MULS64_A_R32 B_32
	_LSR_64 ACC_1,10
	//ACC_1  =(v_x1_u32r * H6) >> 10
	_MULS64_A_R32 E_32
	_LSR_64 ACC_1,10
	//ACC_1 = (ACC_1 * E_32)>> 10
	_LDI_32 L_32, 2097152
	EXP_64_F_32 HL_64
	_ADD_64 ACC_1, HL_64
	_MOV_16s_to_32 D_32, BME_dig_H2s16
	_MULS64_A_R32 D_32
	_LDI_32 L_32, 8192
	EXP_64_F_32 HL_64
	_ADD_64 ACC_1, HL_64
	_LSR_64 ACC_1,14
	//ACC_1 = ((ACC_1 + 2097152) * BME_dig_H2s16 + 8192) >> 14
	_MULS64_A_R32 C_32
	_MOV_32 C_32, ACC_32
	//C_32 = ACC_1 * C_32
	_LSR_64 ACC_1,15
	_MOV_32 B_32, ACC_32
	//B_32 = C_32 >> 15
	_MULS64_A_R32 B_32
	_LSR_64 ACC_1,7
	//ACC = (B_32 * B_32) >> 7
	_CLR_32 D_32
	LDS tmp2, BME_dig_H1u8
	STS D_32 + 3, tmp2
	_MULS64_A_R32 D_32
	_LSR_64 ACC_1,4
	//ACC = (ACC * BME_dig_H1u8) >> 4
	_SUB_32 C_32, ACC_32
	//v_x1_u32r = v_x1_u32r – ACC;
	_LSR_32 C_32,10
	//C_32 = C_32 / 1024

	_TST_32 C_32
	BRGE /*num>=*/ BME250_read_H_notNeg
		//Влажность не может быть меньше 0. Так что тут у нас 0
		CLR tmp2
		STS Humidity, tmp2
		RJMP BME250_read_H_ret
	BME250_read_H_notNeg:
	_LSR_32 C_32,12
	LDS tmp2, C_32 + 3
	CPI tmp2, 100
	BRLO /*<*/ BME250_read_H_less100
		//У нас 100 и больше... Не может быть!
		STI Humidity,100
		RJMP BME250_read_H_ret
	BME250_read_H_less100:
		STS Humidity, tmp2

	BME250_read_H_ret:
	CBIFlag TWI_Busy
RET