/*!
    \file    main.h
    \brief   the header file of main 

   \version 2024-12-20, V3.0.1, firmware for GD32F30x
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#ifndef MAIN_H
#define MAIN_H


#include "gd32f30x.h"
#include <arm_math.h>
#include "systick.h"
#include "gd32f307c_eval.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>


/* led spark function */
void led_spark(void);
void TIMER2_IRQHandler(void);
void runPIcontrol(void);
void write_virtual_eeprom(void);
void read_virtual_eeprom(void);
void autodetect(void);
extern uint16_t slow_loop_counter;
extern uint16_t switchtime[3];
extern uint32_t timeout;
extern uint8_t transmit_mailbox;
extern can_trasnmit_message_struct transmit_message;
extern can_receive_message_struct receive_message;
extern FlagStatus receive_flag;
extern int32_t Z_position;
extern int32_t PWM_offset;

enum state {Stop, SixStep, Regen, Running, BatteryCurrentLimit, Interpolation, PLL, IdleRun, Sensorless, OpenLoop};
enum com_mode {Hallsensor, Sensorless_openloop, Sensorless_startkick, Hallsensor_Sensorless};
enum operation {WRITE_CMD,READ_CMD,NORMAL_ACK,ERROR_ACK, LONG_START_CMD, LONG_TRANG_CMD ,LONG_END_CMD,LONG_WARNING_CMD};

typedef struct
{

	int32_t       	Voltage;
	uint32_t       	Speedx100;
	int32_t         i_d;
	int32_t         i_q;
	int32_t 		i_q_setpoint;
	int32_t 		i_d_setpoint;
	int32_t 		i_setpoint_abs;
	int32_t 		i_q_setpoint_temp;
	int32_t 		i_d_setpoint_temp;
	int32_t         u_d;
	int32_t         u_q;
	int32_t         u_abs;
	int32_t         Battery_Current;
	int32_t			teta_obs;
	float       	distance_since_startup;
	int32_t       	sin_delay_filter;
	int32_t       	cos_delay_filter;
	int16_t 		torque_on_crank;
	uint16_t 		torque_filtered;
	uint16_t 		p_human;
	uint16_t        calories;
	uint16_t        range;
	int16_t         int_Temperature;
	int16_t 		KV_detect_flag;
	uint8_t 		hall_angle_detect_flag;
	uint8_t 		char_dyn_adc_state;
	uint8_t 		assist_level;
	uint8_t 		SOC;
	int8_t         	system_state;
	int8_t         	level_counter_global;
	FlagStatus      offroadflag;
	uint8_t         offroadtics;
	int8_t         	error_state;
	int8_t 			angle_est;
	uint8_t 		cadence;
	int8_t 			Obs_flag;
	int8_t 			TQfilter;
	int8_t 			ext_boost_duration;
	int8_t 			ext_boost_strength;
	FlagStatus 		pushassist_flag;
	FlagStatus 		light_flag;
	FlagStatus 		button_up_flag;
	FlagStatus 		button_down_flag;
	FlagStatus 		brake_active_flag;
	FlagStatus 		PAS_active_flag;

}MotorState_t;

typedef struct
{

	uint16_t       	wheel_cirumference;
	uint16_t       	decay_base;
	uint16_t       	Cadence_exponent;
	uint16_t       	Override_Duration;
	uint16_t       	MagicNumber;
	uint16_t       	TS_coeff;
	uint16_t       	PAS_timeout;
	uint16_t       	ramp_end;
	uint16_t       	throttle_offset;
	uint16_t       	throttle_max;
	uint16_t       	torque_offset;
	uint16_t       	torque_max;
	uint16_t       	gear_ratio;
	uint16_t       	phase_current_max;
	uint16_t		battery_current_max;
	int16_t       	voltage_min;
	uint16_t       	speedLimitx100;
	uint16_t       	TQO_threshold[6];
	uint8_t       	throttle_exponent;
	int8_t       	system_voltage;
	int8_t       	max_voltage;
	int8_t       	reverse; //use field Motor Type (Para1[18]) 1 = 1, 0 = -1
	int8_t       	legalflag; //use field Coaster Brake Support

	uint8_t       	pulses_per_revolution;
	uint8_t 		assist_profile[5][6]; //five assist levels with 6 assist factors each
	uint8_t 		assist_settings[6][3]; //six  assist levels (including level zero) with 0: current limit, 1 speed limit, 2 ride mode
	uint8_t 		ext_boost_duration[6];
	uint8_t 		ext_boost_strength[6];
	uint16_t 		battery_capacity;
	uint16_t 		capacity_used;
	uint16_t 		distance_gone;
	uint16_t       	walk_assist_speed; // in km/h x100, e.g. 600 = 6.0 km/h
	uint8_t        	walk_assist_current; // 0-100 %, maps to Para1[36] (speed_limit_enabled in JS)
	int16_t       	PAS_direction;
	uint16_t 		Kv_constant;
	uint8_t			freerun_strenght; //Limp Mode SoC Limit field

}MotorParams_t;

typedef struct
{
	float       	gain_p;
	float       	gain_i;
	int16_t       	limit_i;
	int16_t       	limit_output;
	int16_t       	recent_value;
	int32_t       	setpoint;
	float       	integral_part;
	int16_t       	max_step;
	int32_t       	out;
	int8_t       	shift;

}PI_control_t;

#endif /* MAIN_H */
