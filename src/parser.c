/*
 * parser.c
 *
 *  Created on: 30.11.2025
 *      Author: stancecoke
 */

#include "main.h"
#include "CAN_Display.h"

uint16_t l=0;

void parse_DPparams(MotorParams_t* MP){
	MP->system_voltage = Para1[0];
	MP->battery_current_max=Para1[1]*1000;
	MP->max_voltage = Para1[2];
	MP->freerun_strenght=Para1[10]; //Limp Mode SoC Limit field
	MP->phase_current_max=Para1[9]*1000/CAL_I; //uses field Max Current on Low Charge
	MP->gear_ratio=Para1[19];
	MP->MagicNumber=Para1[24]+(Para1[25]<<8);
	MP->throttle_offset=(Para1[34]<<12)/50; //map 5.0V to 12 bit ADC resolution
	MP->throttle_max=(Para1[35]<<12)/50; //map 5.0V to 12 bit ADC resolution
	MP->voltage_min=(Para1[3]+(Para1[4]<<8))/CAL_BAT_V;
	MP->throttle_exponent=Para1[15];//field cadence sensor signals per rotation
	MP->Cadence_exponent=Para1[12];
	MP->legalflag=Para1[14];
	if (!Para1[18])MP->reverse=-1;
	else MP->reverse=1;
	MP->pulses_per_revolution=Para1[20];
	MP->decay_base=Para1[21];
	MP->Override_Duration=Para1[37]*40;
	MP->PAS_timeout= Para1[38]*400; //in Zehntelsekunden, use field Current Loading Time (Ramp Up)
	MP->ramp_end = 11250/Para1[39]; //use field Current Shedding Time (Ramp Down), calculate timer tics from theshold cadence
	MP->walk_assist_speed = Para1[60]+(Para1[61]<<8);
	if (MP->walk_assist_speed == 0) MP->walk_assist_speed = 600; // fallback: 6.0 km/h
	MP->walk_assist_current = Para1[36];
	if (MP->walk_assist_current == 0 || MP->walk_assist_current > 100) MP->walk_assist_current = 30; // fallback: 30%
	memcpy(&MP->assist_profile[0][0],&Para2[0],30);
	memcpy(&MP->ext_boost_duration[0]+1,&Para2[0]+31,5);
	memcpy(&MP->ext_boost_strength[0]+1,&Para2[0]+37,5);

	for (k=0; k < 4; k++){
		MP->assist_settings[k+1][0]=Para1[k*2+41]; //current limit (%)
		MP->assist_settings[k+1][1]=Para1[k*2+50]; //speed limit (%)
		MP->assist_settings[k+1][2]=Para0[k*2+2];  //ride mode (Acceleraton in Canable Tool)
	}
	MP->assist_settings[5][0]=Para1[48];
	MP->assist_settings[5][1]=Para1[57];
	MP->assist_settings[5][2]=Para0[9];

	MP->assist_settings[0][0]=0;
	MP->assist_settings[0][1]=0;
	MP->assist_settings[0][2]=0;
	MP->ext_boost_strength[0]=0;
	MP->ext_boost_duration[0]=0;

	//Torque override Threshold
	for (k=0; k < 4; k++){
		MP->TQO_threshold[k+1]=Para0[k*4+12]+(Para0[k*4+13]<<8);  // use field Assist ratio
	}
	MP->TQO_threshold[5]=Para0[26]+(Para0[27]<<8);
	MP->TQO_threshold[0]=3299;
}


void parse_MOparams(MotorParams_t* MP){
	Para1[0] = MP->system_voltage;
	Para1[1] = MP->battery_current_max/1000;
	Para1[2] = MP->max_voltage;
	Para1[3] = (MP->voltage_min*CAL_BAT_V)&0xFF;
	Para1[4] = ((MP->voltage_min*CAL_BAT_V)>>8)&0xFF;
	Para1[9]= (MP->phase_current_max*CAL_I/1000);
	Para1[10]= MP->freerun_strenght;
	Para1[12]= MP->Cadence_exponent;
	Para1[14]= MP->legalflag;
	if (MP->reverse==-1)Para1[18]=0;
	else Para1[18]=1;
	Para1[19]= MP->gear_ratio;
	Para1[20]= MP->pulses_per_revolution;
	Para1[21]= MP->decay_base;
	Para1[24]= (MP->MagicNumber)&0xFF;
	Para1[25]= ((MP->MagicNumber)>>8)&0xFF;
	Para1[34]= ((MP->throttle_offset*50)>>12)+1; //map 5.0V to 12 bit ADC resolution
	Para1[35]= ((MP->throttle_max*50)>>12)+1; //map 5.0V to 12 bit ADC resolution
//	Para1[34] = ((uint32_t)MP->throttle_offset * 50u + 2048u) >> 12; //suggestion of Claude for correct rounding
//	Para1[35] = ((uint32_t)MP->throttle_max    * 50u + 2048u) >> 12;
	Para1[15] = MP->throttle_exponent;//field cadence sensor signals per rotation
	Para1[37]= MP->Override_Duration/40;// used for override duration
	Para1[38]= MP->PAS_timeout*10/4000; //in Zehntelsekunden, use field Current Loading Time (Ramp Up)
	Para1[39]= 11250/MP->ramp_end; //use field Current Shedding Time (Ramp Down), calculate threshold cadence from timer tics
	Para1[36]= MP->walk_assist_current;
	Para1[60]= MP->walk_assist_speed&0xFF;
	Para1[61]= (MP->walk_assist_speed>>8)&0xFF;

	memcpy(&Para2[0],&MP->assist_profile[0][0],30);
	memcpy(&Para2[0]+31,&MP->ext_boost_duration[0]+1,5);
	memcpy(&Para2[0]+37,&MP->ext_boost_strength[0]+1,5);
	for (k=0; k < 4; k++){
		Para1[k*2+41]= MP->assist_settings[k+1][0]; //current limit (%)
		Para1[k*2+50]= MP->assist_settings[k+1][1]; //speed limit (%)
		Para0[k*2+2]= MP->assist_settings[k+1][2];  //ride mode (Acceleraton in Canable Tool)
	}
	Para1[48]= MP->assist_settings[5][0];
	Para1[57]= MP->assist_settings[5][1];
	Para0[9]= MP->assist_settings[5][2];

	//Torque override Threshold
	for (k=0; k < 4; k++){
		Para0[k*4+12]=MP->TQO_threshold[k+1]&0xFF;
		Para0[k*4+13]=MP->TQO_threshold[k+1]>>8;  // use field Assist ratio
	}


	Para0[26]= MP->TQO_threshold[5]&0xFF;
	Para0[27]= MP->TQO_threshold[5]>>8;

	update_checksum();
}

void InitEEPROM(MotorParams_t* MP){
	MP->TS_coeff=TS_COEF;
	MP->MagicNumber=202;
	MP->battery_current_max=BATTERYCURRENT_MAX;
	MP->gear_ratio=GEAR_RATIO;
	MP->throttle_offset=THROTTLE_OFFSET; 
	MP->throttle_max=THROTTLE_MAX; 
	MP->throttle_exponent=100; //value/100: default value 1
	MP->reverse=REVERSE;
	MP->Cadence_exponent=10;
	MP->pulses_per_revolution=PULSES_PER_REVOLUTION;
	MP->phase_current_max = PH_CURRENT_MAX;
	MP->voltage_min=VOLTAGE_MIN;
	MP->legalflag = LEGALFLAG;
	MP->Override_Duration=4000;
	MP->PAS_timeout = PAS_TIMEOUT;
	MP->ramp_end = RAMP_END;
	MP->walk_assist_speed = 600; // default 6.0 km/h
	MP->walk_assist_current = 30; // default 80%
	MP->system_voltage = SYSTEM_VOLTAGE;
	MP->max_voltage = MAX_VOLTAGE;
	MP->decay_base =20;
	MP->walk_assist_current = 10;
	MP->walk_assist_speed = 600;
	MP->freerun_strenght=0;
	for (k=0; k < 6; k++){
		for (l=0; l < 7; l++){
			MP->assist_profile[k][l]=(k+1)*20;
		}
	}
	for (l=0; l < 6; l++){
		MP->ext_boost_duration[l]=l*20;
		MP->ext_boost_strength[l]=l*20-5;
	}

	for (k=0; k < 4; k++){
		MP->assist_settings[k+1][0]=100; //current limit (%)
		MP->assist_settings[k+1][1]=100; //speed limit (%)
		MP->assist_settings[k+1][2]=TQFILTER;  //ride mode (Acceleraton in Canable Tool)
	}
	MP->assist_settings[5][0]=100;
	MP->assist_settings[5][1]=100;
	MP->assist_settings[5][2]=TQFILTER;

	MP->assist_settings[0][0]=0;
	MP->assist_settings[0][1]=0;
	MP->assist_settings[0][2]=0;

	for (k=0; k < 5; k++){
		MP->TQO_threshold[k+1]=3299;
	}
	MP->TQO_threshold[0]=3299;
	PWM_offset=0;
	Z_position=60;
	write_virtual_eeprom();
}
