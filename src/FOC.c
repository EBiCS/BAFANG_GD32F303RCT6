/*
 * FOC.c
 *
 *  Created on: 25.01.2019
 *      Author: Stancecoke
 */
#include "main.h"
#include "config.h"
#include "FOC.h"
#include <arm_math.h>

//q31_t	T_halfsample = 0.00003125;
//q31_t	counterfrequency = 64000000;
//q31_t	U_max = (1/_SQRT3)*_U_DC;
q31_t	temp1;
q31_t	temp2;
q31_t	temp3;
q31_t	temp4;
q31_t	temp5;
q31_t	temp6;

q31_t q31_i_q_fil = 0;
q31_t q31_i_d_fil = 0;

q31_t x1;
q31_t x2;


q31_t z;
q31_t startup_counter=0;

q31_t q31_erps_counter=10000;
q31_t q31_erps_filtered=5000;
int8_t debug[720];

q31_t z;
uint16_t j=0;
uint8_t ui8_debug_state=0;

char PI_flag=0;


//const q31_t _T = 2048;



void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target, MotorState_t* MS_FOC, MotorParams_t* MP_FOC);
void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta);
q31_t atan2_LUT(q31_t e_alpha, q31_t e_beta);
void observer_update(long long v_alpha, long long v_beta, long long i_alpha, long long i_beta,  q31_t *e_alpha,q31_t *e_beta);
int utils_truncate_number_abs(long long *number, q31_t max);

void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target, MotorState_t* MS_FOC, MotorParams_t* MP_FOC)
{

	 q31_t q31_i_alpha = 0;
	 q31_t q31_i_beta = 0;
	 q31_t q31_u_alpha = 0;
	 q31_t q31_u_beta = 0;
	 q31_t q31_i_d = 0;
	 q31_t q31_i_q = 0;
	 q31_t q31_i_alpha_corr = 0;
	 q31_t q31_i_beta_corr = 0;
	 static q31_t q31_angle_old = 0;
	 q31_t sinevalue=0, cosinevalue = 0;
//	 temp6= (((q31_teta >> 23) * 180) >> 9);
//	 if(temp6 !=temp5){
//		 debug[j]=(int8_t)temp6;
//
//		 temp5=debug[j];
//		 if(j<720) j++;
//		 else j=0;
//	 }


	// temp5=(q31_t)int16_i_as;
	// temp6=(q31_t)int16_i_bs;

	// Clark transformation
	arm_clarke_q31((q31_t)int16_i_as, (q31_t)int16_i_bs, &q31_i_alpha, &q31_i_beta);

	arm_sin_cos_q31(q31_teta, &sinevalue, &cosinevalue);
	if(sinevalue==-2147483648)sinevalue=-2147483647;
	if(cosinevalue==2147483648)cosinevalue=2147483647;

	if(MS_FOC->Obs_flag){
		arm_park_q31(q31_i_alpha, q31_i_beta, &q31_i_alpha_corr, &q31_i_beta_corr, MS_FOC->sin_delay_filter,  MS_FOC->cos_delay_filter);
	}


	// Park transformation
	arm_park_q31(q31_i_alpha, q31_i_beta, &q31_i_d, &q31_i_q, sinevalue, cosinevalue);


	q31_i_q_fil -= q31_i_q_fil>>3;
	q31_i_q_fil += q31_i_q;
	MS_FOC->i_q=q31_i_q_fil>>3;


	q31_i_d_fil -= q31_i_d_fil>>3;
	q31_i_d_fil += q31_i_d;
	MS_FOC->i_d=q31_i_d_fil>>3;


	if(MS_FOC->i_d>(PH_CURRENT_MAX<<2)){
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);
		timer_primary_output_config(TIMER0,DISABLE)	;	//disable PWM if overcurrent detected
		while(1){}						//stay here until hard reset
	}






	if(!MS_FOC->hall_angle_detect_flag){
		MS_FOC->u_d = 200;
		MS_FOC->u_q = 0;
	}
	else{ //workaround, as long as no current control is implemented
		runPIcontrol();
	}


	//inverse Park transformation
	arm_inv_park_q31(MS_FOC->u_d, MS_FOC->u_q, &q31_u_alpha, &q31_u_beta, -sinevalue, cosinevalue);


		if(q31_erps_counter<10000){
				q31_erps_counter++;
				//MS_FOC->system_state = Sensorless;
			}


		if (q31_angle_old>(1<<25)&&MS_FOC->teta_obs<-(1<<25)&&q31_erps_counter>15){   //Find switch from +180� to -179,999� to detect one completed electric revolution.

			q31_erps_filtered-=q31_erps_filtered>>4;
			q31_erps_filtered+=q31_erps_counter;
			if(MS_FOC->Obs_flag)MS_FOC->Speedx100=q31_erps_filtered>>4;
			q31_erps_counter=0;
		}
		q31_angle_old=MS_FOC->teta_obs;





	//call SVPWM calculation
	svpwm(q31_u_alpha, q31_u_beta);
	//temp6=__HAL_TIM_GET_COUNTER(&htim1);

}
//PI Control for quadrature current iq (torque)
q31_t PI_control (PI_control_t* PI_c)
{
	float Delta = (PI_c->setpoint - PI_c->recent_value);
	    float p_part; //proportional part
	    p_part= Delta*PI_c->gain_p;
	    temp5=p_part;
	    PI_c->integral_part += Delta*PI_c->gain_i;


	  if (PI_c->integral_part > PI_c->limit_i) PI_c->integral_part = PI_c->limit_i;
	  if (PI_c->integral_part < -(PI_c->limit_i)) PI_c->integral_part = -(PI_c->limit_i);
	//  if(!(TIMER_CCHP(TIMER0)&(uint32_t)TIMER_CCHP_POEN))PI_c->integral_part = 0 ; //reset integral part if PWM is disabled

	    //avoid too big steps in one loop run
	  if (p_part+PI_c->integral_part > PI_c->out+PI_c->max_step) PI_c->out+=PI_c->max_step;
	  else if  (p_part+PI_c->integral_part < PI_c->out-PI_c->max_step)PI_c->out-=PI_c->max_step;
	  else PI_c->out=(p_part+PI_c->integral_part);


	  if (PI_c->out>PI_c->limit_output) PI_c->out = PI_c->limit_output;
	  if (PI_c->out<-(PI_c->limit_output)) PI_c->out = -(PI_c->limit_output); // allow no negative voltage.
	 // if(!(TIMER_CCHP(TIMER0)&(uint32_t)TIMER_CCHP_POEN))PI_c->integral_part = 0  ; //reset output if PWM is disabled

	  return (PI_c->out);
}


void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta)	{

/*
SVPWM according to chapter 4.9 of UM1052, ualpha and ubeta are in range 0 ... 1 in the paper
could be done by floating point with the GD32F303. scaled to 0 ... 2^11 in EBiCS due to missing FPU
*/

	q31_t q31_U_alpha = (_SQRT3 *_T * q31_u_alpha)>>4;
	q31_t q31_U_beta = -_T * q31_u_beta;
	q31_t X = q31_U_beta;
	q31_t Y = (q31_U_alpha+q31_U_beta)>>1;
	q31_t Z = (q31_U_beta-q31_U_alpha)>>1;

	//Sector 1 & 4
	if ((Y>=0 && Z<0 && X>0)||(Y < 0 && Z>=0 && X<=0)){
		switchtime[0] = ((_T+X-Z)>>12) + (_T>>1); //right shift 11 scaling to ualpha, ubeta range to 0...2048, right shift 1 for dividing by 2
		switchtime[1] = switchtime[0] + (Z>>11);
		switchtime[2] = switchtime[1] - (X>>11);
		//temp4=1;
	}

	//Sector 2 & 5
	if ((Y>=0 && Z>=0) || (Y<0 && Z<0) ){
		switchtime[0] = ((_T+Y-Z)>>12) + (_T>>1);
		switchtime[1] = switchtime[0] + (Z>>11);
		switchtime[2] = switchtime[0] - (Y>>11);
		//temp4=2;
	}

	//Sector 3 & 6
	if ((Y<0 && Z>=0 && X>0)||(Y >= 0 && Z<0 && X<=0)){
		switchtime[0] = ((_T+Y-X)>>12) + (_T>>1);
		switchtime[2] = switchtime[0] - (Y>>11);
		switchtime[1] = switchtime[2] + (X>>11);
		//temp4=3;
	}


}


int utils_truncate_number_abs(long long *number, q31_t max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < -max) {
		*number = -max;
		did_trunc = 1;
	}

	return did_trunc;
}





