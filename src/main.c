/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example

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

#include "main.h"
#include "FOC.h"
#include "CAN_Display.h"
#include "parser.h"
uint16_t adc_value[9];

#define FMC_PAGE_SIZE           ((uint16_t)0x800U)
#define FMC_WRITE_START_ADDR    ((uint32_t)0x0803F000U) //Page 126, Page size 2kB
#define FMC_WRITE_END_ADDR      ((uint32_t)0x0803F800U) //just one page
//#define FMC_OFFSET_PARA0      	((uint32_t)28) //starts after hall angles
//#define FMC_OFFSET_PARA1      	FMC_OFFSET_PARA0 + ((uint32_t)64) //starts after Para1
//#define FMC_OFFSET_PARA2      	FMC_OFFSET_PARA1 + ((uint32_t)64) //starts after Para1
#define FMC_OFFSET_MP			((uint32_t)28) //starts after hall angles
uint32_t *ptrd;
uint32_t address = 0x00000000U;
uint32_t data0   = 0x01234567U;
/* calculate the number of page to be programmed/erased */
uint32_t PageNum = (FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR) / FMC_PAGE_SIZE;
/* calculate the number of words to be programmed/erased */
uint32_t WordNum = ((FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR) >> 2);

can_trasnmit_message_struct transmit_message;
can_receive_message_struct receive_message;
FlagStatus receive_flag;
FlagStatus PAS_flag=0;
FlagStatus Speed_flag=0;
FlagStatus reg_ADC_flag=0;
FlagStatus OnOffButton_flag=0;
FlagStatus BC_limit_flag=0;
uint8_t Backwards_counter=0;

void nvic_config(void);
void led_config(void);
void gpio_config(void);
void rcu_config(void);
void dma_config(void);
void adc_config(void);
void timer0_config(void); //PWM for Mosfet driver
void timer1_config(void); //PWM for triggering regular ADC
void get_torque_correction(void);
void timer3_config(void); // Input capture for signal Z of encoder
void timer4_config(void); // Input capture for signal PWM of encoder
void Encoder_Init(void); // Quadrature signal A/B on timer2
ErrStatus can_networking(void);
void can_networking_init(void);
int32_t speed_PLL (int32_t ist, int32_t soll, uint8_t speedadapt);
void runPIcontrol(void);
void autodetect(void);
int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t map_exp (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void get_standstill_position();
void dyn_adc_state(q31_t angle);
void fmc_program_hall_angles(void);
void fmc_erase_pages(void);
void PAS_processing(void);
void reg_ADC_processing(void);
void UART4_init(void);
int16_t internal_tics_to_speedx100 (uint32_t tics);
int16_t external_tics_to_speedx100 (uint32_t tics);
fmc_state_enum fmc_multi_word_program(uint32_t offset, uint8_t* data, uint8_t words);
void write_virtual_eeprom(void);
void read_virtual_eeprom(void);
uint8_t interpolate_assistfactor(void);
int8_t calculate_SOC(uint16_t voltage, uint8_t cells_in_series);
void print_debug_on_CAN(void);
void Speed_processing(void);
uint16_t map_rezi(int32_t actual_value, int32_t actual_time, int32_t timeout, int32_t decay_base);
uint16_t update_setpoint(void);
int16_t T_NTC(uint16_t ADC);
float u32_to_deg=0.00000008381903171539;
uint16_t slow_loop_counter=0;
uint16_t PAS_counter=0;
uint16_t torque_counter=0;
uint16_t Speed_counter=0;
int32_t ButtonVoltageCumulated=620<<6;
#define iabs(x) (((x) >= 0)?(x):-(x))
#define sign(x) (((x) >= 0)?(1):(-1))
MotorState_t MS;
MotorParams_t MP;
//structs for PI_control
PI_control_t PI_iq;
PI_control_t PI_id;
PI_control_t PI_speed;

uint16_t ui16_timertics=0;
uint8_t ui8_6step_flag=0;
uint8_t ui8_hall_state=0;
uint8_t ui8_hall_state_old=0;
uint8_t ui8_hall_case=0;
uint32_t uint32_tics_filtered=128000;
uint16_t uint16_cadence_filtered=0;
uint8_t ui8_overflow_flag=0;
uint8_t ui8_SPEED_control_flag=0;
uint32_t voltage_raw_cumulated=0;
uint16_t voltage_raw_filtered=0;
int32_t torque_offset_correction=0;
int32_t q31_rotorposition_hall=0;
q31_t q31_rotorposition_absolute=0;
int8_t i8_recent_rotor_direction=0;

uint16_t ui16_tim2_recent=0;
uint16_t uint16_full_rotation_counter=0;
uint16_t uint16_half_rotation_counter=0;
uint16_t speedlimitx100_scaled=0;
int16_t phase_current_max_scaled=0;
int8_t assist_level_old=0;
q31_t q31_u_d_temp=0;
q31_t q31_u_q_temp=0;
//Hall64	691967230
//Hall26	-11930205
//Hall32	-811271360
//Hall13	-1479377400
//Hall51	2123622926
//Hall45	1348142805

int8_t statehistory[36];
uint8_t historycounter=0;
int32_t i32_full_rotation_flag =-1;
int32_t Z_position_cumulated = 0;
int32_t Z_position = 450;
int32_t Hall_26 = -966367405;
int32_t Encoder_raw_angle = 0;
int32_t PWM_offset_cumulated = 0;
int32_t PWM_offset = 0;
float 	helper_cadence=0;
float 	helper_throttle=0;
int32_t q31_PLL_error=0;
int32_t q31_rotorposition_PLL=0;
uint8_t ui_8_PLL_counter=0;
uint8_t shutoffcounter=0;
uint16_t offroadcode=0;
uint16_t offroadcounter=0;
uint8_t ui_8_PWM_ON_Flag=0;
int32_t q31_angle_per_tic=0;
//Rotor angle scaled from degree to q31 for arm_math. -180Ã‚Â°-->-2^31, 0Ã‚Â°-->0, +180Ã‚Â°-->+2^31
const int32_t deg_30 = 357913941;
uint16_t switchtime[3];
uint16_t ui16_erps=0;
uint16_t ui16_erps_counter=0;
uint16_t mapped_throttle=0;
uint16_t mapped_torque=0;
char char_dyn_adc_state_old=1;
int16_t i16_ph1_current=0;
int16_t i16_ph2_current=0;
int16_t i16_ph3_current=0;

int8_t i8_reverse_flag = 1;
const q31_t tics_lower_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*SPEEDLIMIT*10); //tics=wheelcirc*timerfrequency/(no. of hallevents per rev*gear-ratio*speedlimit)*3600/1000000
const q31_t tics_higher_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*(SPEEDLIMIT+2)*10);
const uint16_t tq_threshold = TQ_THRESHOLD;
uint8_t i = 0;
uint16_t p = 0;
uint16_t Overrun_strength = 0;
uint16_t Overrun_counter = 0;
uint8_t Overrun_flag = 0;
uint32_t timeout = 0xFFFF;
uint8_t transmit_mailbox = 0;
int32_t battery_current_cumulated=0;
uint32_t Speedx100_cumulated=0;
uint32_t torque_cumulated=0;
uint8_t array_temp[88];

uint8_t level_to_array_element[10]={0,0,1,0,2,0,3,0,4,5}; //map assist Level to array element
uint16_t Poll_commands[3]={0x3201,0x3200,0x3205};
uint8_t pollnumber=0;
int32_t ic1value = 0,AngleFromPWM = 0;
__IO uint16_t dutycycle = 0;
__IO uint16_t frequency = 0;



/*!
    \brief      toggle the led every 500ms
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_spark(void)
{
    static __IO uint32_t timingdelaylocal = 0U;

    if(timingdelaylocal){

        if(timingdelaylocal < 500U){
            gd_eval_led_on(LED2);
        }else{
            gd_eval_led_off(LED2);
        }

        timingdelaylocal--;
    }else{
        timingdelaylocal = 1000U;
    }
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
#if (BOOTLOADER== 3)
	nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x4000); //for bootloader v3

#elif (BOOTLOADER== 38)
	nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0xA800); //for bootloader v3.8

#elif (BOOTLOADER== 820)
	nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x5000); //for bootloader from M820
#endif


    __enable_irq();

	//SCB->VTOR = 0x08004000;
	fwdgt_config(65000, FWDGT_PSC_DIV256);
#ifdef __FIRMWARE_VERSION_DEFINE
     uint32_t fw_ver = 0;
#endif
    fwdgt_counter_reload();
    //receive_flag = RESET;
    SystemInit();
    /* system clocks configuration */

    rcu_config();
    /* configure systick */
    systick_config();

    /* initialize the LED */
    gd_eval_led_init(LED2);
    gd_eval_hall_init();
    //gd_eval_com_init(EVAL_COM0);
    nvic_config();
    gpio_config();
    /* TIMER configuration */
    timer0_config(); // PWM for Mosfet driver
    timer1_config(); //trigger regular ADC for testing
    //timer2_config(); //for hall sensor handling
    timer3_config(); //for encoder z signal handling
    timer4_config(); //for encoder PWM handling
    Encoder_Init();

    /* DMA configuration */
    dma_config();
    /* ADC configuration */
    adc_config();

    /* initialize CAN and CAN filter */
    can_networking_init();

    /* enable CAN receive FIFO0 not empty interrupt */
    receive_flag = RESET;

    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE1);
#ifdef PRINTDEBUG_UART
    //start UART4 for debug messages
    UART4_init();
#endif
    /* initialize transmit message */
    transmit_message.tx_sfid = 0x7ab;
    transmit_message.tx_efid = 0x00;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;
    //write_virtual_eeprom();


    //initialize MS struct.
	MS.hall_angle_detect_flag=1;
	MS.Speedx100=0; //in km/h*100
	MS.assist_level=2;
	MS.i_q_setpoint = 0;
	MS.i_d_setpoint = 0;
	MS.angle_est=SPEED_PLL;
	MS.pushassist_flag=SET;
	MS.light_flag=SET;
	MS.button_up_flag=SET;
	MS.button_down_flag=SET;
	MS.offroadflag=RESET;
	MS.offroadtics=0;
	MS.pushassist_flag=RESET;
	MS.distance_since_startup=0;

	MP.pulses_per_revolution = PULSES_PER_REVOLUTION;
	MP.wheel_cirumference = WHEEL_CIRCUMFERENCE;
	MP.speedLimitx100=SPEEDLIMIT;
	MP.battery_current_max = BATTERYCURRENT_MAX;
	MP.phase_current_max = PH_CURRENT_MAX;
	MP.TS_coeff = TS_COEF;
	MP.reverse = REVERSE;
	MP.MagicNumber=202;
	MP.Override_Duration=8000;
	MP.decay_base=16;


	//init PI structs
	PI_id.gain_i=I_FACTOR_I_D;
	PI_id.gain_p=P_FACTOR_I_D;
	PI_id.setpoint = 0;
	PI_id.limit_output = _U_MAX;
	PI_id.max_step=5000;
	PI_id.shift=8;
	PI_id.limit_i=1800;

	PI_iq.gain_i=I_FACTOR_I_Q;
	PI_iq.gain_p=P_FACTOR_I_Q;
	PI_iq.setpoint = 0;
	PI_iq.limit_output = _U_MAX;
	PI_iq.max_step=5000;
	PI_iq.shift=8;
	PI_iq.limit_i=_U_MAX;

    //Check, if virtual EEPROM was ever written. If not, fill it with default values
    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;
    if(0xFFFFFFFF == (*(ptrd+1))){
    	InitEEPROM(&MP);
    }
    //read parameters from virtual EEPROM and overwrite the default values
    read_virtual_eeprom();
    parse_MOparams(&MP);

    for (int i = 0; i < 1000; i++) {//let the ADC stabilize
    	while(!reg_ADC_flag);
    	reg_ADC_flag=0;
    }

    get_torque_correction();


    while((adc_value[1])>3000);//safety for bricked throttle

	helper_cadence=((float)1.0/((float)1.0+(float)MP.Cadence_exponent));
	helper_throttle= (float)MP.throttle_exponent/100.0;
    //autodetect();

    while (1){
    	fwdgt_counter_reload();
    	temp1++;
#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
    	if(receive_flag){
    		receive_flag = RESET;
    		processCAN_Rx(&MP, &MS);
    	}

#endif
    	if(offroadcounter>4000){
    		offroadcode=0;
    		MS.offroadtics=0;
    	}
    	if(PAS_flag)PAS_processing();
    	if(Speed_flag)Speed_processing();
    	if(reg_ADC_flag)reg_ADC_processing();
//    	if(PAS_counter>MP.PAS_timeout){
//    		if(!Overrun_flag){
//				Backwards_counter=0;
//				MS.cadence=0;
//				//MS.torque_on_crank=750;
//				MS.p_human=0;
//				uint16_cadence_filtered=0;
//				if(!MS.i_q_setpoint){
//					PI_iq.integral_part=0;
//					PI_id.integral_part=0;
//					}
//				if(torque_cumulated)torque_cumulated--;
//    		}

//    	}
    	// switch lights
    	if(MS.light_flag&&!gpio_input_bit_get(GPIOB,GPIO_PIN_10))GPIO_BOP(GPIOB) = GPIO_PIN_10;
    	if(!MS.light_flag&&gpio_input_bit_get(GPIOB,GPIO_PIN_10)) GPIO_BC(GPIOB) = GPIO_PIN_10;

    	//check brake sensor state
    	if(!gpio_input_bit_get(GPIOC,GPIO_PIN_13))MS.brake_active_flag=1;
    	else MS.brake_active_flag=0;
    	// update scaled current and speed
    	if(MS.assist_level!=assist_level_old){
    		speedlimitx100_scaled=MP.speedLimitx100*MP.assist_settings[level_to_array_element[MS.assist_level]][1]/100;
    		phase_current_max_scaled=MP.phase_current_max*MP.assist_settings[level_to_array_element[MS.assist_level]][0]/100;
        	MS.TQfilter=level_to_array_element[MS.assist_level];
        	MS.TQfilter=MP.assist_settings[MS.TQfilter][2];
        	MS.ext_boost_duration=MP.ext_boost_duration[level_to_array_element[MS.assist_level]];
        	MS.ext_boost_strength=MP.ext_boost_strength[level_to_array_element[MS.assist_level]];
        	if(offroadcounter<4000&&offroadcounter>1000){
        		offroadcode+=pow(10,MS.offroadtics)*MS.assist_level;
        		MS.offroadtics++;
        	}

        	if(offroadcode==MP.MagicNumber){
        		MS.offroadflag=!MS.offroadflag;
        		if(MS.offroadflag)MS.offroadtics=9;
        		else MS.offroadtics=8;
        	}
        	offroadcounter=0;

    		assist_level_old=MS.assist_level;

    	}

            if (slow_loop_counter > 200){ //slow loop every 500ms, Timer1 @4kHz interrupt frequency
            	gd_eval_led_toggle(LED2);
            	//if(PAS_counter>16000&&!ui_8_PWM_ON_Flag&&iabs(MS.torque_on_crank-740)>40)get_torque_correction();
#ifdef PRINTDEBUG_UART

            	//printf("%d, %d, %d, %d, %d\r\n",MS.Battery_Current,MS.i_q_setpoint,MP.reverse*MS.i_q,ui16_erps,temp2);
            	printf("%d, %d, %d, %d, %d\r\n",MS.Battery_Current,MS.i_q_setpoint,MP.reverse*MS.i_q,MS.p_human,MS.Speedx100);
#endif

#ifdef PRINTDEBUG_CAN
            	print_debug_on_CAN();
#endif
//            	if((Overrun_strength-mapped_torque)>>3>0){
//            		Overrun_strength-=(Overrun_strength-mapped_torque)>>3;
//            	}
            	temp2=temp1;
            	temp1=0;
            	if(pollnumber>2)pollnumber=0;
            	sendCAN_Poll(&MP,&MS,Poll_commands[pollnumber]);
            	pollnumber++;
            	MS.int_Temperature = T_NTC(adc_value[6]);
            	MS.SOC = calculate_SOC(MS.Voltage, (uint8_t) ((float)MP.system_voltage/3.6));
            	//toggle speed pin
            	//gpio_bit_write(GPIOB, GPIO_PIN_0,(bit_status)(1-gpio_input_bit_get(GPIOB, GPIO_PIN_0)));
            	if(Speed_counter>20000) MS.Speedx100=0;
				slow_loop_counter = 0;
#if (BOOTLOADER!= 0)
				if(adc_value[5]<2800)shutoffcounter++; //raw value is 4095 without button pressed, about 3300 with "down" button pressed and about 2400 with on/off button pressed.
				else shutoffcounter=0;
				if(shutoffcounter>50){
					timer_primary_output_config(TIMER0,DISABLE); //stop PWM output
					GPIO_BC(GPIOB) = GPIO_PIN_4; //reset Pin4 from Bootloader
				    GPIO_BC(GPIOB) = GPIO_PIN_5; // Display off
				    GPIO_BC(GPIOB) = GPIO_PIN_6; // DC/DC off
				}
#endif
            }//end slow loop

            if(MS.i_q_setpoint){
            	if(!ui_8_PWM_ON_Flag){
					timer_primary_output_config(TIMER0,ENABLE);
					uint16_half_rotation_counter=0;
					ui_8_PWM_ON_Flag=1;
            	}
            }
            if(uint16_half_rotation_counter>4000) {
            	if(ui_8_PWM_ON_Flag){
					timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);
					timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);
					timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);
					timer_primary_output_config(TIMER0,DISABLE); //Disable PWM if motor is not turning
					ui_8_PWM_ON_Flag=0;
					i8_recent_rotor_direction=0;

            	}
            }//end half rotation counter


    }
}

/*!
    \brief      configure the different system clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOC clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA0);
    /* enable TIMER0 clock */
    rcu_periph_clock_enable(RCU_TIMER0);
    /* enable ADC0 clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* enable ADC1 clock */
    rcu_periph_clock_enable(RCU_ADC1);
    /* enable ADC1 clock */
    rcu_periph_clock_enable(RCU_ADC2);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
}

/*!
    \brief      initialize CAN and filter
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_networking_init(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;
    /* initialize CAN structures */
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    /* initialize CAN register */
    can_deinit(CAN0);

    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    /* baudrate 1Mbps */
    can_parameter.prescaler = 24;
    can_init(CAN0, &can_parameter);

    /* initialize filter */
    /* CAN0 filter number */
    can_filter.filter_number = 0;

    /* initialize filter */
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO1;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);


}

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* enable can clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    rcu_periph_clock_enable(RCU_AF);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    /* configure CAN0 GPIO, CAN0_TX(PA12) and CAN0_RX(PA11) */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
	
    /* config the GPIO as analog mode */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_3); //Battery Voltage
    gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0); // Motor Temp

    //gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    //gpio_init(GPIOB, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    //PB6: switch for DC/DC
    //PB5: switch for BatteryPlus display supply
    //delay_1ms(500);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_10);

    GPIO_BOP(GPIOB) = GPIO_PIN_6; //DC/DC on
    delay_1ms(50);
    GPIO_BOP(GPIOB) = GPIO_PIN_5; // Display on
    delay_1ms(50);
    GPIO_BOP(GPIOB) = GPIO_PIN_4; //reset Pin4 from Bootloader
    delay_1ms(50);
	GPIO_BOP(GPIOB) = GPIO_PIN_3; //12V on
	delay_1ms(200);
#if (BOOTLOADER!= 0)
	GPIO_BC(GPIOB) = GPIO_PIN_6; //reset Pin6 for releasing on/off button
#endif
    //PA15 Dual PAS input pin (green wire)
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
    //PC0 light short circuit detectionß1
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
	//PC10 PAS1 (white), PC11 PAS2 (pink), PC13 brake
    gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_11); //Pas2 interrupt
    /* configure key EXTI line */
    exti_init(EXTI_11, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_11);
    //PB2 for external speed sensor
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_2);
    /* configure key EXTI line */
    exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_2);

    /*configure PA8 PA9 PA10(TIMER0 CH0 CH1 CH2) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);

    /*configure PB13 PB14 PB15(TIMER0 CH0N CH1N CH2N) as alternate function*/
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_14);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_15);
}
/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;

    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);

    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&adc_value);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = 9;
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA0, DMA_CH0);

    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}
/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{

    /* configure the ADC sync mode */
    adc_mode_config(ADC_DAUL_INSERTED_PARALLEL_REGULAL_FOLLOWUP_FAST);
    /* ADC scan mode function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(ADC2, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC2, ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC2, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL,9);
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL,1);
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL,1);
    adc_channel_length_config(ADC2, ADC_INSERTED_CHANNEL,1);

    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5); // PA0 Battery Current
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_6, ADC_SAMPLETIME_239POINT5); // PA6 Throttle?
    adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_7, ADC_SAMPLETIME_239POINT5); // PA7 Torque
    adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_13, ADC_SAMPLETIME_239POINT5);// PC3 battery voltage
    adc_regular_channel_config(ADC0, 4, ADC_CHANNEL_5, ADC_SAMPLETIME_239POINT5); // Phase3 current
    adc_regular_channel_config(ADC0, 5, ADC_CHANNEL_4, ADC_SAMPLETIME_239POINT5); // on/off button
    adc_regular_channel_config(ADC0, 6, ADC_CHANNEL_8, ADC_SAMPLETIME_239POINT5);//Motor temperature
    adc_regular_channel_config(ADC0, 7, ADC_CHANNEL_2, ADC_SAMPLETIME_239POINT5);// Phase1 current
    adc_regular_channel_config(ADC0, 8, ADC_CHANNEL_3, ADC_SAMPLETIME_239POINT5);// Phase2 current

    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_5, ADC_SAMPLETIME_55POINT5);
    adc_inserted_channel_offset_config(ADC0, ADC_INSERTED_CHANNEL_0, 2033); //hardcoded, to be improved

    adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_3, ADC_SAMPLETIME_55POINT5);
//    adc_inserted_channel_config(ADC1, 1, ADC_CHANNEL_5, ADC_SAMPLETIME_55POINT5);

    adc_inserted_channel_offset_config(ADC1, ADC_INSERTED_CHANNEL_0, 2045); //hardcoded, to be improved
//    adc_inserted_channel_offset_config(ADC1, ADC_INSERTED_CHANNEL_1, 2033); //hardcoded, to be improved


    adc_inserted_channel_config(ADC2, 0, ADC_CHANNEL_2, ADC_SAMPLETIME_55POINT5);
    adc_inserted_channel_offset_config(ADC2, ADC_INSERTED_CHANNEL_0, 2033); //hardcoded, to be improved


    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_T1_CH1);
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_CH3);
    adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);
    adc_external_trigger_source_config(ADC2, ADC_INSERTED_CHANNEL, ADC2_EXTTRIG_INSERTED_T0_CH3);
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC2, ADC_INSERTED_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
    /* enable ADC interface */
    adc_enable(ADC1);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);
//     /* enable ADC interface */
    adc_enable(ADC2);
    delay_1ms(1);
//    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC2);
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);
    /* enable ADC interrupt */
    adc_interrupt_enable(ADC1, ADC_INT_EOIC);

    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);
}

/*!
    \brief      configure the timer peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer0_config(void)
{
	/* -----------------------------------------------------------------------
	    TIMER0 configuration to:
	    generate 3 complementary PWM signals with 3 different duty cycles:
	    TIMER0CLK is fixed to systemcoreclock, the TIMER0 prescaler is equal to 6000 so the
	    TIMER0 counter clock used is 20KHz.
	    the three duty cycles are computed as the following description:
	    the channel 0 duty cycle is set to 25% so channel 1N is set to 75%.
	    the channel 1 duty cycle is set to 50% so channel 2N is set to 50%.
	    the channel 2 duty cycle is set to 75% so channel 3N is set to 25%.
	  ----------------------------------------------------------------------- */
	    timer_oc_parameter_struct timer_ocintpara;
	    timer_parameter_struct timer_initpara;
	    timer_break_parameter_struct timer_breakpara;
	    rcu_periph_clock_enable(RCU_TIMER0);

	    timer_deinit(TIMER0);

	    /* TIMER0 configuration */
	    timer_initpara.prescaler         = 0;
	    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_BOTH;
	    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	    timer_initpara.period            = _T; //for 32 kHz center aligned --> 16kHz PWM frequency
	    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	    timer_initpara.repetitioncounter = 0;
	    timer_init(TIMER0,&timer_initpara);

	     /* CH1,CH2 and CH3 configuration in PWM mode */
	    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
	    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
	    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;//inverted logic to make ouput in timer logic
	    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
	    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
	    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;

	    timer_channel_output_config(TIMER0,TIMER_CH_0,&timer_ocintpara);
	    timer_channel_output_config(TIMER0,TIMER_CH_1,&timer_ocintpara);
	    timer_channel_output_config(TIMER0,TIMER_CH_2,&timer_ocintpara);
	    timer_channel_output_config(TIMER0,TIMER_CH_3,&timer_ocintpara);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);//grün
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);//gelb
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);//blau
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,(TRIGGER_DEFAULT));//(_T>>1)+500 in the middle of the PWM cycle
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_3,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);
	    timer_automatic_output_disable(TIMER0);
	    /* automatic output enable, break, dead time and lock configuration*/
	    timer_breakpara.runoffstate      = TIMER_ROS_STATE_DISABLE;
	    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_DISABLE ;
	    timer_breakpara.deadtime         = 32;
	    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_HIGH;
	    timer_breakpara.outputautostate  = TIMER_OUTAUTO_DISABLE;
	    timer_breakpara.protectmode      = TIMER_CCHP_PROT_0;
	    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
	    timer_break_config(TIMER0,&timer_breakpara);

	    timer_primary_output_config(TIMER0,DISABLE);

	    /* auto-reload preload disable */
	    timer_auto_reload_shadow_disable(TIMER0);
	    timer_enable(TIMER0);

}

void timer1_config(void) //running at 6kHz interrupt frequency
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 2;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1, &timer_initpara);

    /* CH0 configuration in PWM mode0 */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 2000);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);



    /* TIMER0 primary output enable */
    timer_primary_output_config(TIMER1, ENABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);
        /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER1,TIMER_INT_FLAG_UP);

    /* enable TIMER0 */
    timer_enable(TIMER1);
}


void Encoder_Init(void)//encoder A+B processing
{
    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;

    /* enable the key clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_AF);

    gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8);


    timer_deinit(TIMER2);

    /* TIMER configuration */
    timer_initpara.prescaler = 1 - 1;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 819;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2, &timer_initpara);

    /* TIMER3 CH0,1 input capture configuration */
    timer_icinitpara.icpolarity = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter = 0xFF;  // 0x05

    timer_input_capture_config(TIMER2, TIMER_CH_0, &timer_icinitpara);
    timer_input_capture_config(TIMER2, TIMER_CH_1, &timer_icinitpara);
    timer_input_capture_config(TIMER2, TIMER_CH_2, &timer_icinitpara);

    /* TIMER_ENCODER_MODE2 */
    timer_quadrature_decoder_mode_config(TIMER2, TIMER_ENCODER_MODE2, TIMER_IC_POLARITY_RISING,
                                         TIMER_IC_POLARITY_FALLING);
    timer_slave_mode_select(TIMER2, TIMER_ENCODER_MODE2);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);

//    timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH2);
//    timer_interrupt_enable(TIMER2, TIMER_INT_CH2);

    timer_enable(TIMER2);
}

void timer3_config(void) //encoder Z-signal
{


    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);


    /* TIMER2 configuration: input capture mode -------------------
    the external signal is connected to TIMER2 CH0 pin (PB4)
    the rising edge is used as active edge
    the TIMER2 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    timer_deinit(TIMER3);

    /* TIMER2 configuration */
    timer_initpara.prescaler         = 256;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 65535;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3,&timer_initpara);

    /* TIMER2  configuration */
    /* TIMER2 CH0 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0F;
    timer_input_capture_config(TIMER3,TIMER_CH_2,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH2);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER3,TIMER_INT_CH2);

    /* TIMER2 slow_loop_counter enable */
    timer_enable(TIMER3);

}
void timer4_config(void) //PWM input from Encoder
{
 /* TIMER2 configuration: PWM input mode ------------------------
     the external signal is connected to TIMER2 CH0 pin
     the rising edge is used as active edge
     the TIMER2 CH0CV is used to compute the frequency value
     the TIMER2 CH1CV is used to compute the duty cycle value
  ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    rcu_periph_clock_enable(RCU_TIMER4);

    timer_deinit(TIMER4);

    /* TIMER2 configuration */
    timer_initpara.prescaler         = 8;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 65535;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER4,&timer_initpara);

    /* TIMER2 configuration */
    /* TIMER2 CH0 PWM input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x05;
    timer_input_pwm_capture_config(TIMER4,TIMER_CH_1,&timer_icinitpara);

    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER4,TIMER_SMCFG_TRGSEL_CI0FE0);
    timer_slave_mode_select(TIMER4,TIMER_SLAVE_MODE_RESTART);

    /* select the master slave mode */
    timer_master_slave_mode_config(TIMER4,TIMER_MASTER_SLAVE_MODE_ENABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER4);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER4,TIMER_INT_FLAG_CH1);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER4,TIMER_INT_CH1);

    /* TIMER2 slow_loop_counter enable */
    timer_enable(TIMER4);
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{




	/* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX1_IRQn,0,0);
    nvic_irq_enable(EXTI10_15_IRQn, 2U, 0U); //for PAS
    nvic_irq_enable(EXTI2_IRQn, 2U, 0U); //for external speed sensor

    //timer2 interrupt for Halls
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 0, 0);
    //nvic_irq_enable(TIMER2_IRQn, 0, 0);
    nvic_irq_enable(ADC0_1_IRQn, 0, 0);
    nvic_irq_enable(TIMER3_IRQn, 0, 0);
    nvic_irq_enable(TIMER4_IRQn, 0, 0);

}

void UART4_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC); //for UART4 Tx on PC12
    //rcu_periph_clock_enable(RCU_GPIOD); //for UART4 Rx on PD2

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_UART4);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    /* connect port to USARTx_Rx */
   // gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    /* USART configure */
    usart_deinit(UART4);
    usart_baudrate_set(UART4, 115200U);
    //usart_receive_config(UART4, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);
    usart_enable(UART4);

    printf("\n\ra Bafang Debug on UART4!\n\r");

}



void TIMER1_IRQHandler(void) // regular ADC processing and common slow timing tasks
{
	fwdgt_counter_reload();
	if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_UP)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);

        reg_ADC_flag=1;
    }
}

void TIMER3_IRQHandler(void) //Z-signal processing
{
    if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_CH2)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH2);
        if(ui_8_PWM_ON_Flag){
			if((int16_t)TIMER_CNT(TIMER2)>0)i8_recent_rotor_direction=1;
			else i8_recent_rotor_direction=-1;
        }
        p++;
        if(MS.hall_angle_detect_flag)TIMER_CNT(TIMER2) = (uint16_t)Z_position; //reset encoder slow_loop_counter at full rotation
        else{

        	//Z_position_cumulated+=q31_rotorposition_absolute/5237765; //Caclulate the timer2 value for the z-Position 5244160=2^32/820
        	//Z_position=Z_position_cumulated/p;
        	Z_position=q31_rotorposition_absolute/5237765;
        }
        TIMER_CNT(TIMER3) = 0;
        ui16_erps=4000*5/(ui16_erps_counter);//4kHz counterfrequency, 5 polepairs, only one interrupt per mechanical revolution
        ui16_erps_counter=0;
        uint16_full_rotation_counter=0;
        uint16_half_rotation_counter=0;
        i32_full_rotation_flag=1;
    }

}

void TIMER4_IRQHandler(void) // PWM position reading from Encoder
{
    if(SET == timer_interrupt_flag_get(TIMER4,TIMER_INT_FLAG_CH1)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER4,TIMER_INT_FLAG_CH1);
        /* read channel 0 capture value */
        ic1value = timer_channel_capture_value_register_read(TIMER4,TIMER_CH_1)+1;

        if(0 != ic1value){
            /* read channel 1 capture value */
        	Encoder_raw_angle=((timer_channel_capture_value_register_read(TIMER4,TIMER_CH_0)+1)%2643)*1625035;//%2643 is for 5pole pairs, is factor from timer4 tic to 32bit angle
            AngleFromPWM = Encoder_raw_angle+PWM_offset;

            /* calculate the duty cycle value */
            dutycycle = (AngleFromPWM * 100) / ic1value;
            /* calculate the frequency value */
            frequency = 1000000 / ic1value;
            TIMER_CNT(TIMER4) = 0;
        }else{
            dutycycle = 0;
            frequency = 0;
        }
    }
}

void EXTI10_15_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(EXTI_11)) {
    	PAS_flag = 1;
        exti_interrupt_flag_clear(EXTI_11);
    }
}

void EXTI2_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(EXTI_2)) {
    	Speed_flag = 1;
        exti_interrupt_flag_clear(EXTI_2);
    }
}

void PAS_processing(void)
{
	if(PAS_counter>70){
		MS.cadence=10000/PAS_counter;//24 Pulses per crank revolution, 4000 Hz Timer interrupt frequency (for M560 about 48 pulses on speed/direction pin)(4000*60/24)=10000
		uint16_cadence_filtered-=uint16_cadence_filtered>>3;
		uint16_cadence_filtered+=MS.cadence;


		PAS_flag = 0;
		if(gpio_input_bit_get(GPIOC,GPIO_PIN_10)){
			if(Backwards_counter)Backwards_counter--;
			//PAS_counter=0;
		}
		else{
			if(Backwards_counter<10)Backwards_counter++;
		}
		torque_cumulated-=torque_cumulated>>MS.TQfilter;
		if(MS.torque_on_crank>tq_threshold){
			torque_cumulated+=(MS.torque_on_crank-tq_threshold);
		}
		//Power=2*Pi*speed*torque, calibration factors: rpm to 1/s for cadence: /60, mV to Nm: 750 to 3200 --> 0 to 80 Nm. (from Bafang data sheet)
		MS.torque_filtered=(torque_cumulated>>MS.TQfilter);
		MS.p_human=(uint16_t)((float)(MS.cadence*MS.torque_filtered)*0.00342); //in Watt

		PAS_counter=0;
	}
}

void Speed_processing(void)
{
	if(Speed_counter>(300/MP.pulses_per_revolution)){ //debouncing, max speed that can be detected about 100kph
		Speedx100_cumulated-=Speedx100_cumulated/MP.pulses_per_revolution;
		Speedx100_cumulated+=MP.wheel_cirumference*4*360/(MP.pulses_per_revolution*Speed_counter);// 4000 Hz Timer interrupt frequency
		MS.Speedx100=Speedx100_cumulated/MP.pulses_per_revolution;
		Speed_counter=0;
		Speed_flag=0;
		MS.distance_since_startup+=MP.wheel_cirumference/(MP.pulses_per_revolution*1000); //in m
	}
}

void reg_ADC_processing(void)
{
	battery_current_cumulated-=battery_current_cumulated>>6;
	battery_current_cumulated+= (adc_value[0]-CAL_BAT_I_OFFSET);
	MS.Battery_Current=(int32_t)((float)(battery_current_cumulated>>6)*CAL_BAT_I); //Battery current in mA
	voltage_raw_cumulated-=voltage_raw_cumulated>>6;
	voltage_raw_cumulated+=adc_value[3];
	voltage_raw_filtered=voltage_raw_cumulated>>6;

	MS.Voltage=voltage_raw_filtered*CAL_BAT_V;//Battery voltage in mV
	MS.calories=MS.torque_on_crank;

	MS.torque_on_crank=(((adc_value[2])*3300)>>12)+torque_offset_correction; //map ADC value to mV
	if(MS.torque_on_crank>tq_threshold&&PAS_counter<MP.PAS_timeout)torque_counter=0;//reset counter, if pressure on pedal and pedals rotating
	MS.range=Overrun_flag*100;//on/off button line
    slow_loop_counter ++;
    if(torque_counter<64000)torque_counter++;
    if(PAS_counter<64000)PAS_counter++;
    if(Speed_counter<64000)Speed_counter++;
    if(uint16_half_rotation_counter<64000)uint16_half_rotation_counter++;
    if(offroadcounter<64000)offroadcounter++;
    if(ui16_erps_counter<64000)ui16_erps_counter++;
    if(Overrun_counter<64000)Overrun_counter++;
    MS.i_q_setpoint = update_setpoint();
    if (torque_counter>4000&&!Overrun_flag){ //reset after one second without torque on the pedal
		Backwards_counter=0;
		MS.cadence=0;
		MS.p_human=0;
		uint16_cadence_filtered=0;
		torque_cumulated=0;
		if (!MS.i_q_setpoint){//reset integral part, if no power from throttle signal is wanted
			PI_iq.integral_part=0;
			PI_id.integral_part=0;
		}
    }

	reg_ADC_flag=0;
}

int16_t internal_tics_to_speedx100 (uint32_t tics){
	return WHEEL_CIRCUMFERENCE*50*3600/(6*GEAR_RATIO*tics);
}

int16_t external_tics_to_speedx100 (uint32_t tics){
	return MP.wheel_cirumference*4*360/(MP.pulses_per_revolution*tics);
}

int32_t speed_PLL (int32_t ist, int32_t soll, uint8_t speedadapt)
  {
    int32_t q31_p;
    static int32_t q31_d_i = 0;
    static int32_t q31_d_dc = 0;
    //temp6 = soll-ist;
  //  temp5 = speedadapt;
    q31_p=(soll - ist)>>(P_FACTOR_PLL-speedadapt);   				//7 for Shengyi middrive, 10 for BionX IGH3
    q31_d_i+=(soll - ist)>>(I_FACTOR_PLL-speedadapt);				//11 for Shengyi middrive, 10 for BionX IGH3

    //clamp i part to twice the theoretical value from hall interrupts
    if (q31_d_i>((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i = ((deg_30>>18)*500/ui16_timertics)<<16;
    if (q31_d_i<-((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i =- ((deg_30>>18)*500/ui16_timertics)<<16;


    if (!ist&&!soll)q31_d_i=0;

    q31_d_dc=q31_p+q31_d_i;
    return (q31_d_dc);
  }

void runPIcontrol(void){

	//check, if Battery Current limit is exceeded
	if(MS.Battery_Current>MP.battery_current_max) BC_limit_flag=1;
	//check, if theoretical Battery current would be below limit with some hysteresis
	if((MS.i_q_setpoint*CAL_I*MS.u_abs)>>11<(MP.battery_current_max*0.9)) BC_limit_flag=0; //duty cycle is scaled to 2048 = 2^11

	if(!BC_limit_flag){
	//control iq
	  PI_iq.recent_value = MS.i_q;
	  PI_iq.setpoint = MP.reverse*i8_reverse_flag*MS.i_q_setpoint;

	}
	else{
	 //control Battery_Current
	  PI_iq.recent_value = MP.reverse*i8_reverse_flag*MS.Battery_Current>>6;
	  PI_iq.setpoint = MP.reverse*i8_reverse_flag*(MP.battery_current_max>>6);

	}
	q31_u_q_temp =  PI_control(&PI_iq);
	//control id
	  PI_id.recent_value = MS.i_d;
	  PI_id.setpoint = MS.i_d_setpoint;
	  q31_u_d_temp = -PI_control(&PI_id); //control direct current to zero

	  //circle limitation

	  MS.u_abs = (int32_t)sqrtf((float)(q31_u_d_temp*q31_u_d_temp+q31_u_q_temp*q31_u_q_temp));
//	  arm_sqrt_q31((q31_u_d_temp*q31_u_d_temp+q31_u_q_temp*q31_u_q_temp)<<1,&MS.u_abs);
//	  MS.u_abs = (MS.u_abs>>16)+1;

	  if (MS.u_abs > _U_MAX){
			MS.u_q = (q31_u_q_temp*_U_MAX)/MS.u_abs; //division!
			MS.u_d = (q31_u_d_temp*_U_MAX)/MS.u_abs; //division!
			MS.u_abs = _U_MAX;
		}
	  else{
			MS.u_q=q31_u_q_temp;
			MS.u_d=q31_u_d_temp;
		}
	  PI_flag=0;

}

void autodetect(void) {
	timer_primary_output_config(TIMER0,ENABLE);
	ui_8_PWM_ON_Flag=1;
	MS.hall_angle_detect_flag = 0; //set uq to contstant value in FOC.c for open loop control
	q31_rotorposition_absolute = 1 << 31;
	i32_full_rotation_flag = 0;
	Z_position_cumulated=0;
	Z_position=0;
	PWM_offset_cumulated = 0;
	PWM_offset = 0;
	p=0;
	
	MS.i_d_setpoint= 200; //set MS.id to appr. 2000mA
	MS.i_q_setpoint= 0;
	delay_1ms(100);
    TIMER_CNT(TIMER2) = 0; //reset counter for Encoder A/B quadrature signal
    TIMER_CNT(TIMER3) = 0; //reset counter for Encoder Z signal for full rotation detection

	

	for (int i = 0; i < 5400; i++) { //15 x 360° to get at least 3 z-pulses
		q31_rotorposition_absolute += 11930465; //drive motor in open loop with steps of 1 deg
		delay_1ms(1);
		if(!(i%360)){
			TIMER_CNT(TIMER2)=0; //reset counter @ position 180°
		}

		if(!(i%60)) { //print debug data every electrical 60°
			PWM_offset_cumulated-=PWM_offset_cumulated>>2;
			PWM_offset_cumulated+=q31_rotorposition_absolute-Encoder_raw_angle;
			PWM_offset=PWM_offset_cumulated>>2;
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = 0x00010203; //ID for debug message
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 8;
            transmit_message.tx_data[0] = (((int16_t) ((float)q31_rotorposition_absolute*u32_to_deg))>>8)&0xFF;//scale q31 angle to -90 .. +90 for 1 Byte representation
            transmit_message.tx_data[1] = (int16_t) ((float)q31_rotorposition_absolute*u32_to_deg)&0xFF;
            transmit_message.tx_data[2] = (int8_t) (p>>8)&0xFF;//scale q31 angle to -90 .. +90 for 1 Byte representation
            transmit_message.tx_data[3] = (int8_t) (p)&0xFF;
            transmit_message.tx_data[4] = (int8_t) (Z_position>>8)&0xFF;
            transmit_message.tx_data[5] = (int8_t) (Z_position)&0xFF;
            transmit_message.tx_data[6] = (((int16_t)((float)PWM_offset*u32_to_deg))>>8)&0xFF;
            transmit_message.tx_data[7] = (int16_t)(((float)PWM_offset*u32_to_deg))&0xFF;

            /* transmit message */
            transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
            /* waiting for transmit completed */
            timeout = 0xFFFF;
            while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
                timeout--;
            	}
			
		}
	}

	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);
	delay_1ms(25);
	timer_primary_output_config(TIMER0,DISABLE); //Disable PWM if motor is not turning

	//ui_8_PWM_ON_Flag=0;

    MS.i_d = 0;
    MS.i_q = 0;
    MS.u_d=0;
    MS.u_q=0;
    MS.i_d_setpoint= 0;
    temp6=0;//reset Temp6 for second part of calibration
    uint32_tics_filtered=1000000;

    if(Z_position<0)Z_position+=820;
	if (i8_recent_rotor_direction == 1) {

		i32_full_rotation_flag = 1;
	} else {

		i32_full_rotation_flag = -1;
	}



	MS.hall_angle_detect_flag = 2;

	delay_1ms(20);
   // ui8_KV_detect_flag = 30;


}



void ADC0_1_IRQHandler(void)
{
    /* clear the ADC flag */
	fwdgt_counter_reload();
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);
    /* read ADC inserted group data register */

    //gpio_bit_write(GPIOB, GPIO_PIN_0,1);
    __disable_irq();
    i16_ph1_current = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0);
    i16_ph2_current = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);
    i16_ph3_current = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
	switch (MS.char_dyn_adc_state) //read in according to state
		{
		case 1: //Phase C at high dutycycles, read from A+B directly
			{
				//first reading is correct, do nothing
			}
			break;
		case 2: //Phase A at high dutycycles, read from B+C (A = -B -C)
			{

				//overwrite A with -B-C
				i16_ph1_current = -i16_ph2_current-i16_ph3_current;

			}
			break;
		case 3: //Phase B at high dutycycles, read from A+C (B=-A-C)
			{

				//overwrite B with -A-C
				i16_ph2_current = -i16_ph1_current-i16_ph3_current;
			}
			break;

		case 0: //timeslot too small for ADC
			{
				//do nothing
			}
			break;




		} // end case

	if(MS.hall_angle_detect_flag){
		if(i8_recent_rotor_direction)q31_rotorposition_absolute=(int32_t)(TIMER_CNT(TIMER2)*5237765); //=2^32/820, 4096 pulses per mechanical revolution, 5 pole pairs
		else q31_rotorposition_absolute=AngleFromPWM;
	}
	//get the Phase with highest duty cycle for dynamic phase current reading
	dyn_adc_state(q31_rotorposition_absolute);

    //q31_rotorposition_absolute=(int16_t)((180.0/75.0)*(float)(1<<31));
    if(ui_8_PWM_ON_Flag){
		FOC_calculation(i16_ph1_current, i16_ph2_current,
					q31_rotorposition_absolute,
					(((int16_t) MP.reverse * i8_reverse_flag)
							* MS.i_q_setpoint), &MS, &MP);

		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,switchtime[0]);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,switchtime[1]);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,switchtime[2]);

    }
    __enable_irq();
    //gpio_bit_write(GPIOB, GPIO_PIN_0,0);
}

int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  // if input is smaller/bigger than expected return the min/max out ranges value
  if (x < in_min)
    return out_min;
  else if (x > in_max)
    return out_max;

  // map the input to the output range.
  // round up if mapping bigger ranges to smaller ranges
  else  if ((in_max - in_min) > (out_max - out_min))
    return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
  // round down if mapping smaller ranges to bigger ranges
  else
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int32_t map_exp (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  // if input is smaller/bigger than expected return the min/max out ranges value
  if (x < in_min)
    return out_min;
  else if (x > in_max)
    return out_max;

  else{
	  float mapped_exp = (float)(x-in_min)/(float)(in_max-in_min);
	  mapped_exp= powf(mapped_exp,helper_throttle)*out_max;
	  return (int32_t)mapped_exp;
  }
}


//assuming, a proper AD conversion takes 350 timer tics, to be confirmed. DT+TR+TS deadtime + noise subsiding + sample time
void dyn_adc_state(q31_t angle){
	if (switchtime[2]>switchtime[0] && switchtime[2]>switchtime[1]){
		MS.char_dyn_adc_state = 1; // -90Ã‚Â° .. +30Ã‚Â°: Phase C at high dutycycles
		if(switchtime[2]>DYNAMIC_ADC_THRESHOLD)timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,(switchtime[2]-TRIGGER_OFFSET_ADC));
		else timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,TRIGGER_DEFAULT);
	}

	if (switchtime[0]>switchtime[1] && switchtime[0]>switchtime[2]) {
		MS.char_dyn_adc_state = 2; // +30Ã‚Â° .. 150Ã‚Â° Phase A at high dutycycles
		if(switchtime[0]>DYNAMIC_ADC_THRESHOLD)timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,(switchtime[0]-TRIGGER_OFFSET_ADC));
		else timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,TRIGGER_DEFAULT);
	}

	if (switchtime[1]>switchtime[0] && switchtime[1]>switchtime[2]){
		MS.char_dyn_adc_state = 3; // +150 .. -90Ã‚Â° Phase B at high dutycycles
		if(switchtime[1]>DYNAMIC_ADC_THRESHOLD)timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,(switchtime[1]-TRIGGER_OFFSET_ADC));
		else timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,TRIGGER_DEFAULT);
	}
}

uint8_t interpolate_assistfactor(void){
	uint16_t interval= speedlimitx100_scaled/5 ;
	uint8_t ui8_speedfactor=0;
	uint8_t ui8_speedcase=0;
	if (MS.Speedx100 < interval)ui8_speedcase=0;
	else if (MS.Speedx100 < 2*interval)ui8_speedcase=1;
	else if (MS.Speedx100 < 3*interval)ui8_speedcase=2;
	else if (MS.Speedx100 < 4*interval)ui8_speedcase=3;
	else ui8_speedcase=4;

	ui8_speedfactor = map(
			MS.Speedx100,
			ui8_speedcase*interval,
			(ui8_speedcase+1)*interval,
			MP.assist_profile[level_to_array_element[MS.assist_level]-1][ui8_speedcase],
			MP.assist_profile[level_to_array_element[MS.assist_level]-1][ui8_speedcase+1]);
	return ui8_speedfactor;
}

void print_debug_on_CAN(void){


	transmit_message.tx_sfid = 0x00;
	transmit_message.tx_efid = 0x00010203; //ID for debug message
	transmit_message.tx_ft = CAN_FT_DATA;
	transmit_message.tx_ff = CAN_FF_EXTENDED;
	transmit_message.tx_dlen = 8;
	transmit_message.tx_data[0] = (MS.cadence>>8)&0xFF;//(GPIO_ISTAT(GPIOC)>>6)&0x07;
	transmit_message.tx_data[1] = (MS.cadence)&0xFF; //ui16_timertics>>8;//(GPIO_ISTAT(GPIOA)>>8)&0xFF;
	transmit_message.tx_data[2] = (MS.torque_on_crank>>8)&0xFF;;
	transmit_message.tx_data[3] = (MS.torque_on_crank)&0xFF;
	transmit_message.tx_data[4] = (iabs(MS.i_q_setpoint)>>8)&0xFF;//
	transmit_message.tx_data[5] = (iabs(MS.i_q_setpoint))&0xFF;
	transmit_message.tx_data[6] = (iabs(MS.i_q)>>8)&0xFF;
	transmit_message.tx_data[7] = (iabs(MS.i_q))&0xFF;

	/* transmit message */
	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	/* waiting for transmit completed */
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

}

int16_t T_NTC(uint16_t ADC) // ADC 12 Bit, 10k NTC, RÃ¼ckgabewert in Â°C

{
	int R = R_TEMP_PULLUP;                                    // Spannungsteiler, fester Widerstand
	float Rn = 20000;                                         // gemessen (Ohm)
	float Tn = 23;                                            // gemessen (°C)
	float B = 3398;
    float U_ntc = (3.3 * ADC) / 4095;             // Spannung
    float R_ntc = (U_ntc * R) / (3.3 - U_ntc);           // Widerstand
                                                          // Temperatur-Berechnung
        // Rt = Rn * e hoch B*(1/T - 1/Tn)                // Ausgangsformel
        // T = 1 / [(log(Rt/Rn)/B + 1/Tn] - 273,15        // umgestellt in °K
        // Tnk = 26,2 + 273,15                            // °K
    float A1 = log(R_ntc / Rn) / B;
    float A2 = A1 + 1 / (Tn + 273.15);
    float T = (1 / A2) - 273.15;
	return (int)T; // Rundung

}

int8_t calculate_SOC(uint16_t voltage, uint8_t cells_in_series){ //interpolate from lookup table
    float voltages[] = {2.9, 3.15, 3.30, 3.42, 3.55, 3.60, 3.65, 3.70, 3.75, 3.80, 3.85, 3.90, 4.00, 4.10, 4.20};
    float soc_values[] = {0, 12, 22, 32, 44, 48, 55, 60, 68, 72, 77, 88, 95, 97, 100};
    int length = sizeof(voltages) / sizeof(voltages[0]);
    float cell_voltage = (float)voltage/((float)cells_in_series*1000);
    if (cell_voltage <= voltages[0]) {
        return (int8_t)soc_values[0];
    }
    if (cell_voltage >= voltages[length - 1]) {
        return (int8_t)soc_values[length - 1];
    }

    for (int i = 0; i < length - 1; i++) {
        if (cell_voltage < voltages[i+1]) {
            float slope = (soc_values[i+1] - soc_values[i]) / (voltages[i+1] - voltages[i]);
            float soc = soc_values[i] + slope * (cell_voltage - voltages[i]);
            return (int8_t)soc;
        }
    }
    return (int8_t)soc_values[length - 1];
}

/*!
    \brief      erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_erase_pages(void)
{
    uint32_t EraseCounter;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < PageNum; EraseCounter++){
        fmc_page_erase(FMC_WRITE_START_ADDR + (FMC_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

/*!
    \brief      program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_program_hall_angles(void)
{
    /* unlock the flash program/erase controller */
    fmc_unlock();

    address = FMC_WRITE_START_ADDR;

    /* program flash */

        fmc_word_program(address, (uint32_t)i32_full_rotation_flag);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Z_position_cumulated);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Z_position);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_26);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Encoder_raw_angle);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)PWM_offset_cumulated);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)PWM_offset);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);


    /* lock the main FMC after the program operation */
    fmc_lock();
}

fmc_state_enum fmc_multi_word_program(uint32_t offset, uint8_t* data, uint8_t words)
{
	uint32_t temp=0;
	fmc_state_enum returnvalue;
	uint32_t target_address = FMC_WRITE_START_ADDR+offset;
    fmc_unlock();
            	for (k=0; k < words; k++){
            		memcpy(&temp, data+k*4,4);
            		returnvalue = fmc_word_program(target_address, (uint32_t)temp);
                    target_address += 4;
                    fmc_flag_clear(FMC_FLAG_BANK0_END);
                    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
                    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
            	}


     fmc_lock();
    return returnvalue;
}

void write_virtual_eeprom(void)
	{
		fmc_erase_pages();
		fmc_program_hall_angles();
		fmc_multi_word_program(FMC_OFFSET_MP, (uint8_t*)&MP, (sizeof(MP)+3)/4); //Did not know padding yet :-)
	}

void read_virtual_eeprom(void)
	{
    //read individual hall angles from virtual EEPROM
    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;
    if(0xFFFFFFFF != (*(ptrd+1))){
    	i32_full_rotation_flag=(int32_t)(*ptrd);
    	ptrd++;
    	Z_position_cumulated = (int32_t)(*ptrd);
    	ptrd++;
    	Z_position = (int32_t)(*ptrd);
    	ptrd++;
    	Hall_26 = (int32_t)(*ptrd);
    	ptrd++;
    	Encoder_raw_angle = (int32_t)(*ptrd);
    	ptrd++;
    	PWM_offset_cumulated = (int32_t)(*ptrd);
    	ptrd++;
    	PWM_offset = (int32_t)(*ptrd);
    	ptrd++;
    }

     memcpy(&MP,(uint32_t *)(FMC_WRITE_START_ADDR+FMC_OFFSET_MP),sizeof(MP));
	}

uint16_t map_rezi(int32_t actual_value, int32_t actual_time, int32_t timeout, int32_t decay_base){
    if(actual_time<timeout)return actual_value;
    else if(actual_time<4000) return (uint16_t)((float)actual_value/((1+(float)(actual_value*decay_base)/175000*(float)(actual_time-timeout))));
    else return 0;
}

void get_torque_correction(void){
	torque_offset_correction=0;
    for (int i = 0; i < 64; i++) {// get torquesensor offset
    	torque_offset_correction+=adc_value[2];
    	while(!reg_ADC_flag);
    	reg_ADC_flag=0;

    }
    torque_offset_correction=(torque_offset_correction>>6);
    torque_offset_correction=740-((torque_offset_correction*3300)>>12);
}

uint16_t update_setpoint(void){

				//calculate iq setpoint
	            //check brake with first priority
	            if(MS.brake_active_flag)MS.i_q_setpoint_temp=0;
	            // check push assist active
	            else if(MS.pushassist_flag)MS.i_q_setpoint_temp=100;
	            //calculate setpoint, if brake is not activated
	            else{
					mapped_throttle= map_exp(adc_value[1], MP.throttle_offset, MP.throttle_max, 0, phase_current_max_scaled);
					mapped_torque= map(MS.torque_on_crank, MP.TQO_threshold[level_to_array_element[MS.assist_level]], 3300, 0, phase_current_max_scaled);

					if(Backwards_counter<4){//normal ride mode, motor power only if pedals are not turned backwards
						MS.i_q_setpoint_temp=(uint32_t)((float) (MP.TS_coeff*powf((float)MS.cadence,helper_cadence))*(MS.torque_filtered)*0.0005*interpolate_assistfactor());//factor 0.0005 from various constants
						//limit setpoint to the max value according to the current setting.
						if(MS.i_q_setpoint_temp>phase_current_max_scaled)MS.i_q_setpoint_temp = phase_current_max_scaled;
						MS.i_q_setpoint_temp=map_rezi(MS.i_q_setpoint_temp, torque_counter, MP.PAS_timeout, MP.decay_base);


						//torque override
						if(mapped_torque>MS.i_q_setpoint_temp){
							if(mapped_torque>Overrun_strength){
								Overrun_strength=mapped_torque;
								Overrun_counter = 0;
							}
							MS.i_q_setpoint_temp=mapped_torque;
						}
					}
					else MS.i_q_setpoint_temp=0; //cut motor power on pedaling backwards
						//throttle override
					if(mapped_throttle>MS.i_q_setpoint_temp)MS.i_q_setpoint_temp=mapped_throttle;

					//apply Extended Boost
					if(Overrun_counter<(MP.Override_Duration*MS.ext_boost_duration)/100&&Backwards_counter<4){
						MS.i_q_setpoint_temp=(Overrun_strength*MS.ext_boost_strength)/100;
						if(MS.i_q_setpoint_temp>MP.phase_current_max)MS.i_q_setpoint_temp = MP.phase_current_max;
						Overrun_flag=1;
						torque_counter=MP.PAS_timeout+1;
					}
					else {
						Overrun_strength=0;
						Overrun_flag=0;
					}


	            }// else brake not active

	            //low battery ramp down with 3V above battery min voltage
	            MS.i_q_setpoint_temp=map(voltage_raw_filtered, MP.voltage_min,(MP.voltage_min+176),0,MS.i_q_setpoint_temp);

	            //motor temperature ramp down between 110 and 130°C
	            MS.i_q_setpoint_temp=map(MS.int_Temperature,110,130,MS.i_q_setpoint_temp,0);
	            if(MP.legalflag&&!MS.offroadflag){

					if((uint16_cadence_filtered>>3)>15){
						MS.i_q_setpoint_temp=map(MS.Speedx100, speedlimitx100_scaled,(speedlimitx100_scaled+200),MS.i_q_setpoint_temp,0);
					}
					else{ //limit to 6km/h if pedals are not turning
						MS.i_q_setpoint_temp=map(MS.Speedx100, 500,700,MS.i_q_setpoint_temp,0);
					}

				}//end legalflag
				if(MS.hall_angle_detect_flag>1){ // part 2 of positions calibration
					MS.i_q_setpoint_temp=200;
					temp6-=temp6>>4;
					temp6+=MS.u_d;
					if (p>30){
						p=0;
						if ((MP.reverse*temp6>>4)>-40)Z_position++;
						else if ((MP.reverse*temp6>>4)<-50)Z_position--;
						else {
							MS.i_q_setpoint_temp=0;
							timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);
							timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);
							timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);
							timer_primary_output_config(TIMER0,DISABLE); //Disable PWM if motor is not turning
							write_virtual_eeprom();
							MS.hall_angle_detect_flag=1;
						}
					}

				}

				//reset integral part to avoid undefined overrun
				if(!MS.i_q_setpoint_temp&&PI_iq.integral_part){
					PI_iq.integral_part=0;
					PI_id.integral_part=0;
				}

	    		return MS.i_q_setpoint_temp;

}


#ifdef GD_ECLIPSE_GCC
/* retarget the C library printf function to the USART, in Eclipse GCC environment */
int __io_putchar(int ch)
{
    usart_data_transmit(UART4, (uint8_t)ch);
    while(RESET == usart_flag_get(UART4, USART_FLAG_TBE));

    return ch;
}
#else
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(UART4, (uint8_t)ch);
    while(RESET == usart_flag_get(UART4, USART_FLAG_TBE));

    return ch;
}
#endif /* GD_ECLIPSE_GCC */

