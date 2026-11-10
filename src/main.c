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
uint16_t adc_value[8];

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
FlagStatus reg_ADC_flag=0;
FlagStatus OnOffButton_flag=0;

void nvic_config(void);
void led_config(void);
void gpio_config(void);
void rcu_config(void);
void dma_config(void);
void adc_config(void);
void timer0_config(void); //PWM for Mosfet driver
void timer1_config(void); //PWM for triggering regular ADC
void timer2_config(void); // Input capture for hall sensors
ErrStatus can_networking(void);
void can_networking_init(void);
int32_t speed_PLL (int32_t ist, int32_t soll, uint8_t speedadapt);
void runPIcontrol(void);
void autodetect(void);
int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
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
uint16_t counter=0;
uint16_t PAS_counter=0;
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
uint8_t ui8_overflow_flag=0;
uint8_t ui8_SPEED_control_flag=0;

int32_t q31_rotorposition_hall=0;
q31_t q31_rotorposition_absolute=0;
int8_t i8_recent_rotor_direction=1;

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
int32_t i32_hall_order =-1;
int32_t Hall_13 = 1825361405;
int32_t Hall_32 = -1789569490;
int32_t Hall_26 = -966367405;
int32_t Hall_64 = -322122295;
int32_t Hall_45 = 381775140;
int32_t Hall_51 = 1169185830;

int32_t q31_PLL_error=0;
int32_t q31_rotorposition_PLL=0;
uint8_t ui_8_PLL_counter=0;
uint8_t shutoffcounter=0;
uint8_t ui_8_PWM_ON_Flag=0;
int32_t q31_angle_per_tic=0;
//Rotor angle scaled from degree to q31 for arm_math. -180Ã‚Â°-->-2^31, 0Ã‚Â°-->0, +180Ã‚Â°-->+2^31
const int32_t deg_30 = 357913941;
uint16_t switchtime[3];
uint16_t ui16_erps=0;
uint32_t ui32_erps_cumulated=0;
uint16_t mapped_throttle=0;
char char_dyn_adc_state_old=1;
int16_t i16_ph1_current=0;
int16_t i16_ph2_current=0;
int16_t i16_ph3_current=0;

int8_t i8_reverse_flag = 1;
const q31_t tics_lower_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*SPEEDLIMIT*10); //tics=wheelcirc*timerfrequency/(no. of hallevents per rev*gear-ratio*speedlimit)*3600/1000000
const q31_t tics_higher_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*(SPEEDLIMIT+2)*10);
uint8_t i = 0;
uint32_t timeout = 0xFFFF;
uint8_t transmit_mailbox = 0;
int32_t battery_current_cumulated=0;
uint32_t torque_cumulated=0;
uint8_t array_temp[88];

uint8_t level_to_array_element[10]={0,0,1,0,2,0,3,0,4,5}; //map assist Level to array element




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

    nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0xA800);
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
    timer2_config(); //for hall sensor handling
    /* DMA configuration */
    dma_config();
    /* ADC configuration */
    adc_config();

    /* initialize CAN and CAN filter */
    can_networking_init();

    /* enable CAN receive FIFO0 not empty interrupt */
    receive_flag = RESET;

    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE1);
#ifdef PRINTDEBUG
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
	MS.assist_level=127;
	MS.regen_level=7;
	MS.i_q_setpoint = 0;
	MS.i_d_setpoint = 0;
	MS.angle_est=SPEED_PLL;
	MS.pushassist_flag=SET;
	MS.light_flag=SET;
	MS.button_up_flag=SET;
	MS.button_down_flag=SET;


	MP.pulses_per_revolution = PULSES_PER_REVOLUTION;
	MP.wheel_cirumference = WHEEL_CIRCUMFERENCE;
	MP.speedLimitx100=SPEEDLIMIT;
	MP.battery_current_max = BATTERYCURRENT_MAX;
	MP.phase_current_max = PH_CURRENT_MAX;
	MP.TS_coeff = TS_COEF;
	MP.reverse = REVERSE;


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

    while((adc_value[1])>3000){

    }
    //autodetect();

    while (1){
    	fwdgt_counter_reload();

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
    	if(receive_flag){
    		receive_flag = RESET;
    		processCAN_Rx(&MP, &MS);
    	}

#endif
    	if(PAS_flag)PAS_processing();
    	if(reg_ADC_flag)reg_ADC_processing();
    	if(PAS_counter>MP.PAS_timeout){
    		MS.cadence=0;
    		MS.torque_on_crank=750;
    		MS.p_human=0;
    	}
    	// update scaled current and speed
    	if(MS.assist_level!=assist_level_old){
    		speedlimitx100_scaled=MP.speedLimitx100*MP.assist_settings[level_to_array_element[MS.assist_level]][1]/100;
    		phase_current_max_scaled=MP.phase_current_max*MP.assist_settings[level_to_array_element[MS.assist_level]][0]/100;
    		assist_level_old=MS.assist_level;
    	}

            if (counter > 2000){ //slow loop every 500ms, Timer1 @4kHz interrupt frequency
            	gd_eval_led_toggle(LED2);
#ifdef PRINTDEBUG
            	//printf("%d, %d, %d, %d, %d\r\n",MS.Battery_Current,MS.i_q_setpoint,MP.reverse*MS.i_q,ui16_erps,temp2);
            	printf("%d, %d, %d, %d, %d\r\n",MS.Battery_Current,MS.i_q_setpoint,MP.reverse*MS.i_q,MS.p_human,MS.Speedx100);
#endif
            	//toggle speed pin
            	//gpio_bit_write(GPIOB, GPIO_PIN_0,(bit_status)(1-gpio_input_bit_get(GPIOB, GPIO_PIN_0)));
            	if(ui16_timertics<10000)MS.Speedx100=internal_tics_to_speedx100(uint32_tics_filtered>>3);
            	else MS.Speedx100=0;
				counter = 0;
				if((((adc_value[3]>>2)+1555)-adc_value[5])+100>300)shutoffcounter++;
				else shutoffcounter=0;
				if(shutoffcounter>5){
					timer_primary_output_config(TIMER0,DISABLE); //stop PWM output
				    GPIO_BC(GPIOB) = GPIO_PIN_5; // Display off
				    GPIO_BC(GPIOB) = GPIO_PIN_6; // DC/DC off


				}

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
				transmit_message.tx_data[0] = (MS.i_d>>8)&0xFF;//(GPIO_ISTAT(GPIOC)>>6)&0x07;
				transmit_message.tx_data[1] = (MS.i_d)&0xFF; //ui16_timertics>>8;//(GPIO_ISTAT(GPIOA)>>8)&0xFF;
				transmit_message.tx_data[2] = (MS.i_q>>8)&0xFF;;
				transmit_message.tx_data[3] = (MS.i_q)&0xFF;
				transmit_message.tx_data[4] = (MS.u_abs>>8)&0xFF;
				transmit_message.tx_data[5] = (MS.u_abs)&0xFF;
				transmit_message.tx_data[6] = (ui_8_PWM_ON_Flag)&0xFF; //(adc_value[1]>>8)&0xFF;
				transmit_message.tx_data[7] = (ui8_6step_flag)&0xFF;

				/* transmit message */
				transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
				/* waiting for transmit completed */
				timeout = 0xFFFF;
				while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
					timeout--;
					}

#endif
            }
            //calculate iq setpoint
            mapped_throttle= map(adc_value[1], THROTTLE_OFFSET, THROTTLE_MAX, 0, PH_CURRENT_MAX);

            //MS.Speedx100=250;

    		MS.i_q_setpoint_temp= MP.TS_coeff*MS.p_human*interpolate_assistfactor()/100;
    		//limit setpoint to the max value according to the current setting.


    		if(mapped_throttle>MS.i_q_setpoint_temp)MS.i_q_setpoint_temp=mapped_throttle;
    		if(MS.i_q_setpoint_temp>phase_current_max_scaled)MS.i_q_setpoint_temp = phase_current_max_scaled;
    		if(MP.legalflag){
				if(!MS.brake_active_flag){ //only ramp down if no regen active
					if(PAS_counter<MP.PAS_timeout){
						MS.i_q_setpoint_temp=map(MS.Speedx100, speedlimitx100_scaled,(speedlimitx100_scaled+200),MS.i_q_setpoint_temp,0);
					}
					else{ //limit to 6km/h if pedals are not turning
						MS.i_q_setpoint_temp=map(MS.Speedx100, 500,700,MS.i_q_setpoint_temp,0);
					}
				}
    		}
    		MS.i_q_setpoint_temp=map(MS.Battery_Current, MP.battery_current_max-500,MP.battery_current_max+500,MS.i_q_setpoint_temp,0);

    		MS.i_q_setpoint=MS.i_q_setpoint_temp;
            if(MS.i_q_setpoint){
            	if(!ui_8_PWM_ON_Flag){
            		get_standstill_position();
            		//=20000; //set interval between two hallevents to a large value
            		//uint32_tics_filtered=128000;
            		i8_recent_rotor_direction=MP.reverse*i8_reverse_flag;
            		timer_counter_value_config(TIMER2, 0);
					timer_primary_output_config(TIMER0,ENABLE);
					ui_8_PWM_ON_Flag=1;
            	}
            }
            else if(uint16_half_rotation_counter>4000) {
            	if(ui_8_PWM_ON_Flag){
					timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,0);
					timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,0);
					timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,0);
					timer_primary_output_config(TIMER0,DISABLE); //Disable PWM if motor is not turning
					ui_8_PWM_ON_Flag=0;

            	}
            	 //Disable PWM if motor is not turning
    			if(TIMER_CCHP(TIMER0)&(uint32_t)TIMER_CCHP_POEN){
    				timer_primary_output_config(TIMER0,DISABLE);
    				PI_id.integral_part=0;
    				PI_iq.integral_part=0;
    				temp1=0;

    			}
            }


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
    rcu_periph_clock_enable(RCU_GPIOD);
    /* configure CAN0 GPIO, CAN0_TX(PD1) and CAN0_RX(PD0) */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    /* config the GPIO as analog mode */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    //gpio_init(GPIOB, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    //PB6: switch for DC/DC
    //PB5: switch for BatteryPlus display supply
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6);
    GPIO_BC(GPIOB) = GPIO_PIN_4; //reset Pin4 from Bootloader
    GPIO_BOP(GPIOB) = GPIO_PIN_6; //DC/DC on
    GPIO_BOP(GPIOB) = GPIO_PIN_5; // Display on
    //PA15 Dual PAS input pin (green wire)
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_15);
    /* configure key EXTI line */
    exti_init(EXTI_15, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_15);

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
    dma_data_parameter.number       = 8;
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
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL,8);
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL,1);
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL,1);
    adc_channel_length_config(ADC2, ADC_INSERTED_CHANNEL,1);

    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5); // PA0 Battery Current
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_6, ADC_SAMPLETIME_239POINT5); // PA6 Throttle?
    adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_7, ADC_SAMPLETIME_239POINT5); // PA7 Torque
    adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_13, ADC_SAMPLETIME_239POINT5);// PC3 battery voltage
    adc_regular_channel_config(ADC0, 4, ADC_CHANNEL_1, ADC_SAMPLETIME_239POINT5); // shunt current unfiltered
    adc_regular_channel_config(ADC0, 5, ADC_CHANNEL_4, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 6, ADC_CHANNEL_7, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(ADC0, 7, ADC_CHANNEL_8, ADC_SAMPLETIME_239POINT5);

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
	    timer_breakpara.deadtime         = 16;
	    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_HIGH;
	    timer_breakpara.outputautostate  = TIMER_OUTAUTO_DISABLE;
	    timer_breakpara.protectmode      = TIMER_CCHP_PROT_0;
	    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
	    timer_break_config(TIMER0,&timer_breakpara);

	    timer_primary_output_config(TIMER0,ENABLE);

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

void timer2_config(void) //for hall sensor processing.
{

    /* TIMER2 configuration: input capture mode */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);


    timer_deinit(TIMER2);
    /* hall mode config */
    timer_hall_mode_config(TIMER2, TIMER_HALLINTERFACE_ENABLE);



    /* TIMER2 configuration */
    timer_initpara.prescaler         = 239; //120MHz/(239+1)=500kHz
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* TIMER2  configuration */
    /* TIMER2 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0xFF;
    timer_input_capture_config(TIMER2,TIMER_CH_0,&timer_icinitpara);

    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0xFF;
    timer_input_capture_config(TIMER2,TIMER_CH_1,&timer_icinitpara);

    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0xFF;
    timer_input_capture_config(TIMER2,TIMER_CH_2,&timer_icinitpara);

    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER2,TIMER_SMCFG_TRGSEL_CI0F_ED);
    timer_slave_mode_select(TIMER2,TIMER_SLAVE_MODE_RESTART);

    /* hall mode config */
    timer_hall_mode_config(TIMER2, TIMER_HALLINTERFACE_ENABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_CH0);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER2,TIMER_INT_CH0);

    /* TIMER2 counter enable */
    timer_enable(TIMER2);

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
    nvic_irq_enable(EXTI10_15_IRQn, 2U, 0U);


    //timer2 interrupt for Halls
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 0, 0);
    nvic_irq_enable(TIMER2_IRQn, 0, 0);
    nvic_irq_enable(ADC0_1_IRQn, 0, 0);

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

void TIMER2_IRQHandler(void)
{
	fwdgt_counter_reload();
    if(SET == timer_interrupt_flag_get(TIMER2,TIMER_INT_FLAG_CH0)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_CH0);

       // if(TIM2->CCR1>20)ui16_timertics = TIM2->CCR1; //debounce hall signals
            /* read channel 0 capture value */

        	ui16_timertics=timer_channel_capture_value_register_read(TIMER2,TIMER_CH_0);
        	ui32_erps_cumulated-=ui32_erps_cumulated>>5;
        	ui32_erps_cumulated+=500000/(ui16_timertics*6);
        	ui16_erps=ui32_erps_cumulated>>5;
                  	//Hall sensor event processing

            		ui8_hall_state = (GPIO_ISTAT(GPIOC)>>6)&0x07; //Mask input register with Hall 1 - 3 bits

            		ui8_hall_case=ui8_hall_state_old*10+ui8_hall_state;
            		statehistory[historycounter]=ui8_hall_case;
            		historycounter++;
            		if (historycounter>35)historycounter=0;

            		if(MS.hall_angle_detect_flag){ //only process, if autodetect procedere is fininshed
            		ui8_hall_state_old=ui8_hall_state;
            		}

            			uint32_tics_filtered-=uint32_tics_filtered>>3;
            			uint32_tics_filtered+=ui16_timertics;

            		   ui8_overflow_flag=0;
            		   ui8_SPEED_control_flag=1;



            		switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
            				{
            			//6 cases for forward direction
            		//6 cases for forward direction
            		case 64:
            			q31_rotorposition_hall = Hall_64;

            			i8_recent_rotor_direction = -i32_hall_order;
            			uint16_full_rotation_counter = 0;
            			break;
            		case 45:
            			q31_rotorposition_hall = Hall_45;

            			i8_recent_rotor_direction = -i32_hall_order;
            			break;
            		case 51:
            			q31_rotorposition_hall = Hall_51;

            			i8_recent_rotor_direction = -i32_hall_order;
            			break;
            		case 13:
            			q31_rotorposition_hall = Hall_13;

            			i8_recent_rotor_direction = -i32_hall_order;
            			uint16_half_rotation_counter = 0;
            			break;
            		case 32:
            			q31_rotorposition_hall = Hall_32;

            			i8_recent_rotor_direction = -i32_hall_order;
            			break;
            		case 26:
            			q31_rotorposition_hall = Hall_26;

            			i8_recent_rotor_direction = -i32_hall_order;
            			break;

            			//6 cases for reverse direction
            		case 46:
            			q31_rotorposition_hall = Hall_64;

            			i8_recent_rotor_direction = i32_hall_order;
            			break;
            		case 62:
            			q31_rotorposition_hall = Hall_26;

            			i8_recent_rotor_direction = i32_hall_order;
            			break;
            		case 23:
            			q31_rotorposition_hall = Hall_32;

            			i8_recent_rotor_direction = i32_hall_order;
            			uint16_half_rotation_counter = 0;
            			break;
            		case 31:
            			q31_rotorposition_hall = Hall_13;

            			i8_recent_rotor_direction = i32_hall_order;
            			break;
            		case 15:
            			q31_rotorposition_hall = Hall_51;

            			i8_recent_rotor_direction = i32_hall_order;
            			break;
            		case 54:
            			q31_rotorposition_hall = Hall_45;

            			i8_recent_rotor_direction = i32_hall_order;
            			uint16_full_rotation_counter = 0;
            			break;

            		} // end case

            		if(MS.angle_est){
            			q31_PLL_error=q31_rotorposition_PLL-q31_rotorposition_hall;
            			if(iabs(q31_PLL_error) < deg_30){
            				if(ui_8_PLL_counter<12)ui_8_PLL_counter++;
            			}
            			else ui_8_PLL_counter=0;
            			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall,0);
            		}

            	#ifdef SPEED_PLL
            		if(ui16_erps>30){   //360 interpolation at higher erps
            			if(ui8_hall_case==32||ui8_hall_case==23){
            				q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall, SPDSHFT*tics_higher_limit/(uint32_tics_filtered>>3));

            			}
            		}
            		else{

            			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall, SPDSHFT*tics_higher_limit/(uint32_tics_filtered>>3));
            		}

            	#endif


    }

}

void TIMER1_IRQHandler(void)
{
	fwdgt_counter_reload();
	if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_UP)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);


            /* read channel 0 capture value */
        counter ++;
        if(PAS_counter<64000)PAS_counter++;
        if(uint16_half_rotation_counter<64000)uint16_half_rotation_counter++;
        reg_ADC_flag=1;
    }
}

void EXTI10_15_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(EXTI_15)) {
    	PAS_flag = 1;
        exti_interrupt_flag_clear(EXTI_15);
    }
}

void PAS_processing(void)
{
		MS.cadence=7500/PAS_counter;//32 Pulses per crank revolution, 4000 Hz Timer interrupt frequency
		MS.torque_on_crank=(adc_value[2]*3300)>>12; //map ADC value to mV
		PAS_counter=0;
    	PAS_flag = 0;
    	if(MS.torque_on_crank>750){
    	torque_cumulated-=torque_cumulated>>5;
    	torque_cumulated+=(MS.torque_on_crank-750);
    	//Power=2*Pi*speed*torque, calibration factors: rpm to 1/s for cadence: /60, mV to Nm: 750 to 3200 --> 0 to 80 Nm. (from Bafang data sheet)
    	MS.p_human=(uint16_t)((float)(MS.cadence*(torque_cumulated>>5))*0.00342); //in Watt
    	}
    	else MS.p_human = 0;
}

void reg_ADC_processing(void)
{
	battery_current_cumulated-=battery_current_cumulated>>6;
	battery_current_cumulated+= (adc_value[0]-CAL_BAT_I_OFFSET);
	MS.Battery_Current=(int32_t)((float)(battery_current_cumulated>>6)*CAL_BAT_I); //Battery current in mA
	MS.Voltage=adc_value[3]*CAL_BAT_V;//Battery voltage in mV
	MS.calories=ui16_erps;//temp2;//(((adc_value[3]>>2)+1555)-adc_value[5])+100; temp2;// *CAL_I
	reg_ADC_flag=0;
}

int16_t internal_tics_to_speedx100 (uint32_t tics){
	return WHEEL_CIRCUMFERENCE*50*3600/(6*GEAR_RATIO*tics);
}

int16_t external_tics_to_speedx100 (uint32_t tics){
	return WHEEL_CIRCUMFERENCE*8*360/(MP.pulses_per_revolution*tics);
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
	//control iq
	  PI_iq.recent_value = MS.i_q;
	  PI_iq.setpoint = MP.reverse*i8_reverse_flag*MS.i_q_setpoint;
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
	i32_hall_order = 1;//reset hall order
	MS.i_d_setpoint= 200; //set MS.id to appr. 2000mA
	MS.i_q_setpoint= 0;

	for (int i = 0; i < 1080; i++) {
		q31_rotorposition_absolute += 11930465; //drive motor in open loop with steps of 1 deg
		delay_1ms(25);


		if (ui8_hall_state_old != ui8_hall_state) {
//			printf_("angle: %d, hallstate:  %d, hallcase %d \n",
//					(int16_t) (((q31_rotorposition_absolute >> 23) * 180) >> 8),
//					ui8_hall_state, ui8_hall_case);

			switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
			{
			//6 cases for forward direction
			case 64:
				Hall_64=q31_rotorposition_absolute;
				break;
			case 45:
				Hall_45=q31_rotorposition_absolute;
				break;
			case 51:
				Hall_51=q31_rotorposition_absolute;
				break;
			case 13:
				Hall_13=q31_rotorposition_absolute;
				break;
			case 32:
				Hall_32=q31_rotorposition_absolute;
				break;
			case 26:
				Hall_26=q31_rotorposition_absolute;
				break;

				//6 cases for reverse direction
			case 46:
				Hall_64=q31_rotorposition_absolute;
				break;
			case 62:
				Hall_26=q31_rotorposition_absolute;
				break;
			case 23:
				Hall_32=q31_rotorposition_absolute;
				break;
			case 31:
				Hall_13=q31_rotorposition_absolute;
				break;
			case 15:
				Hall_51=q31_rotorposition_absolute;
				break;
			case 54:
				Hall_45=q31_rotorposition_absolute;
				break;

			} // end case

            transmit_message.tx_data[0] = (int8_t) (((Hall_64 >> 23) * 180) >> 9);//scale q31 angle to -90 .. +90 for 1 Byte representation
            transmit_message.tx_data[1] = (int8_t) (((Hall_45 >> 23) * 180) >> 9);
            transmit_message.tx_data[2] = (int8_t) (((Hall_51 >> 23) * 180) >> 9);
            transmit_message.tx_data[3] = (int8_t) (((Hall_13 >> 23) * 180) >> 9);
            transmit_message.tx_data[4] = (int8_t) (((Hall_32 >> 23) * 180) >> 9);
            transmit_message.tx_data[5] = (int8_t) (((Hall_26 >> 23) * 180) >> 9);
            transmit_message.tx_data[6] = (adc_value[1]>>8)&0xFF;
            transmit_message.tx_data[7] = (adc_value[1])&0xFF;

            /* transmit message */
            transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
            /* waiting for transmit completed */
            timeout = 0xFFFF;
            while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
                timeout--;
            	}
			ui8_hall_state_old = ui8_hall_state;
		}
	}
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,0);
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,0);
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,0);
	delay_1ms(25);
	timer_primary_output_config(TIMER0,DISABLE); //Disable PWM if motor is not turning

	//ui_8_PWM_ON_Flag=0;

    MS.i_d = 0;
    MS.i_q = 0;
    MS.u_d=0;
    MS.u_q=0;
    MS.i_d_setpoint= 0;
    uint32_tics_filtered=1000000;


	if (i8_recent_rotor_direction == 1) {

		i32_hall_order = 1;
	} else {

		i32_hall_order = -1;
	}

	write_virtual_eeprom();

	MS.hall_angle_detect_flag = 1;

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

    //get the recent timer value from the Hall timer
    ui16_tim2_recent = timer_counter_read(TIMER2);
    if (ui16_tim2_recent>SIXSTEPTHRESHOLD<<1){
    	ui16_timertics=SIXSTEPTHRESHOLD<<1;
    	uint32_tics_filtered=ui16_timertics<<3;
    }
    //check the speed for sixstep threshold
	if (ui16_timertics < SIXSTEPTHRESHOLD && ui16_tim2_recent < 200)
		ui8_6step_flag = 0;
	if (ui16_timertics > (SIXSTEPTHRESHOLD * 6) >> 2)
		ui8_6step_flag = 1;

    // extrapolate rotorposition from filtered speed reading
    if(MS.hall_angle_detect_flag){//q31_rotorposition_absolute = q31_rotorposition_hall + (q31_t) ((float)(i8_recent_rotor_direction * (deg_30<<1) * ui16_tim2_recent)/(float)(uint32_tics_filtered>>3));//
//Speed PLL not implemented yet.
    	if(!ui8_6step_flag){
    	q31_rotorposition_absolute = q31_rotorposition_hall
    									+ (q31_t) (i8_recent_rotor_direction
    											* ((10923 * ui16_tim2_recent)
    													/ (uint32_tics_filtered>>3)) << 16);//interpolate angle between two hallevents by scaling timer2 tics, 10923<<16 is 715827883 = 60deg
    	}
    	else q31_rotorposition_absolute = q31_rotorposition_hall - MP.reverse * deg_30; //offset of 30 degree to get the middle of the sector

    }

	//get the Phase with highest duty cycle for dynamic phase current reading
	dyn_adc_state(q31_rotorposition_absolute);

    //q31_rotorposition_absolute=(int16_t)((180.0/75.0)*(float)(1<<31));
    if(ui_8_PWM_ON_Flag){
		FOC_calculation(i16_ph1_current, i16_ph2_current,
					q31_rotorposition_absolute,
					(((int16_t) MP.reverse * i8_reverse_flag)
							* MS.i_q_setpoint), &MS, &MP);
		if(switchtime[0]>switchtime[1])temp2=switchtime[0];
		else temp2=switchtime[1];
		if(temp2<switchtime[2])temp2=switchtime[2];
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,switchtime[0]);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,switchtime[1]);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,switchtime[2]);

//		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);
//		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,(_T>>1));
//		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,(_T>>1)-500);
		//timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,-MS.i_q_setpoint*2+1);

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

void get_standstill_position(){

	  delay_1ms(25);
	  ui8_hall_state = (GPIO_ISTAT(GPIOC)>>6)&0x07;
		switch (ui8_hall_state) {
			//6 cases for forward direction
			case 2:
				q31_rotorposition_hall = Hall_32;
				break;
			case 6:
				q31_rotorposition_hall = Hall_26;
				break;
			case 4:
				q31_rotorposition_hall = Hall_64;
				break;
			case 5:
				q31_rotorposition_hall = Hall_45;
				break;
			case 1:
				q31_rotorposition_hall = Hall_51;

				break;
			case 3:
				q31_rotorposition_hall = Hall_13;
				break;

			}

			q31_rotorposition_absolute = q31_rotorposition_hall;
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

        fmc_word_program(address, (uint32_t)i32_hall_order);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_13);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_32);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_26);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_64);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_45);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_51);
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
//		fmc_multi_word_program(FMC_OFFSET_PARA0, &Para0[0]);
//		fmc_multi_word_program(FMC_OFFSET_PARA1, &Para1[0]);
//		fmc_multi_word_program(FMC_OFFSET_PARA2, &Para2[0]);
		fmc_multi_word_program(FMC_OFFSET_MP, (uint8_t*)&MP, 22); //86byte in MP
	}

void read_virtual_eeprom(void)
	{
    //read individual hall angles from virtual EEPROM
    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;
    if(0xFFFFFFFF != (*(ptrd+1))){
    	i32_hall_order=(int32_t)(*ptrd);
    	ptrd++;
    	Hall_13 = (int32_t)(*ptrd);
    	ptrd++;
    	Hall_32 = (int32_t)(*ptrd);
    	ptrd++;
    	Hall_26 = (int32_t)(*ptrd);
    	ptrd++;
    	Hall_64 = (int32_t)(*ptrd);
    	ptrd++;
    	Hall_45 = (int32_t)(*ptrd);
    	ptrd++;
    	Hall_51 = (int32_t)(*ptrd);
    	ptrd++;
    }
    //read Para0 to Para2  from virtual EEPROM
//    memcpy(&Para0[0],(uint32_t *)(FMC_WRITE_START_ADDR+FMC_OFFSET_PARA0),64);
//    memcpy(&Para1[0],(uint32_t *)(FMC_WRITE_START_ADDR+FMC_OFFSET_PARA1),64);
//    memcpy(&Para2[0],(uint32_t *)(FMC_WRITE_START_ADDR+FMC_OFFSET_PARA2),64);

     memcpy(&MP,(uint32_t *)(FMC_WRITE_START_ADDR+FMC_OFFSET_MP),88);
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
