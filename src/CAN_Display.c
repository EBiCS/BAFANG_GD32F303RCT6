/*
 * CAN_Display.c
 *
 *  Created on: 19.10.2025
 *  Author: stancecoke
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "main.h"
#include "CAN_Display.h"
#include "parser.h"
#include "FOC.h"

Ext_ID_t Ext_ID_Rx;
Ext_ID_t Ext_ID_Tx;
void processCAN_Rx(MotorParams_t* MP, MotorState_t* MS);
void sendCAN_Tx(MotorParams_t* MP, MotorState_t* MS);
void sendCAN_Poll(MotorParams_t* MP, MotorState_t* MS, uint16_t command);
void sendAcknoledge(void);
void send_multiframe(uint16_t command, char* data, uint8_t length );
void append_multiframe(uint16_t command, char* data);
void update_checksum(void);
//int16_t abs(int16_t value);
char tx_data[64];
uint8_t Para0[64];
uint8_t Para1[64];
uint8_t Para2[64];
uint8_t tx_data_length;
uint8_t rx_data_length;
uint8_t nbrofframes;
float last_distance =0;
float last_kilometer =0;
float distance =0;
uint16_t display_distance=0;
uint16_t delay_counter =0;
uint16_t k=0;
uint8_t level_code;
uint8_t level_code_old;
uint8_t level_counter;

uint16_t Rx_MF_active=0;
uint16_t checksum=0;

void processCAN_Rx(MotorParams_t* MP, MotorState_t* MS){

	Ext_ID_Rx.command = (receive_message.rx_efid)&0xFFFF;
	Ext_ID_Rx.operation = (receive_message.rx_efid>>16)&0x07; //only 3 bit width
	Ext_ID_Rx.target = (receive_message.rx_efid>>19)&0x1F; //only 5 bit width
	Ext_ID_Rx.source = (receive_message.rx_efid>>24)&0x1F;

	if(Ext_ID_Rx.target==2){
		switch (Ext_ID_Rx.operation){
			case WRITE_CMD:

				if (receive_message.rx_dlen==1 && receive_message.rx_data[0]>8 && Ext_ID_Rx.source==5){
					Rx_MF_active=Ext_ID_Rx.command;
					rx_data_length=receive_message.rx_data[0];
				}
				else if(Ext_ID_Rx.command==0x62D9){ //Startup angle, used as multiplyer here
					MP->TS_coeff=receive_message.rx_data[0]+(receive_message.rx_data[1]<<8);
					//save received setting
					write_virtual_eeprom();
				}

				else sendCAN_Tx(MP,MS);
				sendAcknoledge();
				break;
			case READ_CMD:
				sendCAN_Tx(MP,MS);
				break;
			case LONG_START_CMD:
				switch (Rx_MF_active){
					case 0x6010: //Para0
						append_multiframe(0, &Para0[0]);
						break;
					case 0x6011: //Para1
						append_multiframe(0, &Para1[0]);
						break;
					case 0x6012: //Para2
						append_multiframe(0, &Para2[0]);
						break;
				}

				break;
			case LONG_TRANG_CMD:
				switch (Rx_MF_active){
					case 0x6010: //Para0
						append_multiframe(Ext_ID_Rx.command+1, &Para0[0]);
						break;
					case 0x6011: //Para1
						append_multiframe(Ext_ID_Rx.command+1, &Para1[0]);
						break;
					case 0x6012: //Para2
						append_multiframe(Ext_ID_Rx.command+1, &Para2[0]);
						break;
				}
				break;

			case LONG_END_CMD:
				switch (Rx_MF_active){
					case 0x6010: //Para0
						append_multiframe(Ext_ID_Rx.command+1, &Para0[0]);
						break;
					case 0x6011: //Para1
						append_multiframe(Ext_ID_Rx.command+1, &Para1[0]);

						break;
					case 0x6012: //Para2
						append_multiframe(Ext_ID_Rx.command+1, &Para2[0]);

						break;
				}
				k = ((Ext_ID_Rx.command+1)<<3)+receive_message.rx_dlen;
				if(((Ext_ID_Rx.command+1)<<3)+receive_message.rx_dlen==rx_data_length){
					//to do send acknoledge OK
					Rx_MF_active=0;
					rx_data_length=0;
					//save received setting
					parse_DPparams(MP);
					write_virtual_eeprom();

				}
				else{
					//to do send acknoledge NOK
					Rx_MF_active=0;
					rx_data_length=0;
				}

				break;
		}


		if(Ext_ID_Rx.command==0x6300){
			level_code=receive_message.rx_data[1];
			if(level_code==level_code_old&&level_counter<3)level_counter++;
			if(level_code!=level_code_old){
				level_counter=0;
				MS->level_counter_global++;
			}
			level_code_old=level_code;

			if(level_counter==3){

				level_counter=4;
				switch (level_code){
					case 0:
						MS->assist_level=0;
						break;
					case 1:
						MS->assist_level=1;
						break;
					case 0x0B:
						MS->assist_level=2; //Eco
						break;
					case 0x0C:
						MS->assist_level=3;
						break;
					case 0x0D:
						MS->assist_level=4; //Tour
						break;
					case 0x02:
						MS->assist_level=5;
						break;
					case 0x15:
						MS->assist_level=6;//Sport
						break;
					case 0x16:
						MS->assist_level=7;
						break;
					case 0x17:
						MS->assist_level=8; //Sport +
						break;
					case 0x03:
						MS->assist_level=9; //Boost
						break;
				}
			}
			if (receive_message.rx_data[1]==6)MS->pushassist_flag=SET;
			else MS->pushassist_flag=RESET;
			if (receive_message.rx_data[2]&0b1)MS->light_flag=SET;
			else MS->light_flag=RESET;
			if (receive_message.rx_data[2]&0b10)MS->button_up_flag=SET;
			else MS->button_up_flag=RESET;
			if (receive_message.rx_data[2]&0b100000)MS->button_down_flag=SET;
			else MS->button_down_flag=RESET;

		}

		if(Ext_ID_Rx.command==0x3203&&Ext_ID_Rx.operation==WRITE_CMD){ //speed limit and wheel size
			MP->speedLimitx100=receive_message.rx_data[0]+(receive_message.rx_data[1]<<8);
			MP->wheel_cirumference=receive_message.rx_data[4]+(receive_message.rx_data[5]<<8);
			//save received setting
			write_virtual_eeprom();

		}

		if(Ext_ID_Rx.command==0x6200){ //Position sensor calibration
			autodetect();
		}
		if(Ext_ID_Rx.command==0x6101){ //Torque sensor calibration normally, here used for factory settings reset
			InitEEPROM(MP);
			read_virtual_eeprom();
			parse_MOparams(MP);

		}



	}
	if(Ext_ID_Rx.command==0x3005){ //jump to bootloader for firmware update
		NVIC_SystemReset();
	}
}
void sendAcknoledge(void){
	Ext_ID_Tx.command = Ext_ID_Rx.command;
	Ext_ID_Tx.operation = 2; //acknoledge OK
	Ext_ID_Tx.target = 0x05; //BESST
	Ext_ID_Tx.source = 0x02; //controller
	transmit_message.tx_sfid = 0x00;
	transmit_message.tx_efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
	transmit_message.tx_ft = CAN_FT_DATA;
	transmit_message.tx_ff = CAN_FF_EXTENDED;
	transmit_message.tx_dlen = 0;

	/* transmit message */
	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	/* waiting for transmit completed */
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

}

void sendCAN_Poll(MotorParams_t* MP, MotorState_t* MS, uint16_t command){

	switch (command){

		case 0x3201: //speed and power
			/* initialize transmit message */
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = 0x02F83201;
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 8;
			if(MS->offroadtics){
				transmit_message.tx_data[0] = (MS->offroadtics*100)&0xFF;
				transmit_message.tx_data[1] = ((MS->offroadtics*100)>>8)&0xFF;
			}
			else{
				transmit_message.tx_data[0] = (MS->Speedx100)&0xFF;
				transmit_message.tx_data[1] = ((MS->Speedx100)>>8)&0xFF;
			}
			transmit_message.tx_data[2] = (abs(MS->Battery_Current)/10)&0xFF;
			transmit_message.tx_data[3] = ((abs(MS->Battery_Current)/10)>>8)&0xFF;
			transmit_message.tx_data[4] = (MS->Voltage/10)&0xFF;
			transmit_message.tx_data[5] = ((MS->Voltage/10)>>8)&0xFF;
			transmit_message.tx_data[6] = MS->u_abs/10+40; //internal temperature
			transmit_message.tx_data[7] = MS->int_Temperature+40; //motor temperature

			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}
			break;

		case 0x3200: //battery and distance
			/* initialize transmit message */
			if(delay_counter<10)delay_counter++;
			else{
				delay_counter=0;
				distance=(MS->distance_since_startup)-last_kilometer;
				if(MS->distance_since_startup!=last_distance)display_distance = distance/10;
				else{
					display_distance = 0;
					last_kilometer=MS->distance_since_startup;
				}
				if(distance>1000)last_kilometer=MS->distance_since_startup;
				last_distance=MS->distance_since_startup;
			}
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = 0x02F83200;
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 8;
			transmit_message.tx_data[0] = MS->SOC;//battery percentage
			transmit_message.tx_data[1] = (uint8_t)display_distance; // in 10m
			transmit_message.tx_data[2] = 0x00;
			transmit_message.tx_data[3] = MS->cadence; //cadence
			transmit_message.tx_data[4] = MS->torque_on_crank&0xFF; //torque mV LSB
			transmit_message.tx_data[5] = (MS->torque_on_crank>>8)&0xFF; //torque mv MSB
			transmit_message.tx_data[6] = MS->range&0xFF;//range LSB
			transmit_message.tx_data[7] = (MS->range>>8)&0xFF;//range MSB

			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}

			break;

		case 0x3205: //to do
			/* initialize transmit message */
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = 0x02F83205;
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 2;
			transmit_message.tx_data[0] = MS->calories&0xFF; //calories
			transmit_message.tx_data[1] = (MS->calories>>8)&0xFF;


			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}
			break;


	}//end case
}

void sendCAN_Tx(MotorParams_t* MP, MotorState_t* MS){

	switch (Ext_ID_Rx.command){


		case 0x6001: //to do
			/* initialize transmit message */

			Ext_ID_Tx.command = 0x6001;
			Ext_ID_Tx.operation = 0; //write
			Ext_ID_Tx.target = 0x05; //BESST
			Ext_ID_Tx.source = 0x02; //controller
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 8;
			transmit_message.tx_data[0] = (char)'E';
			transmit_message.tx_data[1] = (char)'B';
			transmit_message.tx_data[2] = (char)'i';
			transmit_message.tx_data[3] = (char)'C';
			transmit_message.tx_data[4] = (char)'S';
			transmit_message.tx_data[5] = (char)'v';
			transmit_message.tx_data[6] = (char)'B';
			transmit_message.tx_data[7] = (char)'1';


			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}
			break;

		case 0x6002: //to do
			/* initialize transmit message */

			Ext_ID_Tx.command = 0x6002;
			Ext_ID_Tx.operation = 0; //write
			Ext_ID_Tx.target = 0x05; //BESST
			Ext_ID_Tx.source = 0x02; //controller
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 8;
			transmit_message.tx_data[0] = (char)'C';
			transmit_message.tx_data[1] = (char)'R';
			transmit_message.tx_data[2] = (char)'A';
			transmit_message.tx_data[3] = (char)'1';
			transmit_message.tx_data[4] = (char)'0';
			transmit_message.tx_data[5] = (char)'1';
			transmit_message.tx_data[6] = (char)'.';
			transmit_message.tx_data[7] = (char)'C';


			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}
			break;

		case 0x62D9: //startup angle used as multiplyer here
			/* initialize transmit message */

			Ext_ID_Tx.command = 0x62D9;
			Ext_ID_Tx.operation = 0; //write
			Ext_ID_Tx.target = 0x05; //BESST
			Ext_ID_Tx.source = 0x02; //controller
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 2;
			transmit_message.tx_data[0] =  (MP->TS_coeff)&0xFF;
			transmit_message.tx_data[1] =  (MP->TS_coeff>>8)&0xFF;

			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}
			break;

		case 0x3203: //to do
			/* initialize transmit message */

			Ext_ID_Tx.command = 0x3203;
			Ext_ID_Tx.operation = 0; //write
			Ext_ID_Tx.target = 0x05; //BESST
			Ext_ID_Tx.source = 0x02; //controller
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 6;
			transmit_message.tx_data[0] = MP->speedLimitx100&0xFF;
			transmit_message.tx_data[1] = (MP->speedLimitx100>>8)&0xFF;
			transmit_message.tx_data[2] = (char)'A';
			transmit_message.tx_data[3] = (char)'1';
			transmit_message.tx_data[4] = MP->wheel_cirumference&0xFF;
			transmit_message.tx_data[5] = (MP->wheel_cirumference>>8)&0xFF;

			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}
			break;

		case 0x6200: //to do
			/* initialize transmit message */

			Ext_ID_Tx.command = 0x6200;
			Ext_ID_Tx.operation = NORMAL_ACK; //write
			Ext_ID_Tx.target = 0x05; //BESST
			Ext_ID_Tx.source = 0x02; //controller
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 0;
			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}
			break;

		case 0x6003: //to do
			/* initialize transmit message */
			if(Ext_ID_Rx.operation==1){
			tx_data_length=sprintf(tx_data, "EBiCS_for_M510+M560_V0.009");
			send_multiframe(Ext_ID_Rx.command, &tx_data[0],tx_data_length );
			}
			break;
		case 0x6010: //to do
			/* initialize transmit message */
			if(Ext_ID_Rx.operation==1){
			send_multiframe(Ext_ID_Rx.command, &Para0[0],64 );
			}
			break;
		case 0x6011: //to do
			/* initialize transmit message */
			if(Ext_ID_Rx.operation==1){
			send_multiframe(Ext_ID_Rx.command, &Para1[0],64 );
			}
			break;
		case 0x6012: //to do
			/* initialize transmit message */
			if(Ext_ID_Rx.operation==1){
			send_multiframe(Ext_ID_Rx.command, &Para2[0],64 );
			}
			break;

	}//end case
}

void send_multiframe(uint16_t command, char* data, uint8_t length ){


			//send multiframe start
			Ext_ID_Tx.command = command;
			Ext_ID_Tx.operation = LONG_START_CMD;
			Ext_ID_Tx.target = 0x05; //BESST
			Ext_ID_Tx.source = 0x02; //controller
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			transmit_message.tx_dlen = 1;
			transmit_message.tx_data[0] = length;


			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}

			//send multiframe data
			if(length%8)nbrofframes = (length>>3);
			else nbrofframes = (length>>3)-1;

			for (k=0; k < nbrofframes; k++){
				Ext_ID_Tx.command = k;
				Ext_ID_Tx.operation = LONG_TRANG_CMD;
				Ext_ID_Tx.target = 0x05; //BESST
				Ext_ID_Tx.source = 0x02; //controller
				transmit_message.tx_sfid = 0x00;
				transmit_message.tx_efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
				transmit_message.tx_ft = CAN_FT_DATA;
				transmit_message.tx_ff = CAN_FF_EXTENDED;
				transmit_message.tx_dlen = 8;
				memcpy(&transmit_message.tx_data, data+k*8,8);


				/* transmit message */
				transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
				timeout = 0xFFFF;
				while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
					timeout--;
					}
			}

			//send multiframe end
			Ext_ID_Tx.command = k;
			Ext_ID_Tx.operation = LONG_END_CMD;
			Ext_ID_Tx.target = 0x05; //BESST
			Ext_ID_Tx.source = 0x02; //controller
			transmit_message.tx_sfid = 0x00;
			transmit_message.tx_efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
			transmit_message.tx_ft = CAN_FT_DATA;
			transmit_message.tx_ff = CAN_FF_EXTENDED;
			if(length%8){
			transmit_message.tx_dlen = length%8;//rest of data
			memcpy(&transmit_message.tx_data, data+k*8,length%8);
				}
			else{
				transmit_message.tx_dlen = 8;//rest of data
				memcpy(&transmit_message.tx_data, data+k*8,8);
					}
			/* transmit message */
			transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
			/* waiting for transmit completed */
			timeout = 0xFFFF;
			while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
				timeout--;
				}

}

void append_multiframe(uint16_t command, char* data){
	memcpy(data+command*8, &receive_message.rx_data,receive_message.rx_dlen );

}

void update_checksum(void){
	checksum=0;
	for (k=0; k < 63; k++){
		//Para0[k]=k;
		checksum+=Para0[k];
	}
	Para0[63]=checksum%256;
	checksum=0;
	for (k=0; k < 63; k++){
		//Para1[k]=k+64;
		checksum+=Para1[k];
	}
	Para1[63]=checksum%256;
	checksum=0;
	for (k=0; k < 63; k++){
		//Para2[k]=k+128;
		checksum+=Para2[k];
	}
	Para2[63]=checksum%256;
	checksum=0;
}

//int16_t abs(int16_t value){
//
//	if(value>0)return value;
//	else return -value;
//
//}
