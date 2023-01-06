/*
 * Tx_transmitData.h
 *
 *  Created on: Nov 2, 2022
 *      Author: Kalki
 */

#ifndef TX_TRANSMITDATA_H_
#define TX_TRANSMITDATA_H_



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void init_Txpwm();
void transmitIsr();
void sendkey(uint8_t add, uint8_t dat);
void Tx_preProcess(char* argData);
void Tx_sendByte(uint8_t byte);

//Command functions
void setIR(USER_DATA* data);
void setRGB(USER_DATA* data);
void getIR(USER_DATA* data);
void setAck(USER_DATA* data);
void getpoll(USER_DATA* data);
void freeform_Send(USER_DATA* data);
void retransmit();




#endif /* TX_TRANSMITDATA_H_ */
