/*
 * Rx_readData.h
 *
 *  Created on: Oct 25, 2022
 *      Author: Kalki
 */
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for snprintf)

// Hardware configuration:

// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   Configured to 115,200 baud, 8N1
// Frequency counter and timer input:
//   SIGNAL_IN on PC6 (WT1CCP0)
// Timer samples the data from transmitter remote in the form 9ms>4.5ms>ADD>ADD_INV>DATA>DATA_INV>Stop_bit

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef RX_READDATA_H_
#define RX_READDATA_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void enableCounterMode();                           // configure wide timer to count the timestamps
void Rx_Hw();                                      //Initialize hardware for WideTimer1 and PC6 GPO pin interrupts
void post_print_IR();                               //process the global timearray to convert timestamps to binary
void receiveIsr();
void debugIsr();
void post_process_IR();

void initRgb();
void setRgbColor(uint16_t red, uint16_t green, uint16_t blue);
void Rx_cmd_handler();
void get_response();
void initBackoff();
void printPacket();




#endif /* RX_READDATA_H_ */
