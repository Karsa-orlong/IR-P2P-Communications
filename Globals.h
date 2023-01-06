/*
 * Globals.h
 *
 *  Created on: Nov 23, 2022
 *      Author: giskusama
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_


//All Global variables and defines

//*********************************************************************************************
//ParseData.c


//*********************************************************************************************
//Tx_transmitData.c
//-----------------------------------------------------------------------------
#define IR_LED   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4))) // Bit 6 of Port B

#define IR_LED_MASK 64 // PB6  - M0PWM0
#define LOAD_9ms 360000
#define LOAD_4ms 180000
#define LOAD_T 22500

#define Tx_FIFOSize 100
#define LSB

//------- CMDs supported by IR
#define CMD_SET 0x00
#define CMD_GET 0x10
#define CMD_RGB 0x01
#define CMD_POLL 0x20
#define CMD_GETRESPONSE 0x11
#define CMD_SENDACK 0x30

//Max args for args list
#define MAX_ARGS 3


// DATA PACKET(s)
typedef struct _DATA_PACKET {
    uint8_t DST_ADD;
    uint8_t SRC_ADD;
    bool ack;
    uint8_t CMD;
   // uint8_t argsCount;
    uint8_t argList[MAX_ARGS];
    uint8_t checksum;
    } DATA_PACKET;

    DATA_PACKET transmit_packet;
    DATA_PACKET receive_packet;

//*********************************************************************************************
// Rx_ReadData.c

// PortC masks
#define FREQ_IN_MASK 64 // 2^6 so 0-6, 7th bit mask

// PortF masks   NO NEED
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8
#define PUSH_BUTTON_MASK 16

#define compare0 11250   // 0.5T
#define compare1 33750   // 1.5T
#define compare2 56250   // 2.5T
#define compare3 78750   // 3.5T
#define compare4 101250  // 4.5T

#define timeout 135000 // same as 6T
#define Rx_FIFOSize 100

//*********************************************************************************************


// PortF masks
    #define RED_LED_MASK 2
    #define BLUE_LED_MASK 4
    #define GREEN_LED_MASK 8


#endif /* GLOBALS_H_ */
