/*
 * Lab_Kalki.c
 *
 *  Created on: Nov 2, 2022
 *      Author: Kalki
 */

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "Globals.h"
#include "ParseData.h"
#include "Tx_transmitData.h"
#include "Rx_readData.h"
#include "cmdHelp.h"

//-----------------------------------------------------------------------------
//Global Variables included in header file
//-----------------------------------------------------------------------------
bool valid = false;
extern uint32_t time[140];
extern uint8_t count;
extern bool data_received;
extern int pwmState ;
extern uint16_t bitval ;
extern bool HighT;
extern bool dataSent;
extern uint8_t FIFO_rdIndex;
extern uint8_t Tx_temprd ;

//****backoff variables

#define T_eff 200000
extern uint8_t retrans_count;
extern bool ACK_RECEIVED;
extern bool HANDLER_BUSY;

//MSB LSB handling
uint8_t isMSB =1;


//-----------------------------------------------------------------------------
//Subroutines
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(){
    // Initialize system clock to 40 MHz
    Rx_Hw();
    initUart0();
    init_Txpwm();
    setUart0BaudRate(115200, 40e6);
    DebugLED();
    initBackoff();



    // Power-on flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    USER_DATA data = {0};

    while(true){
        if(kbhitUart0()){
                getsUart0(&data);
                parseFields(&data);
                putsUart0("\nProcessing..");
                putcUart0('\n');
                helpfuncs(&data);                       // supports commands in putty like clear and reboot


                if(isCommand(&data, "set", 2)){ //RGB
                    setIR(&data);
                }
                else if(isCommand(&data, "rgb", 4)){ //RGB
                    setRGB(&data); }
                else if(isCommand(&data, "get", 1)){ //GET
                    getIR(&data); }
                else if(isCommand(&data, "poll", 1)){ // POLL SPECFIC DEVICE
                    getpoll(&data); }
                else if(isCommand(&data, "poll", 0)){ // POLL ALL
                    getpoll(&data); }
                else if(isCommand(&data, "Address", 3)){ // SET EEPROM ADDRESS
                    setAddress(&data); }
                else if(isCommand(&data, "ACK", 1)){ // ACK ON/OFF
                    setAck(&data);
                }
                else if(isCommand(&data, "receive", 0)){
                    printPacket();
                }
                else if(isCommand(&data, "send", 0)){
                    freeform_Send(&data);
                }

        }
        if(HANDLER_BUSY){
            post_process_IR();
        }


        } // while ends
} // main ends
