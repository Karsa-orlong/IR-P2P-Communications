/*
 * cmdHelp.c
 *
 *  Created on: Nov 11, 2022
 *      Author: Kalki
 *      DebugLED config and functions
 *      Help commands
 *      clear >> print 50 lines in putty
 *      reboot >> reset the Tiva C board
 *      commands handler
 */

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "Globals.h"
#include "ParseData.h"
#include "cmdHelp.h"


//-----------------------------------------------------------------------------
// Extern vars
//-----------------------------------------------------------------------------
extern uint32_t time[140];
extern uint8_t count;
extern bool data_received;
extern uint8_t isMSB;
extern bool trans_on;


//--------------------------------------------------------
//Subroutines
//--------------------------------------------------------

void DebugLED()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // port F clock enable

    // Configure LED pins
    GPIO_PORTF_DIR_R = 0x0E;  // bits 3, 2, and 1 are outputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x0E;  // enable LEDs
}
//--------------------------------------------------------

void helpfuncs(USER_DATA* data ){
    DebugLED();

    if(isCommand(data, "clear", 0)){
        uint8_t i;
        for(i =0 ; i<50; i++){
            putcUart0('\n');
        }
    }
    else if(isCommand(data, "help", 0)){
        putsUart0("clear >> clears putty \n");
        putsUart0("reboot >> reboots the texas board \n");
        putsUart0("BlueON, BlueOFF \n");
        putsUart0("RedON, RedOFF \n");
        putsUart0("GreenON, GreenOFF \n");
        putcUart0('\n');
        putsUart0("COMMANDS.... \n");
        putsUart0("ACK ON | ACK OFF \n");
        putsUart0("get DST_ADD \n");
        putsUart0("poll | poll DST_ADD .. ACK should be OFF \n");
        putsUart0("set DST_ADD DATA \n");
        putsUart0("rgb DST_ADD DATA1 DATA2 DATA3 \n");
        putsUart0("send DATA1 DATA2.. send free form data \n");
        putsUart0("receive...print the last received packet \n");
        putsUart0("clr .... reset state for receiveISR \n");
        putsUart0("MSB .... process with MSB first \n");
        putsUart0("LSB .... process with LSB first \n");
        putsUart0("trans_off...Turn off retransmission \n");
        putcUart0('\n');

    }

    else if(isCommand(data, "reboot", 0)){
        putsUart0("Rebooting...");
        NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    }
    else if(isCommand(data, "clr", 0)){
   //******** start receiving again
                  count=0;
      } // receive ends
    else if(isCommand(data, "MSB", 0)){
        isMSB =1;
        putsUart0("process with MSB first \n");
    }
    else if(isCommand(data, "LSB", 0)){
        isMSB =0;
        putsUart0("process with LSB first \n");
    }
    else if(isCommand(data, "BlueON", 0)){
        BLUE_LED = 1;
    }
    else if(isCommand(data, "BlueOFF", 0)){
        BLUE_LED = 0;
        }
    else if(isCommand(data, "RedON", 0)){
        RED_LED = 1;
        }
    else if(isCommand(data, "RedOFF", 0)){
        RED_LED = 0;
            }
    else if(isCommand(data, "GreenON", 0)){
        GREEN_LED = 1;
        }
    else if(isCommand(data, "GreenOFF", 0)){
        GREEN_LED = 0;
        }
    else if(isCommand(data, "trans_off", 0)){
        trans_on = false;
        putsUart0("Retransmission Turned OFF \n");

    }
}

void printFields(USER_DATA* data){
    int i;
    for (i = 0; i < data->fieldCount; i++)
                    {
                    putcUart0(data->fieldType[i]);
                    putcUart0('\t');
                    putsUart0(&data->buffer[data->fieldPosition[i]]);
                    putcUart0('\n');
                    }
}
