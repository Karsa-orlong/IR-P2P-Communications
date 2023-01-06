/*
 * Tx_transmitData.c
 *
 *  Created on: Nov 2, 2022
 *      Author: Kalki
 *      Preprocess the data you get from Putty and save it in the Tx_FIFO
 *      Transmit ISR will send config pulse first and
 *      IR LED is configured on the PB6 and driven by PWM configured on PB6 M0PWM0
 *      32 bit Timer 1 is configured in One shot and edge count mode
 */
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

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



int pwmState = 0;
uint16_t bitval =0;
bool HighT =false;
bool dataSent = false;

//---Tx FIFO
char Tx_FIFO[Tx_FIFOSize][8];       // 16*8 bits
uint8_t FIFO_wrIndex =0;
uint8_t FIFO_rdIndex =0;
uint8_t Tx_temprd =0;               // Temp FIFO read index used for saving the state for retransmission

bool global_ACK;                    // ACk ON /OFF sets this true or false
extern DATA_PACKET transmit_packet;
extern uint8_t count;
uint8_t retrans_count =0;
bool trans_on = true;
bool ACK_RECEIVED = false;

extern uint8_t isMSB;               // MSB/LSB handling in putty



int T_rand =0 , T0 =0, T_eff = 1;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// initialize hardware
void init_Txpwm(){
    // Initialize system clock to 40 MHz
       initSystemClockTo40Mhz();
       SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // port B clock enable
       SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R3; // Timer1 enable for transmitter and Timer 3 for backoff
       SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0; // PWM enable for module 0, PB6 is module 0, gen 0
       _delay_cycles(3);

   // Configure IR LED
       GPIO_PORTB_DIR_R |= IR_LED_MASK;
       GPIO_PORTB_DR8R_R |= IR_LED_MASK;
       GPIO_PORTB_DEN_R |= 0;
       GPIO_PORTB_AFSEL_R |= IR_LED_MASK;
       GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB6_M);
       GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB6_M0PWM0;

   // Config the PWM0
       SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
       SYSCTL_SRPWM_R = 0;                              // leave reset state
       PWM0_0_CTL_R  = 0;                               // Turn off the PWM generator 0
       PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO; // output on PWM0, Gen 0a, cmpa
       PWM0_0_LOAD_R = 1053;                            // 37.9 khz to generate a PWM wave fcyc/fdes
       PWM0_0_CMPA_R = 527;                             // Duty cycle 50%

       PWM0_0_CTL_R = PWM_0_CTL_ENABLE;
       PWM0_ENABLE_R = PWM_ENABLE_PWM0EN ;
                                                            // enable outputs



    // Configure Timer 1 as the time base
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACMR;          // configure for one shot mode (count down)
        //TIMER1_TAILR_R = 0;                       // **set load value to 40e6 for 1 Hz interrupt rate
        TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
        //TIMER1_CTL_R |= 0;                  // **turn-on timer
        NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)

        initEeprom();

}
//-----------------------------------------------------------------------------
//--------Global ACK
//-----------------------------------------------------------------------------

void setAck(USER_DATA* data){
    char* off = "OFF";
    char* on = "ON";
    if(!strcmp(getFieldString(data, 1), on)){
        global_ACK = true;
        putsUart0("ACK has been turned ON \n");
    }
    else if(!strcmp(getFieldString(data, 1), off)){
        global_ACK = false;
        putsUart0("ACK has been turned OFF \n");

    }
}

void retransmit(){
    if(trans_on && global_ACK && transmit_packet.CMD!= NULL){
        T_eff = 35000;                                              // 35 ms for max time RGB with ACK
        T0 = 50000;                                                 // Tolerance

        while(retrans_count < 4 &&  !ACK_RECEIVED && count!=1){
            char str[40];
            srand(11);
            T_rand = T0 + (rand()%11)*T_eff*(1<<retrans_count);
            snprintf(str, sizeof(str), "T_rand    %7"PRIu32"\n", T_rand);
            putsUart0(str);
           waitMicrosecond(T_rand);
           putsUart0("Retransmitting \n");
           pwmState = 0;
           bitval = 0;
           HighT = false;
           dataSent = false;
           if(!ACK_RECEIVED){
               FIFO_rdIndex = Tx_temprd ;
               transmitIsr();
           }
           retrans_count++;
        }

        retrans_count =0;
        ACK_RECEIVED = false;
    }

}
//    //take data and create the DATAPACKET
/*
 * set DST_ADD DATA
 * get arguments from the User data input :
 * get the source address >> EEPROM code
 * get the destination address from the first argument  of input data
 * build packet destadd,srcadd, ack, cmd, arglist, checksum
 * call the Tx_sendbyte function and send each byte while count <= argscount + 4
 */
//-----------------------------------------------------------------------------
//--------setIR
//-----------------------------------------------------------------------------
void setIR(USER_DATA* data){
    //initEeprom();
    dataSent = false; // enable transmission

    transmit_packet.DST_ADD = getFieldInteger(data,1 ); // destination add
    transmit_packet.SRC_ADD = readEeprom(Block0); // default source address
    if(global_ACK){
        transmit_packet.ack = true;
        transmit_packet.CMD = 0x80|CMD_SET; }
    else{
        transmit_packet.ack = false;
        transmit_packet.CMD = CMD_SET; }
        transmit_packet.argList[0] = getFieldInteger(data, 2); // get the data bit
        transmit_packet.checksum = ~(transmit_packet.DST_ADD + transmit_packet.SRC_ADD + transmit_packet.CMD + transmit_packet.argList[0]);

// transmit packets for set
    Tx_temprd = FIFO_rdIndex; // store temp index for retransmisssion in case of ACK failure

    Tx_sendByte(transmit_packet.DST_ADD);
    Tx_sendByte(transmit_packet.SRC_ADD);
    Tx_sendByte(transmit_packet.CMD);
    Tx_sendByte(transmit_packet.argList[0]);
    Tx_sendByte(transmit_packet.checksum);

    retransmit();

}
//-----------------------------------------------------------------------------
//--------setRGB
//-----------------------------------------------------------------------------
void setRGB(USER_DATA* data){
    dataSent = false; // enable transmission

    transmit_packet.DST_ADD = getFieldInteger(data,1 );     // destination add
    transmit_packet.SRC_ADD = readEeprom(Block0);           // default source address
    if(global_ACK){
        transmit_packet.ack = true;
        transmit_packet.CMD = 0x80|CMD_RGB; }
    else{
        transmit_packet.ack = false;
        transmit_packet.CMD = CMD_RGB; }
    transmit_packet.argList[0] = getFieldInteger(data, 2); // get the data bit
    transmit_packet.argList[1] = getFieldInteger(data, 3); // get the data bit
    transmit_packet.argList[2] = getFieldInteger(data, 4); // get the data bit
    transmit_packet.checksum = ~(transmit_packet.DST_ADD + transmit_packet.SRC_ADD + transmit_packet.CMD + transmit_packet.argList[0] +transmit_packet.argList[1] +transmit_packet.argList[2]);

// transmit packets for setRGB
    Tx_temprd = FIFO_rdIndex;                               // store temp index for retransmisssion in case of ACK failure

    Tx_sendByte(transmit_packet.DST_ADD);
    Tx_sendByte(transmit_packet.SRC_ADD);
    Tx_sendByte(transmit_packet.CMD);
    Tx_sendByte(transmit_packet.argList[0]);
    Tx_sendByte(transmit_packet.argList[1]);
    Tx_sendByte(transmit_packet.argList[2]);
    Tx_sendByte(transmit_packet.checksum);

    retransmit();

}
//-----------------------------------------------------------------------------
//--------getIR
//-----------------------------------------------------------------------------
void getIR(USER_DATA* data){
    //initEeprom();
    dataSent = false; // enable transmission

transmit_packet.DST_ADD = getFieldInteger(data,1 );         // destination add
    transmit_packet.SRC_ADD = readEeprom(Block0);           // default source address
    transmit_packet.ack = false;
    transmit_packet.CMD = CMD_GET;
    transmit_packet.checksum = ~(transmit_packet.DST_ADD + transmit_packet.SRC_ADD + transmit_packet.CMD );

// transmit packets for getting a response
    Tx_temprd = FIFO_rdIndex;                               // store temp index for retransmisssion in case of ACK failure

    Tx_sendByte(transmit_packet.DST_ADD);
    Tx_sendByte(transmit_packet.SRC_ADD);
    Tx_sendByte(transmit_packet.CMD);
    Tx_sendByte(transmit_packet.checksum);

    retransmit();


}

void getpoll(USER_DATA* data){

    dataSent = false; // enable transmission
if(getFieldInteger(data,1 )){
    transmit_packet.DST_ADD = getFieldInteger(data,1 ); }   // destination add

else{
    transmit_packet.DST_ADD = 0xFF;}                        // destination add is 255 for POLL if no arguments

    transmit_packet.SRC_ADD = readEeprom(Block0);           // default source address
        transmit_packet.ack = true;
        transmit_packet.CMD = CMD_POLL;                     // POLL always requires ACK

    transmit_packet.checksum = ~(transmit_packet.DST_ADD + transmit_packet.SRC_ADD + transmit_packet.CMD );

// transmit packets for POLL
    Tx_temprd = FIFO_rdIndex;                               // store temp index for retransmisssion in case of ACK failure

    Tx_sendByte(transmit_packet.DST_ADD);
    Tx_sendByte(transmit_packet.SRC_ADD);
    Tx_sendByte(transmit_packet.CMD);
    Tx_sendByte(transmit_packet.checksum);

  //  retransmit();

}

void freeform_Send(USER_DATA* data){
    dataSent = false; // enable transmission
    transmit_packet.CMD = getFieldInteger(data,1 );
    transmit_packet.DST_ADD = getFieldInteger(data,2 );
    transmit_packet.SRC_ADD = getFieldInteger(data,3 );

    if(transmit_packet.CMD ==CMD_SET ){
    transmit_packet.argList[0] = getFieldInteger(data,4 );
    transmit_packet.checksum = getFieldInteger(data,5 );
    }
    else if(transmit_packet.CMD ==CMD_RGB ){
        transmit_packet.argList[0] = getFieldInteger(data, 4); // get the data bit
        transmit_packet.argList[1] = getFieldInteger(data, 5); // get the data bit
        transmit_packet.argList[2] = getFieldInteger(data, 6); // get the data bit
        transmit_packet.checksum = getFieldInteger(data,7 );
    }
    else{
        transmit_packet.checksum = getFieldInteger(data,4 );
    }

    Tx_sendByte(transmit_packet.DST_ADD);
    Tx_sendByte(transmit_packet.SRC_ADD);
    Tx_sendByte(transmit_packet.CMD);
    if(transmit_packet.CMD ==CMD_SET ){
        Tx_sendByte(transmit_packet.argList[0]);
    }
    else if(transmit_packet.CMD ==CMD_RGB ){
        Tx_sendByte(transmit_packet.argList[0]);
        Tx_sendByte(transmit_packet.argList[1]);
        Tx_sendByte(transmit_packet.argList[2]);
    }
    Tx_sendByte(transmit_packet.checksum);


}
//-----------------------------------------------------------------------------
//Transmit ISR
//-----------------------------------------------------------------------------

void transmitIsr(){
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;                      // clear interrupt flag

if(FIFO_wrIndex != FIFO_rdIndex && dataSent == false){
      if(bitval == 8 ){                                     // bitval  == 8 and FIFO is NOT empty
          FIFO_rdIndex = (FIFO_rdIndex + 1)%Tx_FIFOSize ;
          bitval = 0;                                       // reset bitval
//putcUart0(':');

          if(FIFO_wrIndex == FIFO_rdIndex){                 // Tx FIFO is empty condition
               GPIO_PORTB_DEN_R |= IR_LED_MASK;
               TIMER1_TAILR_R = LOAD_T;
               TIMER1_CTL_R |= TIMER_CTL_TAEN;
               HighT = true;
               dataSent = true;                             // stop transmission in main
                                                            // Send the last bit only if there is no more data
           }
      }

    if(pwmState == 0){                                      // Configuration pulse
        GPIO_PORTB_DEN_R |= IR_LED_MASK;
        TIMER1_TAILR_R = LOAD_9ms;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
        pwmState = 1;

    }
    else if(pwmState ==1){
        GPIO_PORTB_DEN_R &= ~IR_LED_MASK;
        TIMER1_TAILR_R = LOAD_4ms;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
        pwmState = 2;
    }

                                                            //send 1
    else if(Tx_FIFO[FIFO_rdIndex][bitval]){
        if(pwmState == 2 && HighT == false){
            GPIO_PORTB_DEN_R |= IR_LED_MASK;
            TIMER1_TAILR_R = LOAD_T;
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            pwmState = 3;
            HighT = true;
        }

        else if(pwmState == 3 && HighT == true){
            GPIO_PORTB_DEN_R &= ~IR_LED_MASK;
            TIMER1_TAILR_R = 3*LOAD_T;
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            pwmState = 2;
            HighT = false;
            bitval ++ ;

//putsUart0("1 ");
        }

    }                                                       // Send 0
    else if(!Tx_FIFO[FIFO_rdIndex][bitval]){
        if(pwmState == 2 && HighT == false){
            GPIO_PORTB_DEN_R |= IR_LED_MASK;
            TIMER1_TAILR_R = LOAD_T;
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            pwmState = 4;
            HighT = true;
        }
        else if(pwmState == 4 && HighT == true){
            GPIO_PORTB_DEN_R &= ~IR_LED_MASK;
            TIMER1_TAILR_R = LOAD_T;
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            pwmState = 2;
            HighT = false;
            bitval++ ;
//putsUart0("0 ");
        }
    }
 }
else{
    GPIO_PORTB_DEN_R &= ~IR_LED_MASK;
    putcUart0('F');
    putcUart0('\n');
    pwmState = 0;
    bitval = 0;
    HighT = false;
}

} // Isr ends
//-----------------------------------------------------------------------------

void Tx_sendByte(uint8_t byte){
    int i =0;
    if(isMSB){
    for(i=7; i>=0; --i){
        if(byte & (1<<i)){
            Tx_FIFO[FIFO_wrIndex][7-i] = 1;
        }
        else{
            Tx_FIFO[FIFO_wrIndex][7-i] = 0;
        }
    }
    } // MSB
    else{
        for(i=0; i<8; ++i){
               if(byte & (1<<i)){
                   Tx_FIFO[FIFO_wrIndex][i] = 1;
               }
               else{
                   Tx_FIFO[FIFO_wrIndex][i] = 0;
               }
           }
    }//LSB

    FIFO_wrIndex = (FIFO_wrIndex+1)%Tx_FIFOSize ;               // increment write index
    if(pwmState ==0){
    transmitIsr();
    }
}
