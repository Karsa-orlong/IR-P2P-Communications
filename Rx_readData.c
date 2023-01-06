// Frequency Counter / Timer Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for snprintf)

// Hardware configuration:
//WideTimer1, Timer2A configured
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Frequency counter and timer input:
//   SIGNAL_IN on PC6 (WT1CCP0)

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
#include "Rx_readData.h"
#include "Tx_transmitData.h"



//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

uint32_t time[140];         // Global array to capture timestamps
uint8_t count =0;
uint8_t value =0;
bool ready = false;         // Overall readiness

char Rx_FIFO[Rx_FIFOSize][8];        // 16*8 bits
uint8_t Rx_wrIndex =0;
uint8_t Rx_rdIndex =0;
bool data_received  = false;
int shift =0; // 2^7 = 128 used for bitshift

char Rx_packet_handle[Rx_FIFOSize];
int handle_size = 0; // used for getting variable packet sizes

bool HANDLER_BUSY = false;

extern DATA_PACKET receive_packet;
extern bool dataSent;
extern bool ACK_RECEIVED;
extern uint8_t isMSB;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void enableCounterMode()
{

    // Configure Wide Timer 1 as counter of external events on CCP0 pin
	WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                      // turn-off counter before reconfiguring // counter events configured as inputs to the wide timer
	WTIMER1_CFG_R = 0x4 ; //TIMER_CFG_16_BIT;              // configure as 32-bit counter (A only) 0x4 is used for selecting the 32/64 32 bit wide timer mode
	WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR; // configure for edge count mode TACMR to 1 without using the timer modes, count up
	                                                       // TIMER_TAMR_TACDIR = 0x10 which is setting bit 4 to one to configure count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_NEG;
   // WTIMER1_IMR_R = 0;                                   // turn-off interrupts
	WTIMER1_TAV_R = 0;                                     // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                       // turn-on counter

   // Timer2 config
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                                    // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;                              // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACMR;          // configure for one shot  (count down)
    //TIMER2_TAILR_R = 0;                                               // **set load value to 40e6 for 1 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                                    // turn-on interrupts
    //TIMER2_CTL_R |= 0;                                                // **turn-on timer
    NVIC_EN0_R |= 1 << (INT_TIMER2A-16);                                // turn-on interrupt 37 (TIMER1A)
}
//-----------------------------------------------------------------------------
// Initialize Hardware
void Rx_Hw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;          // wide timer 1 is used so that we can use other timers in other classes
                                                         // its not necessary to match wide timer1 with timer 1
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 ;            // PORT C
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;          // Timer2 enable
    _delay_cycles(3);

    enableCounterMode(); // configure wide timer to count up using the GPIO and not the the timer itself

    GPIO_PORTC_DIR_R &= ~FREQ_IN_MASK;               // PC6 digital input
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK;                // enable bit 6 for digital input
    GPIO_PORTC_PUR_R |= FREQ_IN_MASK;               // set pull up register on PC6 for detector to work
    GPIO_PORTC_IM_R |= FREQ_IN_MASK;                // Setting the GPIO Interrupt mask so that the interrupt works with PC6 pin 6
    NVIC_EN0_R |= 1 << (INT_GPIOC-16);              // NVIC INTERRUPT FOR GPIO PORTC

   initRgb();


}
// Initialize RGB
void initRgb()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1; // turn on pwm generator
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // portf enable
    _delay_cycles(3);

    // Configure three LEDs
    GPIO_PORTF_DEN_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK; // led pins mask DEN
    GPIO_PORTF_AFSEL_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M); // selection of PWM alternate selection
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7; // pwm is not powered up yet

    // Configure PWM module 1 to drive RGB LED
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 5 on PWM1, gen 2b, cmpb, red
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 6 on PWM1, gen 3a, cmpa, blue
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 7 on PWM1, gen 3b, cmpb, green

    PWM1_2_LOAD_R = 256;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz // this affects the OUT4 of Module 1 PWM
    PWM1_3_LOAD_R = 256;                            // The counter counts down

    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs
}

void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM1_2_CMPB_R = red;
    PWM1_3_CMPB_R = green;
    PWM1_3_CMPA_R = blue;

}

void initBackoff(){
    // Configure Timer 1 as the time base
        TIMER3_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACMR;          // configure for periodic mode (count down)
        TIMER3_TAILR_R = 5000000;                       // **set load value to 40e6 for 1 Hz interrupt rate
        TIMER3_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
        TIMER3_CTL_R |= TIMER_CTL_TAEN;                  // **turn-on timer
        NVIC_EN1_R |= 1 << (INT_TIMER3A-48);             // turn-on interrupt 37 (TIMER1A)
}


//-----------------------------------------------------------------------------
//Receive Handler for Commands
//-----------------------------------------------------------------------------

/*
 * RxFIFO will already be stored
 * Extract from the FIFO DEST Add(match it current source) >> SRC add(now the dest add for ack)>>
 * ACK/CMD, Checksum and then finally the arguments
 * Store it in a receive_packet struct. STEP1
 * Handle the command STEP2
 * Create a response packet and send ACK back using a modified transmit_packet if necessary STEP3
 */
//-----------------------------------------------------------------------------
void sendACK(){
    //count =0;
    dataSent = false; // enable transmission
    transmit_packet.DST_ADD = receive_packet.SRC_ADD;    // destination add is received SRC add for ACK cmd
    transmit_packet.SRC_ADD = readEeprom(Block0);           // device source address
    transmit_packet.ack = false;
    transmit_packet.CMD = CMD_SENDACK;

    transmit_packet.checksum = ~(transmit_packet.DST_ADD + transmit_packet.SRC_ADD + transmit_packet.CMD );

// transmit packets for set
    Tx_sendByte(transmit_packet.DST_ADD);
    Tx_sendByte(transmit_packet.SRC_ADD);
    Tx_sendByte(transmit_packet.CMD);
    Tx_sendByte(transmit_packet.checksum);
putsUart0("sent ack\n");
}

void get_response(){
//    pwmState = 0; // Reset transmitter state machine
//    bitval = 0;
//    HighT = false;
    dataSent = false;

    transmit_packet.DST_ADD = receive_packet.SRC_ADD;
    transmit_packet.SRC_ADD = receive_packet.DST_ADD;
    transmit_packet.CMD = CMD_GETRESPONSE;

    if(receive_packet.DST_ADD ==readEeprom(Block0) ){
    transmit_packet.argList[0] = PWM1_2_CMPB_R;} // red cmp value
    else if(receive_packet.DST_ADD ==readEeprom(Block1) ){
    transmit_packet.argList[0] = PWM1_3_CMPB_R;} //blue cmp value
    else if(receive_packet.DST_ADD ==readEeprom(Block2) ){
    transmit_packet.argList[0] = PWM1_3_CMPA_R;} // green cmp value

    transmit_packet.checksum = ~(transmit_packet.DST_ADD + transmit_packet.SRC_ADD + transmit_packet.CMD + transmit_packet.argList[0]);
    waitMicrosecond(10000);
    Tx_sendByte(transmit_packet.DST_ADD);
    Tx_sendByte(transmit_packet.SRC_ADD);
    Tx_sendByte(transmit_packet.CMD);
    Tx_sendByte(transmit_packet.argList[0]);
    Tx_sendByte(transmit_packet.checksum);
}

void Rx_cmd_handler(){
    //initRgb();
    //extract the pure command from the receive packet by removing the ACK mask
    int extract_cmd = receive_packet.CMD;
    extract_cmd &= ~(0x80); // remove the ACK bit

    if(extract_cmd == CMD_SET){                                 //SET
        if(Rx_packet_handle[0] == readEeprom(Block0)){
            setRgbColor(receive_packet.argList[0], 0,0);
        }
        else if(Rx_packet_handle[0] == readEeprom(Block1)){
            setRgbColor(0, receive_packet.argList[0],0);
        }
        else if(Rx_packet_handle[0] == readEeprom(Block2)){
            setRgbColor(0,0, receive_packet.argList[0]);
        }
    }
    else if(extract_cmd == CMD_RGB){                            //RGB
            setRgbColor(receive_packet.argList[0], receive_packet.argList[1],receive_packet.argList[2]); // only accept 1 or 0 as args

        }
    else if(extract_cmd == CMD_GET){                            // GET >> GET_RESPONSE
            get_response();
        }
    else if(extract_cmd == CMD_POLL){
        if( receive_packet.SRC_ADD != readEeprom(Block0)){
//        int T_poll = (rand()%11)*(200000);
//            waitMicrosecond(T_poll);
            sendACK();
        }
        else{
            putsUart0("Waiting for nodes..\n");
        }
    }
    else{
        putsUart0("Command not handled\n");
    }

    //send ack
    if(receive_packet.ack == true ){ // POLL AND ACK
        int T_ack = (rand()%11)*(200000);
        waitMicrosecond(T_ack);
        sendACK();
    }

}



//-----------------------------------------------------------------------------
// receiveIsr() is called when a falling edge occurs

void receiveIsr(){
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                     // Turn off Timeout ISR

    GPIO_PORTC_ICR_R = FREQ_IN_MASK;                    // clear interrupt flag
  //  if((Rx_wrIndex + 1)%Rx_FIFOSize != Rx_rdIndex && (data_received == false)){ // FIFO is NOT FULL and data_received is false
        if((Rx_wrIndex + 1)%Rx_FIFOSize != Rx_rdIndex ){ // FIFO is NOT FULL and data_received is false
        time[count]  = WTIMER1_TAV_R;                    //*** store the timestamps or counts in the time array.
                                                         //Register returns counts and count*(1/fcyc) will give timestamps

        // Config pulse receive
        if(count==0){
        WTIMER1_TAV_R = 0;                              //Clear the counter
            time[count] = 0;
            count++;
        }
                                                        //Receive and process timestamps
        else if(count==1){
            if((time[1]-time[0]) >= 520000 && (time[1]-time[0]) <= 560000 ){
                count++;
                putcUart0('S');
            }
            else{                                       // Not a valid start of the IR command
            count=0;                                    // this will go back to the count=0 condition and clear the counter from timer again
            }
        }
        else if(count>1){
            if((time[count] - time[count-1]) >= compare1 && (time[count] - time[count-1] <= compare2)){
               count++;
                   Rx_FIFO[Rx_wrIndex][shift] = 0 + '0';
                   shift++;

               putcUart0('0');


                }
            else if((time[count] - time[count-1]) >= compare3 && (time[count] - time[count-1] <= compare4)){
                count++;
                   Rx_FIFO[Rx_wrIndex][shift] = 1 + '0';
                   shift++;
                putcUart0('1');

                }
            else{                                       // Not a valid bit length
                count=0;
                }
            TIMER2_TAILR_R = timeout;                   // turn on timeout every hit of receiveISR
            TIMER2_CTL_R |= TIMER_CTL_TAEN;
        }


            if(shift == 8){
                        shift = 0;                      // Write into FIFO, increment wrIndex, reset value
                        putcUart0(':');
                        Rx_wrIndex  = (Rx_wrIndex + 1)%Rx_FIFOSize;
                    }
    }                                                   //FIFO is not full and data received is false ends
                                                        // Receive Isr ends

}
//-----------------------------------------------------------------------------
//TimeoutIsr
void timeoutIsr(){
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;                   // clear interrupt flag
    putcUart0('R');
    putcUart0('\n');
    WTIMER1_TAV_R = 0;
    count=0;
    shift = 0;
    HANDLER_BUSY = true;                                // start the post process IR in main

}

void BackoffISR(){
    TIMER3_ICR_R = TIMER_ICR_TATOCINT;                  // clear interrupt flag
    if(count <= 3 ){                                    // Count issue patch fix
        count =0;
    }

}

void post_process_IR(){
    //********STEP 1 create packet from FIFO
        initEeprom();
        int p=0;
        int temp =0;

        if(Rx_rdIndex > Rx_wrIndex){
            handle_size = Rx_wrIndex + (Rx_FIFOSize - Rx_rdIndex); }
        else{
            handle_size = Rx_wrIndex - Rx_rdIndex; }

        int handle_index = 0;
            while(Rx_rdIndex != Rx_wrIndex ){
                if(isMSB){
                    for(p=0; p<8; p++){
                        temp = Rx_FIFO[Rx_rdIndex][p] - '0';
                        if(temp)
                        {
                            value += 1 << (7-p);
                        }
                    }
                }//MSB
                else{
                    for(p=0; p<8; p++){
                        temp = Rx_FIFO[Rx_rdIndex][p] - '0';
                        if(temp)
                        {
                            value += 1 << p;
                        }
                    }
                } // LSB
                Rx_packet_handle[handle_index] = value;               // single dimensional FIFO that flushes data every time "receive" is called in putty
                handle_index++ ;

                Rx_rdIndex  =(Rx_rdIndex + 1)%Rx_FIFOSize ;
                value=0;
            }//while ends

            if(Rx_packet_handle[0] == readEeprom(Block0) || Rx_packet_handle[0] == readEeprom(Block1) || Rx_packet_handle[0] == readEeprom(Block2) ||Rx_packet_handle[0] == 0xFF ){
                char str[300] ;
                snprintf(str, sizeof(str), "R:DST_ADD    %7"PRIu32"\n", Rx_packet_handle[0]);
                putsUart0(str);
                snprintf(str, sizeof(str), "R:SRC_ADD    %7"PRIu32"\n", Rx_packet_handle[1]);
                putsUart0(str);
                snprintf(str, sizeof(str), "R:CMD    %7"PRIu32"\n", Rx_packet_handle[2]);
                putsUart0(str);

                uint8_t calc_checksum =0; // Calculated CHECKSUM
                                                                        // Build the receive packet
                receive_packet.DST_ADD = Rx_packet_handle[0];           // ***** if this is not equal to SRC address flush data ; how to? increment read pointer
                receive_packet.SRC_ADD = Rx_packet_handle[1];
                receive_packet.CMD = Rx_packet_handle[2];
                if(receive_packet.CMD & 0x80){
                    receive_packet.ack = true; }
                else{
                    receive_packet.ack = false; }
                calc_checksum = Rx_packet_handle[0] + Rx_packet_handle[1] + Rx_packet_handle[2];

                int argpointer = 3;
                while(argpointer < handle_size -1 ){ // its not the last byte which is the checksum
                    receive_packet.argList[argpointer -3] = Rx_packet_handle[argpointer];

                    snprintf(str, sizeof(str), "R:ARG_LIST    %7"PRIu32"\n", Rx_packet_handle[argpointer]);
                    putsUart0(str);

                    calc_checksum += Rx_packet_handle[argpointer];

                    argpointer++;
                }
                calc_checksum = ~calc_checksum;
                receive_packet.checksum = Rx_packet_handle[handle_size -1];
                snprintf(str, sizeof(str), "R:CHECKSUM    %7"PRIu32"\n", Rx_packet_handle[handle_size -1]);
                putsUart0(str);

                                                                            //***** HANDLE CHECKSUM verification
                if(receive_packet.checksum == calc_checksum){
                                                                            //********STEP 2 Handle the commands
                    Rx_cmd_handler();                                       //Handle the commands
                }
                else{
                    putsUart0("CHECKSUM MISMATCH \n");
                    Rx_rdIndex = Rx_wrIndex;                                // flush the receive fifo
                }




            }                                                               // if ends: only process if the dest address in command is the same as
                                                                            //current device address in EEPROM
            else{
               // putcUart0('E');
                Rx_rdIndex = Rx_wrIndex;                                    // flush the receive fifo
                putsUart0("\n ADD MISMATCH / CMD FROM THIS DEVICE \n");
            }
     //   data_received = false;                                            //******** start receiving again
        if(transmit_packet.DST_ADD == receive_packet.SRC_ADD && receive_packet.CMD == CMD_SENDACK ){
            ACK_RECEIVED = true;
        }

        HANDLER_BUSY = false;                                               // POST PROCESSING FLAG


}

void printPacket(){
    char str[300] ;
    snprintf(str, sizeof(str), "R:DST_ADD    %7"PRIu32"\n", Rx_packet_handle[0]);
    putsUart0(str);
    snprintf(str, sizeof(str), "R:SRC_ADD    %7"PRIu32"\n", Rx_packet_handle[1]);
    putsUart0(str);
    snprintf(str, sizeof(str), "R:CMD    %7"PRIu32"\n", Rx_packet_handle[2]);
    putsUart0(str);

    int argpointer =3;

    while(argpointer < handle_size -1 ){ // its not the last byte which is the checksum
       snprintf(str, sizeof(str), "R:ARG_LIST    %7"PRIu32"\n", Rx_packet_handle[argpointer]);
       putsUart0(str);
       argpointer++;
                   }
    snprintf(str, sizeof(str), "R:CHECKSUM    %7"PRIu32"\n", Rx_packet_handle[handle_size -1]);
    putsUart0(str);
}


