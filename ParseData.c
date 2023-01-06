/*
 * ParseData.c
 *
 *  Created on: Nov 2, 2022
 *      Author: Kalki
 *      Methods:
 *      getsUart0 >> get the data from putty and store in a buffer
 *      parseFields >> Parse the data and store the fields, field positions of command, arguments
 *      getFieldInteger
 *      getFieldString >> extract string and integer arguments
 *      strcmp >> string compare
 *      isCommand >> check if a command is valid or not
 */



//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
//#include "Globals.h"
#include "ParseData.h"

//-----------------------------------------------------------------------------
//Global Variables included in header file
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Subroutines
/*
 ASCII Space 32
 BACKSPACE 127
 Carriage return  CR 13
 Line feed  LF \n 10

 */


void getsUart0(USER_DATA* data){
    int counter =0;
    char c= 'a';

        while(counter <= MAX_CHARS ){ //CR HANDLING
             c = getcUart0();

        // BACKSPACE HANDLING ASCII == 127
        if(c == 127 ){
            if(counter >0){
                counter--; // backspace entered so decrement the count
            }
        }
        else
        {
            if(counter < MAX_CHARS){
                   data->buffer[counter++] =c;
            }
                // LF HANDLING // when a LF  is entered, set buffer[count] =0 (ASCII 0 is for null terminator that indicates
               // end of string and break out of the program
            if(c == 10 || c==13 || counter == MAX_CHARS){
               data ->buffer[counter-1] = '\0'; // set null pointer
               break; // break out of for loop
           }
            }
        }
}
//-----------------------------------------------------------------------------

void parseFields(USER_DATA* data){
    int counter =0;
   // int fieldcnt =0;
    int isPrev_delimiter = 0;
    int isFirstPass =0;

    do{ //loop through the buffer array until we get a null

                                                                    // parse fields according to a,n,d (Alphabet, numeric, delimiter)
                                                                    //65-90 A-Z ; 97-122 a-z ; 0-9 digits
        if(counter == 0){
            isFirstPass =1;                                         // assume the prev is a delimiter for the start of the string
            data->fieldCount =0;

        }


        if(isPrev_delimiter ==1 || isFirstPass ==1){
            if((data->buffer[counter] >='a' && data->buffer[counter] <='z') || (data->buffer[counter] >='A' && data->buffer[counter] <='Z')){
                data->fieldType[data->fieldCount] = 'a';            // alphabet type
                data->fieldPosition[data->fieldCount] = counter;
                isPrev_delimiter =0;                                // set the flag for the next iteration to indicate that the prev character was not a delimiter
               // data->fieldCount++;                                // field count for the starting command
            }
            else if(data->buffer[counter] >='0' && data->buffer[counter] <='9'){
            data->fieldType[data->fieldCount] = 'n';                // number type
                data->fieldPosition[data->fieldCount]  = counter;   // counter is the position of the field
                isPrev_delimiter =0;
            }
        }
        else if(isPrev_delimiter == 0){                             // previous char was not a delimiter AND the current character is a delimiter
            if(!(data->buffer[counter] >='A' && data->buffer[counter] <='Z') ){
                if(!(data->buffer[counter] >='a' && data->buffer[counter] <='z')){
                    if(!(data->buffer[counter] >='0' && data->buffer[counter] <='9')){

                        isPrev_delimiter =1;                        // set the flag to true so that the next iteration knows that a delimiter was found earlier
                        data->buffer[counter] = '\0';
                        data->fieldCount++;                         // at the transition update the field count
                    }
                }

            }

        }
        counter++;
        isFirstPass =0;                                             // change flag to false isFirstPass after counter is incremented

               }while(data->buffer[counter] != '\0' );
data->fieldCount++;                                                 // count the last field before '/0'
} // end parsefields
//-----------------------------------------------------------------------------


char* getFieldString(USER_DATA* data, uint8_t fieldNumber){

    if(data->fieldType[fieldNumber] == 'a' && fieldNumber <=MAX_FIELDS){
        return &data->buffer[data->fieldPosition[fieldNumber]];      // data->fieldPosition[fieldNumber] returns the position of the field
    }
    else
        return NULL;
} // end of getFieldString
//-----------------------------------------------------------------------------

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber){
    if(data->fieldType[fieldNumber] == 'n' && fieldNumber <=MAX_FIELDS){
        int32_t result =0 ;
        int32_t digitValue;
        int count = data->fieldPosition[fieldNumber];
        while( data->buffer[count]!= '\0'){ // parse through the
            digitValue = (int)(data->buffer[count] - '0');
            result = result*10 + digitValue;
            count++;
        }

        return result;

    }
    else
        return 0;
} // end of getFieldInteger
//-----------------------------------------------------------------------------


// Function to implement strcmp function
int strcmp(const char *X, const char *Y)
{
    while (*X)
    {
                                                                    // if characters differ, or end of the second string is reached
        if (*X != *Y) {
            break;
        }

                                                                    // move to the next pair of characters
        X++;
        Y++;
    }
                                                                    // return the ASCII difference after converting `char*` to `unsigned char*`
    return *(const unsigned char*)X - *(const unsigned char*)Y;
}

//-----------------------------------------------------------------------------

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments){
                                                                            // minArguments is Max_fields -1
    bool flag ;
        char *command = getFieldString(data, 0);                            // get the first command

        int ret = strcmp(strCommand, command);
        if(ret == 0 && (data->fieldCount -1 >= minArguments)){              // if the string commands are the same and the min arguments are
                                                                           // not greater than MAX_FIELDS -1 1 is for the string command itself
            flag = true;
        }
        else
            flag = false;

    return flag;

}
//-----------------------------------------------------------------------------
//EEPROM and set addresses
//-----------------------------------------------------------------------------
void initEeprom(){
    SYSCTL_RCGCEEPROM_R = 1;
    _delay_cycles(3);
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}
void writeEeprom(uint16_t add , uint32_t data){
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0XF;
    EEPROM_EERDWR_R  = data;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

uint32_t readEeprom(uint16_t add){
    EEPROM_EEBLOCK_R  = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    return EEPROM_EERDWR_R;
}
//-----------------------------------------------------------------------------


void setAddress(USER_DATA* data){

        initEeprom();


        writeEeprom(Block0 , getFieldInteger(data,1 ));
        writeEeprom(Block1 , getFieldInteger(data,2 ));
        writeEeprom(Block2 , getFieldInteger(data,3 ));

        char str[40];
        snprintf(str, sizeof(str), "Block0:    %7"PRIu32"\n", readEeprom(Block0));
        putsUart0(str);
        putcUart0('\n');
        snprintf(str, sizeof(str), "Block1:    %7"PRIu32"\n", readEeprom(Block1));
        putsUart0(str);
        putcUart0('\n');
        snprintf(str, sizeof(str), "Block2:    %7"PRIu32"\n", readEeprom(Block2));
        putsUart0(str);
        putcUart0('\n');


}

