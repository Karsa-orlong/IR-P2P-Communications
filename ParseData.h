/*
 * ParseData.h
 *
 *  Created on: Nov 2, 2022
 *      Author: Kalki
 */

#ifndef PARSEDATA_H_
#define PARSEDATA_H_

#define MAX_CHARS 2000
#define MAX_FIELDS 255


typedef struct _USER_DATA {
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
    } USER_DATA;


//--EEPROM pointers for storing RGB address values
#define Block0 0
#define Block1 1
#define Block2 2
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void getsUart0(USER_DATA* data);
void parseFields(USER_DATA* data);
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
int strcmp(const char *X, const char *Y);
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments);
void helpfuncs(USER_DATA* data);
void setAddress(USER_DATA* data);


//EEPROM funcs
void initEeprom();
uint32_t readEeprom(uint16_t add);




#endif /* PARSEDATA_H_ */
