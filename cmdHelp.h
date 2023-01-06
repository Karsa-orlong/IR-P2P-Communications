/*
 * cmdHelp.h
 *
 *  Created on: Nov 11, 2022
 *      Author: giskusama
 */

#ifndef CMDHELP_H_
#define CMDHELP_H_


#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

void helpfuncs();
void DebugLED();
void printFields(USER_DATA* data);


#endif /* CMDHELP_H_ */
