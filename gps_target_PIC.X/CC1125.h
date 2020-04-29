/* 
 * File:   CC1125.h
 * Author: Pedro Gois
 *
 * Created on 10 de Outubro de 2014, 17:56
 */
 
#ifndef CC1125_H
#define CC1125_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "main.h"
#include "HardwareProfile.h"

#define INT_PRIORITY_CC			5
//#define CC_BROADCAST_ADDR		0xFFFF		// FFFF broadcast
//#define CC_MY_ADDR				0x3958		//"9X"
#define CC_MAX_PACKET_DATA_SIZE			9
#define CC_SCK			1000000
#define CC_BRG			((PBCLK / (2 * CC_SCK)) -1)
	
#define CC1125_CHANNEL_MIN		1
#define CC1125_CHANNEL_MAX		69

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned int dword;


typedef struct _ccsts
{
	byte rssi;
    unsigned rxGo :1;
    unsigned txGo :1;
	unsigned active :1;
	unsigned :5;
}CC1125_STATUS;



void CC1125_Init(byte channel);
void CC1125_Process();
void CC1125_TX(byte *data, byte length);
void CC1125_RX(byte *data, byte length);
void CC1125_Disable();



#ifdef	__cplusplus
}
#endif

#endif	/* CC1125_H */


