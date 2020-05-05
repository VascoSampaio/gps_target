/* 
 * File:   HardwareProfile.h
 * Author: pgois
 *
 * Created on 21 de Abril de 2016, 16:25
 */

#ifndef HARDWAREPROFILE_H
#define	HARDWAREPROFILE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "Typedefs.h"
    
    //LEDS
#define WHITE_LED		LATREG(E,3)
#define ORANGE_LED	    LATREG(E,4)
    
 
//IO PINS
#define IN1			PORTREG(B,9)
#define IN2			PORTREG(B,8)
#define IN3			PORTREG(B,7)
#define IN4			PORTREG(B,6)
#define OUT1		LATREG(B,9)
#define OUT2		LATREG(B,8)
#define OUT3		LATREG(B,7)
#define OUT4		LATREG(B,6)
#define IO1_TRIS	TRISREG(B,9)
#define IO2_TRIS	TRISREG(B,8)
#define IO3_TRIS	TRISREG(B,7)
#define IO4_TRIS	TRISREG(B,6)

//SPI
#define CC_CS       LATREG(D,4)
#define CC_RESET    LATREG(B,7)
#define CC_IO0		IN4
#define CC_IO1		PORTREG(D,3)
#define CC_SPI_REMAP()	{SDI1Rbits.SDI1R = 0x0; RPD1Rbits.RPD1R = 0x8; INT3Rbits.INT3R = 0x5;}	//RPD3 -> SI; RPD1 -> SDO3; CS -> RPD4; External Interrupt 3 on RPB9
    

#define CONFIG_IOS()	{Clr(ANSELE); Clr(ANSELG); Output(TRISEbits.TRISE3); Output(TRISEbits.TRISE4); Set(LED1); Set(LED2); Output(TRISDbits.TRISD4); Set(CC_CS);}
#define CONFIG_REMAP()	{SPI_REMAP(); CAN_REMAP();}


#ifdef	__cplusplus
}
#endif

#endif	/* HARDWAREPROFILE_H */

