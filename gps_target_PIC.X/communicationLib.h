/* 
 * File:   communication_lib.h
 * Author: Vasco
 *
 * Created on 28 de Abril de 2020, 17:16
 */

#ifndef COMMUNICATION_LIB_H
#define	COMMUNICATION_LIB_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "main.h"

#define CPU_CT_HZ          (SYSCLK/2)
#define US_TO_CT_TICKS  (CPU_CT_HZ/1000000UL)    // uS to CoreTimer Ticks


void UART4_Initializer(unsigned long);
void SPI_INT_SETUP();
void GENERAL_INTERRUPT_SETUP();
void TIMER1_SETUP();
void U4INT_SETUP();
void T1INT_SETUP();
void ShortDelay(uint32_t);

#ifdef	__cplusplus
}
#endif

#endif	/* COMMUNICATION_LIB_H */

