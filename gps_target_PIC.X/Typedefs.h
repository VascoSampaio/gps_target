/* 
 * File:   Typedefs.h
 * Author: Pedro Gois
 *
 * Created on 9 de Agosto de 2015, 11:36
 */

#ifndef TYPEDEFS_H
#define	TYPEDEFS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "main.h"
    
typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned int dword;



#define LATREG(a,b)		LAT##a##bits.LAT##a##b
#define TRISREG(a,b)    TRIS##a##bits.TRIS##a##b
#define PORTREG(a,b)	PORT##a##bits.R##a##b
#define CNSTATREG(a,b)	CNSTAT##a##bits.CNSTAT##a##b



#define TurnOn(x)	{x = 1;}
#define TurnOff(x)	{x = 0;}
#define Toggle(x)	{x = ~x;}
#define Swap(a,b)	{int r; r=a; a=b; b=r;}
#define Set(x)		{x = 1;}
#define Clr(x)		{x = 0;}
#define Input(x)	{x = 1;}
#define Output(x)	{x = 0;}


#define BUFFER_COPY(o, i, size)		{int k; for(k = 0; k < size; k++) o[k] = i[k];}



#ifdef	__cplusplus
}
#endif

#endif	/* TYPEDEFS_H */

