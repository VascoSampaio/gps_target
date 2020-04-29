/* 
 * File:   main.h
 * Author: Vasco
 *
 * Created on 2 de Abril de 2020, 16:19
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

#define _SUPPRESS_PLIB_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define _DISABLE_OPENADC10_CONFIGSCAN_WARNING

#include <xc.h>
#include <cp0defs.h>
#include <plib.h>
#include <sys/attribs.h>
#include <stdint.h> 
#include <stdbool.h>


#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#define SYSCLK			      60000000
#define GetSystemClock()      (SYSCLK)
#define GetPeripheralClock()  (SYSCLK/2)
#define PBCLK			      GetPeripheralClock()

#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

