// PIC32MX570F512H Configuration Bit Settings
// 'C' source line config statements
// DEVCFG3

#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)
#pragma config FPLLIDIV = DIV_6         // PLL Input Divider (6x Divider)
#pragma config FPLLMUL = MUL_15         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config UPLLIDIV = DIV_6         // USB PLL Input Divider (6x Divider)
#pragma config UPLLEN = OFF              // USB PLL Enable (Enabled)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)

#pragma config WDTPS = PS16384          // Watchdog Timer Postscaler (1:16384) 500ms
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF              // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "main.h"

void IO_SETUP(){
    ANSELD= 0x00; //Set as digital port
    ANSELB= 0x00; //Set as digital port
    
    // 0 - output; 1 - input
    TRISE = 0x00; //Set as output
    TRISD = 0x14; //0b000000010100;
    TRISB = 0x40; //0b000001000000;
    
    //Write to port latch
    LATE =  0x00; //Turn on LED's 
    
}
//void UART1_Initializer(unsigned long baudRate){
//    
//    U1MODEbits.UEN                    = 2; //Enable RTS, CTS, URx and  UTx
//    U1MODEbits.BRGH                   = 0; // Setup High baud rates.
//    U1MODEbits.RTSMD                  = 0; // flow control
//    U1MODEbits.WAKE                   = 0; // No wake up on low batery
//    U1MODEbits.PDSEL                  = 0; // 8-bit data no parity
//    U1MODEbits.STSEL                  = 0; // 1 stop-bit 
//    U1MODEbits.IREN                   = 0; 
//    unsigned long int baudRateDivider = ((GetPeripheralClock()/(16*baudRate))-1);
//    U1BRG                             = baudRateDivider; // set BRG     
// 
//   //PIN SELECTION
//    U1RXRbits.U1RXR                   = 0b0000; //Map to RPD2
//    U1CTSRbits.U1CTSR                 = 0b0100; //Remap to RPD4
//    RPD3Rbits.RPD3R                   = 0b0011; //Remap U1Tx
//    RPD1Rbits.RPD1R                   = 0b0100; //Remap to RTS
//    CFGCONbits.IOLOCK                 = 1;    // PPS Lock
//    
//  // UART Configuration
//    U1STA                            = 0;
//    U1STAbits.UTXEN                  = 1;  // TX is enabled
//    U1STAbits.URXEN                  = 1;  // RX is enabled
//    U1STAbits.URXISEL                = 0;  //Interrupt only if 6 bytes are avaible in buffer
//    U1MODEbits.ON                    = 1;  // UART1 module is Enabled
//}

void UART4_Initializer(unsigned long baudRate){
    
    U4MODEbits.UEN                    = 0; //Enable RTS, CTS, URx and  UTx
    U4MODEbits.BRGH                   = 0; // Setup High baud rates.
    U4MODEbits.RTSMD                  = 0; // flow control
    U4MODEbits.WAKE                   = 0; // No wake up on low batery
    U4MODEbits.PDSEL                  = 0; // 8-bit data no parity
    U4MODEbits.STSEL                  = 0; // 1 stop-bit 
    U4MODEbits.IREN                   = 0; 
    unsigned long int baudRateDivider = ((GetPeripheralClock()/(16*baudRate))-1);
    U4BRG                             = baudRateDivider;                         // set BRG     
 
   //PIN SELECTION
    U4RXRbits.U4RXR                   = 0b0101; //Map to RPB6
    RPB7Rbits.RPB7R                   = 0b0010; //Remap U4Tx
    CFGCONbits.IOLOCK                 = 1;    // PPS Lock
    
  // UART Configuration
    U4STA                             = 0;
    U4STAbits.UTXEN                   = 1;  // TX is enabled
    U4STAbits.URXEN                   = 1;  // RX is enabled
    U4STAbits.URXISEL                 = 0;
    U4MODEbits.ON                     = 1;  // UART4 module is Enabled
}


void GENERAL_INTERRUPT_SETUP(){ //INTERRUPT CONTROL   
    INTDisableInterrupts();
    INTCONbits.MVEC   = 1;   //CPU interrupt enable to Multi vectored
    INTCONbits.INT0EP = 1; //External interrupt Polarity
    
}

void TIMER1_SETUP(){ //TIMER1
    PR1 = GetPeripheralClock()/256/2-1;
    TMR1 = 0;
    T1CONbits.TCKPS = 3; // Prescaler= 256
    T1CONbits.TCS   = 0;   // Clock Source 
    T1CONbits.ON    = 1;    // Turn on 
}

void SPI_INT_SETUP(){ //UART1 INTERRUPT SETUP
    IEC1bits.U1EIE  = 1;
    IPC7bits.U1IP   = 7;  // Rx Interrurpt priority level
    IPC7bits.U1IS   = 2;  // Rx Interrurpt sub priority level
    IFS1bits.U1RXIF = 0;  // Flag status
    IEC1bits.U1RXIE = 1;  // Rx interrupt enable
}

void U4INT_SETUP(){ //UART4 INTERRUPT SETUP
    IEC2bits.U4EIE  = 1;
    IPC9bits.U4IP   = 6;  // Rx Interrurpt priority level
    IPC9bits.U4IS   = 3;  // Rx Interrurpt sub priority level
    IFS2bits.U4RXIF = 0;  // Flag status
    IEC2bits.U4RXIE = 1;  // Rx interrupt enable
}

T1INT_SETUP(){
    IPC1bits.T1IP = 5; //Timer 1 priority
    IPC1bits.T1IS = 0; //Timer 1 Subpriority
    IFS0bits.T1IF = 0; //Flag status
    IEC0bits.T1IE = 1; //Enable Timer1 Interrupt
}

void __ISR(_UART_1_VECTOR, IPL7SAVEALL) UART1ISR(void){ //UART FROM PC TO GPS  - LED2 Orange
    unsigned char curChar;
    while(U1STAbits.URXDA){
        curChar = U1RXREG;
        U4TXREG    = curChar; //   U1RXREG; //
    }
    IFS1bits.U1RXIF = 0;
};

void __ISR(_UART_4_VECTOR, IPL6SAVEALL) UART4ISR(void){ //UART FROM GPS TO PC - LED1 White
     while(U4STAbits.URXDA){
        U1TXREG    = U4RXREG; // curChar; //  
    }
    LATEINV         = 0x8; 
    IFS2bits.U4RXIF = 0;    
};

void __ISR(_TIMER_1_VECTOR, IPL5SAVEALL) Timer1ISR(void){ 
    LATEINV = 0xff;
    U1TXREG = 100;
    IFS0bits.T1IF = 0;
};

void main(){ 

    IO_SETUP(); //INITIALIZE I/O
    
   //INITIALIZE UART
//    UART1_Initializer(115200); //baud rate =  115K
    UART4_Initializer(115200);   //baud rate =  115K    
   
    GENERAL_INTERRUPT_SETUP(); //SETUP INTERRUPT    
   
    TIMER1_SETUP(); //TIMER1
     
//    U1INT_SETUP();  //INTERRUPT UART    
    U4INT_SETUP();
    
//    T1INT_SETUP(); //INTERRUPT1
    
    INTEnableInterrupts();   
    
    while (1){;}
}
    
