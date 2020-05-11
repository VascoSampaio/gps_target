#include "communicationLib.h"

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

//inline void SPI1_Init()
//{
//    int c;
//    Clr(SPI1CONbits.ENHBUF);
//    Clr(SPI1CONbits.SMP);
//    Set(SPI1CONbits.CKE);
//    Clr(SPI1CONbits.CKP);
//    Set(SPI1CONbits.MSTEN);
//    SPI1BRG = CC_BRG;
//    Set(SPI1CONbits.ON);
//    c = SPI1BUF;
//}

void GENERAL_INTERRUPT_SETUP(){ //INTERRUPT CONTROL   
    INTDisableInterrupts();
    INTCONbits.MVEC   = 1;   //CPU interrupt enable to Multi vectored
    INTCONbits.INT0EP = 1;   //External interrupt Polarity   
}


void IO_SETUP(){
    ANSELD= 0x00; //Set as digital port
    ANSELB= 0x00; //Set as digital port
    
    // 0 - output; 1 - input
    TRISE = 0x00; //Set as output
    
    //Write to port latch
    LATE =  0x00; //Turn on LED's 
    
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

void T1INT_SETUP(){
    IPC1bits.T1IP = 5; //Timer 1 priority
    IPC1bits.T1IS = 0; //Timer 1 Subpriority
    IFS0bits.T1IF = 0; //Flag status
    IEC0bits.T1IE = 1; //Enable Timer1 Interrupt
}

void Delay_ms(uint32_t DelayCount){
  uint32_t StartTime;                    // Start Time
  volatile uint32_t a = GetPeripheralClock()/1000*DelayCount;
  
  StartTime = _CP0_GET_COUNT();         // Get CoreTimer value for StartTime
  while ( (uint32_t)(_CP0_GET_COUNT() - StartTime) < a) { 
  }
}
