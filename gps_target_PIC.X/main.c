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

#include "CC1125.h"
#include "communicationLib.h"

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
    Toggle(ORANGE_LED);
    IFS0bits.T1IF = 0;
};

void main(){ 

    byte a = 0;
    IO_SETUP(); //INITIALIZE I/O
    
   //INITIALIZE UART
    //UART1_Initializer(115200); //baud rate =  115K    
    UART4_Initializer(115200);   //baud rate =  115K    
    
    GENERAL_INTERRUPT_SETUP(); //SETUP INTERRUPT 
    //SPI1_Init();

    //TIMER1_SETUP(); //TIMER1
//    U1INT_SETUP();  //INTERRUPT UART    
    //U4INT_SETUP();
//    T1INT_SETUP(); //INTERRUPT1
    
    CC1125_Init(10);
    Delay_ms(10);
    a = ReadStatus();
    a = ReadReg(REG_RFEND_CFG1);
    a = ReadReg(REG_RFEND_CFG0);
    WriteStrobe(STROBE_STX);
    Delay_ms(10);
    //INTEnableInterrupts();   
    while (1){
    byte c[CC_MAX_PACKET_DATA_SIZE] = {1,2,3,4,5,6,7,8,9};
    a = ReadExtendedReg(EXT_NUM_TXBYTES);
    WriteFIFO(c, 9);
    if(ReadStatus() == 0x6F){
        WriteStrobe(STROBE_SFTX);
        WriteStrobe(STROBE_STX);
    }
    Delay_ms(800);
    Toggle(WHITE_LED);
    }
}
   