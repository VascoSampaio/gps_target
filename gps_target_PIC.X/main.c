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

enum {START_WAIT = 0, RECEIVING_NMEA, RECEIVING_UBX, RECEIVING_UBX_PAYLOAD, PRE_RECV_UBX};
enum {POS = 0, SURV};
int state = START_WAIT;
unsigned char nmeaBuffer[120];
unsigned char ubxBuffer[120];
unsigned char *nmeaPtr = nmeaBuffer;
unsigned char *ubxPtr = ubxBuffer;
unsigned char to_send[120];
int msg_len = 0, i = 0;
bool mRcv, mSend;

unsigned char* chooseNMEA(int opt){
    int n_virg = 0;
    int i = 0;
    
    unsigned char *to_send_ptr = to_send;
    switch(opt){
        case POS:
            for(i = 0; i < 109; i++){
                if(nmeaBuffer[i] == ',') n_virg++;
                
                if(n_virg < 3) continue;
                if(n_virg == 8) break;
                
                *(to_send_ptr++) = nmeaBuffer[i]; 
            }
            return to_send;
        break;
        case SURV:
            return nmeaBuffer;
        break;    
    }
    
}

bool getMessage(unsigned char currChar){
    switch(state){
        case START_WAIT:               		 // Waiting for start of message
			if(currChar == '$'){
				nmeaPtr = nmeaBuffer;		// Start character received...
				state = RECEIVING_NMEA;
			}
			else if(currChar == 0xb5){
                state = PRE_RECV_UBX;
			}    	 //                        	 // and start receiving data
       	  	break;
            
        case PRE_RECV_UBX:
			if(currChar == 0x62){
				ubxPtr = ubxBuffer;		// Start character received...
				state = RECEIVING_UBX;								
			}
            break;
     	case RECEIVING_NMEA:                       // Message Start received
			if(currChar == '*'){              // If end of message...
				state  = START_WAIT;
				nmeaPtr = nmeaBuffer;
                //Toggle(ORANGE_LED);
				return true;
			}                             		      	 
       	   	else{
                U1TXREG = U4RXREG;
				*(nmeaPtr++) = currChar; 
            }            
       	    break;
					   
		case RECEIVING_UBX:                  // Message Start received
			*(ubxPtr++) = currChar;
			if ((ubxPtr - ubxBuffer) == 4){ 
				msg_len = *(ubxPtr-2) | *(ubxPtr-1) << 8; 
				state = RECEIVING_UBX_PAYLOAD;
                return true;
			}

			break;
						
		case RECEIVING_UBX_PAYLOAD:	
			*(ubxPtr++) = currChar;	
			if(ubxPtr - ubxBuffer - 6 == msg_len){ 
				Toggle(WHITE_LED);
                state = START_WAIT;
			}
			break;
	}
    return false;
}  	

void __ISR(_UART_1_VECTOR, IPL7SAVEALL) UART1ISR(void){ //UART FROM PC TO GPS  - LED2 Orange
    unsigned char curChar;
    while(U1STAbits.URXDA){
        curChar = U1RXREG;
        //U4TXREG    = curChar; //   U1RXREG; //
    }
    IFS1bits.U1RXIF = 0;
    //Toggle(WHITE_LED);
    
};

void __ISR(_UART_4_VECTOR, IPL6SAVEALL) UART4ISR(void){ //UART FROM GPS TO PC - LED1 White
     while(U4STAbits.URXDA){
        //U1TXREG    = U4RXREG; // curChar; //
         //mRcv = true;
         
         if(getMessage((unsigned char) U4RXREG)){
             
             
             byte a = ReadStatus();      
             
             if ((a >> 4) == 0x07){
                 Toggle(WHITE_LED);
                 WriteStrobe(STROBE_SFTX);
                 WriteStrobe(STROBE_STX);
             }
             a = ReadStatus();
             if(i == 7){
             WriteFIFO(nmeaBuffer, 108);
             } else if (i == 10){
               i = 0;  
             } else {
             WriteFIFO(nmeaBuffer+17, 36);  
             }
             i++;
            Toggle(ORANGE_LED);
        }
         //Toggle(WHITE_LED);
         
    }
    //LATEINV         = 0x8; 
    IFS2bits.U4RXIF = 0;
        
};

void __ISR(_TIMER_1_VECTOR, IPL5SAVEALL) Timer1ISR(void){ 
    //LATEINV = 0xff;
    U1TXREG = 100;
    //Toggle(ORANGE_LED);
    IFS0bits.T1IF = 0;
};

void main(){ 

    byte a = 0;
   
    IO_SETUP(); //INITIALIZE I/O
    
    
    GENERAL_INTERRUPT_SETUP(); //SETUP INTERRUPT 
    UART4_Initializer(115200);   //baud rate =  115K    
    CC1125_Init(10);
    a = ReadStatus();
    //WriteStrobe(STROBE_SRES);
    U4INT_SETUP();
    
    
    Delay_ms(10);
    a = ReadStatus();
    Toggle(WHITE_LED);
    a = ReadReg(REG_RFEND_CFG1);
    a = ReadReg(REG_RFEND_CFG0);
    WriteStrobe(STROBE_STX);
    
    Delay_ms(10);
    INTEnableInterrupts();   
    
    while (1){
        
    
    
    //byte c[CC_MAX_PACKET_DATA_SIZE] = {'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0};
    a = ReadExtendedReg(EXT_NUM_TXBYTES);
    a= ReadStatus();
    
    if(mRcv){
        
        mRcv = false;
    }
    //WriteFIFO(c, 109);
    if(ReadStatus() == 0x6F){
        WriteStrobe(STROBE_SFTX);
        WriteStrobe(STROBE_STX);
    }
    Delay_ms(800);
    //Toggle(WHITE_LED);
    //Toggle(ORANGE_LED);
    }
}
   