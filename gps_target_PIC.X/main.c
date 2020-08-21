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
#define PUBX_CFG  0x209100ed
#define RATE_CFG  0x30210001
#define MONN_CFG  0x20910350
#define UBX_CFG   0x10740001

enum {START_WAIT = 0, RECEIVING_NMEA, RECEIVING_UBX, RECEIVING_UBX_PAYLOAD, PRE_RECV_UBX};
enum {POS = 0, SURV};
int state = START_WAIT;
unsigned char nmeaBuffer[120];
unsigned char ubxBuffer[120];
unsigned char *nmeaPtr = nmeaBuffer;
unsigned char *ubxPtr = ubxBuffer;
unsigned char to_send[120];
int msg_len = 0, i = 0;
bool mRcv, mSend, ack;

void chooseUBX(){
    if(ubxBuffer[2] == 0x05){
        if(ubxBuffer[3] == 0x01){
            ack = true;
        } else if(ubxBuffer[3] == 0x00) {
            //TODO NACK
        }
    }
    
    if(ubxBuffer[2] == 0x0a){
        if(ubxBuffer[3] == 0x36){
            
            /*byte a = ReadStatus();      
             
             if ((a >> 4) != 0x02){
             if ((a >> 4) == 0x07){
                 //Toggle(WHITE_LED);
                 WriteStrobe(STROBE_SFTX);
                 
             }
             WriteStrobe(STROBE_STX);
             }
            
            WriteFIFO(ubxBuffer, 108);
            a = ReadStatus();  
            a = ReadStatus(); 
            */
        }
    }      
}

bool cfg_gps(uint16_t rate){
    
    send_cfg(PUBX_CFG, 17,(char) 0, 0);
    U4STAbits.URXEN = 1; 
    
    send_cfg(RATE_CFG, 18,(char) 0, rate);
    
    while(!ack);
    ack = false;
    
    /*send_cfg(UBX_CFG, 17,(char) 1, 0);
    
    while(!ack);
    ack = false;
    
    send_cfg(MONN_CFG, 17,(char) 1, 0);
    
    while(!ack);
    ack = false;
*/   
    send_cfg(PUBX_CFG, 17,(char) 1, 0);
    
    while(!ack);
    ack = false;
    
}

void send_cfg(int keyValue, int write_size, char enable, uint16_t rate){
    unsigned char buf[20];
    int ct = 0;
    
    buf[0]  = 0xb5; /*Header sync1*/
	buf[1]  = 0x62; /*Header sync2*/
	buf[2]  = 0x06; /*class ID: CFG*/
	buf[3]  = 0x8a;
	buf[4]  = write_size-8;
	buf[5]  = 0;    /*lenght MSB*/
	buf[6]  = 0x00; //version
	buf[7]  = 0x01; //layers
	buf[8]  = 0x00; //reserved
	buf[9]  = 0x00; //reserved
	buf[10] = keyValue & 0xFF; 
	buf[11] = (keyValue >>  8) & 0xFF;
	buf[12] = (keyValue >> 16) & 0xFF;
	buf[13] = (keyValue >> 24) & 0xFF;
    
    switch(write_size){
        case 17:
            memmove(buf + 14, &enable, sizeof(enable));
            break;
        case 18:
            memmove(buf + 14, &rate, sizeof(rate));
            break;
    }
    
    ubx_checksum(buf + 2, write_size-4, buf + write_size - 2, 0, 0);
    
    for (ct = 0; ct < write_size; ct++){
        //while(!IFS2bits.U4TXIF);
        U4TXREG = buf[ct];
        Delay_ms(10);
    }
    
}

static bool ubx_checksum(const unsigned char *data,  unsigned len, unsigned char ck[2], unsigned char comparator_a, unsigned char comparator_b)
{
	const unsigned char *buffer = data;
	unsigned char ck_a = 0;
	unsigned char ck_b = 0;

	while(len--){
		ck_a += *buffer++;
		ck_b += ck_a;				
	}

	if (ck != NULL){
		ck[0] = ck_a;
		ck[1] = ck_b;
	}

	if(comparator_a == ck_a && comparator_b == ck_b) {
		return true;
	}
	return false;
}

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
            to_send[51] = '&';
            return to_send;
        break;
        case SURV:
            nmeaBuffer[109] = '%';
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
                ubxPtr = ubxBuffer;
                *(ubxPtr++) = currChar; 
                state = PRE_RECV_UBX;
			}    	 //                        	 // and start receiving data
       	  	break;
            
        case PRE_RECV_UBX:
			if(currChar == 0x62){
				*(ubxPtr++) = currChar; 		// Start character received...
				state = RECEIVING_UBX;								
			}
            break;
     	case RECEIVING_NMEA:                       // Message Start received
			if(currChar == '*'){              // If end of message...
				state  = START_WAIT;
				nmeaPtr = nmeaBuffer;
				return true;
			}                             		      	 
       	   	else{
				*(nmeaPtr++) = currChar; 
            }            
       	    break;
					   
		case RECEIVING_UBX:                  // Message Start received
			*(ubxPtr++) = currChar;
			if ((ubxPtr - ubxBuffer) == 6){ 
				msg_len = *(ubxPtr-2) | *(ubxPtr-1) << 8; 
				state = RECEIVING_UBX_PAYLOAD;
			}

			break;
						
		case RECEIVING_UBX_PAYLOAD:	
			*(ubxPtr++) = currChar;	
            int b = ubxPtr - ubxBuffer;
			if(ubxPtr - ubxBuffer - 8 == msg_len){ 
				//Toggle(WHITE_LED);
                state = START_WAIT;
                chooseUBX();
                return false;
			}
			break;
	}
    return false;
}  	

void __ISR(_UART_1_VECTOR, IPL7SAVEALL) UART1ISR(void){ //UART FROM PC TO GPS  - LED2 Orange
    unsigned char curChar;
    while(U1STAbits.URXDA){
        curChar = U1RXREG;
        U4TXREG = curChar;
    }
    IFS1bits.U1RXIF = 0;   
};

void __ISR(_UART_4_VECTOR, IPL6SAVEALL) UART4ISR(void){ //UART FROM GPS TO PC - LED1 White
     while(U4STAbits.URXDA){
         
         if(getMessage((unsigned char) U4RXREG)){
             
             byte a = ReadStatus();      
             
             if ((a >> 4) != 0x02){
             if ((a >> 4) == 0x07){
                 //Toggle(WHITE_LED);
                 WriteStrobe(STROBE_SFTX);
                 
             }
             WriteStrobe(STROBE_STX);
             }
             a = ReadStatus();
             if(i == 20){
                 nmeaBuffer[110] = '&';
                 nmeaBuffer[3] = '!';
                //WriteStrobe(STROBE_STX);
                a = ReadStatus();
                a = CC_IO0;
                WriteFIFO(nmeaBuffer+3, 108);
                a = ReadStatus();
                if((a >> 4) == 0x07) WriteStrobe(STROBE_SFTX); 
                if((a >> 4) == 0x06) WriteStrobe(STROBE_SFRX); 
             } else if (i == 23){
               i = 0;  
             } else {
                 nmeaBuffer[52] = '%';
                 nmeaBuffer[17] = '@';
                //WriteStrobe(STROBE_STX);
                 //a = ReadStatus();
                WriteFIFO(nmeaBuffer+17, 36);
                a = ReadStatus();
                if((a >> 4) == 0x07) WriteStrobe(STROBE_SFTX); 
                if((a >> 4) == 0x06) WriteStrobe(STROBE_SFRX); 
             }
             i++;
             a = ReadStatus();
            Toggle(ORANGE_LED);
        }       
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
    uint16_t rate = 40;
    
    IO_SETUP(); //INITIALIZE I/O
    
    GENERAL_INTERRUPT_SETUP(); //SETUP INTERRUPT 
    UART4_Initializer(115200);   //baud rate =  115K    
    CC1125_Init(10);
    a = ReadStatus();
    U4INT_SETUP();
    RECINT_SETUP();
    //SPI_INT_SETUP();
    
    
    Delay_ms(10);
    a = ReadStatus();
    Toggle(WHITE_LED);
    a = ReadReg(REG_RFEND_CFG1);
    a = ReadReg(REG_RFEND_CFG0);
    a = ReadStatus();
    if((a >> 4) == 0x07) WriteStrobe(STROBE_SFTX); 
    if((a >> 4) == 0x06) WriteStrobe(STROBE_SFRX); 
    WriteStrobe(STROBE_SRX);
    a= ReadStatus();
    
    Delay_ms(10);
    INTEnableInterrupts();
    
    cfg_gps(rate);
    
    while (1){
    
    //byte c[CC_MAX_PACKET_DATA_SIZE] = {'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0,0,'P','U','B','X',0,0,0,0,0};
    //a = ReadExtendedReg(EXT_NUM_TXBYTES);
    a= ReadStatus();
    if((a >> 4) == 0x07) WriteStrobe(STROBE_SFTX); 
    if((a >> 4) == 0x06) WriteStrobe(STROBE_SFRX); 


    /*if(ReadStatus() == 0x6F){
        WriteStrobe(STROBE_SFTX);
        WriteStrobe(STROBE_STX);
    }*/
    Delay_ms(800);

    }
}
   