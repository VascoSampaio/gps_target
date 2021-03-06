#include "/home/dsor/target/src/gps_target/teensy_library/binaryOperators/binaryOperators.h"
#include "/home/dsor/target/src/gps_target/teensy_library/Radio/Radio.cpp"


//#define SENDER
#define RECEIVER


enum class Send {
  ss  = 10,
  mosi= 11,
  miso= 12,
  sck = 13,
  RESET= 3,
  IO0  = 2,
  IO1  =12,
  IO2  =14  
};

enum class Rec {
  ss   =  6,
  mosi = 21,
  miso =  5,
  sck  = 20,
  RESET=  7,
  IO0  = 22,
  IO1  =  5,
  IO2  =  4  
};

//SPI 0
#define SS_SENDER      10
#define MOSI_SENDER    11
#define MISO_SENDER    12
#define SCK_SENDER     13
#define CC_RESET_SENDER 3
#define CC_IO0_SENDER   2
#define CC_IO1_SENDER  12
#define CC_IO2_SENDER  14

//SPI 1
#define SS_RECEIVER       6
#define MOSI_RECEIVER    21
#define MISO_RECEIVER     5
#define SCK_RECEIVER     20
#define CC_RESET_RECEIVER 7
#define CC_IO0_RECEIVER  22
#define CC_IO1_RECEIVER   5
#define CC_IO2_RECEIVER   4

enum {WAIT, RTK, DATA}
    state = WAIT;
int ctr = 0, size_m = 0;  
byte rtk_data[300];
bool a = true;
bool pr = false;
byte stat = 0;
int ct = 0;
int ct_rtk = 0;
int last_time = millis();
byte incomingByte = 0;
const int ledPin = 13;
SPISettings settingsA(1000000, MSBFIRST, SPI_MODE0);

bool getMessage(byte in){
  switch(state){
    case WAIT:
      if(in == 'R') state = RTK;
      break;
    case RTK:
      size_m = (int) in;
      Serial.println(size_m);
      state = DATA;
      break;
    case DATA:
      Serial.print("DATA:    ");
      Serial.println(ctr);
      rtk_data[ctr] = in;
      ctr++;
      if(ctr == size_m){
        state = WAIT;
        return true;
      }
      break;
  }
  return false;
}

#ifdef SENDER
rfend_cfg sender_rfend_cfg  = {0x20, 0x2f};
pins sender_pins =  {SS_SENDER, MOSI_SENDER, MISO_SENDER, SCK_SENDER, CC_RESET_SENDER, CC_IO0_SENDER, CC_IO1_SENDER, CC_IO2_SENDER};
Radio sender(SPI, settingsA, sender_pins, "sender", sender_rfend_cfg);

void senderWrapper() {
  sender.interruptHandler();
}
#endif

#ifdef RECEIVER
rfend_cfg receiver_rfend_cfg  = {0x30, 0x3f};
pins receiver_pins = {6, 21, 5, 20, 7, 22, 5, 4};
Radio receiver(SPI1, settingsA, receiver_pins, "receiver", receiver_rfend_cfg);

void receiverWrapper() {
  receiver.interruptHandler();
}
#endif

void setup (void) {

  Serial.begin(115200);
  while (!Serial && millis() < 4000 );
  Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  pinMode(ledPin, OUTPUT);
#ifdef SENDER
  sender.configurator(10);
  attachInterrupt(digitalPinToInterrupt(sender.ccIO0), senderWrapper, FALLING); // interrrupt 1 is data ready
   delay(10);
   sender.WriteStrobe(STROBE_STX);
#endif

#ifdef RECEIVER
  receiver.configurator(10);
  attachInterrupt(digitalPinToInterrupt(receiver.ccIO0), receiverWrapper, FALLING); // interrrupt 1 is data ready
  delay(10);
  receiver.WriteStrobe(STROBE_SRX);
#endif
}


void loop (void) {

  //byte c[CC_MAX_PACKET_DATA_SIZE];

  //last_time = millis();
  //int2bytearr(c, last_time, sizeof(c));

  #ifdef SENDER
  sender.WriteFIFO(c, sizeof(c));
  delay(10);
    if (sender.ReadStatus() == TXFIFOERROR) {
      sender.WriteStrobe(STROBE_SFTX);  //Flux TX;
      sender.WriteStrobe(STROBE_STX);
    }
  #endif
  //receiver.ReadExtendedReg(EXT_NUM_RXBYTES);
  //Serial.println(receiver.ReadExtendedReg(EXT_NUM_RXBYTES));

  

  //Serial.println(receiver.ReadStatus() >> 4);
  if(ct == 20000){
    stat = receiver.ReadStatus() >> 4;
    //Serial.println(stat >> 4);
  
    
    if ((stat) != 0x01){
      if (stat == 0x06){             
      receiver.WriteStrobe(STROBE_SFRX);
    }
      receiver.WriteStrobe(STROBE_SRX);
       
      digitalWrite(ledPin, LOW);             
    } else digitalWrite(ledPin, HIGH);
    ct = 0;
  }

      if (stat == 0x07){ 
                   
      receiver.WriteStrobe(STROBE_SFTX);
    }
     
  
  /*if (receiver.ReadStatus() == RXFIFOERROR) {
    receiver.WriteStrobe(STROBE_SFRX);
    receiver.WriteStrobe(STROBE_SRX);
  }*/

  if(Serial.available()){
    
   //stat = receiver.ReadStatus() >> 4;
    incomingByte = Serial.read();
    if (incomingByte == 211) {
      ct_rtk = 0;
      receiver.WriteStrobe(STROBE_STX);
      receiver.WriteFIFO(rtk_data, ct_rtk);
      digitalWrite(ledPin, LOW);
    }
    rtk_data[ct_rtk] = incomingByte;
    ct_rtk++;
    //Serial.print(incomingByte);
    //Serial.print(',');
    //getMessage(incomingByte);
    
    
 
    
    //Serial.flush();
    //receiver.WriteStrobe(STROBE_SRX);
    pr = true;
  }
  

  //Serial.println("\n");*/
  //delay(200);
  ct++;
}
