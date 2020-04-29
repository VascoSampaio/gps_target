/*
   File:   radio.h
   Author: Vasco Sampaio

   Created on 1 of April, 2020

   Useful links: https://community.platformio.org/t/compiling-isr-on-a-teensy/9018/2
*/

/*
   File:   RF_SPI_master.c
   Author: Vasco Sampaio

   Created on 10 de Outubro de 2014, 17:56

   Useful links: https://community.platformio.org/t/compiling-isr-on-a-teensy/9018/2
*/

#include <SPI.h>
#include "pins_arduino.h"
#include <map>
#include "binaryOperators.h"
#include <string.h>

#define SENDER
#define RECEIVER

#define EXTENDED_ADDRESS 0x2F
#define DIRECT_MEMORY_ACCESS 0x3E
#define FIFOS 0x7F

//#define APPENDED
#define CC_MAX_PACKET_DATA_SIZE 10
#define CC_PACKET (CC_MAX_PACKET_DATA_SIZE + 2)

#define REG_IOCFG3 0x00
#define REG_IOCFG2 0x01
#define REG_IOCFG1 0x02
#define REG_IOCFG0 0x03
#define REG_SYNC3 0x04
#define REG_SYNC2 0x05
#define REG_SYNC1 0x06
#define REG_SYNC0 0x07
#define REG_SYNC_CFG1 0x08
#define REG_SYNC_CFG0 0x09
#define REG_DEVIATION_M 0x0A
#define REG_MODCFG_DEV_E 0x0B
#define REG_DCFILT_CFG 0x0C
#define REG_PREAMBLE_CFG1 0x0D
#define REG_PREAMBLE_CFG0 0x0E
#define REG_FREQ_IF_CFG 0x0F
#define REG_IQIC 0x10
#define REG_CHAN_BW 0x11
#define REG_MDMCFG1 0x12
#define REG_MDMCFG0 0x13
#define REG_SYMBOL_RATE2 0x14
#define REG_SYMBOL_RATE1 0x15
#define REG_SYMBOL_RATE0 0x16
#define REG_AGC_REF 0x17
#define REG_AGC_CS_THR 0x18
#define REG_AGC_GAIN_ADJUST 0x19
#define REG_AGC_CFG3 0x1A
#define REG_AGC_CFG2 0x1B
#define REG_AGC_CFG1 0x1C
#define REG_AGC_CFG0 0x1D
#define REG_FIFO_CFG 0x1E
#define REG_DEV_ADDR 0x1F
#define REG_SETTLING_CFG 0x20
#define REG_FS_CFG 0x21
#define REG_WOR_CFG1 0x22
#define REG_WOR_CFG0 0x23
#define REG_WOR_EVENT0_MSB 0x24
#define REG_WOR_EVENT0_LSB 0x25
#define REG_PKT_CFG2 0x26
#define REG_PKT_CFG1 0x27
#define REG_PKT_CFG0 0x28
#define REG_RFEND_CFG1 0x29
#define REG_RFEND_CFG0 0x2A
#define REG_PA_CFG2 0x2B
#define REG_PA_CFG1 0x2C
#define REG_PA_CFG0 0x2D
#define REG_PKT_LEN 0x2E

#define STROBE_SRES 0x30
#define STROBE_SFSTXON 0x31
#define STROBE_SXOFF 0x32
#define STROBE_SCAL 0x33
#define STROBE_SRX 0x34
#define STROBE_STX 0x35
#define STROBE_SIDLE 0x36
#define STROBE_SAFC 0x37
#define STROBE_SWOR 0x38
#define STROBE_SPWD 0x39
#define STROBE_SFRX 0x3A
#define STROBE_SFTX 0x3B
#define STROBE_SWORRST 0x3C
#define STROBE_SNOP 0x3D

#define EXT_IF_MIX_CFG 0x00
#define EXT_FREQOFF_CFG 0x01
#define EXT_TOC_CFG 0x02
#define EXT_MARC_SPARE 0x03
#define EXT_ECG_CFG 0x04
#define EXT_CFM_DATA_CFG 0x05
#define EXT_EXT_CTRL 0x06
#define EXT_RCCAL_FINE 0x07
#define EXT_RCCAL_COARSE 0x08
#define EXT_RCCAL_OFFSET 0x09
#define EXT_FREQOFF1 0x0A
#define EXT_FREQOFF0 0x0B
#define EXT_FREQ2 0x0C
#define EXT_FREQ1 0x0D
#define EXT_FREQ0 0x0E
#define EXT_IF_ADC2 0x0F
#define EXT_IF_ADC1 0x10
#define EXT_IF_ADC0 0x11
#define EXT_FS_DIG1 0x12
#define EXT_FS_DIG0 0x13
#define EXT_FS_CAL3 0x14
#define EXT_FS_CAL2 0x15
#define EXT_FS_CAL1 0x16
#define EXT_FS_CAL0 0x17
#define EXT_FS_CHP 0x18
#define EXT_FS_DIVTWO 0x19
#define EXT_FS_DSM1 0x1A
#define EXT_FS_DSM0 0x1B
#define EXT_FS_DVC1 0x1C
#define EXT_FS_DVC0 0x1D
#define EXT_FS_LBI 0x1E
#define EXT_FS_PFD 0x1F
#define EXT_FS_PRE 0x20
#define EXT_FS_REG_DIV_CML 0x21
#define EXT_FS_SPARE 0x22
#define EXT_FS_VCO4 0x23
#define EXT_FS_VCO3 0x24
#define EXT_FS_VCO2 0x25
#define EXT_FS_VCO1 0x26
#define EXT_FS_VCO0 0x27
#define EXT_GBIAS6 0x28
#define EXT_GBIAS5 0x29
#define EXT_GBIAS4 0x2A
#define EXT_GBIAS3 0x2B
#define EXT_GBIAS2 0x2C
#define EXT_GBIAS1 0x2D
#define EXT_GBIAS0 0x2E
#define EXT_IFAMP 0x2F
#define EXT_LNA 0x30
#define EXT_RXMIX 0x31
#define EXT_XOSC5 0x32
#define EXT_XOSC4 0x33
#define EXT_XOSC3 0x34
#define EXT_XOSC2 0x35
#define EXT_XOSC1 0x36
#define EXT_XOSC0 0x37
#define EXT_ANALOG_SPARE 0x38
#define EXT_PA_CFG3 0x39
#define EXT_WOR_TIME1 0x64
#define EXT_WOR_TIME0 0x65
#define EXT_WOR_CAPTURE1 0x66
#define EXT_WOR_CAPTURE0 0x67
#define EXT_BIST 0x68
#define EXT_DCFILTOFFSET_I1 0x69
#define EXT_DCFILTOFFSET_I0 0x6A
#define EXT_DCFILTOFFSET_Q1 0x6B
#define EXT_DCFILTOFFSET_Q0 0x6C
#define EXT_IQIE_I1 0x6D
#define EXT_IQIE_I0 0x6E
#define EXT_IQIE_Q1 0x6F
#define EXT_IQIE_Q0 0x70
#define EXT_RSSI1 0x71
#define EXT_RSSI0 0x72
#define EXT_MARCSTATE 0x73
#define EXT_LQI_VAL 0x74
#define EXT_PQT_SYNC_ERR 0x75
#define EXT_DEM_STATUS 0x76
#define EXT_FREQOFF_EST1 0x77
#define EXT_FREQOFF_EST0 0x78
#define EXT_AGC_GAIN3 0x79
#define EXT_AGC_GAIN2 0x7A
#define EXT_AGC_GAIN1 0x7B
#define EXT_AGC_GAIN0 0x7C
#define EXT_CFM_RX_DATA_OUT 0x7D
#define EXT_CFM_TX_DATA_IN 0x7E
#define EXT_ASK_SOFT_RX_DATA 0x7F
#define EXT_RNDGEN 0x80
#define EXT_MAGN2 0x81
#define EXT_MAGN1 0x82
#define EXT_MAGN0 0x83
#define EXT_ANG1 0x84
#define EXT_ANG0 0x85
#define EXT_CHFILT_I2 0x86
#define EXT_CHFILT_I1 0x87
#define EXT_CHFILT_I0 0x88
#define EXT_CHFILT_Q2 0x89
#define EXT_CHFILT_Q1 0x8A
#define EXT_CHFILT_Q0 0x8B
#define EXT_GPIO_STATUS 0x8C
#define EXT_FSCAL_CTRL 0x8D
#define EXT_PHASE_ADJUST 0x8E
#define EXT_PARTNUMBER 0x8F
#define EXT_PARTVERSION 0x90
#define EXT_SERIAL_STATUS 0x91
#define EXT_MODEM_STATUS1 0x92
#define EXT_MODEM_STATUS0 0x93
#define EXT_MARC_STATUS1 0x94
#define EXT_MARC_STATUS0 0x95
#define EXT_PA_IFAMP_TEST 0x96
#define EXT_FSRF_TEST 0x97
#define EXT_PRE_TEST 0x98
#define EXT_PRE_OVR 0x99
#define EXT_ADC_TEST 0x9A
#define EXT_DVC_TEST 0x9B
#define EXT_ATEST 0x9C
#define EXT_ATEST_LVDS 0x9D
#define EXT_ATEST_MODE 0x9E
#define EXT_XOSC_TEST1 0x9F
#define EXT_XOSC_TEST0 0xA0
#define EXT_RXFIRST 0xD2
#define EXT_TXFIRST 0xD3
#define EXT_RXLAST 0xD4
#define EXT_TXLAST 0xD5
#define EXT_NUM_TXBYTES 0xD6
#define EXT_NUM_RXBYTES 0xD7
#define EXT_FIFO_NUM_TXBYTES 0xD8
#define EXT_FIFO_NUM_RXBYTES 0xD9

#define AGC_CFG1_INIT 0xa0
#define AGC_CS_THR_INIT 0x19
#define AGC_REF_INIT 0x20
#define DCFILT_CFG_INIT 0x1c
#define FREQ_IF_CFG_INIT 0x33
#define FS_CAL0_INIT 0x0e
#define FS_CFG_INIT 0x14
#define FS_DIG0_INIT 0x5f
#define FS_DIG1_INIT 0x00
#define FS_DIVTWO_INIT 0x03
#define FS_DSM0_INIT 0x33
#define FS_DVC0_INIT 0x17
#define FS_PFD_INIT 0x50
#define FS_PRE_INIT 0x6e
#define FS_REG_DIV_CML_INIT 0x14
#define FS_SPARE_INIT 0xac
#define IF_ADC0_INIT 0x05
#define IF_MIX_CFG_INIT 0x00
#define MDMCFG0_INIT 0x05
#define XOSC1_INIT 0x07
#define XOSC3_INIT 0xc7
#define XOSC5_INIT 0x0e
#define PA_CFG0_INIT 0x7E

#define FREQOFF_CFG_INIT 0x33
#define SYNC_CFG1_INIT 0x08
#define SYNC_CFG0_INIT 0x08
#define PREAMBLE_CFG1_INIT 0x14 //can reduce the number of preamble bits to be faster but less secure)
#define PREAMBLE_CFG0_INIT 0x2A
#define IQIC_INIT 0xC6 // channel bandwitdh 19.2KHz
#define CHAN_BW_INIT 0x0D
#define SYMBOL_RATE0_INIT 0x10 // symbol rate 9.6ksps
#define SYMBOL_RATE1_INIT 0x75
#define SYMBOL_RATE2_INIT 0x6F
#define MODCFG_DEV_E_INIT 0x0A // modulation 2-gfsk
#define DEVIATION_M_INIT 0xF7  // deviation 4.8kHz
#define FREQOFF0_INIT 0x40     // diferent for each module
#define FREQOFF1_INIT 0x02

#define PKT_CFG0_INIT 0x00
#define PKT_CFG2_INIT 0x08
#ifdef APPENDED
#define FIFO_CFG_INIT (0x80 | CC_PACKET)
#define PKT_CFG1_INIT 0x45
#else
#define FIFO_CFG_INIT (0x80 | CC_MAX_PACKET_DATA_SIZE)
#define PKT_CFG1_INIT 0x44
#endif

//#define RFEND_CFG1_INIT 0x3F
//#define RFEND_CFG0_INIT 0x30

#define IOCFG0_INIT 0x41
#define IOCFG1_INIT 0xb0
#define IOCFG2_INIT 0x06
#define IOCFG3_INIT 0xb0
#define AGC_GAIN_ADJUST_INIT 12

#define INT_PRIORITY_CC 5
//#define CC_BROADCAST_ADDR   0xFFFF    // FFFF broadcast
//#define CC_MY_ADDR        0x3958    //"9X"
#define CC_SCK 1000000
#define CC_BRG ((PBCLK / (2 * CC_SCK)) - 1)

#define CC1125_CHANNEL_MIN 1
#define CC1125_CHANNEL_MAX 69

#define Set(x)             \
  {                        \
    digitalWrite(x, HIGH); \
  }
#define Clr(x)            \
  {                       \
    digitalWrite(x, LOW); \
  }

//SPI 0
#define CC_RESET_SENDER 3
#define CC_IO0_SENDER 2
#define CC_IO1_SENDER 12
#define SS_SENDER 10

//SPI 1
#define CC_RESET_RECEIVER 7
#define CC_IO0_RECEIVER 22
#define CC_IO1_RECEIVER 5
#define CC_IO2_RECEIVER 4
#define SS_RECEIVER 6
#define SCK_RECEIVER 20
#define MOSI_RECEIVER 21
#define MISO_RECEIVER 5

SPISettings settingsA(1000000, MSBFIRST, SPI_MODE0);

//438.025 MHz - 439.975 MHz
//const int channelListCC[40] = {
//  0x579AE1, 0x579D70, 0x57A000, 0x57A28F, 0x57A51E, 0x57A7AE, 0x57AA3D, 0x57ACCC, 0x57AF5C, 0x57B1EB,
//  0x57B47A, 0x57B70A, 0x57B999, 0x57BC28, 0x57BEB8, 0x57C147, 0x57C3D7, 0x57C666, 0x57C8F5, 0x57CB85,
//  0x57CE14, 0x57D0A3, 0x57D333, 0x57D5C2, 0x57D851, 0x57DAE1, 0x57DD70, 0x57E000, 0x57E28F, 0x57E51E,
//  0x57E7AE, 0x57EA3D, 0x57ECCC, 0x57EF5C, 0x57F1EB, 0x57F47A, 0x57F70A, 0x57F999, 0x57FC28, 0x57FEB8
//};

const long int channelListCC[69] = {
  0x569D70, 0x569EB8, 0x56A000, 0x56A147, 0x56A28F, 0x56A3D7, 0x56A51E, 0x56A666, 0x56A7AE, 0x56A8F5, 0x56AA3D, 0x56AB85,
  0x56ACCC, 0x56AE14, 0x56AF5C, 0x56B0A3, 0x56B1EB, 0x56B333, 0x56B47A, 0x56B5C2, 0x56B70A, 0x56B851, 0x56B999, 0x56BAE1,
  0x56BC28, 0x56BD70, 0x56BEB8, 0x56C000, 0x56C147, 0x56C28F, 0x56C3D7, 0x56C51E, 0x56C666, 0x56C7AE, 0x56C8F5, 0x56CA3D,
  0x56CB85, 0x56CCCC, 0x56CE14, 0x56CF5C, 0x56D0A3, 0x56D1EB, 0x56D333, 0x56D47A, 0x56D5C2, 0x56D70A, 0x56D851, 0x56D999,
  0x56DAE1, 0x56DC28, 0x56DD70, 0x56DEB8, 0x56E000, 0x56E147, 0x56E28F, 0x56E3D7, 0x56E51E, 0x56E666, 0x56E7AE, 0x56E8F5,
  0x56EA3D, 0x56EB85, 0x56ECCC, 0x56EE14, 0x56EF5C, 0x56F0A3, 0x56F1EB, 0x56F333, 0x56F47A
};

typedef struct pins {
  byte spiSS;
  byte spiMOSI;
  byte spiMISO;
  byte spiSCK;
  byte ccReset;
  byte ccIO0;
  byte ccIO1;
  byte ccIO2;
} pins;

typedef struct rfend_cfg {
  byte RFEND_CFG0_INIT;
  byte RFEND_CFG1_INIT;
} rfend_cfg;

void interruptHandlerWrapper();

class Radio
{
  private:
    typedef struct radio_status
    {
      byte rssi;
      unsigned rxGo : 1;
      unsigned txGo : 1;
      unsigned active : 1;
      unsigned : 5;
    } radio_status;

    typedef union radio_packet {
      struct
      {
        byte data[CC_MAX_PACKET_DATA_SIZE];
        byte RSSI;
        byte LQI;
      };
      byte raw[CC_PACKET];
    } radio_packet;

    volatile radio_status Status;
    radio_packet tx, rx;

    inline byte SPIGetSet(byte x);
    static void interruptHandlerWrapper();

    SPIClass spiObject;

    byte RFEND_CFG0_INIT;
    byte RFEND_CFG1_INIT;
    byte spiSS;
    byte spiMOSI;
    byte spiMISO;
    byte spiSCK;
    byte ccReset;
    byte ccIO1;
    byte ccIO2;
    String radioName;

  public :

    Radio(SPIClass &, const pins &pinsStruct, String _radioName, const rfend_cfg &RFEND_CFG); // : spiObject(_spiObject) {};
    void configurator(int);
    void WriteReg(byte, byte);
    void WriteExtendedReg(byte, byte);
    void WriteStrobe(byte);
    void WriteFIFO(byte *data, byte bytesToWrite);
    byte ReadStatus();
    byte ReadReg(byte);
    byte ReadExtendedReg(byte);
    void ReadFIFO(byte *, byte);

    void interruptHandler(void);
    void cc1125Process();
    void cc1125Tx(byte *, byte);
    void cc1125Disable();

    void printMemberName();
    byte ccIO0;
};

Radio::Radio(SPIClass &_spiObject, const pins &pinsStruct, String _radioName, const rfend_cfg &RFEND_CFG) :
  spiObject(_spiObject),
  RFEND_CFG0_INIT(RFEND_CFG.RFEND_CFG0_INIT),
  RFEND_CFG1_INIT(RFEND_CFG.RFEND_CFG1_INIT),
  spiSS(pinsStruct.spiSS),
  spiMOSI(pinsStruct.spiMOSI),
  spiMISO(pinsStruct.spiMISO),
  spiSCK(pinsStruct.spiSCK),
  ccReset(pinsStruct.ccReset),
  ccIO1(pinsStruct.ccIO1),
  ccIO2(pinsStruct.ccIO2),
  radioName(_radioName),
  ccIO0(pinsStruct.ccIO0)
{
  pinMode (spiSS, OUTPUT);
  pinMode (ccIO0, INPUT_PULLUP);
  pinMode (ccIO2, INPUT_PULLUP);
  pinMode (ccReset, OUTPUT);
  spiObject.setMOSI(spiMOSI);
  spiObject.setMISO(spiMISO);
  spiObject.setSCK(spiSCK);
}

void Radio::configurator(int channel)
{
  int i = 0xffff, timeout = 5;
  Set(ccReset);
  Clr(spiSS);
  Status.rxGo = 0;
  Status.txGo = 0;
  Status.active = 0;

  while (ccIO1 == 1) {
    i--;
    if (i <= 0)
      return;
  }
  Set(spiSS);

  spiObject.begin();
  if (channel < 1)
    channel = 1;
  else if (channel > 40)
    channel = 40;
  channel--;

  WriteReg(REG_IOCFG3, IOCFG3_INIT);
  WriteReg(REG_IOCFG2, IOCFG2_INIT);
  WriteReg(REG_IOCFG1, IOCFG1_INIT);
  WriteReg(REG_IOCFG0, IOCFG0_INIT);
  WriteReg(REG_SYNC_CFG1, SYNC_CFG1_INIT);
  WriteReg(REG_SYNC_CFG0, SYNC_CFG0_INIT);
  //WriteReg(REG_PREAMBLE_CFG1, PREAMBLE_CFG1_INIT);
  //WriteReg(REG_PREAMBLE_CFG0, PREAMBLE_CFG0_INIT);
  WriteReg(REG_DEVIATION_M, DEVIATION_M_INIT);
  WriteReg(REG_MODCFG_DEV_E, MODCFG_DEV_E_INIT);
  WriteReg(REG_DCFILT_CFG, DCFILT_CFG_INIT);
  WriteReg(REG_FREQ_IF_CFG, FREQ_IF_CFG_INIT);
  WriteReg(REG_IQIC, IQIC_INIT);
  WriteReg(REG_CHAN_BW, CHAN_BW_INIT);
  WriteReg(REG_MDMCFG0, MDMCFG0_INIT);
  WriteReg(REG_SYMBOL_RATE2, SYMBOL_RATE2_INIT);
  WriteReg(REG_SYMBOL_RATE1, SYMBOL_RATE1_INIT);
  WriteReg(REG_SYMBOL_RATE0, SYMBOL_RATE0_INIT);
  WriteReg(REG_AGC_REF, AGC_REF_INIT);
  WriteReg(REG_AGC_CS_THR, AGC_CS_THR_INIT);
  WriteReg(REG_AGC_CFG1, AGC_CFG1_INIT);
  WriteReg(REG_FIFO_CFG, FIFO_CFG_INIT);
  WriteReg(REG_FS_CFG, FS_CFG_INIT);
  WriteReg(REG_PKT_CFG0, PKT_CFG0_INIT);
  WriteReg(REG_PKT_CFG1, PKT_CFG1_INIT);
  WriteReg(REG_PKT_CFG2, PKT_CFG2_INIT);
  WriteReg(REG_PKT_LEN, CC_MAX_PACKET_DATA_SIZE);


  WriteReg(REG_RFEND_CFG1, RFEND_CFG1_INIT);
  WriteReg(REG_RFEND_CFG0, RFEND_CFG0_INIT);

  WriteReg(REG_AGC_GAIN_ADJUST, AGC_GAIN_ADJUST_INIT);
  WriteReg(REG_PA_CFG0, PA_CFG0_INIT);

  WriteExtendedReg(EXT_IF_MIX_CFG, IF_MIX_CFG_INIT);
  WriteExtendedReg(EXT_FREQOFF_CFG, FREQOFF_CFG_INIT);
  WriteExtendedReg(EXT_FREQ2, channelListCC[channel] >> 16);
  WriteExtendedReg(EXT_FREQ1, channelListCC[channel] >> 8);
  WriteExtendedReg(EXT_FREQ0, channelListCC[channel]);
  WriteExtendedReg(EXT_FREQOFF0, FREQOFF0_INIT);
  WriteExtendedReg(EXT_FREQOFF1, FREQOFF1_INIT);
  WriteExtendedReg(EXT_IF_ADC0, IF_ADC0_INIT);
  WriteExtendedReg(EXT_FS_DIG1, FS_DIG1_INIT);
  WriteExtendedReg(EXT_FS_DIG0, FS_DIG0_INIT);
  WriteExtendedReg(EXT_FS_CAL0, FS_CAL0_INIT);
  WriteExtendedReg(EXT_FS_DIVTWO, FS_DIVTWO_INIT);
  WriteExtendedReg(EXT_FS_DSM0, FS_DSM0_INIT);
  WriteExtendedReg(EXT_FS_DVC0, FS_DVC0_INIT);
  WriteExtendedReg(EXT_FS_PFD, FS_PFD_INIT);
  WriteExtendedReg(EXT_FS_PRE, FS_PRE_INIT);
  WriteExtendedReg(EXT_FS_REG_DIV_CML, FS_REG_DIV_CML_INIT);
  WriteExtendedReg(EXT_FS_SPARE, FS_SPARE_INIT);
  WriteExtendedReg(EXT_XOSC5, XOSC5_INIT);
  WriteExtendedReg(EXT_XOSC3, XOSC3_INIT);
  WriteExtendedReg(EXT_XOSC1, XOSC1_INIT);

  do
  {
    i = 0xfff;
    while (i > 0)
      i--;
    i = ReadStatus();
    if (timeout <= 0)
      return;
    timeout--;
  } while ((i & 0xFF) != 0x0F);

  WriteStrobe(STROBE_STX); //Enable TX
  Status.active = 1;
}

inline byte Radio::SPIGetSet(byte x)
{
  spiObject.beginTransaction(settingsA);
  x = spiObject.transfer(x); // send test string
  spiObject.endTransaction();
  return x;
}

void Radio::WriteReg(byte reg, byte data)
{
  Clr(spiSS);
  SPIGetSet(reg);
  SPIGetSet(data);
  Set(spiSS);
}

void Radio::WriteExtendedReg(byte extReg, byte data)
{
  Clr(spiSS);
  SPIGetSet(EXTENDED_ADDRESS);
  SPIGetSet(extReg);
  SPIGetSet(data);
  Set(spiSS);
}

void Radio::WriteStrobe(byte reg)
{
  Clr(spiSS);
  SPIGetSet(reg);
  Set(spiSS);
}

void Radio::WriteFIFO(byte *data, byte bytesToWrite)
{
  int k;
  Clr(spiSS);
  SPIGetSet(FIFOS);
  Serial.println("Write FIFO " + radioName);
  for (k = 0; k < bytesToWrite; k++) {
    SPIGetSet(data[k]);
    Serial.print(data[k]);
    Serial.print("\t");
  }
  Serial.println();
  Set(spiSS);
}

byte Radio::ReadReg(byte reg)
{
  Serial.print("Read register " + radioName);
  byte x;
  Clr(spiSS);
  SPIGetSet(reg | 0x80);
  x = SPIGetSet(0);
  Serial.println(x);
  Set(spiSS);
  return x;
}

byte Radio::ReadExtendedReg(byte extReg)
{
  byte x;
  Clr(spiSS);
  Serial.print("Read Extended register: " + radioName + ": " );
  SPIGetSet(EXTENDED_ADDRESS | 0x80);
  SPIGetSet(extReg);
  x = SPIGetSet(0);
  Serial.println(x);
  Set(spiSS);
  return x;
}

void Radio::ReadFIFO(byte *data, byte bytesToRead)
{
  int k;
  Clr(spiSS);
  Serial.print("Read FIFO " + radioName + " ");
  SPIGetSet(FIFOS | 0x80);
  for (k = 0; k < bytesToRead; k++) {
    data[k] = SPIGetSet(0);
    Serial.print(data[k]);
    Serial.print("\t");
  }
  bytearr2int(data, k, sizeof(data));
  Serial.println(millis() - k);
  Set(spiSS);
}

byte Radio::ReadStatus()
{
  byte s;

  Clr(spiSS);
  Serial.print("Read STATUS " + radioName + " ");
  s = SPIGetSet(STROBE_SNOP);
  s = SPIGetSet(STROBE_SNOP);
  s &= (~0x80);
  s = s >> 4;
  printStatus(s);
  Set(spiSS);
  return s;
}

void Radio::interruptHandler(void)
{
  if (digitalRead(ccIO0) == 0)
  {
    int num_rx_bytes = ReadExtendedReg(EXT_NUM_RXBYTES);
    ReadFIFO(rx.raw, num_rx_bytes);
#ifdef APPENDED
    if (num_rx_bytes >= CC_PACKET)
      Status.rxGo = 1;
#else
    if (num_rx_bytes >= CC_MAX_PACKET_DATA_SIZE)
      Status.rxGo = 1;
#endif
  }

  if (Status.txGo)
  {
    WriteStrobe(STROBE_SIDLE);                  //Put in idle
    WriteStrobe(STROBE_SFTX);                   //Flux TX
    WriteStrobe(STROBE_STX);                    //Enable TX
    WriteFIFO(tx.raw, CC_MAX_PACKET_DATA_SIZE); //Write to FIFO
    Status.txGo = 0;
  }
  Serial.println("Asserted " + radioName);
}

void Radio::cc1125Process()
{
  if (Status.rxGo)
  {
    Status.rxGo = 0;
    if (rx.RSSI > 120)
      rx.RSSI = 0;
    Status.rssi = rx.RSSI;
  }
}

void Radio::cc1125Tx(byte *data, byte length)
{
  int s;
  if (!Status.active)
    return;
  if (length > CC_MAX_PACKET_DATA_SIZE)
    return;
  for (s = 0; s < length; s++)
    tx.data[s] = data[s];
  Status.txGo = 1;
}

void Radio::cc1125Disable()
{
  Clr(ccReset);
  Status.active = 0;
  Status.rssi = 0;
}

void Radio::printMemberName() {
  Serial.println((int)spiSS);
  Serial.println((int)spiMOSI);
  Serial.println((int)spiMISO);
  Serial.println((int)spiSCK);
  Serial.println((int)ccReset);
  Serial.println((int)ccIO0);
  Serial.println((int)ccIO1);
  Serial.println((int)ccIO2);
  Serial.println((int)RFEND_CFG0_INIT);
  Serial.println((int)RFEND_CFG1_INIT);
}

#ifdef SENDER
rfend_cfg sender_rfend_cfg  = {0x20, 0x2f};
pins sender_pins =  {10, 11, 12, 13, 3, 2, 12, 14};
Radio sender(SPI, sender_pins, "sender", sender_rfend_cfg);

void senderWrapper() {
  sender.interruptHandler();
}
#endif

#ifdef RECEIVER
rfend_cfg receiver_rfend_cfg  = {0x30, 0x3f};
pins receiver_pins = {6, 21, 5, 20, 7, 22, 5, 4};
Radio receiver(SPI1, receiver_pins, "receiver", receiver_rfend_cfg);

void receiverWrapper() {
  receiver.interruptHandler();
}
#endif
