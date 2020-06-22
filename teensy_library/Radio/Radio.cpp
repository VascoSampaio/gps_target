/*
  Radio.cpp - Library for interacting with CC1125 transceiver.
  Created by Vasco G. Sampaio, April 2, 2020.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Radio.h"

void printStatus(byte inByte)
{
  const char *s = 0;
  inByte &= (~0x80);
  inByte = inByte >> 4;

  switch (inByte)
  {
    PROCESS_VAL(IDLER);
    PROCESS_VAL(RX);
    PROCESS_VAL(TX);
    PROCESS_VAL(FSTXON);
    PROCESS_VAL(CALIBRATE);
    PROCESS_VAL(SETTLING);
    PROCESS_VAL(RXFIFOERROR);
    PROCESS_VAL(TXFIFOERROR);
  }
  Serial.println(s);
}

Radio::Radio(SPIClass &_spiObject, SPISettings &_spisettingsObject, const pins &pinsStruct, String _radioName, const rfend_cfg &RFEND_CFG) : spiObject(_spiObject),
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
  pinMode(spiSS, OUTPUT);
  pinMode(ccIO0, INPUT_PULLUP);
  pinMode(ccIO2, INPUT_PULLUP);
  pinMode(ccReset, OUTPUT);
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

  while (ccIO1 == 1)
  {
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
  //Serial.println("Write FIFO " + radioName);
  for (k = 0; k < bytesToWrite; k++)
  {
    SPIGetSet(data[k]);
    //Serial.print(data[k]);
    //Serial.print("\t");
  }
  //Serial.println();
  Set(spiSS);
}

byte Radio::ReadReg(byte reg)
{
  //Serial.print("Read register " + radioName);
  byte x;
  Clr(spiSS);
  SPIGetSet(reg | 0x80);
  x = SPIGetSet(0);
  //Serial.println(x);
  Set(spiSS);
  return x;
}

byte Radio::ReadExtendedReg(byte extReg)
{
  byte x;
  Clr(spiSS);
  //Serial.print("Read Extended register: " + radioName + ": ");
  SPIGetSet(EXTENDED_ADDRESS | 0x80);
  SPIGetSet(extReg);
  x = SPIGetSet(0);
  //Serial.println(x);
  Set(spiSS);
  return x;
}

void Radio::ReadFIFO(byte *data, byte bytesToRead)
{
  int k;
  double a = 80;
  Clr(spiSS);
  //Serial.print("Read FIFO " + radioName + " ");
  SPIGetSet(FIFOS | 0x80);
  //a = millis();
  for (k = 0; k < bytesToRead; k++)
  {
    data[k] = SPIGetSet(0);
    //a = (int) data[k];
    Serial.write(data[k]);
    //Serial.print(",");
  }
  //a = millis()-a;
  //Serial.println();
  //Serial.println(a);
  Set(spiSS);
}

byte Radio::ReadStatus()
{
  byte s;

  Clr(spiSS);
  //Serial.print("Read STATUS " + radioName + " ");
  s = SPIGetSet(STROBE_SNOP);
  s = SPIGetSet(STROBE_SNOP);
  Set(spiSS);
  //printStatus(s);
  return s;
}

void Radio::interruptHandler(void)
{

  if (digitalRead(ccIO0) == 0)
  {
    int num_rx_bytes = ReadExtendedReg(EXT_NUM_RXBYTES);
    //Serial.println(num_rx_bytes);
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
  Serial.println();
  //Serial.println("Asserted " + radioName);
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

void Radio::printMemberName()
{
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
