/* 
 * File:   CC1125.c
 * Author: Pedro Gois
 *
 * Created on 10 de Outubro de 2014, 17:56
 */

#include "CC1125.h"

//438.025 MHz - 439.975 MHz
//const int channelListCC[40] = {
//	0x579AE1, 0x579D70, 0x57A000, 0x57A28F, 0x57A51E, 0x57A7AE, 0x57AA3D, 0x57ACCC, 0x57AF5C, 0x57B1EB,
//	0x57B47A, 0x57B70A, 0x57B999, 0x57BC28, 0x57BEB8, 0x57C147, 0x57C3D7, 0x57C666, 0x57C8F5, 0x57CB85,
//	0x57CE14, 0x57D0A3, 0x57D333, 0x57D5C2, 0x57D851, 0x57DAE1, 0x57DD70, 0x57E000, 0x57E28F, 0x57E51E,
//	0x57E7AE, 0x57EA3D, 0x57ECCC, 0x57EF5C, 0x57F1EB, 0x57F47A, 0x57F70A, 0x57F999, 0x57FC28, 0x57FEB8
//};

const int channelListCC[69] = {
	0x569D70, 0x569EB8, 0x56A000, 0x56A147, 0x56A28F, 0x56A3D7, 0x56A51E, 0x56A666, 0x56A7AE, 0x56A8F5, 0x56AA3D, 0x56AB85,
	0x56ACCC, 0x56AE14, 0x56AF5C, 0x56B0A3, 0x56B1EB, 0x56B333, 0x56B47A, 0x56B5C2, 0x56B70A, 0x56B851, 0x56B999, 0x56BAE1,
	0x56BC28, 0x56BD70, 0x56BEB8, 0x56C000, 0x56C147, 0x56C28F, 0x56C3D7, 0x56C51E, 0x56C666, 0x56C7AE, 0x56C8F5, 0x56CA3D,
	0x56CB85, 0x56CCCC, 0x56CE14, 0x56CF5C, 0x56D0A3, 0x56D1EB, 0x56D333, 0x56D47A, 0x56D5C2, 0x56D70A, 0x56D851, 0x56D999,
	0x56DAE1, 0x56DC28, 0x56DD70, 0x56DEB8, 0x56E000, 0x56E147, 0x56E28F, 0x56E3D7, 0x56E51E, 0x56E666, 0x56E7AE, 0x56E8F5,
	0x56EA3D, 0x56EB85, 0x56ECCC, 0x56EE14, 0x56EF5C, 0x56F0A3, 0x56F1EB, 0x56F333, 0x56F47A
};

typedef union _ccpak
{
    struct
	{
		byte data[CC_MAX_PACKET_DATA_SIZE];
        byte RSSI;
		byte LQI;
    };
    byte raw[CC_MAX_PACKET_DATA_SIZE +2];
}CC_Packet;
CC_Packet tx, rx;

volatile CC1125_STATUS ccStatus;

inline void SPI1_Init()
{
    int c;

    Clr(SPI1CONbits.ENHBUF);
    Clr(SPI1CONbits.SMP);
    Set(SPI1CONbits.CKE);
    Clr(SPI1CONbits.CKP);
    Set(SPI1CONbits.MSTEN);
    SPI1BRG = CC_BRG;
    Set(SPI1CONbits.ON);
    c = SPI1BUF;
}



inline byte SPI1GetSet(byte x)
{
    while(SPI1STATbits.SPITBF);
    SPI1BUF = x;
    while(!SPI1STATbits.SPIRBF);
    return SPI1BUF;
}



void WriteReg(byte reg, byte data)
{
	Clr(CC_CS);
	SPI1GetSet(reg);
	SPI1GetSet(data);
	Set(CC_CS);
}


void WriteExtendedReg(byte extReg, byte data)
{
	Clr(CC_CS);
	SPI1GetSet(EXTENDED_ADDRESS);
	SPI1GetSet(extReg);
	SPI1GetSet(data);
	Set(CC_CS);
}



void WriteStrobe(byte reg)
{
	Clr(CC_CS);
	SPI1GetSet(reg);
	Set(CC_CS);
}



void WriteFIFO(byte *data, byte bytesToWrite)
{
	int k;

	Clr(CC_CS);
	SPI1GetSet(FIFOS);
	for(k = 0; k < bytesToWrite; k++)
	{
		SPI1GetSet(data[k]);
	}
	Set(CC_CS);
}



byte ReadReg(byte reg)
{
	byte x;
	
	Clr(CC_CS);
	SPI1GetSet(reg | 0x80);
	x = SPI1GetSet(0);
	Set(CC_CS);
	return x;
}



byte ReadExtendedReg(byte extReg)
{
	byte x;
	
	Clr(CC_CS);
	SPI1GetSet(EXTENDED_ADDRESS | 0x80);
	SPI1GetSet(extReg);
	x = SPI1GetSet(0);
	Set(CC_CS);
	return x;
}



void ReadFIFO(byte *data, byte bytesToRead)
{
	int k;

	Clr(CC_CS);
	SPI1GetSet(FIFOS | 0x80);
	for(k = 0; k < bytesToRead; k++)
	{
		data[k] = SPI1GetSet(0);
	}
	Set(CC_CS);
}



byte ReadStatus()
{
	byte s;
	
	Clr(CC_CS);
	s = SPI1GetSet(STROBE_SNOP);
	s = SPI1GetSet(STROBE_SNOP);
	Set(CC_CS);
	return s;
}



void CC1125_Init(byte channel)
{
	int i = 0xffff, timeout = 5;

	Clr(IEC0bits.INT3IE);
	Set(CC_RESET);
	Clr(CC_CS);
	Clr(ccStatus.rxGo);
    Clr(ccStatus.txGo);
	Clr(ccStatus.active);
    while(CC_IO1 == 1){
		i--;
		if(i <= 0) return;
	}
	Set(CC_CS);
	SPI1_Init();
	
	if(channel < 1) channel = 1;
	else if(channel > 40) channel = 40;
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
	
	WriteReg(REG_RFEND_CFG1, 0x2f);//RFEND_CFG1_INIT);
	WriteReg(REG_RFEND_CFG0, 0x20);//RFEND_CFG0_INIT);
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
        while(i > 0) i--;
        i = ReadStatus();
        if(timeout <= 0) return;
        timeout--;
    }
    while((i & 0xFF) != 0x0F);
	
	WriteStrobe(STROBE_SRX);	// start RX
	Set(ccStatus.active);
	Clr(INTCONbits.INT3EP);		//falling edge
    Clr(IFS0bits.INT3IF);
	IPC3bits.INT3IP = INT_PRIORITY_CC;
	Set(IEC0bits.INT3IE);
}



void __ISR(_EXTERNAL_3_VECTOR) CC_Handler(void)
{
	Clr(IFS0bits.INT3IF);
	
	if(CC_IO0 == 0)
	{
		int num_rx_bytes = ReadExtendedReg(EXT_NUM_RXBYTES);
		ReadFIFO(rx.raw, num_rx_bytes);
		if(num_rx_bytes >= (CC_MAX_PACKET_DATA_SIZE +2)) Set(ccStatus.rxGo);
	}
	if(ccStatus.txGo)
	{
		//while(CC_IO2);
        WriteStrobe(STROBE_SIDLE);
		WriteStrobe(STROBE_SFTX);
		WriteStrobe(STROBE_STX);
		WriteFIFO(tx.raw, CC_MAX_PACKET_DATA_SIZE);
		Clr(ccStatus.txGo);
	}
}



void CC1125_Process()
{
	if(ccStatus.rxGo)
    {
		Clr(ccStatus.rxGo);
		if(rx.RSSI > 120) rx.RSSI = 0;
		ccStatus.rssi = rx.RSSI;
//		CC1125_RX(rx.data, CC_MAX_PACKET_DATA_SIZE);
    }
}



void CC1125_TX(byte *data, byte length)
{
    int s;

	if(!ccStatus.active) return;
	if(length > CC_MAX_PACKET_DATA_SIZE) return;
    for(s = 0; s < length; s++) tx.data[s] = data[s];
	Set(ccStatus.txGo);
	Set(IFS0bits.INT3IF);
}



void CC1125_Disable()
{
	Clr(IEC0bits.INT3IE);
	Clr(IFS0bits.INT3IF);
	Clr(CC_RESET);
	Clr(SPI3CONbits.ON);
	Clr(ccStatus.active);
	ccStatus.rssi = 0;
}
