/* 
 * File:   CC1125.h
 * Author: Pedro Gois
 *
 * Created on 10 de Outubro de 2014, 17:56
 */
 
#ifndef CC1125_H
#define CC1125_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "HardwareProfile.h"

#define INT_PRIORITY_CC			5
//#define CC_BROADCAST_ADDR		0xFFFF		// FFFF broadcast
//#define CC_MY_ADDR				0x3958		//"9X"
#define CC_MAX_PACKET_DATA_SIZE			36
	
#define CC1125_CHANNEL_MIN		1
#define CC1125_CHANNEL_MAX		69

    
#define EXTENDED_ADDRESS		0x2F
#define DIRECT_MEMORY_ACCESS	0x3E
#define FIFOS					0x7F

#define REG_IOCFG3			0x00
#define REG_IOCFG2			0x01
#define REG_IOCFG1			0x02
#define REG_IOCFG0			0x03
#define REG_SYNC3			0x04
#define REG_SYNC2			0x05
#define REG_SYNC1			0x06
#define REG_SYNC0			0x07
#define REG_SYNC_CFG1		0x08
#define REG_SYNC_CFG0		0x09
#define REG_DEVIATION_M		0x0A
#define REG_MODCFG_DEV_E	0x0B
#define REG_DCFILT_CFG		0x0C
#define REG_PREAMBLE_CFG1	0x0D
#define REG_PREAMBLE_CFG0	0x0E
#define REG_FREQ_IF_CFG		0x0F
#define REG_IQIC			0x10
#define REG_CHAN_BW			0x11
#define REG_MDMCFG1			0x12
#define REG_MDMCFG0			0x13
#define REG_SYMBOL_RATE2	0x14
#define REG_SYMBOL_RATE1	0x15
#define REG_SYMBOL_RATE0	0x16
#define REG_AGC_REF			0x17
#define REG_AGC_CS_THR		0x18
#define REG_AGC_GAIN_ADJUST	0x19
#define REG_AGC_CFG3		0x1A
#define REG_AGC_CFG2		0x1B
#define REG_AGC_CFG1		0x1C
#define REG_AGC_CFG0		0x1D
#define REG_FIFO_CFG		0x1E
#define REG_DEV_ADDR		0x1F
#define REG_SETTLING_CFG	0x20
#define REG_FS_CFG			0x21
#define REG_WOR_CFG1		0x22
#define REG_WOR_CFG0		0x23
#define REG_WOR_EVENT0_MSB	0x24
#define REG_WOR_EVENT0_LSB	0x25
#define REG_PKT_CFG2		0x26
#define REG_PKT_CFG1		0x27
#define REG_PKT_CFG0		0x28
#define REG_RFEND_CFG1		0x29
#define REG_RFEND_CFG0		0x2A
#define REG_PA_CFG2			0x2B
#define REG_PA_CFG1			0x2C
#define REG_PA_CFG0			0x2D
#define REG_PKT_LEN			0x2E

#define STROBE_SRES		0x30
#define STROBE_SFSTXON	0x31
#define STROBE_SXOFF	0x32
#define STROBE_SCAL		0x33
#define STROBE_SRX		0x34
#define STROBE_STX		0x35
#define STROBE_SIDLE	0x36
#define STROBE_SAFC		0x37
#define STROBE_SWOR		0x38
#define STROBE_SPWD		0x39
#define STROBE_SFRX		0x3A
#define STROBE_SFTX		0x3B
#define STROBE_SWORRST	0x3C
#define STROBE_SNOP		0x3D

#define EXT_IF_MIX_CFG			0x00
#define EXT_FREQOFF_CFG			0x01
#define EXT_TOC_CFG				0x02
#define EXT_MARC_SPARE			0x03
#define EXT_ECG_CFG				0x04
#define EXT_CFM_DATA_CFG		0x05
#define EXT_EXT_CTRL			0x06
#define EXT_RCCAL_FINE			0x07
#define EXT_RCCAL_COARSE		0x08
#define EXT_RCCAL_OFFSET		0x09
#define EXT_FREQOFF1			0x0A
#define EXT_FREQOFF0			0x0B
#define EXT_FREQ2				0x0C
#define EXT_FREQ1				0x0D
#define EXT_FREQ0				0x0E
#define EXT_IF_ADC2				0x0F
#define EXT_IF_ADC1				0x10
#define EXT_IF_ADC0				0x11
#define EXT_FS_DIG1				0x12
#define EXT_FS_DIG0				0x13
#define EXT_FS_CAL3				0x14
#define EXT_FS_CAL2				0x15
#define EXT_FS_CAL1				0x16
#define EXT_FS_CAL0				0x17
#define EXT_FS_CHP				0x18
#define EXT_FS_DIVTWO			0x19
#define EXT_FS_DSM1				0x1A
#define EXT_FS_DSM0				0x1B
#define EXT_FS_DVC1				0x1C
#define EXT_FS_DVC0				0x1D
#define EXT_FS_LBI				0x1E
#define EXT_FS_PFD				0x1F
#define EXT_FS_PRE				0x20
#define EXT_FS_REG_DIV_CML		0x21
#define EXT_FS_SPARE			0x22
#define EXT_FS_VCO4				0x23
#define EXT_FS_VCO3				0x24
#define EXT_FS_VCO2				0x25
#define EXT_FS_VCO1				0x26
#define EXT_FS_VCO0				0x27
#define EXT_GBIAS6				0x28
#define EXT_GBIAS5				0x29
#define EXT_GBIAS4				0x2A
#define EXT_GBIAS3				0x2B
#define EXT_GBIAS2				0x2C
#define EXT_GBIAS1				0x2D
#define EXT_GBIAS0				0x2E
#define EXT_IFAMP				0x2F
#define EXT_LNA					0x30
#define EXT_RXMIX				0x31
#define EXT_XOSC5				0x32
#define EXT_XOSC4				0x33
#define EXT_XOSC3				0x34
#define EXT_XOSC2				0x35
#define EXT_XOSC1				0x36
#define EXT_XOSC0				0x37
#define EXT_ANALOG_SPARE		0x38
#define EXT_PA_CFG3				0x39
#define EXT_WOR_TIME1			0x64
#define EXT_WOR_TIME0			0x65
#define EXT_WOR_CAPTURE1		0x66
#define EXT_WOR_CAPTURE0		0x67
#define EXT_BIST				0x68
#define EXT_DCFILTOFFSET_I1		0x69
#define EXT_DCFILTOFFSET_I0		0x6A
#define EXT_DCFILTOFFSET_Q1		0x6B
#define EXT_DCFILTOFFSET_Q0		0x6C
#define EXT_IQIE_I1				0x6D
#define EXT_IQIE_I0				0x6E
#define EXT_IQIE_Q1				0x6F
#define EXT_IQIE_Q0				0x70
#define EXT_RSSI1				0x71
#define EXT_RSSI0				0x72
#define EXT_MARCSTATE			0x73
#define EXT_LQI_VAL				0x74
#define EXT_PQT_SYNC_ERR		0x75
#define EXT_DEM_STATUS			0x76
#define EXT_FREQOFF_EST1		0x77
#define EXT_FREQOFF_EST0		0x78
#define EXT_AGC_GAIN3			0x79
#define EXT_AGC_GAIN2			0x7A
#define EXT_AGC_GAIN1			0x7B
#define EXT_AGC_GAIN0			0x7C
#define EXT_CFM_RX_DATA_OUT		0x7D
#define EXT_CFM_TX_DATA_IN		0x7E
#define EXT_ASK_SOFT_RX_DATA	0x7F
#define EXT_RNDGEN				0x80
#define EXT_MAGN2				0x81
#define EXT_MAGN1				0x82
#define EXT_MAGN0				0x83
#define EXT_ANG1				0x84
#define EXT_ANG0				0x85
#define EXT_CHFILT_I2			0x86
#define EXT_CHFILT_I1			0x87
#define EXT_CHFILT_I0			0x88
#define EXT_CHFILT_Q2			0x89
#define EXT_CHFILT_Q1			0x8A
#define EXT_CHFILT_Q0			0x8B
#define EXT_GPIO_STATUS			0x8C
#define EXT_FSCAL_CTRL			0x8D
#define EXT_PHASE_ADJUST		0x8E
#define EXT_PARTNUMBER			0x8F
#define EXT_PARTVERSION			0x90
#define EXT_SERIAL_STATUS		0x91
#define EXT_MODEM_STATUS1		0x92
#define EXT_MODEM_STATUS0		0x93
#define EXT_MARC_STATUS1		0x94
#define EXT_MARC_STATUS0		0x95
#define EXT_PA_IFAMP_TEST		0x96
#define EXT_FSRF_TEST			0x97
#define EXT_PRE_TEST			0x98
#define EXT_PRE_OVR				0x99
#define EXT_ADC_TEST			0x9A
#define EXT_DVC_TEST			0x9B
#define EXT_ATEST				0x9C
#define EXT_ATEST_LVDS			0x9D
#define EXT_ATEST_MODE			0x9E
#define EXT_XOSC_TEST1			0x9F
#define EXT_XOSC_TEST0			0xA0
#define EXT_RXFIRST				0xD2
#define EXT_TXFIRST				0xD3
#define EXT_RXLAST				0xD4
#define EXT_TXLAST				0xD5
#define EXT_NUM_TXBYTES			0xD6
#define EXT_NUM_RXBYTES			0xD7
#define EXT_FIFO_NUM_TXBYTES	0xD8
#define EXT_FIFO_NUM_RXBYTES	0xD9



#define AGC_CFG1_INIT		0xa0
#define AGC_CS_THR_INIT		0x19
#define AGC_REF_INIT		0x20
#define DCFILT_CFG_INIT		0x1c
#define FREQ_IF_CFG_INIT	0x33
#define FS_CAL0_INIT		0x0e
#define FS_CFG_INIT			0x14
#define FS_DIG0_INIT		0x5f
#define FS_DIG1_INIT		0x00
#define FS_DIVTWO_INIT		0x03
#define FS_DSM0_INIT		0x33
#define FS_DVC0_INIT		0x17
#define FS_PFD_INIT			0x50
#define FS_PRE_INIT			0x6e
#define FS_REG_DIV_CML_INIT	0x14
#define FS_SPARE_INIT		0xac
#define IF_ADC0_INIT		0x05
#define IF_MIX_CFG_INIT		0x00
#define MDMCFG0_INIT		0x05
#define XOSC1_INIT			0x07
#define XOSC3_INIT			0xc7
#define XOSC5_INIT			0x0e
#define PA_CFG0_INIT		0x7E

#define FREQOFF_CFG_INIT	0x33
#define SYNC_CFG1_INIT		0x08
#define SYNC_CFG0_INIT		0x08
#define PREAMBLE_CFG1_INIT	0x14
#define PREAMBLE_CFG0_INIT	0x2A
#define IQIC_INIT			0xC6		// channel bandwitdh 19.2KHz
#define CHAN_BW_INIT		0x0D
#define SYMBOL_RATE0_INIT	0x10		// symbol rate 9.6ksps
#define SYMBOL_RATE1_INIT	0x75
#define SYMBOL_RATE2_INIT	0x6F	
#define MODCFG_DEV_E_INIT	0x0A		// modulation 2-gfsk
#define DEVIATION_M_INIT	0xF7		// deviation 4.8kHz
#define FREQOFF0_INIT		0x40		// diferent for each module
#define FREQOFF1_INIT		0x02

#define PKT_CFG0_INIT		0x00
#define PKT_CFG1_INIT		0x45
#define PKT_CFG2_INIT		0x08
#define FIFO_CFG_INIT		(0x80 | (CC_MAX_PACKET_DATA_SIZE +2))
#define RFEND_CFG1_INIT		0x3F
#define RFEND_CFG0_INIT		0x30
#define IOCFG0_INIT			0x41
#define IOCFG1_INIT			0xb0
#define IOCFG2_INIT			0x06
#define IOCFG3_INIT			0xb0
#define AGC_GAIN_ADJUST_INIT	12
    
typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned int dword;


typedef struct _ccsts
{
	byte rssi;
    unsigned rxGo :1;
    unsigned txGo :1;
	unsigned active :1;
	unsigned :5;
}CC1125_STATUS;


inline void SPI1_Init();
inline byte SPI1GetSet(byte);
void WriteReg(byte, byte);
void WriteExtendedReg(byte, byte);
void WriteStrobe(byte);
void WriteFIFO(byte*, byte);
byte ReadReg(byte);
byte ReadExtendedReg(byte);
void ReadFIFO(byte*, byte);
byte ReadStatus();
void CC1125_Init(byte channel);
void CC1125_Process();
void CC1125_TX(byte *data, byte length);
void CC1125_RX(byte *data, byte length);
void CC1125_Disable();



#ifdef	__cplusplus
}
#endif

#endif	/* CC1125_H */


