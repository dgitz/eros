/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-07-29 08:09:41.999248***/
/***Target: Arduino ***/
#include "spimessage.h"
int encode_TestMessageCounterSPI(unsigned char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = value1;
	*p_outbuffer++ = value2;
	*p_outbuffer++ = value3;
	*p_outbuffer++ = value4;
	*p_outbuffer++ = value5;
	*p_outbuffer++ = value6;
	*p_outbuffer++ = value7;
	*p_outbuffer++ = value8;
	*p_outbuffer++ = value9;
	*p_outbuffer++ = value10;
	*p_outbuffer++ = value11;
	*p_outbuffer++ = value12;
	unsigned char checksum = 0;
	for(int i = 0; i < 12;i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	length[0] = 12;
	return 1;
}
int encode_Get_DIO_Port1SPI(unsigned char* outbuffer,int* length,unsigned int EncoderA_TickSpeed_Offset,unsigned int EncoderB_TickSpeed_Offset)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = EncoderA_TickSpeed_Offset>>8;
	*p_outbuffer++ = EncoderA_TickSpeed_Offset;
	*p_outbuffer++ = EncoderB_TickSpeed_Offset>>8;
	*p_outbuffer++ = EncoderB_TickSpeed_Offset;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	unsigned char checksum = 0;
	for(int i = 0; i < 12;i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	length[0] = 12;
	return 1;
}
int encode_Get_ANA_Port1SPI(unsigned char* outbuffer,int* length,unsigned int Pin1_Value,unsigned int Pin2_Value,unsigned int Pin3_Value,unsigned int Pin4_Value,unsigned int Pin5_Value,unsigned int Pin6_Value)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = Pin1_Value>>8;
	*p_outbuffer++ = Pin1_Value;
	*p_outbuffer++ = Pin2_Value>>8;
	*p_outbuffer++ = Pin2_Value;
	*p_outbuffer++ = Pin3_Value>>8;
	*p_outbuffer++ = Pin3_Value;
	*p_outbuffer++ = Pin4_Value>>8;
	*p_outbuffer++ = Pin4_Value;
	*p_outbuffer++ = Pin5_Value>>8;
	*p_outbuffer++ = Pin5_Value;
	*p_outbuffer++ = Pin6_Value>>8;
	*p_outbuffer++ = Pin6_Value;
	unsigned char checksum = 0;
	for(int i = 0; i < 12;i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	length[0] = 12;
	return 1;
}

int decode_LEDStripControlSPI(unsigned char* inbuffer,int* length,unsigned char checksum,unsigned char * LEDPixelMode,unsigned char * Param1,unsigned char * Param2)
{
	*LEDPixelMode = inbuffer[0];
	*Param1 = inbuffer[1];
	*Param2 = inbuffer[2];
	unsigned char calc_checksum = SPI_LEDStripControl_ID;
	for(int i = 0; i < 12; i++)
	{
		calc_checksum ^= inbuffer[i];
	}
	if(calc_checksum == checksum)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}