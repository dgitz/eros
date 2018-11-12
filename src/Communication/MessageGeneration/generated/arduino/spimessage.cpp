/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-11-12 06:50:47.053223***/
/***Target: Arduino ***/
#include "spimessage.h"
int encode_DiagnosticSPI(unsigned char* outbuffer,int* length,unsigned char System,unsigned char SubSystem,unsigned char Component,unsigned char Diagnostic_Type,unsigned char Level,unsigned char Diagnostic_Message)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = System;
	*p_outbuffer++ = SubSystem;
	*p_outbuffer++ = Component;
	*p_outbuffer++ = Diagnostic_Type;
	*p_outbuffer++ = Level;
	*p_outbuffer++ = Diagnostic_Message;
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
int encode_Get_DIO_Port1SPI(unsigned char* outbuffer,int* length,unsigned int u1,unsigned int u2)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = u1>>8;
	*p_outbuffer++ = u1;
	*p_outbuffer++ = u2>>8;
	*p_outbuffer++ = u2;
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