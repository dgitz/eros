/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-08-23 07:05:58.972954***/
/***Target: Arduino ***/
#include "i2cmessage.h"
int encode_DiagnosticI2C(unsigned char* outbuffer,int* length,unsigned char System,unsigned char SubSystem,unsigned char Component,unsigned char Diagnostic_Type,unsigned char Level,unsigned char Diagnostic_Message)
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
int encode_TestMessageCounterI2C(unsigned char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12)
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
int encode_Get_DIO_Port1I2C(unsigned char* outbuffer,int* length,unsigned int u1,unsigned int u2,unsigned int u3,unsigned int u4)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = u1>>8;
	*p_outbuffer++ = u1;
	*p_outbuffer++ = u2>>8;
	*p_outbuffer++ = u2;
	*p_outbuffer++ = u3>>8;
	*p_outbuffer++ = u3;
	*p_outbuffer++ = u4>>8;
	*p_outbuffer++ = u4;
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
