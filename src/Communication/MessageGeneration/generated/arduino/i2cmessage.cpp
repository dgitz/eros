/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2019-10-20 03:43:36.664920***/
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
int encode_Get_ANA_Port1I2C(unsigned char* outbuffer,int* length,unsigned int Pin1_Value,unsigned int Pin2_Value,unsigned int Pin3_Value,unsigned int Pin4_Value,unsigned int Pin5_Value,unsigned int Pin6_Value)
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
int encode_Get_ANA_Port2I2C(unsigned char* outbuffer,int* length,unsigned int Pin1_Value,unsigned int Pin2_Value,unsigned int Pin3_Value,unsigned int Pin4_Value,unsigned int Pin5_Value,unsigned int Pin6_Value)
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
int encode_Get_IMUAccI2C(unsigned char* outbuffer,int* length,unsigned int acc1_x,unsigned int acc1_y,unsigned int acc1_z,unsigned int acc2_x,unsigned int acc2_y,unsigned int acc2_z)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = acc1_x>>8;
	*p_outbuffer++ = acc1_x;
	*p_outbuffer++ = acc1_y>>8;
	*p_outbuffer++ = acc1_y;
	*p_outbuffer++ = acc1_z>>8;
	*p_outbuffer++ = acc1_z;
	*p_outbuffer++ = acc2_x>>8;
	*p_outbuffer++ = acc2_x;
	*p_outbuffer++ = acc2_y>>8;
	*p_outbuffer++ = acc2_y;
	*p_outbuffer++ = acc2_z>>8;
	*p_outbuffer++ = acc2_z;
	unsigned char checksum = 0;
	for(int i = 0; i < 12;i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	length[0] = 12;
	return 1;
}
int encode_Get_IMUGyroI2C(unsigned char* outbuffer,int* length,unsigned int gyro1_x,unsigned int gyro1_y,unsigned int gyro1_z,unsigned int gyro2_x,unsigned int gyro2_y,unsigned int gyro2_z)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = gyro1_x>>8;
	*p_outbuffer++ = gyro1_x;
	*p_outbuffer++ = gyro1_y>>8;
	*p_outbuffer++ = gyro1_y;
	*p_outbuffer++ = gyro1_z>>8;
	*p_outbuffer++ = gyro1_z;
	*p_outbuffer++ = gyro2_x>>8;
	*p_outbuffer++ = gyro2_x;
	*p_outbuffer++ = gyro2_y>>8;
	*p_outbuffer++ = gyro2_y;
	*p_outbuffer++ = gyro2_z>>8;
	*p_outbuffer++ = gyro2_z;
	unsigned char checksum = 0;
	for(int i = 0; i < 12;i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	length[0] = 12;
	return 1;
}
int encode_Get_IMUMagI2C(unsigned char* outbuffer,int* length,unsigned int mag1_x,unsigned int mag1_y,unsigned int mag1_z,unsigned int mag2_x,unsigned int mag2_y,unsigned int mag2_z)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = mag1_x>>8;
	*p_outbuffer++ = mag1_x;
	*p_outbuffer++ = mag1_y>>8;
	*p_outbuffer++ = mag1_y;
	*p_outbuffer++ = mag1_z>>8;
	*p_outbuffer++ = mag1_z;
	*p_outbuffer++ = mag2_x>>8;
	*p_outbuffer++ = mag2_x;
	*p_outbuffer++ = mag2_y>>8;
	*p_outbuffer++ = mag2_y;
	*p_outbuffer++ = mag2_z>>8;
	*p_outbuffer++ = mag2_z;
	unsigned char checksum = 0;
	for(int i = 0; i < 12;i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	length[0] = 12;
	return 1;
}
