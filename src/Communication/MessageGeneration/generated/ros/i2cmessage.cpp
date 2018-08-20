/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-08-19 07:10:30.243560***/
/***Target: Raspberry Pi ***/
#include "../include/i2cmessage.h"
I2CMessageHandler::I2CMessageHandler(){}
I2CMessageHandler::~I2CMessageHandler(){}
int I2CMessageHandler::decode_DiagnosticI2C(unsigned char* inbuffer,int * length,unsigned char* System,unsigned char* SubSystem,unsigned char* Component,unsigned char* Diagnostic_Type,unsigned char* Level,unsigned char* Diagnostic_Message)
{
	*System = inbuffer[0];
	*SubSystem = inbuffer[1];
	*Component = inbuffer[2];
	*Diagnostic_Type = inbuffer[3];
	*Level = inbuffer[4];
	*Diagnostic_Message = inbuffer[5];
	return 1;
}
int I2CMessageHandler::decode_TestMessageCounterI2C(unsigned char* inbuffer,int * length,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8,unsigned char* value9,unsigned char* value10,unsigned char* value11,unsigned char* value12)
{
	*value1 = inbuffer[0];
	*value2 = inbuffer[1];
	*value3 = inbuffer[2];
	*value4 = inbuffer[3];
	*value5 = inbuffer[4];
	*value6 = inbuffer[5];
	*value7 = inbuffer[6];
	*value8 = inbuffer[7];
	*value9 = inbuffer[8];
	*value10 = inbuffer[9];
	*value11 = inbuffer[10];
	*value12 = inbuffer[11];
	return 1;
}
int I2CMessageHandler::decode_Get_DIO_Port1I2C(unsigned char* inbuffer,int * length,uint16_t* u1,uint16_t* u2,uint16_t* u3,uint16_t* u4)
{
	int v_u1 = inbuffer[0]<<8;
	*u1 = v_u1 + inbuffer[1];
	int v_u2 = inbuffer[2]<<8;
	*u2 = v_u2 + inbuffer[3];
	int v_u3 = inbuffer[4]<<8;
	*u3 = v_u3 + inbuffer[5];
	int v_u4 = inbuffer[6]<<8;
	*u4 = v_u4 + inbuffer[7];
	return 1;
}
