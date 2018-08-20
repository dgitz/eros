/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-08-19 07:10:30.243478***/
/***Target: Raspberry Pi ***/
#ifndef I2CMESSAGE_H
#define I2CMESSAGE_H
#include "ros/ros.h"
#include "Definitions.h"
#include "ros/time.h"
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <iostream>


class I2CMessageHandler
{
public:
	enum MessageID
	{
		I2C_Diagnostic_ID = 0x12,
		I2C_TestMessageCounter_ID = 0x14,
		I2C_Get_DIO_Port1_ID = 0x19,
	};
	I2CMessageHandler();
	~I2CMessageHandler();

	int decode_DiagnosticI2C(unsigned char* inbuffer,int * length,unsigned char* System,unsigned char* SubSystem,unsigned char* Component,unsigned char* Diagnostic_Type,unsigned char* Level,unsigned char* Diagnostic_Message);

	int decode_TestMessageCounterI2C(unsigned char* inbuffer,int * length,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8,unsigned char* value9,unsigned char* value10,unsigned char* value11,unsigned char* value12);

	int decode_Get_DIO_Port1I2C(unsigned char* inbuffer,int * length,uint16_t* u1,uint16_t* u2,uint16_t* u3,uint16_t* u4);
private:
};
#endif