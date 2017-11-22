/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2017-11-21 20:37:54.556312***/
/***Target: Raspberry Pi ***/
#ifndef SPIMESSAGE_H
#define SPIMESSAGE_H
#include "ros/ros.h"
#include "Definitions.h"
#include "ros/time.h"
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <iostream>


class SPIMessageHandler
{
public:
	enum MessageID
	{
		SPI_TestMessageCounter_ID = 0x14,
		SPI_Get_DIO_Port1_ID = 0x19,
		SPI_Get_ANA_Port1_ID = 0x20,
	};
	SPIMessageHandler();
	~SPIMessageHandler();

	int decode_TestMessageCounterSPI(unsigned char* inbuffer,int * length,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8,unsigned char* value9,unsigned char* value10,unsigned char* value11,unsigned char* value12);

	int decode_Get_DIO_Port1SPI(unsigned char* inbuffer,int * length,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value);

	int decode_Get_ANA_Port1SPI(unsigned char* inbuffer,int * length,uint16_t* Pin1_Value,uint16_t* Pin2_Value,uint16_t* Pin3_Value,uint16_t* Pin4_Value,uint16_t* Pin5_Value,uint16_t* Pin6_Value);
private:
};
#endif