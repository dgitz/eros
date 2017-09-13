/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2017-09-12 22:13:32.315970***/
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
	};
	SPIMessageHandler();
	~SPIMessageHandler();
	int decode_TestMessageCounterSPI(unsigned char* inbuffer,int * length,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8,unsigned char* value9,unsigned char* value10,unsigned char* value11,unsigned char* value12);
private:
};
#endif