/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-10-24 06:49:44.094085***/
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

#define BYTE2_OFFSET 32768

class SPIMessageHandler
{
public:
	enum MessageID
	{
		SPI_Diagnostic_ID = 0x12,
		SPI_TestMessageCounter_ID = 0x14,
		SPI_Get_DIO_Port1_ID = 0x19,
		SPI_Get_ANA_Port1_ID = 0x20,
		SPI_Get_IMUAcc_ID = 0x27,
		SPI_Get_IMUGyro_ID = 0x28,
		SPI_Get_IMUMag_ID = 0x29,
		SPI_LEDStripControl_ID = 0x42,
	};
	SPIMessageHandler();
	~SPIMessageHandler();

	int decode_DiagnosticSPI(unsigned char* inbuffer,int * length,unsigned char* System,unsigned char* SubSystem,unsigned char* Component,unsigned char* Diagnostic_Type,unsigned char* Level,unsigned char* Diagnostic_Message);

	int decode_TestMessageCounterSPI(unsigned char* inbuffer,int * length,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8,unsigned char* value9,unsigned char* value10,unsigned char* value11,unsigned char* value12);

	int decode_Get_DIO_Port1SPI(unsigned char* inbuffer,int * length,uint16_t* u1,uint16_t* u2);

	int decode_Get_ANA_Port1SPI(unsigned char* inbuffer,int * length,uint16_t* Pin1_Value,uint16_t* Pin2_Value,uint16_t* Pin3_Value,uint16_t* Pin4_Value,uint16_t* Pin5_Value,uint16_t* Pin6_Value);

	int decode_Get_IMUAccSPI(unsigned char* inbuffer,int * length,uint16_t* acc1_x,uint16_t* acc1_y,uint16_t* acc1_z,uint16_t* acc2_x,uint16_t* acc2_y,uint16_t* acc2_z);

	int decode_Get_IMUGyroSPI(unsigned char* inbuffer,int * length,uint16_t* gyro1_x,uint16_t* gyro1_y,uint16_t* gyro1_z,uint16_t* gyro2_x,uint16_t* gyro2_y,uint16_t* gyro2_z);

	int decode_Get_IMUMagSPI(unsigned char* inbuffer,int * length,uint16_t* mag1_x,uint16_t* mag1_y,uint16_t* mag1_z,uint16_t* mag2_x,uint16_t* mag2_y,uint16_t* mag2_z);

	int encode_LEDStripControlSPI(unsigned char* outbuffer,int * length,unsigned char LEDPixelMode,unsigned char Param1,unsigned char Param2);
private:
};
#endif