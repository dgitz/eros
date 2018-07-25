/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-07-24 07:28:40.257196***/
/***Target: Arduino ***/
#ifndef SPIMESSAGE_H
#define SPIMESSAGE_H
#define SPI_TestMessageCounter_ID 0x14
#define SPI_Get_DIO_Port1_ID 0x19
#define SPI_Get_ANA_Port1_ID 0x20
#define SPI_LEDStripControl_ID 0x42

int encode_TestMessageCounterSPI(unsigned char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12);

int encode_Get_DIO_Port1SPI(unsigned char* outbuffer,int* length,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value);

int encode_Get_ANA_Port1SPI(unsigned char* outbuffer,int* length,unsigned int Pin1_Value,unsigned int Pin2_Value,unsigned int Pin3_Value,unsigned int Pin4_Value,unsigned int Pin5_Value,unsigned int Pin6_Value);

int decode_LEDStripControlSPI(unsigned char* inbuffer,int* length,unsigned char checksum,unsigned char * LEDPixelMode,unsigned char * Param1,unsigned char * Param2);
#endif