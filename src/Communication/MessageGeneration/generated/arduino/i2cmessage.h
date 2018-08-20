/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-08-19 07:10:30.243301***/
/***Target: Arduino ***/
#ifndef I2CMESSAGE_H
#define I2CMESSAGE_H
#define I2C_Diagnostic_ID 0x12
#define I2C_TestMessageCounter_ID 0x14
#define I2C_Get_DIO_Port1_ID 0x19

int encode_DiagnosticI2C(unsigned char* outbuffer,int* length,unsigned char System,unsigned char SubSystem,unsigned char Component,unsigned char Diagnostic_Type,unsigned char Level,unsigned char Diagnostic_Message);

int encode_TestMessageCounterI2C(unsigned char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12);

int encode_Get_DIO_Port1I2C(unsigned char* outbuffer,int* length,unsigned int u1,unsigned int u2,unsigned int u3,unsigned int u4);
#endif