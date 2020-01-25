/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2020-01-25 07:38:19.291584***/
/***Target: Arduino ***/
#ifndef I2CMESSAGE_H
#define I2CMESSAGE_H
#define I2C_Diagnostic_ID 0x12
#define I2C_TestMessageCounter_ID 0x14
#define I2C_Get_DIO_Port1_ID 0x19
#define I2C_Get_ANA_Port1_ID 0x20
#define I2C_Get_ANA_Port2_ID 0x21
#define I2C_Get_IMUAcc_ID 0x27
#define I2C_Get_IMUGyro_ID 0x28
#define I2C_Get_IMUMag_ID 0x29
#define I2C_TestProgram_ID 0x47

int encode_DiagnosticI2C(unsigned char* outbuffer,int* length,unsigned char System,unsigned char SubSystem,unsigned char Component,unsigned char Diagnostic_Type,unsigned char Level,unsigned char Diagnostic_Message);

int encode_TestMessageCounterI2C(unsigned char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12);

int encode_Get_DIO_Port1I2C(unsigned char* outbuffer,int* length,unsigned int u1,unsigned int u2,unsigned int u3,unsigned int u4);

int encode_Get_ANA_Port1I2C(unsigned char* outbuffer,int* length,unsigned int Pin1_Value,unsigned int Pin2_Value,unsigned int Pin3_Value,unsigned int Pin4_Value,unsigned int Pin5_Value,unsigned int Pin6_Value);

int encode_Get_ANA_Port2I2C(unsigned char* outbuffer,int* length,unsigned int Pin1_Value,unsigned int Pin2_Value,unsigned int Pin3_Value,unsigned int Pin4_Value,unsigned int Pin5_Value,unsigned int Pin6_Value);

int encode_Get_IMUAccI2C(unsigned char* outbuffer,int* length,unsigned int acc1_x,unsigned int acc1_y,unsigned int acc1_z,unsigned int acc2_x,unsigned int acc2_y,unsigned int acc2_z);

int encode_Get_IMUGyroI2C(unsigned char* outbuffer,int* length,unsigned int gyro1_x,unsigned int gyro1_y,unsigned int gyro1_z,unsigned int gyro2_x,unsigned int gyro2_y,unsigned int gyro2_z);

int encode_Get_IMUMagI2C(unsigned char* outbuffer,int* length,unsigned int mag1_x,unsigned int mag1_y,unsigned int mag1_z,unsigned int mag2_x,unsigned int mag2_y,unsigned int mag2_z);

int encode_TestProgramI2C(unsigned char* outbuffer,int* length,unsigned char ProgramState);
#endif