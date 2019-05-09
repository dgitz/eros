/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2019-05-08 08:00:47.522511***/
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
int I2CMessageHandler::decode_Get_IMUAccI2C(unsigned char* inbuffer,int * length,uint16_t* acc1_x,uint16_t* acc1_y,uint16_t* acc1_z,uint16_t* acc2_x,uint16_t* acc2_y,uint16_t* acc2_z)
{
	int v_acc1_x = inbuffer[0]<<8;
	*acc1_x = v_acc1_x + inbuffer[1];
	int v_acc1_y = inbuffer[2]<<8;
	*acc1_y = v_acc1_y + inbuffer[3];
	int v_acc1_z = inbuffer[4]<<8;
	*acc1_z = v_acc1_z + inbuffer[5];
	int v_acc2_x = inbuffer[6]<<8;
	*acc2_x = v_acc2_x + inbuffer[7];
	int v_acc2_y = inbuffer[8]<<8;
	*acc2_y = v_acc2_y + inbuffer[9];
	int v_acc2_z = inbuffer[10]<<8;
	*acc2_z = v_acc2_z + inbuffer[11];
	return 1;
}
int I2CMessageHandler::decode_Get_IMUGyroI2C(unsigned char* inbuffer,int * length,uint16_t* gyro1_x,uint16_t* gyro1_y,uint16_t* gyro1_z,uint16_t* gyro2_x,uint16_t* gyro2_y,uint16_t* gyro2_z)
{
	int v_gyro1_x = inbuffer[0]<<8;
	*gyro1_x = v_gyro1_x + inbuffer[1];
	int v_gyro1_y = inbuffer[2]<<8;
	*gyro1_y = v_gyro1_y + inbuffer[3];
	int v_gyro1_z = inbuffer[4]<<8;
	*gyro1_z = v_gyro1_z + inbuffer[5];
	int v_gyro2_x = inbuffer[6]<<8;
	*gyro2_x = v_gyro2_x + inbuffer[7];
	int v_gyro2_y = inbuffer[8]<<8;
	*gyro2_y = v_gyro2_y + inbuffer[9];
	int v_gyro2_z = inbuffer[10]<<8;
	*gyro2_z = v_gyro2_z + inbuffer[11];
	return 1;
}
int I2CMessageHandler::decode_Get_IMUMagI2C(unsigned char* inbuffer,int * length,uint16_t* mag1_x,uint16_t* mag1_y,uint16_t* mag1_z,uint16_t* mag2_x,uint16_t* mag2_y,uint16_t* mag2_z)
{
	int v_mag1_x = inbuffer[0]<<8;
	*mag1_x = v_mag1_x + inbuffer[1];
	int v_mag1_y = inbuffer[2]<<8;
	*mag1_y = v_mag1_y + inbuffer[3];
	int v_mag1_z = inbuffer[4]<<8;
	*mag1_z = v_mag1_z + inbuffer[5];
	int v_mag2_x = inbuffer[6]<<8;
	*mag2_x = v_mag2_x + inbuffer[7];
	int v_mag2_y = inbuffer[8]<<8;
	*mag2_y = v_mag2_y + inbuffer[9];
	int v_mag2_z = inbuffer[10]<<8;
	*mag2_z = v_mag2_z + inbuffer[11];
	return 1;
}
