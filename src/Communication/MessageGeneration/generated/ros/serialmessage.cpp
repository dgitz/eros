/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2017-11-19 13:28:18.630295***/
/***Target: Raspberry Pi ***/
#include "../include/serialmessage.h"
SerialMessageHandler::SerialMessageHandler(){}
SerialMessageHandler::~SerialMessageHandler(){}
int SerialMessageHandler::encode_UserMessageSerial(char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x1;
	*p_outbuffer++ = 12;
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
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_UserMessageSerial(unsigned char* inpacket,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8,unsigned char* value9,unsigned char* value10,unsigned char* value11,unsigned char* value12)
{
	*value1=inpacket[0];
	*value2=inpacket[1];
	*value3=inpacket[2];
	*value4=inpacket[3];
	*value5=inpacket[4];
	*value6=inpacket[5];
	*value7=inpacket[6];
	*value8=inpacket[7];
	*value9=inpacket[8];
	*value10=inpacket[9];
	*value11=inpacket[10];
	*value12=inpacket[11];
	return 1;
}
int SerialMessageHandler::encode_CommandSerial(char* outbuffer,int* length,unsigned char Command,unsigned char Option1,unsigned char Option2,unsigned char Option3)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x2;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = Command;
	*p_outbuffer++ = Option1;
	*p_outbuffer++ = Option2;
	*p_outbuffer++ = Option3;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_CommandSerial(unsigned char* inpacket,unsigned char* Command,unsigned char* Option1,unsigned char* Option2,unsigned char* Option3)
{
	*Command=inpacket[0];
	*Option1=inpacket[1];
	*Option2=inpacket[2];
	*Option3=inpacket[3];
	return 1;
}
int SerialMessageHandler::encode_DiagnosticSerial(char* outbuffer,int* length,unsigned char System,unsigned char SubSystem,unsigned char Component,unsigned char Diagnostic_Type,unsigned char Level,unsigned char Diagnostic_Message)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x12;
	*p_outbuffer++ = 12;
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
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_DiagnosticSerial(unsigned char* inpacket,unsigned char* System,unsigned char* SubSystem,unsigned char* Component,unsigned char* Diagnostic_Type,unsigned char* Level,unsigned char* Diagnostic_Message)
{
	*System=inpacket[0];
	*SubSystem=inpacket[1];
	*Component=inpacket[2];
	*Diagnostic_Type=inpacket[3];
	*Level=inpacket[4];
	*Diagnostic_Message=inpacket[5];
	return 1;
}
int SerialMessageHandler::encode_TestMessageCommandSerial(char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x15;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = value1;
	*p_outbuffer++ = value2;
	*p_outbuffer++ = value3;
	*p_outbuffer++ = value4;
	*p_outbuffer++ = value5;
	*p_outbuffer++ = value6;
	*p_outbuffer++ = value7;
	*p_outbuffer++ = value8;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::encode_Configure_DIO_PortSerial(char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char MessageIndex,unsigned char MessageCount,unsigned char Pin1_Mode,unsigned char Pin2_Mode,unsigned char Pin3_Mode,unsigned char Pin4_Mode,unsigned char Pin5_Mode,unsigned char Pin6_Mode,unsigned char Pin7_Mode,unsigned char Pin8_Mode)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x16;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = ShieldID;
	*p_outbuffer++ = PortID;
	*p_outbuffer++ = MessageIndex;
	*p_outbuffer++ = MessageCount;
	*p_outbuffer++ = Pin1_Mode;
	*p_outbuffer++ = Pin2_Mode;
	*p_outbuffer++ = Pin3_Mode;
	*p_outbuffer++ = Pin4_Mode;
	*p_outbuffer++ = Pin5_Mode;
	*p_outbuffer++ = Pin6_Mode;
	*p_outbuffer++ = Pin7_Mode;
	*p_outbuffer++ = Pin8_Mode;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Configure_DIO_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* MessageIndex,unsigned char* MessageCount,unsigned char* Pin1_Mode,unsigned char* Pin2_Mode,unsigned char* Pin3_Mode,unsigned char* Pin4_Mode,unsigned char* Pin5_Mode,unsigned char* Pin6_Mode,unsigned char* Pin7_Mode,unsigned char* Pin8_Mode)
{
	*ShieldID=inpacket[0];
	*PortID=inpacket[1];
	*MessageIndex=inpacket[2];
	*MessageCount=inpacket[3];
	*Pin1_Mode=inpacket[4];
	*Pin2_Mode=inpacket[5];
	*Pin3_Mode=inpacket[6];
	*Pin4_Mode=inpacket[7];
	*Pin5_Mode=inpacket[8];
	*Pin6_Mode=inpacket[9];
	*Pin7_Mode=inpacket[10];
	*Pin8_Mode=inpacket[11];
	return 1;
}
int SerialMessageHandler::encode_ModeSerial(char* outbuffer,int* length,unsigned char DeviceType,unsigned char ID,unsigned char Mode)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x17;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = DeviceType;
	*p_outbuffer++ = ID;
	*p_outbuffer++ = Mode;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_ModeSerial(unsigned char* inpacket,unsigned char* DeviceType,unsigned char* ID,unsigned char* Mode)
{
	*DeviceType=inpacket[0];
	*ID=inpacket[1];
	*Mode=inpacket[2];
	return 1;
}
int SerialMessageHandler::encode_Set_DIO_PortSerial(char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x18;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = ShieldID;
	*p_outbuffer++ = PortID;
	*p_outbuffer++ = Pin1_Value;
	*p_outbuffer++ = Pin2_Value;
	*p_outbuffer++ = Pin3_Value;
	*p_outbuffer++ = Pin4_Value;
	*p_outbuffer++ = Pin5_Value;
	*p_outbuffer++ = Pin6_Value;
	*p_outbuffer++ = Pin7_Value;
	*p_outbuffer++ = Pin8_Value;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Set_DIO_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value)
{
	*ShieldID=inpacket[0];
	*PortID=inpacket[1];
	*Pin1_Value=inpacket[2];
	*Pin2_Value=inpacket[3];
	*Pin3_Value=inpacket[4];
	*Pin4_Value=inpacket[5];
	*Pin5_Value=inpacket[6];
	*Pin6_Value=inpacket[7];
	*Pin7_Value=inpacket[8];
	*Pin8_Value=inpacket[9];
	return 1;
}
int SerialMessageHandler::decode_FirmwareVersionSerial(unsigned char* inpacket,unsigned char* majorVersion,unsigned char* minorVersion,unsigned char* buildNumber)
{
	*majorVersion=inpacket[0];
	*minorVersion=inpacket[1];
	*buildNumber=inpacket[2];
	return 1;
}
int SerialMessageHandler::encode_Arm_StatusSerial(char* outbuffer,int* length,unsigned char Status)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x30;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = Status;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Arm_StatusSerial(unsigned char* inpacket,unsigned char* Status)
{
	*Status=inpacket[0];
	return 1;
}
int SerialMessageHandler::encode_Set_DIO_Port_DefaultValueSerial(char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char MessageIndex,unsigned char MessageCount,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x32;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = ShieldID;
	*p_outbuffer++ = PortID;
	*p_outbuffer++ = MessageIndex;
	*p_outbuffer++ = MessageCount;
	*p_outbuffer++ = Pin1_Value;
	*p_outbuffer++ = Pin2_Value;
	*p_outbuffer++ = Pin3_Value;
	*p_outbuffer++ = Pin4_Value;
	*p_outbuffer++ = Pin5_Value;
	*p_outbuffer++ = Pin6_Value;
	*p_outbuffer++ = Pin7_Value;
	*p_outbuffer++ = Pin8_Value;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Set_DIO_Port_DefaultValueSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* MessageIndex,unsigned char* MessageCount,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value)
{
	*ShieldID=inpacket[0];
	*PortID=inpacket[1];
	*MessageIndex=inpacket[2];
	*MessageCount=inpacket[3];
	*Pin1_Value=inpacket[4];
	*Pin2_Value=inpacket[5];
	*Pin3_Value=inpacket[6];
	*Pin4_Value=inpacket[7];
	*Pin5_Value=inpacket[8];
	*Pin6_Value=inpacket[9];
	*Pin7_Value=inpacket[10];
	*Pin8_Value=inpacket[11];
	return 1;
}
int SerialMessageHandler::encode_PPSSerial(char* outbuffer,int* length,unsigned char counter)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x35;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = counter;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_PPSSerial(unsigned char* inpacket,unsigned char* counter)
{
	*counter=inpacket[0];
	return 1;
}
int SerialMessageHandler::encode_Configure_ANA_PortSerial(char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char MessageIndex,unsigned char MessageCount,unsigned char Pin1_Mode,unsigned char Pin2_Mode,unsigned char Pin3_Mode,unsigned char Pin4_Mode)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x36;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = ShieldID;
	*p_outbuffer++ = PortID;
	*p_outbuffer++ = MessageIndex;
	*p_outbuffer++ = MessageCount;
	*p_outbuffer++ = Pin1_Mode;
	*p_outbuffer++ = Pin2_Mode;
	*p_outbuffer++ = Pin3_Mode;
	*p_outbuffer++ = Pin4_Mode;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Configure_ANA_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* MessageIndex,unsigned char* MessageCount,unsigned char* Pin1_Mode,unsigned char* Pin2_Mode,unsigned char* Pin3_Mode,unsigned char* Pin4_Mode)
{
	*ShieldID=inpacket[0];
	*PortID=inpacket[1];
	*MessageIndex=inpacket[2];
	*MessageCount=inpacket[3];
	*Pin1_Mode=inpacket[4];
	*Pin2_Mode=inpacket[5];
	*Pin3_Mode=inpacket[6];
	*Pin4_Mode=inpacket[7];
	return 1;
}
int SerialMessageHandler::encode_IDSerial(char* outbuffer,int* length,unsigned char DeviceID,unsigned long PartNumber)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x40;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = DeviceID;
	*p_outbuffer++ = PartNumber >> 24;
	*p_outbuffer++ = PartNumber >> 16;
	*p_outbuffer++ = PartNumber >> 8;
	*p_outbuffer++ = (PartNumber & 0xFF);
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_IDSerial(unsigned char* inpacket,unsigned char* DeviceID,unsigned long* PartNumber)
{
	*DeviceID=inpacket[0];
	int v_PartNumber3=inpacket[1]<<24;
	int v_PartNumber2=inpacket[2]<<16;
	int v_PartNumber1=inpacket[3]<<8;
	*PartNumber=inpacket[4] + v_PartNumber1 + v_PartNumber2 + v_PartNumber3;
	return 1;
}
int SerialMessageHandler::encode_IMUSerial(char* outbuffer,int* length,unsigned long timemS,int counter,long AccX_mg,long AccY_mg,long AccZ_mg,long GyroX_mdegps,long GyroY_mdepgs,long GyroZ_mdegps,long MagX,long MagY,long MagZ)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x41;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = timemS >> 24;
	*p_outbuffer++ = timemS >> 16;
	*p_outbuffer++ = timemS >> 8;
	*p_outbuffer++ = (timemS & 0xFF);
	*p_outbuffer++ = counter >> 8;
	*p_outbuffer++ = (counter & 0xFF);
	*p_outbuffer++ = AccX_mg >> 24;
	*p_outbuffer++ = AccX_mg >> 16;
	*p_outbuffer++ = AccX_mg >> 8;
	*p_outbuffer++ = (AccX_mg & 0xFF);
	*p_outbuffer++ = AccY_mg >> 24;
	*p_outbuffer++ = AccY_mg >> 16;
	*p_outbuffer++ = AccY_mg >> 8;
	*p_outbuffer++ = (AccY_mg & 0xFF);
	*p_outbuffer++ = AccZ_mg >> 24;
	*p_outbuffer++ = AccZ_mg >> 16;
	*p_outbuffer++ = AccZ_mg >> 8;
	*p_outbuffer++ = (AccZ_mg & 0xFF);
	*p_outbuffer++ = GyroX_mdegps >> 24;
	*p_outbuffer++ = GyroX_mdegps >> 16;
	*p_outbuffer++ = GyroX_mdegps >> 8;
	*p_outbuffer++ = (GyroX_mdegps & 0xFF);
	*p_outbuffer++ = GyroY_mdepgs >> 24;
	*p_outbuffer++ = GyroY_mdepgs >> 16;
	*p_outbuffer++ = GyroY_mdepgs >> 8;
	*p_outbuffer++ = (GyroY_mdepgs & 0xFF);
	*p_outbuffer++ = GyroZ_mdegps >> 24;
	*p_outbuffer++ = GyroZ_mdegps >> 16;
	*p_outbuffer++ = GyroZ_mdegps >> 8;
	*p_outbuffer++ = (GyroZ_mdegps & 0xFF);
	*p_outbuffer++ = MagX >> 24;
	*p_outbuffer++ = MagX >> 16;
	*p_outbuffer++ = MagX >> 8;
	*p_outbuffer++ = (MagX & 0xFF);
	*p_outbuffer++ = MagY >> 24;
	*p_outbuffer++ = MagY >> 16;
	*p_outbuffer++ = MagY >> 8;
	*p_outbuffer++ = (MagY & 0xFF);
	*p_outbuffer++ = MagZ >> 24;
	*p_outbuffer++ = MagZ >> 16;
	*p_outbuffer++ = MagZ >> 8;
	*p_outbuffer++ = (MagZ & 0xFF);
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_IMUSerial(unsigned char* inpacket,unsigned long* timemS,int* counter,long* AccX_mg,long* AccY_mg,long* AccZ_mg,long* GyroX_mdegps,long* GyroY_mdepgs,long* GyroZ_mdegps,long* MagX,long* MagY,long* MagZ)
{
	int v_timemS3=inpacket[0]<<24;
	int v_timemS2=inpacket[1]<<16;
	int v_timemS1=inpacket[2]<<8;
	*timemS=inpacket[3] + v_timemS1 + v_timemS2 + v_timemS3;
	int v_counter1=inpacket[4]<<8;
	*counter=inpacket[5] + v_counter1;
	int v_AccX_mg3=inpacket[6]<<24;
	int v_AccX_mg2=inpacket[7]<<16;
	int v_AccX_mg1=inpacket[8]<<8;
	*AccX_mg=inpacket[9] + v_AccX_mg1 + v_AccX_mg2 + v_AccX_mg3;
	int v_AccY_mg3=inpacket[10]<<24;
	int v_AccY_mg2=inpacket[11]<<16;
	int v_AccY_mg1=inpacket[12]<<8;
	*AccY_mg=inpacket[13] + v_AccY_mg1 + v_AccY_mg2 + v_AccY_mg3;
	int v_AccZ_mg3=inpacket[14]<<24;
	int v_AccZ_mg2=inpacket[15]<<16;
	int v_AccZ_mg1=inpacket[16]<<8;
	*AccZ_mg=inpacket[17] + v_AccZ_mg1 + v_AccZ_mg2 + v_AccZ_mg3;
	int v_GyroX_mdegps3=inpacket[18]<<24;
	int v_GyroX_mdegps2=inpacket[19]<<16;
	int v_GyroX_mdegps1=inpacket[20]<<8;
	*GyroX_mdegps=inpacket[21] + v_GyroX_mdegps1 + v_GyroX_mdegps2 + v_GyroX_mdegps3;
	int v_GyroY_mdepgs3=inpacket[22]<<24;
	int v_GyroY_mdepgs2=inpacket[23]<<16;
	int v_GyroY_mdepgs1=inpacket[24]<<8;
	*GyroY_mdepgs=inpacket[25] + v_GyroY_mdepgs1 + v_GyroY_mdepgs2 + v_GyroY_mdepgs3;
	int v_GyroZ_mdegps3=inpacket[26]<<24;
	int v_GyroZ_mdegps2=inpacket[27]<<16;
	int v_GyroZ_mdegps1=inpacket[28]<<8;
	*GyroZ_mdegps=inpacket[29] + v_GyroZ_mdegps1 + v_GyroZ_mdegps2 + v_GyroZ_mdegps3;
	int v_MagX3=inpacket[30]<<24;
	int v_MagX2=inpacket[31]<<16;
	int v_MagX1=inpacket[32]<<8;
	*MagX=inpacket[33] + v_MagX1 + v_MagX2 + v_MagX3;
	int v_MagY3=inpacket[34]<<24;
	int v_MagY2=inpacket[35]<<16;
	int v_MagY1=inpacket[36]<<8;
	*MagY=inpacket[37] + v_MagY1 + v_MagY2 + v_MagY3;
	int v_MagZ3=inpacket[38]<<24;
	int v_MagZ2=inpacket[39]<<16;
	int v_MagZ1=inpacket[40]<<8;
	*MagZ=inpacket[41] + v_MagZ1 + v_MagZ2 + v_MagZ3;
	return 1;
}
