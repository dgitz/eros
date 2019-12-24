/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2019-12-11 06:12:15.303632***/
/***Target: ROS ***/
#include "../include/serialmessage.h"
SerialMessageHandler::SerialMessageHandler(){}
SerialMessageHandler::~SerialMessageHandler(){}
int SerialMessageHandler::encode_UserMessageSerial(unsigned char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12)
{
	unsigned char *p_outbuffer;
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
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_UserMessageSerial(unsigned char* inpacket,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8,unsigned char* value9,unsigned char* value10,unsigned char* value11,unsigned char* value12)
{
	*value1=inpacket[3];
	*value2=inpacket[4];
	*value3=inpacket[5];
	*value4=inpacket[6];
	*value5=inpacket[7];
	*value6=inpacket[8];
	*value7=inpacket[9];
	*value8=inpacket[10];
	*value9=inpacket[11];
	*value10=inpacket[12];
	*value11=inpacket[13];
	*value12=inpacket[14];
	return 1;
}
int SerialMessageHandler::encode_CommandSerial(unsigned char* outbuffer,int* length,unsigned char Command,unsigned char Option1,unsigned char Option2,unsigned char Option3)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x2;
	*p_outbuffer++ = 4;
	*p_outbuffer++ = Command;
	*p_outbuffer++ = Option1;
	*p_outbuffer++ = Option2;
	*p_outbuffer++ = Option3;
	int checksum = 0;
	for(int i = 3; i < (3+4);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_CommandSerial(unsigned char* inpacket,unsigned char* Command,unsigned char* Option1,unsigned char* Option2,unsigned char* Option3)
{
	*Command=inpacket[3];
	*Option1=inpacket[4];
	*Option2=inpacket[5];
	*Option3=inpacket[6];
	return 1;
}
int SerialMessageHandler::encode_DiagnosticSerial(unsigned char* outbuffer,int* length,unsigned char System,unsigned char SubSystem,unsigned char Component,unsigned char Diagnostic_Type,unsigned char Level,unsigned char Diagnostic_Message)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x12;
	*p_outbuffer++ = 6;
	*p_outbuffer++ = System;
	*p_outbuffer++ = SubSystem;
	*p_outbuffer++ = Component;
	*p_outbuffer++ = Diagnostic_Type;
	*p_outbuffer++ = Level;
	*p_outbuffer++ = Diagnostic_Message;
	int checksum = 0;
	for(int i = 3; i < (3+6);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_DiagnosticSerial(unsigned char* inpacket,unsigned char* System,unsigned char* SubSystem,unsigned char* Component,unsigned char* Diagnostic_Type,unsigned char* Level,unsigned char* Diagnostic_Message)
{
	*System=inpacket[3];
	*SubSystem=inpacket[4];
	*Component=inpacket[5];
	*Diagnostic_Type=inpacket[6];
	*Level=inpacket[7];
	*Diagnostic_Message=inpacket[8];
	return 1;
}
int SerialMessageHandler::encode_TestMessageCommandSerial(unsigned char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x15;
	*p_outbuffer++ = 8;
	*p_outbuffer++ = value1;
	*p_outbuffer++ = value2;
	*p_outbuffer++ = value3;
	*p_outbuffer++ = value4;
	*p_outbuffer++ = value5;
	*p_outbuffer++ = value6;
	*p_outbuffer++ = value7;
	*p_outbuffer++ = value8;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::encode_Configure_DIO_PortSerial(unsigned char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char MessageIndex,unsigned char MessageCount,unsigned char Pin1_Mode,unsigned char Pin2_Mode,unsigned char Pin3_Mode,unsigned char Pin4_Mode,unsigned char Pin5_Mode,unsigned char Pin6_Mode,unsigned char Pin7_Mode,unsigned char Pin8_Mode)
{
	unsigned char *p_outbuffer;
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
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Configure_DIO_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* MessageIndex,unsigned char* MessageCount,unsigned char* Pin1_Mode,unsigned char* Pin2_Mode,unsigned char* Pin3_Mode,unsigned char* Pin4_Mode,unsigned char* Pin5_Mode,unsigned char* Pin6_Mode,unsigned char* Pin7_Mode,unsigned char* Pin8_Mode)
{
	*ShieldID=inpacket[3];
	*PortID=inpacket[4];
	*MessageIndex=inpacket[5];
	*MessageCount=inpacket[6];
	*Pin1_Mode=inpacket[7];
	*Pin2_Mode=inpacket[8];
	*Pin3_Mode=inpacket[9];
	*Pin4_Mode=inpacket[10];
	*Pin5_Mode=inpacket[11];
	*Pin6_Mode=inpacket[12];
	*Pin7_Mode=inpacket[13];
	*Pin8_Mode=inpacket[14];
	return 1;
}
int SerialMessageHandler::encode_ModeSerial(unsigned char* outbuffer,int* length,unsigned char DeviceType,unsigned char ID,unsigned char Mode)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x17;
	*p_outbuffer++ = 3;
	*p_outbuffer++ = DeviceType;
	*p_outbuffer++ = ID;
	*p_outbuffer++ = Mode;
	int checksum = 0;
	for(int i = 3; i < (3+3);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_ModeSerial(unsigned char* inpacket,unsigned char* DeviceType,unsigned char* ID,unsigned char* Mode)
{
	*DeviceType=inpacket[3];
	*ID=inpacket[4];
	*Mode=inpacket[5];
	return 1;
}
int SerialMessageHandler::encode_Set_DIO_PortSerial(unsigned char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x18;
	*p_outbuffer++ = 10;
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
	int checksum = 0;
	for(int i = 3; i < (3+10);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Set_DIO_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value)
{
	*ShieldID=inpacket[3];
	*PortID=inpacket[4];
	*Pin1_Value=inpacket[5];
	*Pin2_Value=inpacket[6];
	*Pin3_Value=inpacket[7];
	*Pin4_Value=inpacket[8];
	*Pin5_Value=inpacket[9];
	*Pin6_Value=inpacket[10];
	*Pin7_Value=inpacket[11];
	*Pin8_Value=inpacket[12];
	return 1;
}
int SerialMessageHandler::decode_FirmwareVersionSerial(unsigned char* inpacket,unsigned char* majorVersion,unsigned char* minorVersion,unsigned char* buildNumber)
{
	*majorVersion=inpacket[3];
	*minorVersion=inpacket[4];
	*buildNumber=inpacket[5];
	return 1;
}
int SerialMessageHandler::encode_Arm_StatusSerial(unsigned char* outbuffer,int* length,unsigned char Status)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x30;
	*p_outbuffer++ = 1;
	*p_outbuffer++ = Status;
	int checksum = 0;
	for(int i = 3; i < (3+1);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Arm_StatusSerial(unsigned char* inpacket,unsigned char* Status)
{
	*Status=inpacket[3];
	return 1;
}
int SerialMessageHandler::encode_Set_DIO_Port_DefaultValueSerial(unsigned char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char MessageIndex,unsigned char MessageCount,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value)
{
	unsigned char *p_outbuffer;
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
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Set_DIO_Port_DefaultValueSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* MessageIndex,unsigned char* MessageCount,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value)
{
	*ShieldID=inpacket[3];
	*PortID=inpacket[4];
	*MessageIndex=inpacket[5];
	*MessageCount=inpacket[6];
	*Pin1_Value=inpacket[7];
	*Pin2_Value=inpacket[8];
	*Pin3_Value=inpacket[9];
	*Pin4_Value=inpacket[10];
	*Pin5_Value=inpacket[11];
	*Pin6_Value=inpacket[12];
	*Pin7_Value=inpacket[13];
	*Pin8_Value=inpacket[14];
	return 1;
}
int SerialMessageHandler::encode_PPSSerial(unsigned char* outbuffer,int* length,unsigned char counter)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x35;
	*p_outbuffer++ = 1;
	*p_outbuffer++ = counter;
	int checksum = 0;
	for(int i = 3; i < (3+1);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_PPSSerial(unsigned char* inpacket,unsigned char* counter)
{
	*counter=inpacket[3];
	return 1;
}
int SerialMessageHandler::encode_Configure_ANA_PortSerial(unsigned char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char MessageIndex,unsigned char MessageCount,unsigned char Pin1_Mode,unsigned char Pin2_Mode,unsigned char Pin3_Mode,unsigned char Pin4_Mode)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x36;
	*p_outbuffer++ = 8;
	*p_outbuffer++ = ShieldID;
	*p_outbuffer++ = PortID;
	*p_outbuffer++ = MessageIndex;
	*p_outbuffer++ = MessageCount;
	*p_outbuffer++ = Pin1_Mode;
	*p_outbuffer++ = Pin2_Mode;
	*p_outbuffer++ = Pin3_Mode;
	*p_outbuffer++ = Pin4_Mode;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*p_outbuffer++ = 10;
	*p_outbuffer++ = 13;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Configure_ANA_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* MessageIndex,unsigned char* MessageCount,unsigned char* Pin1_Mode,unsigned char* Pin2_Mode,unsigned char* Pin3_Mode,unsigned char* Pin4_Mode)
{
	*ShieldID=inpacket[3];
	*PortID=inpacket[4];
	*MessageIndex=inpacket[5];
	*MessageCount=inpacket[6];
	*Pin1_Mode=inpacket[7];
	*Pin2_Mode=inpacket[8];
	*Pin3_Mode=inpacket[9];
	*Pin4_Mode=inpacket[10];
	return 1;
}
int SerialMessageHandler::decode_IDSerial(unsigned char* inpacket,unsigned char* DeviceID,unsigned long* PartNumber)
{
	*DeviceID=inpacket[3];
	int v_PartNumber3=inpacket[4]<<24;
	int v_PartNumber2=inpacket[5]<<16;
	int v_PartNumber1=inpacket[6]<<8;
	*PartNumber=inpacket[7] + v_PartNumber1 + v_PartNumber2 + v_PartNumber3;
	return 1;
}
