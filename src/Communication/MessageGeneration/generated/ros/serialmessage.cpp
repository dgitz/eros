/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2017-04-01 18:51:14.306690***/
/***Target: Raspberry Pi ***/
#include "serialmessage.h"
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
int SerialMessageHandler::encode_TestMessageCounterSerial(char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x14;
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
int SerialMessageHandler::decode_TestMessageCounterSerial(unsigned char* inpacket,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8)
{
	*value1=inpacket[0];
	*value2=inpacket[1];
	*value3=inpacket[2];
	*value4=inpacket[3];
	*value5=inpacket[4];
	*value6=inpacket[5];
	*value7=inpacket[6];
	*value8=inpacket[7];
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
int SerialMessageHandler::encode_Configure_DIO_PortSerial(char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char Pin1_Mode,unsigned char Pin2_Mode,unsigned char Pin3_Mode,unsigned char Pin4_Mode,unsigned char Pin5_Mode,unsigned char Pin6_Mode,unsigned char Pin7_Mode,unsigned char Pin8_Mode)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x16;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = ShieldID;
	*p_outbuffer++ = PortID;
	*p_outbuffer++ = Pin1_Mode;
	*p_outbuffer++ = Pin2_Mode;
	*p_outbuffer++ = Pin3_Mode;
	*p_outbuffer++ = Pin4_Mode;
	*p_outbuffer++ = Pin5_Mode;
	*p_outbuffer++ = Pin6_Mode;
	*p_outbuffer++ = Pin7_Mode;
	*p_outbuffer++ = Pin8_Mode;
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
int SerialMessageHandler::decode_Configure_DIO_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* Pin1_Mode,unsigned char* Pin2_Mode,unsigned char* Pin3_Mode,unsigned char* Pin4_Mode,unsigned char* Pin5_Mode,unsigned char* Pin6_Mode,unsigned char* Pin7_Mode,unsigned char* Pin8_Mode)
{
	*ShieldID=inpacket[0];
	*PortID=inpacket[1];
	*Pin1_Mode=inpacket[2];
	*Pin2_Mode=inpacket[3];
	*Pin3_Mode=inpacket[4];
	*Pin4_Mode=inpacket[5];
	*Pin5_Mode=inpacket[6];
	*Pin6_Mode=inpacket[7];
	*Pin7_Mode=inpacket[8];
	*Pin8_Mode=inpacket[9];
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
int SerialMessageHandler::decode_Get_DIO_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value)
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
int SerialMessageHandler::decode_Get_ANA_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,int* Pin1_Value,int* Pin2_Value,int* Pin3_Value,int* Pin4_Value)
{
	*ShieldID=inpacket[0];
	*PortID=inpacket[1];
	int v_Pin1_Value1=inpacket[2]<<8;
	*Pin1_Value=inpacket[3] + v_Pin1_Value1;
	int v_Pin2_Value1=inpacket[4]<<8;
	*Pin2_Value=inpacket[5] + v_Pin2_Value1;
	int v_Pin3_Value1=inpacket[6]<<8;
	*Pin3_Value=inpacket[7] + v_Pin3_Value1;
	int v_Pin4_Value1=inpacket[8]<<8;
	*Pin4_Value=inpacket[9] + v_Pin4_Value1;
	return 1;
}
int SerialMessageHandler::decode_FirmwareVersionSerial(unsigned char* inpacket,unsigned char* majorVersion,unsigned char* minorVersion,unsigned char* buildNumber)
{
	*majorVersion=inpacket[0];
	*minorVersion=inpacket[1];
	*buildNumber=inpacket[2];
	return 1;
}
int SerialMessageHandler::encode_Arm_CommandSerial(char* outbuffer,int* length,unsigned char Command)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x27;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = Command;
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
int SerialMessageHandler::decode_Arm_CommandSerial(unsigned char* inpacket,unsigned char* Command)
{
	*Command=inpacket[0];
	return 1;
}
int SerialMessageHandler::encode_Setup_ControlGroupSerial(char* outbuffer,int* length,char ID,char Mode,char Input_Port,unsigned char Input_PinMode,unsigned char Input_PinNumber,unsigned char Output_Port,unsigned char Output_PinMode,unsigned char Output_PinNUmber)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x28;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = ID;
	*p_outbuffer++ = Mode;
	*p_outbuffer++ = Input_Port;
	*p_outbuffer++ = Input_PinMode;
	*p_outbuffer++ = Input_PinNumber;
	*p_outbuffer++ = Output_Port;
	*p_outbuffer++ = Output_PinMode;
	*p_outbuffer++ = Output_PinNUmber;
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
int SerialMessageHandler::encode_Tune_ControlGroupSerial(char* outbuffer,int* length,unsigned char ID,unsigned char Mode,int Proportional_Gain,int Integral_Gain,int Derivative_Gain)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x29;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = ID;
	*p_outbuffer++ = Mode;
	*p_outbuffer++ = Proportional_Gain;
	*p_outbuffer++ = Integral_Gain;
	*p_outbuffer++ = Derivative_Gain;
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
int SerialMessageHandler::encode_Set_DIO_Port_DefaultValueSerial(char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x32;
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
int SerialMessageHandler::decode_Set_DIO_Port_DefaultValueSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value)
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
int SerialMessageHandler::encode_Configure_ShieldSerial(char* outbuffer,int* length,unsigned char ShieldCount,unsigned char ShieldType,unsigned char ShieldID,unsigned char PortCount)
{
	char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x33;
	*p_outbuffer++ = 12;
	*p_outbuffer++ = ShieldCount;
	*p_outbuffer++ = ShieldType;
	*p_outbuffer++ = ShieldID;
	*p_outbuffer++ = PortCount;
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
int SerialMessageHandler::decode_Configure_ShieldSerial(unsigned char* inpacket,unsigned char* ShieldCount,unsigned char* ShieldType,unsigned char* ShieldID,unsigned char* PortCount)
{
	*ShieldCount=inpacket[0];
	*ShieldType=inpacket[1];
	*ShieldID=inpacket[2];
	*PortCount=inpacket[3];
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
