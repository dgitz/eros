/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2016-11-21 18:01:01.386683***/
/***Target: Parallax Propeller ***/
#include "serialmessage.h"
int encode_DiagnosticSerial(int* outbuffer,int* length,char System,char SubSystem,char Component,char Diagnostic_Type,char Level,char Diagnostic_Message)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x12;
	outbuffer[byte_counter++] = 8;
	outbuffer[byte_counter++] = System;
	outbuffer[byte_counter++] = SubSystem;
	outbuffer[byte_counter++] = Component;
	outbuffer[byte_counter++] = Diagnostic_Type;
	outbuffer[byte_counter++] = Level;
	outbuffer[byte_counter++] = Diagnostic_Message;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+8+1;
	return 1;
}
int encode_TestMessageCounterSerial(int* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x14;
	outbuffer[byte_counter++] = 8;
	outbuffer[byte_counter++] = value1;
	outbuffer[byte_counter++] = value2;
	outbuffer[byte_counter++] = value3;
	outbuffer[byte_counter++] = value4;
	outbuffer[byte_counter++] = value5;
	outbuffer[byte_counter++] = value6;
	outbuffer[byte_counter++] = value7;
	outbuffer[byte_counter++] = value8;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+8+1;
	return 1;
}
int decode_TestMessageCounterSerial(int* inpacket,int length,int checksum,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
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
int decode_TestMessageCommandSerial(int* inpacket,int length,int checksum,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
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
int decode_Configure_DIO_PortASerial(int* inpacket,int length,int checksum,char* Pin1_Mode,char* Pin2_Mode,char* Pin3_Mode,char* Pin4_Mode,char* Pin5_Mode,char* Pin6_Mode,char* Pin7_Mode,char* Pin8_Mode)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*Pin1_Mode=inpacket[0];
	*Pin2_Mode=inpacket[1];
	*Pin3_Mode=inpacket[2];
	*Pin4_Mode=inpacket[3];
	*Pin5_Mode=inpacket[4];
	*Pin6_Mode=inpacket[5];
	*Pin7_Mode=inpacket[6];
	*Pin8_Mode=inpacket[7];
	return 1;
}
int encode_ModeSerial(int* outbuffer,int* length,char Mode)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x17;
	outbuffer[byte_counter++] = 8;
	outbuffer[byte_counter++] = Mode;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+8+1;
	return 1;
}
int decode_ModeSerial(int* inpacket,int length,int checksum,char* Mode)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*Mode=inpacket[0];
	return 1;
}
int decode_Set_DIO_PortASerial(int* inpacket,int length,int checksum,char* Pin1_Value,char* Pin2_Value,char* Pin3_Value,char* Pin4_Value,char* Pin5_Value,char* Pin6_Value,char* Pin7_Value,char* Pin8_Value)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*Pin1_Value=inpacket[0];
	*Pin2_Value=inpacket[1];
	*Pin3_Value=inpacket[2];
	*Pin4_Value=inpacket[3];
	*Pin5_Value=inpacket[4];
	*Pin6_Value=inpacket[5];
	*Pin7_Value=inpacket[6];
	*Pin8_Value=inpacket[7];
	return 1;
}
int encode_Get_ANA_PortASerial(int* outbuffer,int* length,int Pin1_Value,int Pin2_Value,int Pin3_Value,int Pin4_Value)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x19;
	outbuffer[byte_counter++] = 8;
	int v_Pin1_Value1 = Pin1_Value >> 8;
	outbuffer[byte_counter++] = v_Pin1_Value1;
	int v_Pin1_Value2 = Pin1_Value -(v_Pin1_Value1 << 8);
	outbuffer[byte_counter++] = v_Pin1_Value2;
	int v_Pin2_Value1 = Pin2_Value >> 8;
	outbuffer[byte_counter++] = v_Pin2_Value1;
	int v_Pin2_Value2 = Pin2_Value -(v_Pin2_Value1 << 8);
	outbuffer[byte_counter++] = v_Pin2_Value2;
	int v_Pin3_Value1 = Pin3_Value >> 8;
	outbuffer[byte_counter++] = v_Pin3_Value1;
	int v_Pin3_Value2 = Pin3_Value -(v_Pin3_Value1 << 8);
	outbuffer[byte_counter++] = v_Pin3_Value2;
	int v_Pin4_Value1 = Pin4_Value >> 8;
	outbuffer[byte_counter++] = v_Pin4_Value1;
	int v_Pin4_Value2 = Pin4_Value -(v_Pin4_Value1 << 8);
	outbuffer[byte_counter++] = v_Pin4_Value2;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+8+1;
	return 1;
}
int encode_Get_ANA_PortBSerial(int* outbuffer,int* length,int Pin1_Value,int Pin2_Value,int Pin3_Value,int Pin4_Value)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x20;
	outbuffer[byte_counter++] = 8;
	int v_Pin1_Value1 = Pin1_Value >> 8;
	outbuffer[byte_counter++] = v_Pin1_Value1;
	int v_Pin1_Value2 = Pin1_Value -(v_Pin1_Value1 << 8);
	outbuffer[byte_counter++] = v_Pin1_Value2;
	int v_Pin2_Value1 = Pin2_Value >> 8;
	outbuffer[byte_counter++] = v_Pin2_Value1;
	int v_Pin2_Value2 = Pin2_Value -(v_Pin2_Value1 << 8);
	outbuffer[byte_counter++] = v_Pin2_Value2;
	int v_Pin3_Value1 = Pin3_Value >> 8;
	outbuffer[byte_counter++] = v_Pin3_Value1;
	int v_Pin3_Value2 = Pin3_Value -(v_Pin3_Value1 << 8);
	outbuffer[byte_counter++] = v_Pin3_Value2;
	int v_Pin4_Value1 = Pin4_Value >> 8;
	outbuffer[byte_counter++] = v_Pin4_Value1;
	int v_Pin4_Value2 = Pin4_Value -(v_Pin4_Value1 << 8);
	outbuffer[byte_counter++] = v_Pin4_Value2;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+8+1;
	return 1;
}
int decode_Configure_DIO_PortBSerial(int* inpacket,int length,int checksum,char* Pin1_Mode,char* Pin2_Mode,char* Pin3_Mode,char* Pin4_Mode,char* Pin5_Mode,char* Pin6_Mode,char* Pin7_Mode,char* Pin8_Mode)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*Pin1_Mode=inpacket[0];
	*Pin2_Mode=inpacket[1];
	*Pin3_Mode=inpacket[2];
	*Pin4_Mode=inpacket[3];
	*Pin5_Mode=inpacket[4];
	*Pin6_Mode=inpacket[5];
	*Pin7_Mode=inpacket[6];
	*Pin8_Mode=inpacket[7];
	return 1;
}
int decode_Set_DIO_PortBSerial(int* inpacket,int length,int checksum,char* Pin1_Value,char* Pin2_Value,char* Pin3_Value,char* Pin4_Value,char* Pin5_Value,char* Pin6_Value,char* Pin7_Value,char* Pin8_Value)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*Pin1_Value=inpacket[0];
	*Pin2_Value=inpacket[1];
	*Pin3_Value=inpacket[2];
	*Pin4_Value=inpacket[3];
	*Pin5_Value=inpacket[4];
	*Pin6_Value=inpacket[5];
	*Pin7_Value=inpacket[6];
	*Pin8_Value=inpacket[7];
	return 1;
}
int encode_Get_DIO_PortASerial(int* outbuffer,int* length,char Pin1_Value,char Pin2_Value,char Pin3_Value,char Pin4_Value,char Pin5_Value,char Pin6_Value,char Pin7_Value,char Pin8_Value)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x23;
	outbuffer[byte_counter++] = 8;
	outbuffer[byte_counter++] = Pin1_Value;
	outbuffer[byte_counter++] = Pin2_Value;
	outbuffer[byte_counter++] = Pin3_Value;
	outbuffer[byte_counter++] = Pin4_Value;
	outbuffer[byte_counter++] = Pin5_Value;
	outbuffer[byte_counter++] = Pin6_Value;
	outbuffer[byte_counter++] = Pin7_Value;
	outbuffer[byte_counter++] = Pin8_Value;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+8+1;
	return 1;
}
int encode_Get_DIO_PortBSerial(int* outbuffer,int* length,char Pin1_Value,char Pin2_Value,char Pin3_Value,char Pin4_Value,char Pin5_Value,char Pin6_Value,char Pin7_Value,char Pin8_Value)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x24;
	outbuffer[byte_counter++] = 8;
	outbuffer[byte_counter++] = Pin1_Value;
	outbuffer[byte_counter++] = Pin2_Value;
	outbuffer[byte_counter++] = Pin3_Value;
	outbuffer[byte_counter++] = Pin4_Value;
	outbuffer[byte_counter++] = Pin5_Value;
	outbuffer[byte_counter++] = Pin6_Value;
	outbuffer[byte_counter++] = Pin7_Value;
	outbuffer[byte_counter++] = Pin8_Value;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+8+1;
	return 1;
}
int encode_FirmwareVersionSerial(int* outbuffer,int* length,char majorVersion,char minorVersion,char buildNumber)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x25;
	outbuffer[byte_counter++] = 8;
	outbuffer[byte_counter++] = majorVersion;
	outbuffer[byte_counter++] = minorVersion;
	outbuffer[byte_counter++] = buildNumber;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	outbuffer[byte_counter++] = 0;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+8+1;
	return 1;
}
int decode_Arm_CommandSerial(int* inpacket,int length,int checksum,char* Command)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*Command=inpacket[0];
	return 1;
}
int decode_Setup_ControlGroupSerial(int* inpacket,int length,int checksum,char* ID,char* Mode,char* Input_Port,char* Input_PinMode,char* Input_PinNumber,char* Output_Port,char* Output_PinMode,char* Output_PinNUmber)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*ID=inpacket[0];
	*Mode=inpacket[1];
	*Input_Port=inpacket[2];
	*Input_PinMode=inpacket[3];
	*Input_PinNumber=inpacket[4];
	*Output_Port=inpacket[5];
	*Output_PinMode=inpacket[6];
	*Output_PinNUmber=inpacket[7];
	return 1;
}
int decode_Tune_ControlGroupSerial(int* inpacket,int length,int checksum,char* ID,char* Mode,int* Proportional_Gain,int* Integral_Gain,int* Derivative_Gain)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*ID=inpacket[0];
	*Mode=inpacket[1];
	int v_Proportional_Gain1=inpacket[2]<<8;
	*Proportional_Gain=inpacket[3] + v_Proportional_Gain1;
	int v_Integral_Gain1=inpacket[4]<<8;
	*Integral_Gain=inpacket[5] + v_Integral_Gain1;
	int v_Derivative_Gain1=inpacket[6]<<8;
	*Derivative_Gain=inpacket[7] + v_Derivative_Gain1;
	return 1;
}
int decode_Arm_StatusSerial(int* inpacket,int length,int checksum,char* Status)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*Status=inpacket[0];
	return 1;
}
