/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2017-11-21 20:37:54.556217***/
/***Target: Parallax Propeller ***/
#include "serialmessage.h"
int encode_UserMessageSerial(int* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x1;
	outbuffer[byte_counter++] = 12;
	outbuffer[byte_counter++] = value1;
	outbuffer[byte_counter++] = value2;
	outbuffer[byte_counter++] = value3;
	outbuffer[byte_counter++] = value4;
	outbuffer[byte_counter++] = value5;
	outbuffer[byte_counter++] = value6;
	outbuffer[byte_counter++] = value7;
	outbuffer[byte_counter++] = value8;
	outbuffer[byte_counter++] = value9;
	outbuffer[byte_counter++] = value10;
	outbuffer[byte_counter++] = value11;
	outbuffer[byte_counter++] = value12;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+12+1;
	return 1;
}
int decode_UserMessageSerial(int* inpacket,int length,int checksum,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8,unsigned char* value9,unsigned char* value10,unsigned char* value11,unsigned char* value12)
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
	*value9=inpacket[8];
	*value10=inpacket[9];
	*value11=inpacket[10];
	*value12=inpacket[11];
	return 1;
}
int encode_CommandSerial(int* outbuffer,int* length,unsigned char Command,unsigned char Option1,unsigned char Option2,unsigned char Option3)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x2;
	outbuffer[byte_counter++] = 4;
	outbuffer[byte_counter++] = Command;
	outbuffer[byte_counter++] = Option1;
	outbuffer[byte_counter++] = Option2;
	outbuffer[byte_counter++] = Option3;
	int checksum = 0;
	for(int i = 3; i < (3+4);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+4+1;
	return 1;
}
int decode_CommandSerial(int* inpacket,int length,int checksum,unsigned char* Command,unsigned char* Option1,unsigned char* Option2,unsigned char* Option3)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*Command=inpacket[0];
	*Option1=inpacket[1];
	*Option2=inpacket[2];
	*Option3=inpacket[3];
	return 1;
}
int encode_DiagnosticSerial(int* outbuffer,int* length,unsigned char System,unsigned char SubSystem,unsigned char Component,unsigned char Diagnostic_Type,unsigned char Level,unsigned char Diagnostic_Message)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x12;
	outbuffer[byte_counter++] = 6;
	outbuffer[byte_counter++] = System;
	outbuffer[byte_counter++] = SubSystem;
	outbuffer[byte_counter++] = Component;
	outbuffer[byte_counter++] = Diagnostic_Type;
	outbuffer[byte_counter++] = Level;
	outbuffer[byte_counter++] = Diagnostic_Message;
	int checksum = 0;
	for(int i = 3; i < (3+6);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+6+1;
	return 1;
}
int decode_DiagnosticSerial(int* inpacket,int length,int checksum,unsigned char* System,unsigned char* SubSystem,unsigned char* Component,unsigned char* Diagnostic_Type,unsigned char* Level,unsigned char* Diagnostic_Message)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*System=inpacket[0];
	*SubSystem=inpacket[1];
	*Component=inpacket[2];
	*Diagnostic_Type=inpacket[3];
	*Level=inpacket[4];
	*Diagnostic_Message=inpacket[5];
	return 1;
}
int decode_TestMessageCommandSerial(int* inpacket,int length,int checksum,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8)
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
int encode_Configure_DIO_PortSerial(int* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char MessageIndex,unsigned char MessageCount,unsigned char Pin1_Mode,unsigned char Pin2_Mode,unsigned char Pin3_Mode,unsigned char Pin4_Mode,unsigned char Pin5_Mode,unsigned char Pin6_Mode,unsigned char Pin7_Mode,unsigned char Pin8_Mode)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x16;
	outbuffer[byte_counter++] = 12;
	outbuffer[byte_counter++] = ShieldID;
	outbuffer[byte_counter++] = PortID;
	outbuffer[byte_counter++] = MessageIndex;
	outbuffer[byte_counter++] = MessageCount;
	outbuffer[byte_counter++] = Pin1_Mode;
	outbuffer[byte_counter++] = Pin2_Mode;
	outbuffer[byte_counter++] = Pin3_Mode;
	outbuffer[byte_counter++] = Pin4_Mode;
	outbuffer[byte_counter++] = Pin5_Mode;
	outbuffer[byte_counter++] = Pin6_Mode;
	outbuffer[byte_counter++] = Pin7_Mode;
	outbuffer[byte_counter++] = Pin8_Mode;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+12+1;
	return 1;
}
int decode_Configure_DIO_PortSerial(int* inpacket,int length,int checksum,unsigned char* ShieldID,unsigned char* PortID,unsigned char* MessageIndex,unsigned char* MessageCount,unsigned char* Pin1_Mode,unsigned char* Pin2_Mode,unsigned char* Pin3_Mode,unsigned char* Pin4_Mode,unsigned char* Pin5_Mode,unsigned char* Pin6_Mode,unsigned char* Pin7_Mode,unsigned char* Pin8_Mode)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
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
int encode_ModeSerial(int* outbuffer,int* length,unsigned char DeviceType,unsigned char ID,unsigned char Mode)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x17;
	outbuffer[byte_counter++] = 3;
	outbuffer[byte_counter++] = DeviceType;
	outbuffer[byte_counter++] = ID;
	outbuffer[byte_counter++] = Mode;
	int checksum = 0;
	for(int i = 3; i < (3+3);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+3+1;
	return 1;
}
int decode_ModeSerial(int* inpacket,int length,int checksum,unsigned char* DeviceType,unsigned char* ID,unsigned char* Mode)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*DeviceType=inpacket[0];
	*ID=inpacket[1];
	*Mode=inpacket[2];
	return 1;
}
int encode_Set_DIO_PortSerial(int* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x18;
	outbuffer[byte_counter++] = 10;
	outbuffer[byte_counter++] = ShieldID;
	outbuffer[byte_counter++] = PortID;
	outbuffer[byte_counter++] = Pin1_Value;
	outbuffer[byte_counter++] = Pin2_Value;
	outbuffer[byte_counter++] = Pin3_Value;
	outbuffer[byte_counter++] = Pin4_Value;
	outbuffer[byte_counter++] = Pin5_Value;
	outbuffer[byte_counter++] = Pin6_Value;
	outbuffer[byte_counter++] = Pin7_Value;
	outbuffer[byte_counter++] = Pin8_Value;
	int checksum = 0;
	for(int i = 3; i < (3+10);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+10+1;
	return 1;
}
int decode_Set_DIO_PortSerial(int* inpacket,int length,int checksum,unsigned char* ShieldID,unsigned char* PortID,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
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
int encode_FirmwareVersionSerial(int* outbuffer,int* length,unsigned char majorVersion,unsigned char minorVersion,unsigned char buildNumber)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x25;
	outbuffer[byte_counter++] = 3;
	outbuffer[byte_counter++] = majorVersion;
	outbuffer[byte_counter++] = minorVersion;
	outbuffer[byte_counter++] = buildNumber;
	int checksum = 0;
	for(int i = 3; i < (3+3);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+3+1;
	return 1;
}
int encode_Arm_StatusSerial(int* outbuffer,int* length,unsigned char Status)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x30;
	outbuffer[byte_counter++] = 1;
	outbuffer[byte_counter++] = Status;
	int checksum = 0;
	for(int i = 3; i < (3+1);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+1+1;
	return 1;
}
int decode_Arm_StatusSerial(int* inpacket,int length,int checksum,unsigned char* Status)
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
int encode_Set_DIO_Port_DefaultValueSerial(int* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char MessageIndex,unsigned char MessageCount,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x32;
	outbuffer[byte_counter++] = 12;
	outbuffer[byte_counter++] = ShieldID;
	outbuffer[byte_counter++] = PortID;
	outbuffer[byte_counter++] = MessageIndex;
	outbuffer[byte_counter++] = MessageCount;
	outbuffer[byte_counter++] = Pin1_Value;
	outbuffer[byte_counter++] = Pin2_Value;
	outbuffer[byte_counter++] = Pin3_Value;
	outbuffer[byte_counter++] = Pin4_Value;
	outbuffer[byte_counter++] = Pin5_Value;
	outbuffer[byte_counter++] = Pin6_Value;
	outbuffer[byte_counter++] = Pin7_Value;
	outbuffer[byte_counter++] = Pin8_Value;
	int checksum = 0;
	for(int i = 3; i < (3+12);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+12+1;
	return 1;
}
int decode_Set_DIO_Port_DefaultValueSerial(int* inpacket,int length,int checksum,unsigned char* ShieldID,unsigned char* PortID,unsigned char* MessageIndex,unsigned char* MessageCount,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
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
int encode_PPSSerial(int* outbuffer,int* length,unsigned char counter)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x35;
	outbuffer[byte_counter++] = 1;
	outbuffer[byte_counter++] = counter;
	int checksum = 0;
	for(int i = 3; i < (3+1);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+1+1;
	return 1;
}
int decode_PPSSerial(int* inpacket,int length,int checksum,unsigned char* counter)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
	*counter=inpacket[0];
	return 1;
}
int encode_Configure_ANA_PortSerial(int* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char MessageIndex,unsigned char MessageCount,unsigned char Pin1_Mode,unsigned char Pin2_Mode,unsigned char Pin3_Mode,unsigned char Pin4_Mode)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x36;
	outbuffer[byte_counter++] = 8;
	outbuffer[byte_counter++] = ShieldID;
	outbuffer[byte_counter++] = PortID;
	outbuffer[byte_counter++] = MessageIndex;
	outbuffer[byte_counter++] = MessageCount;
	outbuffer[byte_counter++] = Pin1_Mode;
	outbuffer[byte_counter++] = Pin2_Mode;
	outbuffer[byte_counter++] = Pin3_Mode;
	outbuffer[byte_counter++] = Pin4_Mode;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+8+1;
	return 1;
}
int decode_Configure_ANA_PortSerial(int* inpacket,int length,int checksum,unsigned char* ShieldID,unsigned char* PortID,unsigned char* MessageIndex,unsigned char* MessageCount,unsigned char* Pin1_Mode,unsigned char* Pin2_Mode,unsigned char* Pin3_Mode,unsigned char* Pin4_Mode)
{
	int computed_checksum = 0;
	for(int i = 0; i < length; i++)
	{
		computed_checksum ^= inpacket[i];
	}
	if(computed_checksum != checksum) { return -1; }
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
int encode_IDSerial(int* outbuffer,int* length,unsigned char DeviceID,unsigned long PartNumber)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x40;
	outbuffer[byte_counter++] = 5;
	outbuffer[byte_counter++] = DeviceID;
	int v_PartNumber1 = PartNumber >> 8;
	outbuffer[byte_counter++] = v_PartNumber1;
	int v_PartNumber2 = PartNumber -(v_PartNumber1 << 8);
	outbuffer[byte_counter++] = v_PartNumber2;
	int checksum = 0;
	for(int i = 3; i < (3+5);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+5+1;
	return 1;
}
int encode_IMUSerial(int* outbuffer,int* length,unsigned long timemS,int counter,long AccX_mg,long AccY_mg,long AccZ_mg,long GyroX_mdegps,long GyroY_mdepgs,long GyroZ_mdegps,long MagX,long MagY,long MagZ)
{
	int byte_counter=0;
	outbuffer[byte_counter++] = 0xAB;
	outbuffer[byte_counter++] = 0x41;
	outbuffer[byte_counter++] = 42;
	int v_timemS1 = timemS >> 8;
	outbuffer[byte_counter++] = v_timemS1;
	int v_timemS2 = timemS -(v_timemS1 << 8);
	outbuffer[byte_counter++] = v_timemS2;
	int v_counter1 = counter >> 8;
	outbuffer[byte_counter++] = v_counter1;
	int v_counter2 = counter -(v_counter1 << 8);
	outbuffer[byte_counter++] = v_counter2;
	int v_AccX_mg1 = AccX_mg >> 8;
	outbuffer[byte_counter++] = v_AccX_mg1;
	int v_AccX_mg2 = AccX_mg -(v_AccX_mg1 << 8);
	outbuffer[byte_counter++] = v_AccX_mg2;
	int v_AccY_mg1 = AccY_mg >> 8;
	outbuffer[byte_counter++] = v_AccY_mg1;
	int v_AccY_mg2 = AccY_mg -(v_AccY_mg1 << 8);
	outbuffer[byte_counter++] = v_AccY_mg2;
	int v_AccZ_mg1 = AccZ_mg >> 8;
	outbuffer[byte_counter++] = v_AccZ_mg1;
	int v_AccZ_mg2 = AccZ_mg -(v_AccZ_mg1 << 8);
	outbuffer[byte_counter++] = v_AccZ_mg2;
	int v_GyroX_mdegps1 = GyroX_mdegps >> 8;
	outbuffer[byte_counter++] = v_GyroX_mdegps1;
	int v_GyroX_mdegps2 = GyroX_mdegps -(v_GyroX_mdegps1 << 8);
	outbuffer[byte_counter++] = v_GyroX_mdegps2;
	int v_GyroY_mdepgs1 = GyroY_mdepgs >> 8;
	outbuffer[byte_counter++] = v_GyroY_mdepgs1;
	int v_GyroY_mdepgs2 = GyroY_mdepgs -(v_GyroY_mdepgs1 << 8);
	outbuffer[byte_counter++] = v_GyroY_mdepgs2;
	int v_GyroZ_mdegps1 = GyroZ_mdegps >> 8;
	outbuffer[byte_counter++] = v_GyroZ_mdegps1;
	int v_GyroZ_mdegps2 = GyroZ_mdegps -(v_GyroZ_mdegps1 << 8);
	outbuffer[byte_counter++] = v_GyroZ_mdegps2;
	int v_MagX1 = MagX >> 8;
	outbuffer[byte_counter++] = v_MagX1;
	int v_MagX2 = MagX -(v_MagX1 << 8);
	outbuffer[byte_counter++] = v_MagX2;
	int v_MagY1 = MagY >> 8;
	outbuffer[byte_counter++] = v_MagY1;
	int v_MagY2 = MagY -(v_MagY1 << 8);
	outbuffer[byte_counter++] = v_MagY2;
	int v_MagZ1 = MagZ >> 8;
	outbuffer[byte_counter++] = v_MagZ1;
	int v_MagZ2 = MagZ -(v_MagZ1 << 8);
	outbuffer[byte_counter++] = v_MagZ2;
	int checksum = 0;
	for(int i = 3; i < (3+42);i++)
	{
		checksum ^= outbuffer[i];
	}
	outbuffer[byte_counter] = checksum;
	length[0] = 3+42+1;
	return 1;
}
