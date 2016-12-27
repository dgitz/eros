/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2016-12-27 16:05:02.204751***/
/***Target: Raspberry Pi OR Arduino ***/
#ifndef SERIALMESSAGE_H
#define SERIALMESSAGE_H
#define SERIAL_UserMessage_ID 0x1
#define SERIAL_Diagnostic_ID 0x12
#define SERIAL_TestMessageCounter_ID 0x14
#define SERIAL_TestMessageCommand_ID 0x15
#define SERIAL_Configure_DIO_Port_ID 0x16
#define SERIAL_Mode_ID 0x17
#define SERIAL_Set_DIO_Port_ID 0x18
#define SERIAL_Get_DIO_Port_ID 0x19
#define SERIAL_Get_ANA_Port_ID 0x20
#define SERIAL_FirmwareVersion_ID 0x25
#define SERIAL_Arm_Command_ID 0x27
#define SERIAL_Setup_ControlGroup_ID 0x28
#define SERIAL_Tune_ControlGroup_ID 0x29
#define SERIAL_Arm_Status_ID 0x30
#define SERIAL_Set_DIO_Port_DefaultValue_ID 0x32
#define SERIAL_Configure_Shield_ID 0x33

class SerialMessageHandler
{
public:
	SerialMessageHandler();
	~SerialMessageHandler();
	int encode_UserMessageSerial(char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8,char value9,char value10,char value11,char value12);
	int decode_UserMessageSerial(unsigned char* inpacket,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8,char* value9,char* value10,char* value11,char* value12);
	int decode_DiagnosticSerial(unsigned char* inpacket,char* System,char* SubSystem,char* Component,char* Diagnostic_Type,char* Level,char* Diagnostic_Message);
	int encode_TestMessageCounterSerial(char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8);
	int decode_TestMessageCounterSerial(unsigned char* inpacket,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8);
	int encode_TestMessageCommandSerial(char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8);
	int encode_Configure_DIO_PortSerial(char* outbuffer,int* length,char ShieldID,char PortID,char Pin1_Mode,char Pin2_Mode,char Pin3_Mode,char Pin4_Mode,char Pin5_Mode,char Pin6_Mode,char Pin7_Mode,char Pin8_Mode);
	int decode_Configure_DIO_PortSerial(unsigned char* inpacket,char* ShieldID,char* PortID,char* Pin1_Mode,char* Pin2_Mode,char* Pin3_Mode,char* Pin4_Mode,char* Pin5_Mode,char* Pin6_Mode,char* Pin7_Mode,char* Pin8_Mode);
	int encode_ModeSerial(char* outbuffer,int* length,char DeviceType,char ID,char Mode);
	int decode_ModeSerial(unsigned char* inpacket,char* DeviceType,char* ID,char* Mode);
	int encode_Set_DIO_PortSerial(char* outbuffer,int* length,char ShieldID,char PortID,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value);
	int decode_Set_DIO_PortSerial(unsigned char* inpacket,char* ShieldID,char* PortID,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value);
	int decode_Get_DIO_PortSerial(unsigned char* inpacket,char* ShieldID,char* PortID,char* Pin1_Value,char* Pin2_Value,char* Pin3_Value,char* Pin4_Value,char* Pin5_Value,char* Pin6_Value,char* Pin7_Value,char* Pin8_Value);
	int decode_Get_ANA_PortSerial(unsigned char* inpacket,char* ShieldID,char* PortID,int* Pin1_Value,int* Pin2_Value,int* Pin3_Value,int* Pin4_Value);
	int decode_FirmwareVersionSerial(unsigned char* inpacket,char* majorVersion,char* minorVersion,char* buildNumber);
	int encode_Arm_CommandSerial(char* outbuffer,int* length,char Command);
	int encode_Setup_ControlGroupSerial(char* outbuffer,int* length,char ID,char Mode,char Input_Port,char Input_PinMode,char Input_PinNumber,char Output_Port,char Output_PinMode,char Output_PinNUmber);
	int encode_Tune_ControlGroupSerial(char* outbuffer,int* length,char ID,char Mode,int Proportional_Gain,int Integral_Gain,int Derivative_Gain);
	int encode_Arm_StatusSerial(char* outbuffer,int* length,char Status);
	int encode_Set_DIO_Port_DefaultValueSerial(char* outbuffer,int* length,char ShieldID,char PortID,char Pin1_Value,char Pin2_Value,char Pin3_Value,char Pin4_Value,char Pin5_Value,char Pin6_Value,char Pin7_Value,char Pin8_Value);
	int encode_Configure_ShieldSerial(char* outbuffer,int* length,char ShieldCount,char ShieldType,char ShieldID,char PortCount);
	int decode_Configure_ShieldSerial(unsigned char* inpacket,char* ShieldCount,char* ShieldType,char* ShieldID,char* PortCount);
private:
};
#endif