/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2017-04-01 18:51:14.306661***/
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
#define SERIAL_PPS_ID 0x35

class SerialMessageHandler
{
public:
	SerialMessageHandler();
	~SerialMessageHandler();
	int encode_UserMessageSerial(char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8,unsigned char value9,unsigned char value10,unsigned char value11,unsigned char value12);
	int decode_UserMessageSerial(unsigned char* inpacket,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8,unsigned char* value9,unsigned char* value10,unsigned char* value11,unsigned char* value12);
	int encode_DiagnosticSerial(char* outbuffer,int* length,unsigned char System,unsigned char SubSystem,unsigned char Component,unsigned char Diagnostic_Type,unsigned char Level,unsigned char Diagnostic_Message);
	int decode_DiagnosticSerial(unsigned char* inpacket,unsigned char* System,unsigned char* SubSystem,unsigned char* Component,unsigned char* Diagnostic_Type,unsigned char* Level,unsigned char* Diagnostic_Message);
	int encode_TestMessageCounterSerial(char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8);
	int decode_TestMessageCounterSerial(unsigned char* inpacket,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8);
	int encode_TestMessageCommandSerial(char* outbuffer,int* length,unsigned char value1,unsigned char value2,unsigned char value3,unsigned char value4,unsigned char value5,unsigned char value6,unsigned char value7,unsigned char value8);
	int encode_Configure_DIO_PortSerial(char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char Pin1_Mode,unsigned char Pin2_Mode,unsigned char Pin3_Mode,unsigned char Pin4_Mode,unsigned char Pin5_Mode,unsigned char Pin6_Mode,unsigned char Pin7_Mode,unsigned char Pin8_Mode);
	int decode_Configure_DIO_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* Pin1_Mode,unsigned char* Pin2_Mode,unsigned char* Pin3_Mode,unsigned char* Pin4_Mode,unsigned char* Pin5_Mode,unsigned char* Pin6_Mode,unsigned char* Pin7_Mode,unsigned char* Pin8_Mode);
	int encode_ModeSerial(char* outbuffer,int* length,unsigned char DeviceType,unsigned char ID,unsigned char Mode);
	int decode_ModeSerial(unsigned char* inpacket,unsigned char* DeviceType,unsigned char* ID,unsigned char* Mode);
	int encode_Set_DIO_PortSerial(char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value);
	int decode_Set_DIO_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value);
	int decode_Get_DIO_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value);
	int decode_Get_ANA_PortSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,int* Pin1_Value,int* Pin2_Value,int* Pin3_Value,int* Pin4_Value);
	int decode_FirmwareVersionSerial(unsigned char* inpacket,unsigned char* majorVersion,unsigned char* minorVersion,unsigned char* buildNumber);
	int encode_Arm_CommandSerial(char* outbuffer,int* length,unsigned char Command);
	int decode_Arm_CommandSerial(unsigned char* inpacket,unsigned char* Command);
	int encode_Setup_ControlGroupSerial(char* outbuffer,int* length,char ID,char Mode,char Input_Port,unsigned char Input_PinMode,unsigned char Input_PinNumber,unsigned char Output_Port,unsigned char Output_PinMode,unsigned char Output_PinNUmber);
	int encode_Tune_ControlGroupSerial(char* outbuffer,int* length,unsigned char ID,unsigned char Mode,int Proportional_Gain,int Integral_Gain,int Derivative_Gain);
	int encode_Arm_StatusSerial(char* outbuffer,int* length,unsigned char Status);
	int decode_Arm_StatusSerial(unsigned char* inpacket,unsigned char* Status);
	int encode_Set_DIO_Port_DefaultValueSerial(char* outbuffer,int* length,unsigned char ShieldID,unsigned char PortID,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value);
	int decode_Set_DIO_Port_DefaultValueSerial(unsigned char* inpacket,unsigned char* ShieldID,unsigned char* PortID,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value);
	int encode_Configure_ShieldSerial(char* outbuffer,int* length,unsigned char ShieldCount,unsigned char ShieldType,unsigned char ShieldID,unsigned char PortCount);
	int decode_Configure_ShieldSerial(unsigned char* inpacket,unsigned char* ShieldCount,unsigned char* ShieldType,unsigned char* ShieldID,unsigned char* PortCount);
	int encode_PPSSerial(char* outbuffer,int* length,unsigned char counter);
	int decode_PPSSerial(unsigned char* inpacket,unsigned char* counter);
private:
};
#endif