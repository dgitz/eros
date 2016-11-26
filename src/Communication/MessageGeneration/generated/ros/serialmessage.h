/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2016-11-26 08:18:18.895549***/
/***Target: Raspberry Pi ***/
#ifndef SERIALMESSAGE_H
#define SERIALMESSAGE_H
#define SERIAL_Diagnostic_ID 0x12
#define SERIAL_TestMessageCounter_ID 0x14
#define SERIAL_TestMessageCommand_ID 0x15
#define SERIAL_Configure_DIO_PortA_ID 0x16
#define SERIAL_Mode_ID 0x17
#define SERIAL_Set_DIO_PortA_ID 0x18
#define SERIAL_Get_ANA_PortA_ID 0x19
#define SERIAL_Get_ANA_PortB_ID 0x20
#define SERIAL_Configure_DIO_PortB_ID 0x21
#define SERIAL_Set_DIO_PortB_ID 0x22
#define SERIAL_Get_DIO_PortA_ID 0x23
#define SERIAL_Get_DIO_PortB_ID 0x24
#define SERIAL_FirmwareVersion_ID 0x25
#define SERIAL_Arm_Command_ID 0x27
#define SERIAL_Setup_ControlGroup_ID 0x28
#define SERIAL_Tune_ControlGroup_ID 0x29
#define SERIAL_Arm_Status_ID 0x30
#define SERIAL_Set_DIO_PortA_DefaultValue_ID 0x32
#define SERIAL_Set_DIO_PortB_DefaultValue_ID 0x33

class SerialMessageHandler
{
public:
	SerialMessageHandler();
	~SerialMessageHandler();
	int decode_DiagnosticSerial(unsigned char* inpacket,char* System,char* SubSystem,char* Component,char* Diagnostic_Type,char* Level,char* Diagnostic_Message);
	int encode_TestMessageCounterSerial(char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8);
	int decode_TestMessageCounterSerial(unsigned char* inpacket,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8);
	int encode_TestMessageCommandSerial(char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8);
	int encode_Configure_DIO_PortASerial(char* outbuffer,int* length,char Pin1_Mode,char Pin2_Mode,char Pin3_Mode,char Pin4_Mode,char Pin5_Mode,char Pin6_Mode,char Pin7_Mode,char Pin8_Mode);
	int encode_ModeSerial(char* outbuffer,int* length,char Mode);
	int decode_ModeSerial(unsigned char* inpacket,char* Mode);
	int encode_Set_DIO_PortASerial(char* outbuffer,int* length,char Pin1_Value,char Pin2_Value,char Pin3_Value,char Pin4_Value,char Pin5_Value,char Pin6_Value,char Pin7_Value,char Pin8_Value);
	int decode_Get_ANA_PortASerial(unsigned char* inpacket,int* Pin1_Value,int* Pin2_Value,int* Pin3_Value,int* Pin4_Value);
	int decode_Get_ANA_PortBSerial(unsigned char* inpacket,int* Pin1_Value,int* Pin2_Value,int* Pin3_Value,int* Pin4_Value);
	int encode_Configure_DIO_PortBSerial(char* outbuffer,int* length,char Pin1_Mode,char Pin2_Mode,char Pin3_Mode,char Pin4_Mode,char Pin5_Mode,char Pin6_Mode,char Pin7_Mode,char Pin8_Mode);
	int encode_Set_DIO_PortBSerial(char* outbuffer,int* length,char Pin1_Value,char Pin2_Value,char Pin3_Value,char Pin4_Value,char Pin5_Value,char Pin6_Value,char Pin7_Value,char Pin8_Value);
	int decode_Get_DIO_PortASerial(unsigned char* inpacket,char* Pin1_Value,char* Pin2_Value,char* Pin3_Value,char* Pin4_Value,char* Pin5_Value,char* Pin6_Value,char* Pin7_Value,char* Pin8_Value);
	int decode_Get_DIO_PortBSerial(unsigned char* inpacket,char* Pin1_Value,char* Pin2_Value,char* Pin3_Value,char* Pin4_Value,char* Pin5_Value,char* Pin6_Value,char* Pin7_Value,char* Pin8_Value);
	int decode_FirmwareVersionSerial(unsigned char* inpacket,char* majorVersion,char* minorVersion,char* buildNumber);
	int encode_Arm_CommandSerial(char* outbuffer,int* length,char Command);
	int encode_Setup_ControlGroupSerial(char* outbuffer,int* length,char ID,char Mode,char Input_Port,char Input_PinMode,char Input_PinNumber,char Output_Port,char Output_PinMode,char Output_PinNUmber);
	int encode_Tune_ControlGroupSerial(char* outbuffer,int* length,char ID,char Mode,int Proportional_Gain,int Integral_Gain,int Derivative_Gain);
	int encode_Arm_StatusSerial(char* outbuffer,int* length,char Status);
	int encode_Set_DIO_PortA_DefaultValueSerial(char* outbuffer,int* length,char Pin1_Value,char Pin2_Value,char Pin3_Value,char Pin4_Value,char Pin5_Value,char Pin6_Value,char Pin7_Value,char Pin8_Value);
	int encode_Set_DIO_PortB_DefaultValueSerial(char* outbuffer,int* length,char Pin1_Value,char Pin2_Value,char Pin3_Value,char Pin4_Value,char Pin5_Value,char Pin6_Value,char Pin7_Value,char Pin8_Value);
private:
};
#endif