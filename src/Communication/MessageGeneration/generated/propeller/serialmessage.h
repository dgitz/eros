/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2016-12-27 16:05:02.204816***/
/***Target: Parallax Propeller ***/
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
int encode_UserMessageSerial(int* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8,char value9,char value10,char value11,char value12);
int decode_UserMessageSerial(int* inpacket,int length,int checksum,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8,char* value9,char* value10,char* value11,char* value12);
int encode_DiagnosticSerial(int* outbuffer,int* length,char System,char SubSystem,char Component,char Diagnostic_Type,char Level,char Diagnostic_Message);
int encode_TestMessageCounterSerial(int* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8);
int decode_TestMessageCounterSerial(int* inpacket,int length,int checksum,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8);
int decode_TestMessageCommandSerial(int* inpacket,int length,int checksum,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8);
int encode_Configure_DIO_PortSerial(int* outbuffer,int* length,char ShieldID,char PortID,char Pin1_Mode,char Pin2_Mode,char Pin3_Mode,char Pin4_Mode,char Pin5_Mode,char Pin6_Mode,char Pin7_Mode,char Pin8_Mode);
int decode_Configure_DIO_PortSerial(int* inpacket,int length,int checksum,char* ShieldID,char* PortID,char* Pin1_Mode,char* Pin2_Mode,char* Pin3_Mode,char* Pin4_Mode,char* Pin5_Mode,char* Pin6_Mode,char* Pin7_Mode,char* Pin8_Mode);
int encode_ModeSerial(int* outbuffer,int* length,char DeviceType,char ID,char Mode);
int decode_ModeSerial(int* inpacket,int length,int checksum,char* DeviceType,char* ID,char* Mode);
int encode_Set_DIO_PortSerial(int* outbuffer,int* length,char ShieldID,char PortID,unsigned char Pin1_Value,unsigned char Pin2_Value,unsigned char Pin3_Value,unsigned char Pin4_Value,unsigned char Pin5_Value,unsigned char Pin6_Value,unsigned char Pin7_Value,unsigned char Pin8_Value);
int decode_Set_DIO_PortSerial(int* inpacket,int length,int checksum,char* ShieldID,char* PortID,unsigned char* Pin1_Value,unsigned char* Pin2_Value,unsigned char* Pin3_Value,unsigned char* Pin4_Value,unsigned char* Pin5_Value,unsigned char* Pin6_Value,unsigned char* Pin7_Value,unsigned char* Pin8_Value);
int encode_Get_DIO_PortSerial(int* outbuffer,int* length,char ShieldID,char PortID,char Pin1_Value,char Pin2_Value,char Pin3_Value,char Pin4_Value,char Pin5_Value,char Pin6_Value,char Pin7_Value,char Pin8_Value);
int encode_Get_ANA_PortSerial(int* outbuffer,int* length,char ShieldID,char PortID,int Pin1_Value,int Pin2_Value,int Pin3_Value,int Pin4_Value);
int encode_FirmwareVersionSerial(int* outbuffer,int* length,char majorVersion,char minorVersion,char buildNumber);
int decode_Arm_CommandSerial(int* inpacket,int length,int checksum,char* Command);
int decode_Setup_ControlGroupSerial(int* inpacket,int length,int checksum,char* ID,char* Mode,char* Input_Port,char* Input_PinMode,char* Input_PinNumber,char* Output_Port,char* Output_PinMode,char* Output_PinNUmber);
int decode_Tune_ControlGroupSerial(int* inpacket,int length,int checksum,char* ID,char* Mode,int* Proportional_Gain,int* Integral_Gain,int* Derivative_Gain);
int decode_Arm_StatusSerial(int* inpacket,int length,int checksum,char* Status);
int decode_Set_DIO_Port_DefaultValueSerial(int* inpacket,int length,int checksum,char* ShieldID,char* PortID,char* Pin1_Value,char* Pin2_Value,char* Pin3_Value,char* Pin4_Value,char* Pin5_Value,char* Pin6_Value,char* Pin7_Value,char* Pin8_Value);
int encode_Configure_ShieldSerial(int* outbuffer,int* length,char ShieldCount,char ShieldType,char ShieldID,char PortCount);
int decode_Configure_ShieldSerial(int* inpacket,int length,int checksum,char* ShieldCount,char* ShieldType,char* ShieldID,char* PortCount);
#endif