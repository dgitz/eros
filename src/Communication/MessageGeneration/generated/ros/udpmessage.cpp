/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2020-01-25 07:38:19.291281***/
#include "../include/udpmessage.h"
UDPMessageHandler::UDPMessageHandler(){}
UDPMessageHandler::~UDPMessageHandler(){}
int UDPMessageHandler::decode_CommandUDP(std::vector<std::string> items,uint8_t* Command,uint8_t* Option1,uint8_t* Option2,uint8_t* Option3,std::string* CommandText,std::string* Description)
{
	char tempstr[8];
	sprintf(tempstr,"0x%s",items.at(0).c_str());
	int id = (int)strtol(tempstr,NULL,0);
	if(id != UDP_Command_ID){ return 0; }
	if(items.size() != 7){ return 0; }
	*Command=(uint8_t)atoi(items.at(1).c_str());
	*Option1=(uint8_t)atoi(items.at(2).c_str());
	*Option2=(uint8_t)atoi(items.at(3).c_str());
	*Option3=(uint8_t)atoi(items.at(4).c_str());
	*CommandText=items.at(5);
	*Description=items.at(6);
	return 1;
}
int UDPMessageHandler::decode_RemoteControlUDP(std::vector<std::string> items,uint64_t* Current_Timestamp,int* axis1,int* axis2,int* axis3,int* axis4,int* axis5,int* axis6,int* axis7,int* axis8,uint8_t* button1,uint8_t* button2,uint8_t* button3,uint8_t* button4,uint8_t* button5,uint8_t* button6,uint8_t* button7,uint8_t* button8)
{
	char tempstr[8];
	sprintf(tempstr,"0x%s",items.at(0).c_str());
	int id = (int)strtol(tempstr,NULL,0);
	if(id != UDP_RemoteControl_ID){ return 0; }
	if(items.size() != 18){ return 0; }
	*Current_Timestamp=(uint64_t)strtoull(items.at(1).c_str(),NULL,10);
	*axis1=(int16_t)atoi(items.at(2).c_str());
	*axis2=(int16_t)atoi(items.at(3).c_str());
	*axis3=(int16_t)atoi(items.at(4).c_str());
	*axis4=(int16_t)atoi(items.at(5).c_str());
	*axis5=(int16_t)atoi(items.at(6).c_str());
	*axis6=(int16_t)atoi(items.at(7).c_str());
	*axis7=(int16_t)atoi(items.at(8).c_str());
	*axis8=(int16_t)atoi(items.at(9).c_str());
	*button1=(uint8_t)atoi(items.at(10).c_str());
	*button2=(uint8_t)atoi(items.at(11).c_str());
	*button3=(uint8_t)atoi(items.at(12).c_str());
	*button4=(uint8_t)atoi(items.at(13).c_str());
	*button5=(uint8_t)atoi(items.at(14).c_str());
	*button6=(uint8_t)atoi(items.at(15).c_str());
	*button7=(uint8_t)atoi(items.at(16).c_str());
	*button8=(uint8_t)atoi(items.at(17).c_str());
	return 1;
}
std::string UDPMessageHandler::encode_ResourceUDP(std::string Node_Name,uint16_t RAM_Mb,uint8_t CPU_Used)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_Resource_ID));
	tempstr.append(",");
	tempstr.append(Node_Name);
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)RAM_Mb));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)CPU_Used));
	return tempstr;
}
std::string UDPMessageHandler::encode_DiagnosticUDP(std::string DeviceName,std::string Node_Name,uint8_t System,uint8_t SubSystem,uint8_t Component,uint8_t Diagnostic_Type,uint8_t Level,uint8_t Diagnostic_Message,std::string Description)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_Diagnostic_ID));
	tempstr.append(",");
	tempstr.append(DeviceName);
	tempstr.append(",");
	tempstr.append(Node_Name);
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)System));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)SubSystem));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Component));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Diagnostic_Type));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Level));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Diagnostic_Message));
	tempstr.append(",");
	tempstr.append(Description);
	return tempstr;
}
std::string UDPMessageHandler::encode_DeviceUDP(std::string DeviceParent,std::string DeviceName,std::string DeviceType,std::string Architecture)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_Device_ID));
	tempstr.append(",");
	tempstr.append(DeviceParent);
	tempstr.append(",");
	tempstr.append(DeviceName);
	tempstr.append(",");
	tempstr.append(DeviceType);
	tempstr.append(",");
	tempstr.append(Architecture);
	return tempstr;
}
int UDPMessageHandler::decode_ArmControlUDP(std::vector<std::string> items,uint8_t* device,int* axis1,int* axis2,int* axis3,int* axis4,int* axis5,int* axis6,uint8_t* button1,uint8_t* button2,uint8_t* button3,uint8_t* button4,uint8_t* button5,uint8_t* button6)
{
	char tempstr[8];
	sprintf(tempstr,"0x%s",items.at(0).c_str());
	int id = (int)strtol(tempstr,NULL,0);
	if(id != UDP_ArmControl_ID){ return 0; }
	if(items.size() != 14){ return 0; }
	*device=(uint8_t)atoi(items.at(1).c_str());
	*axis1=(int16_t)atoi(items.at(2).c_str());
	*axis2=(int16_t)atoi(items.at(3).c_str());
	*axis3=(int16_t)atoi(items.at(4).c_str());
	*axis4=(int16_t)atoi(items.at(5).c_str());
	*axis5=(int16_t)atoi(items.at(6).c_str());
	*axis6=(int16_t)atoi(items.at(7).c_str());
	*button1=(uint8_t)atoi(items.at(8).c_str());
	*button2=(uint8_t)atoi(items.at(9).c_str());
	*button3=(uint8_t)atoi(items.at(10).c_str());
	*button4=(uint8_t)atoi(items.at(11).c_str());
	*button5=(uint8_t)atoi(items.at(12).c_str());
	*button6=(uint8_t)atoi(items.at(13).c_str());
	return 1;
}
std::string UDPMessageHandler::encode_Arm_StatusUDP(uint8_t Status)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_Arm_Status_ID));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Status));
	return tempstr;
}
int UDPMessageHandler::decode_HeartbeatUDP(std::vector<std::string> items,std::string* Device,uint64_t* Current_Timestamp,uint64_t* Expected_Timestamp)
{
	char tempstr[8];
	sprintf(tempstr,"0x%s",items.at(0).c_str());
	int id = (int)strtol(tempstr,NULL,0);
	if(id != UDP_Heartbeat_ID){ return 0; }
	if(items.size() != 4){ return 0; }
	*Device=items.at(1);
	*Current_Timestamp=(uint64_t)strtoull(items.at(2).c_str(),NULL,10);
	*Expected_Timestamp=(uint64_t)strtoull(items.at(3).c_str(),NULL,10);
	return 1;
}
int UDPMessageHandler::decode_FindTargetUDP(std::vector<std::string> items,std::string* SearchDevice)
{
	char tempstr[8];
	sprintf(tempstr,"0x%s",items.at(0).c_str());
	int id = (int)strtol(tempstr,NULL,0);
	if(id != UDP_FindTarget_ID){ return 0; }
	if(items.size() != 2){ return 0; }
	*SearchDevice=items.at(1);
	return 1;
}
std::string UDPMessageHandler::encode_PowerUDP(std::string BatteryName,uint8_t PowerLevel,uint8_t PowerState,double Voltage,double Current)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_Power_ID));
	tempstr.append(",");
	tempstr.append(BatteryName);
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)PowerLevel));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)PowerState));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)Voltage));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)Current));
	return tempstr;
}
std::string UDPMessageHandler::encode_EStopUDP(std::string DeviceName,uint8_t State)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_EStop_ID));
	tempstr.append(",");
	tempstr.append(DeviceName);
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)State));
	return tempstr;
}
int UDPMessageHandler::decode_EStopUDP(std::vector<std::string> items,std::string* DeviceName,uint8_t* State)
{
	char tempstr[8];
	sprintf(tempstr,"0x%s",items.at(0).c_str());
	int id = (int)strtol(tempstr,NULL,0);
	if(id != UDP_EStop_ID){ return 0; }
	if(items.size() != 3){ return 0; }
	*DeviceName=items.at(1);
	*State=(uint8_t)atoi(items.at(2).c_str());
	return 1;
}
int UDPMessageHandler::decode_TuneControlGroupUDP(std::vector<std::string> items,std::string* ControlGroupName,double* value1,double* value2,double* value3,int* maxvalue,int* minvalue,int* defaultvalue)
{
	char tempstr[8];
	sprintf(tempstr,"0x%s",items.at(0).c_str());
	int id = (int)strtol(tempstr,NULL,0);
	if(id != UDP_TuneControlGroup_ID){ return 0; }
	if(items.size() != 8){ return 0; }
	*ControlGroupName=items.at(1);
	*value1=(double)atof(items.at(2).c_str());
	*value2=(double)atof(items.at(3).c_str());
	*value3=(double)atof(items.at(4).c_str());
	*maxvalue=(int16_t)atoi(items.at(5).c_str());
	*minvalue=(int16_t)atoi(items.at(6).c_str());
	*defaultvalue=(int16_t)atoi(items.at(7).c_str());
	return 1;
}
std::string UDPMessageHandler::encode_FirmwareUDP(std::string NodeName,std::string Description,uint8_t MajorRelease,uint8_t MinorRelease,uint8_t BuildNumber)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_Firmware_ID));
	tempstr.append(",");
	tempstr.append(NodeName);
	tempstr.append(",");
	tempstr.append(Description);
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)MajorRelease));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)MinorRelease));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)BuildNumber));
	return tempstr;
}
std::string UDPMessageHandler::encode_SubsystemDiagnosticUDP(uint8_t Electrical,uint8_t Software,uint8_t Communications,uint8_t Sensors,uint8_t Actuators,uint8_t Data_Storage,uint8_t Remote_Control,uint8_t Target_Acquisition,uint8_t Pose,uint8_t Timing,uint8_t System_Resource)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_SubsystemDiagnostic_ID));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Electrical));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Software));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Communications));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Sensors));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Actuators));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Data_Storage));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Remote_Control));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Target_Acquisition));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Pose));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Timing));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)System_Resource));
	return tempstr;
}
std::string UDPMessageHandler::encode_SystemSnapshotStateUDP(std::string State,uint8_t PercentComplete,uint16_t SystemSnapshotCount,std::string SourceDevice,std::string SystemSnapshotPath)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_SystemSnapshotState_ID));
	tempstr.append(",");
	tempstr.append(State);
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)PercentComplete));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)SystemSnapshotCount));
	tempstr.append(",");
	tempstr.append(SourceDevice);
	tempstr.append(",");
	tempstr.append(SystemSnapshotPath);
	return tempstr;
}
std::string UDPMessageHandler::encode_ControlGroupValueUDP(double tov,std::string ControlGroupName,double Command,double Sense,double Error,double Error_perc,double Output,double IntegralError,double DerivativeError,double P_Output,double I_Output,double D_Output)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_ControlGroupValue_ID));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)tov));
	tempstr.append(",");
	tempstr.append(ControlGroupName);
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)Command));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)Sense));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)Error));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)Error_perc));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)Output));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)IntegralError));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)DerivativeError));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)P_Output));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)I_Output));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((double)D_Output));
	return tempstr;
}
std::string UDPMessageHandler::encode_SystemStateUDP(uint8_t State,uint8_t Option1,uint8_t Option2,uint8_t Option3,std::string StateText,std::string Description)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_SystemState_ID));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)State));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Option1));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Option2));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Option3));
	tempstr.append(",");
	tempstr.append(StateText);
	tempstr.append(",");
	tempstr.append(Description);
	return tempstr;
}
