/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2017-05-05 07:38:59.368056***/
#ifndef UDPMESSAGE_H
#define UDPMESSAGE_H
#include <QString>
#include <QList>
#define UDP_Command_ID "AB02"
#define UDP_RemoteControl_ID "AB10"
#define UDP_Resource_ID "AB11"
#define UDP_Diagnostic_ID "AB12"
#define UDP_Device_ID "AB13"
#define UDP_ArmControl_ID "AB26"
#define UDP_Arm_Status_ID "AB30"
#define UDP_Heartbeat_ID "AB31"
#define UDP_FindTarget_ID "AB34"

class UDPMessageHandler
{
public:
	UDPMessageHandler();
	~UDPMessageHandler();
	QString encode_CommandUDP(int Command,int Option1,int Option2,int Option3,std::string CommandText,std::string Description);
	QString encode_RemoteControlUDP(int axis1,int axis2,int axis3,int axis4,int axis5,int axis6,int axis7,int axis8,int button1,int button2,int button3,int button4,int button5,int button6,int button7,int button8);
	int decode_ResourceUDP(QList<QByteArray> items,std::string* Node_Name,int* RAM_Mb,int* CPU_Used);
	int decode_DiagnosticUDP(QList<QByteArray> items,std::string* DeviceName,std::string* Node_Name,int* System,int* SubSystem,int* Component,int* Diagnostic_Type,int* Level,int* Diagnostic_Message,std::string* Description);
	int decode_DeviceUDP(QList<QByteArray> items,std::string* DeviceParent,std::string* DeviceName,std::string* DeviceType,std::string* Architecture);
	QString encode_ArmControlUDP(int device,int axis1,int axis2,int axis3,int axis4,int axis5,int axis6,int button1,int button2,int button3,int button4,int button5,int button6);
	int decode_Arm_StatusUDP(QList<QByteArray> items,int* Status);
	QString encode_HeartbeatUDP(std::string Device,uint64_t Current_Timestamp,uint64_t Expected_Timestamp);
	QString encode_FindTargetUDP(std::string SearchDevice);
private:
};
#endif