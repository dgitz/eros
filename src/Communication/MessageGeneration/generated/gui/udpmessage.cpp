/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2017-05-05 07:38:59.368089***/
#include "udpmessage.h"
UDPMessageHandler::UDPMessageHandler(){}
UDPMessageHandler::~UDPMessageHandler(){}
QString UDPMessageHandler::encode_CommandUDP(int Command,int Option1,int Option2,int Option3,std::string CommandText,std::string Description)
{
	QString tempstr = "";
	tempstr.append(UDP_Command_ID);
	tempstr.append(",");
	tempstr.append(QString::number(Command));
	tempstr.append(",");
	tempstr.append(QString::number(Option1));
	tempstr.append(",");
	tempstr.append(QString::number(Option2));
	tempstr.append(",");
	tempstr.append(QString::number(Option3));
	tempstr.append(",");
	tempstr.append(QString::fromStdString(CommandText));
	tempstr.append(",");
	tempstr.append(QString::fromStdString(Description));
	return tempstr;
}
QString UDPMessageHandler::encode_RemoteControlUDP(int axis1,int axis2,int axis3,int axis4,int axis5,int axis6,int axis7,int axis8,int button1,int button2,int button3,int button4,int button5,int button6,int button7,int button8)
{
	QString tempstr = "";
	tempstr.append(UDP_RemoteControl_ID);
	tempstr.append(",");
	tempstr.append(QString::number(axis1));
	tempstr.append(",");
	tempstr.append(QString::number(axis2));
	tempstr.append(",");
	tempstr.append(QString::number(axis3));
	tempstr.append(",");
	tempstr.append(QString::number(axis4));
	tempstr.append(",");
	tempstr.append(QString::number(axis5));
	tempstr.append(",");
	tempstr.append(QString::number(axis6));
	tempstr.append(",");
	tempstr.append(QString::number(axis7));
	tempstr.append(",");
	tempstr.append(QString::number(axis8));
	tempstr.append(",");
	tempstr.append(QString::number(button1));
	tempstr.append(",");
	tempstr.append(QString::number(button2));
	tempstr.append(",");
	tempstr.append(QString::number(button3));
	tempstr.append(",");
	tempstr.append(QString::number(button4));
	tempstr.append(",");
	tempstr.append(QString::number(button5));
	tempstr.append(",");
	tempstr.append(QString::number(button6));
	tempstr.append(",");
	tempstr.append(QString::number(button7));
	tempstr.append(",");
	tempstr.append(QString::number(button8));
	return tempstr;
}
int UDPMessageHandler::decode_ResourceUDP(QList<QByteArray> items,std::string* Node_Name,int* RAM_Mb,int* CPU_Used)
{
	if(items.size() != 4){ return 0; }
	*Node_Name=items.at(1).toStdString();
	*RAM_Mb=(int)items.at(2).toInt();
	*CPU_Used=(int)items.at(3).toInt();
	return 1;
}
int UDPMessageHandler::decode_DiagnosticUDP(QList<QByteArray> items,std::string* DeviceName,std::string* Node_Name,int* System,int* SubSystem,int* Component,int* Diagnostic_Type,int* Level,int* Diagnostic_Message,std::string* Description)
{
	if(items.size() != 10){ return 0; }
	*DeviceName=items.at(1).toStdString();
	*Node_Name=items.at(2).toStdString();
	*System=(int)items.at(3).toInt();
	*SubSystem=(int)items.at(4).toInt();
	*Component=(int)items.at(5).toInt();
	*Diagnostic_Type=(int)items.at(6).toInt();
	*Level=(int)items.at(7).toInt();
	*Diagnostic_Message=(int)items.at(8).toInt();
	*Description=items.at(9).toStdString();
	return 1;
}
int UDPMessageHandler::decode_DeviceUDP(QList<QByteArray> items,std::string* DeviceParent,std::string* DeviceName,std::string* DeviceType,std::string* Architecture)
{
	if(items.size() != 5){ return 0; }
	*DeviceParent=items.at(1).toStdString();
	*DeviceName=items.at(2).toStdString();
	*DeviceType=items.at(3).toStdString();
	*Architecture=items.at(4).toStdString();
	return 1;
}
QString UDPMessageHandler::encode_ArmControlUDP(int device,int axis1,int axis2,int axis3,int axis4,int axis5,int axis6,int button1,int button2,int button3,int button4,int button5,int button6)
{
	QString tempstr = "";
	tempstr.append(UDP_ArmControl_ID);
	tempstr.append(",");
	tempstr.append(QString::number(device));
	tempstr.append(",");
	tempstr.append(QString::number(axis1));
	tempstr.append(",");
	tempstr.append(QString::number(axis2));
	tempstr.append(",");
	tempstr.append(QString::number(axis3));
	tempstr.append(",");
	tempstr.append(QString::number(axis4));
	tempstr.append(",");
	tempstr.append(QString::number(axis5));
	tempstr.append(",");
	tempstr.append(QString::number(axis6));
	tempstr.append(",");
	tempstr.append(QString::number(button1));
	tempstr.append(",");
	tempstr.append(QString::number(button2));
	tempstr.append(",");
	tempstr.append(QString::number(button3));
	tempstr.append(",");
	tempstr.append(QString::number(button4));
	tempstr.append(",");
	tempstr.append(QString::number(button5));
	tempstr.append(",");
	tempstr.append(QString::number(button6));
	return tempstr;
}
int UDPMessageHandler::decode_Arm_StatusUDP(QList<QByteArray> items,int* Status)
{
	if(items.size() != 2){ return 0; }
	*Status=(int)items.at(1).toInt();
	return 1;
}
QString UDPMessageHandler::encode_HeartbeatUDP(std::string Device,uint64_t Current_Timestamp,uint64_t Expected_Timestamp)
{
	QString tempstr = "";
	tempstr.append(UDP_Heartbeat_ID);
	tempstr.append(",");
	tempstr.append(QString::fromStdString(Device));
	tempstr.append(",");
	tempstr.append(QString::number(Current_Timestamp));
	tempstr.append(",");
	tempstr.append(QString::number(Expected_Timestamp));
	return tempstr;
}
QString UDPMessageHandler::encode_FindTargetUDP(std::string SearchDevice)
{
	QString tempstr = "";
	tempstr.append(UDP_FindTarget_ID);
	tempstr.append(",");
	tempstr.append(QString::fromStdString(SearchDevice));
	return tempstr;
}
