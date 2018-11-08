/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-11-08 06:41:05.829892***/
#include "../include/jsonmessage.h"
JSONMessageHandler::JSONMessageHandler(){}
JSONMessageHandler::~JSONMessageHandler(){}
std::string JSONMessageHandler::encode_DiagnosticJSON(icarus_rover_v2::diagnostic diagnostic)
{
	std::string tempstr = "{\"ID\":43794,\"data\":{";
	tempstr+="\"diagnostic\":{";
	tempstr+="\"DeviceName\":\""+diagnostic.DeviceName+"\",";
	tempstr+="\"Node_Name\":\""+diagnostic.Node_Name+"\",";
	tempstr+="\"System\":"+boost::lexical_cast<std::string>((int)diagnostic.System)+",";
	tempstr+="\"SubSystem\":"+boost::lexical_cast<std::string>((int)+diagnostic.SubSystem)+",";
	tempstr+="\"Component\":"+boost::lexical_cast<std::string>((int)+diagnostic.Component)+",";
	tempstr+="\"Diagnostic_Type\":"+boost::lexical_cast<std::string>((int)diagnostic.Diagnostic_Type)+",";
	tempstr+="\"Level\":"+boost::lexical_cast<std::string>((int)diagnostic.Level)+",";
	tempstr+="\"Diagnostic_Message\":"+boost::lexical_cast<std::string>((int)diagnostic.Diagnostic_Message)+",";
	tempstr+="\"Description\":\""+diagnostic.Description+"\"}";
	tempstr+="}}";
	return tempstr;
}
std::string JSONMessageHandler::encode_DeviceJSON(std::vector<icarus_rover_v2::device> devicelist)
{
	std::string tempstr = "{\"ID\":43795,\"data\":{";
	tempstr+="\"devicelist\":[";
	for(std::size_t i = 0; i < devicelist.size(); i++)
	{
		tempstr+="{";
		tempstr+="\"DeviceParent\":\""+devicelist.at(i).DeviceParent+"\",";
		tempstr+="\"PartNumber\":\""+devicelist.at(i).PartNumber+"\",";
		tempstr+="\"DeviceName\":\""+devicelist.at(i).DeviceName+"\",";
		tempstr+="\"DeviceType\":\""+devicelist.at(i).DeviceType+"\",";
		tempstr+="\"PrimaryIP\":\""+devicelist.at(i).PrimaryIP+"\",";
		tempstr+="\"Architecture\":\""+devicelist.at(i).Architecture+"\",";
		tempstr+="\"Capabilities\":[";
		for(std::size_t j = 0; j < devicelist.at(i).Capabilities.size(); j++)
		{
			tempstr+="\""+devicelist.at(i).Capabilities.at(j)+"\"";
			if(j < (devicelist.at(i).Capabilities.size()-1))
			{
				tempstr+=",";
			}
		}
		tempstr+="],";
		tempstr+="\"BoardCount\":"+boost::lexical_cast<std::string>((int)devicelist.at(i).BoardCount)+",";
		tempstr+="\"HatCount\":"+boost::lexical_cast<std::string>((int)devicelist.at(i).HatCount)+",";
		tempstr+="\"ShieldCount\":"+boost::lexical_cast<std::string>((int)devicelist.at(i).ShieldCount)+",";
		tempstr+="\"SensorCount\":"+boost::lexical_cast<std::string>((int)devicelist.at(i).SensorCount);
		tempstr+="}";
		if(i < (devicelist.size()-1))
		{
			tempstr+=",";
		}
	}
	tempstr+="]";
	tempstr+="}}";
	return tempstr;
}
