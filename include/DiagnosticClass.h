#ifndef DIAGNOSTICCLASS_H
#define DIAGNOSTICCLASS_H
#include <map>
#include "eROS_Definitions.h"
class DiagnosticClass
{
public:
	DiagnosticClass()
{
		DiagSystemMap[ROVER] = "ROVER";
		DiagSystemMap[GROUND_STATION] = "GROUND STATION";
		DiagSystemMap[REMOTE_CONTROL] = "REMOTE CONTROL";
		DiagSystemMap[SYSTEM_UNKNOWN] = "SYSTEM UNKNOWN";

		DiagSubSystemMap[ENTIRE_SYSTEM] = "ENTIRE SYSTEM";
		DiagSubSystemMap[ROBOT_CONTROLLER] = "ROBOT CONTROLLER";
		DiagSubSystemMap[SUBSYSTEM_UNKNOWN] = "SUBSYSTEM UNKNOWN";

		DiagComponentMap[ENTIRE_SUBSYSTEM] = "ENTIRE SUBSYSTEM";
		DiagComponentMap[CONTROLLER_NODE] = "CONTROLLER NODE";
		DiagComponentMap[DIAGNOSTIC_NODE] = "DIAGNOSTIC NODE";
		DiagComponentMap[NAVIGATION_NODE] = "NAVIGATION NODE";
		DiagComponentMap[MAPPING_NODE] = "MAPPING NODE";
		DiagComponentMap[EVOLUTION_NODE] = "EVOLUTION NODE";
		DiagComponentMap[TARGETING_NODE] = "TARGETING NODE";
		DiagComponentMap[TIMING_NODE] = "TIMING NODE";
		DiagComponentMap[VISION_NODE] = "VISION NODE";
		DiagComponentMap[GPIO_NODE] = "GPIO NODE";
		DiagComponentMap[COMMUNICATION_NODE] = "COMMUNICATION NODE";
		DiagComponentMap[DYNAMICS_NODE] = "DYNAMICS NODE";
		DiagComponentMap[POWER_NODE] = "POWER NODE";
		DiagComponentMap[POSE_NODE] = "POSE NODE";
		DiagComponentMap[COMPONENT_UNKNOWN] = "COMPONENT UNKNOWN";

		DiagTypeMap[NOERROR] = "NO ERROR";
		DiagTypeMap[ELECTRICAL] = "ELECTRICAL";
		DiagTypeMap[SOFTWARE] = "SOFTWARE";
		DiagTypeMap[COMMUNICATIONS] = "COMMUNICATIONS";
		DiagTypeMap[SENSORS] = "SENSORS";
		DiagTypeMap[ACTUATORS] = "ACTUATORS";
		DiagTypeMap[DATA_STORAGE] = "DATA_STORAGE";
		DiagTypeMap[REMOTE_CONTROL] = "REMOTE_CONTROL";
		DiagTypeMap[TARGET_ACQUISITION] = "TARGET ACQUISITION";
		DiagTypeMap[POWER] = "POWER";
		DiagTypeMap[POSE] = "POSE";
		DiagTypeMap[TIMING] = "TIMING";
		DiagTypeMap[GENERAL_ERROR] = "GENERAL ERROR";

		DiagLevelMap[DEBUG] = "DEBUG";
		DiagLevelMap[INFO] = "INFO";
		DiagLevelMap[NOTICE] = "NOTICE";
		DiagLevelMap[WARN] = "WARN";
		DiagLevelMap[ERROR] = "ERROR";
		DiagLevelMap[FATAL] = "FATAL";
		DiagLevelMap[LEVEL_UNKNOWN] = "LEVEL UNKNOWN";

		DiagMessageMap[NOERROR] = "NO ERROR";
		DiagMessageMap[INITIALIZING] = "INITIALIZING";
		DiagMessageMap[INITIALIZING_ERROR] = "INITIALIZING ERROR";
		DiagMessageMap[DROPPING_PACKETS] = "DROPPING PACKETS";
		DiagMessageMap[MISSING_HEARTBEATS] = "MISSING HEARTBEATS";
		DiagMessageMap[DEVICE_NOT_AVAILABLE] = "DEVICE NOT AVAILABLE";
		DiagMessageMap[ROVER_ARMED] = "ROVER ARMED";
		DiagMessageMap[ROVER_DISARMED] = "ROVER DISARMED";
		DiagMessageMap[TEMPERATURE_HIGH] = "TEMPERATURE HIGH";
		DiagMessageMap[TEMPERATURE_LOW] = "TEMPERATURE LOW";
		DiagMessageMap[DIAGNOSTIC_PASSED] = "DIAGNOSTIC PASSED";
		DiagMessageMap[DIAGNOSTIC_FAILED] = "DIAGNOSTIC FAILED";
		DiagMessageMap[RESOURCE_LEAK] = "RESOURCE LEAK";
		DiagMessageMap[HIGH_RESOURCE_USAGE] = "HIGH RESOURCE USAGE";
		DiagMessageMap[UNKNOWN_STATE] = "UNKNOWN STATE";
		DiagMessageMap[UNKNOWN_MESSAGE] = "UNKNOWN MESSAGE";
}
	~DiagnosticClass()
	{

	}
	std::string get_DiagSystemString(uint8_t v)
	{
		return lookup_key(DiagSystemMap,v,SYSTEM_UNKNOWN);
	}
	std::string get_DiagSubSystemString(uint8_t v)
	{
		return lookup_key(DiagSubSystemMap,v,SUBSYSTEM_UNKNOWN);
	}
	std::string get_DiagComponentString(uint8_t v)
	{
		return lookup_key(DiagComponentMap,v,COMPONENT_UNKNOWN);
	}
	std::string get_DiagTypeString(uint8_t v)
	{
		return lookup_key(DiagTypeMap,v,GENERAL_ERROR);
	}
	std::string get_DiagLevelString(uint8_t v)
	{
		return lookup_key(DiagLevelMap,v,LEVEL_UNKNOWN);
	}
	std::string get_DiagMessageString(uint8_t v)
		{
			return lookup_key(DiagMessageMap,v,UNKNOWN_MESSAGE);
		}

private:
	std::string lookup_key(std::map<uint8_t,std::string> keymap,uint8_t v,uint8_t v_default)
	{
		if(keymap.find(v) != keymap.end())
		{
			return keymap[v];
		}
		else
		{
			return keymap[v_default];
		}
	}
	std::map<uint8_t,std::string> DiagSystemMap;
	std::map<uint8_t,std::string> DiagSubSystemMap;
	std::map<uint8_t,std::string> DiagComponentMap;
	std::map<uint8_t,std::string> DiagTypeMap;
	std::map<uint8_t,std::string> DiagLevelMap;
	std::map<uint8_t,std::string> DiagMessageMap;
};

#endif
