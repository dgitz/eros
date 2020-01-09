
//C System Files
#include <fstream>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm> 
//C++ System Files
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
//ROS Base Functionality
//ROS Messages
#include <std_msgs/Bool.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include <eros/diagnostic.h>
#include <eros/heartbeat.h>
#include <eros/command.h>
#include <eros/resource.h>
#include <eros/loadfactor.h>
#include <eros/pose.h>
#include <eros/uptime.h>
#include <eros/battery.h>
#include <eros/systemsnapshot_state.h>
//Project
#include "../../../include/eROS_Definitions.h"
#include "../../../include/PoseHelper.h"
#define COMMTIMEOUT_THRESHOLD 3.0
#define MINWINDOW_WIDTH 140
#define ACTIVESCENARIO_Y 1 //Centered
#define ROSTIME_COORD_X 1 //Left Justified
#define ROSTIME_COORD_Y 1
#define DATETIME_COORD_X 1 //Right Justified
#define DATETIME_COORD_Y 1
#define RUNTIME_COORD_Y 2 //Centered

#define TASKSTART_COORD_X 1
#define TASKSTART_COORD_Y 1
#define TASKPAGE_COUNT 30

#define NO_COLOR 1
#define RED_COLOR 2
#define YELLOW_COLOR 3
#define GREEN_COLOR 4
#define BLUE_COLOR 5

#define KEY_q 113
#define KEY_Q 81
#define KEY_s 83
#define KEY_S 115
#define KEY_c 99
#define KEY_C 67
#define KEY_g 103
#define KEY_G 71
#define KEY_l 108
#define KEY_L 76
#define KEY_d 100
#define KEY_D 68
#define KEY_r 114
#define KEY_R 82
#define KEY_p 112
#define KEY_P 80
#define KEY_m 109
#define KEY_M 77

#define ENTER_KEY 10
class SystemMonitorNodeProcess {
public:
    //Constants
    //Enums;
	enum class TaskType
	{
		UNKNOWN,
		ROS,
		EROS
	};
    //Structs
	struct Task
	{
		bool initialized;
		uint8_t id;
		uint8_t state;
		uint8_t received_state;
		TaskType type;
		int32_t pid;
		std::string host_device;
		std::string base_node_name;
		std::string node_name;
		int16_t cpu_used_perc;
		int32_t mem_used_perc;
		double last_heartbeat;
		double last_heartbeat_delta;
		double last_ping_time;
		uint64_t restart_count;
	};
	struct Module
	{
		bool initialized;
		std::string name;
		double uptime;
		int8_t RAMFree_Perc;
		int8_t CPUFree_Perc;
		int8_t DISKFree_Perc;
		double loadfactor_1min;
		double loadfactor_5min;
		double loadfactor_15min;
		double last_heartbeat;
		double last_heartbeat_delta;
		uint8_t state;
	};
	struct SystemSnap
	{
		eros::systemsnapshot_state state;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	/*! \brief Initializes Process.  Should be called right after instantiating variable. */
	eros::diagnostic initialize(std::string t_base_node_name,std::string t_node_name,std::string t_hostname)
	{
		run_time = 0.0;
		base_node_name = t_base_node_name;
		node_name = t_node_name;
		diagnostic.Node_Name = node_name;
		diagnostic.DeviceName = t_hostname;
		diagnostic.System = GROUND_STATION;
		diagnostic.SubSystem = ROBOT_MONITOR;
		diagnostic.Component = ENTIRE_SUBSYSTEM;
		diagnostic.Diagnostic_Type = REMOTE_CONTROL;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = INITIALIZING;
		diagnostic.Description = "Initializing Process.";
		mainwindow_width = 0;
		mainwindow_height = 0;
		uptime = -1.0;
		snap.state.state = "UNKNOWN";
		truthpose_state = SIGNALSTATE_UNDEFINED;
		truthpose_string = "Truth Pose: Not Received.";
		powerinfo_string = "--.-V --.-A";
		return diagnostic;
	}
	bool read_nodelist(std::string node_list_path,std::vector<std::string> *hosts,std::vector<std::string> *nodes,
		std::vector<std::string> *resource_topics,
		std::vector<std::string> *heartbeat_topics,
		std::vector<std::string> *loadfactor_topics,
		std::vector<std::string> *deviceuptime_topics,
		std::vector<std::string> *resource_available_topics);
	eros::diagnostic init_nodelist(std::vector<std::string> hosts,std::vector<std::string> nodes);
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	ros::Time get_rostime()
	{
		ros::Time t;
		t.sec = (int64_t)ros_time;
		double rem = ros_time - (double)t.sec;
		t.nsec = (int64_t)(rem * 1000000.0);
		return t;
	}
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	void set_uptime(double v) { uptime = v; };
	double get_uptime() { return uptime; }
	bool set_mainwindow(uint16_t t_mainwindow_width,uint16_t t_mainwindow_height)
	{
		mainwindow_width = t_mainwindow_width;
		mainwindow_height = t_mainwindow_height;
		if(mainwindow_width < MINWINDOW_WIDTH)
		{
			return false;
		}
		return true;
	}
	eros::diagnostic get_diagnostic() { return diagnostic; }
	std::vector<Task> get_alltasks() { return tasklist; }
	std::vector<Module> get_allmodules() { return modulelist; }
	std::vector<std::string> get_taskbuffer();
	std::vector<std::string> get_modulebuffer();
	SystemSnap get_systemsnapinfo() { return snap; }
	eros::diagnostic ping_nodes();
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	int push_topiclist(std::string type,std::string name)
	{
		if(type == "eros/resource")
		{
			for(std::size_t i = 0; i < resource_topics.size();i++)
			{
				if(resource_topics.at(i) == name)
				{
					return 0;
				}
			}
			resource_topics.push_back(name);
			return 1;
		}
		else if(type == "eros/heartbeat")
		{
			for(std::size_t i = 0; i < heartbeat_topics.size();i++)
			{
				if(heartbeat_topics.at(i) == name)
				{
					return 0;
				}
			}
			heartbeat_topics.push_back(name);
			return 1;
		}
		else if(type == "eros/loadfactor")
		{
			for(std::size_t i = 0; i < loadfactor_topics.size();i++)
			{
				if(loadfactor_topics.at(i) == name)
				{
					return 0;
				}
			}
			loadfactor_topics.push_back(name);
			return 1;
		}
		else if(type == "eros/uptime")
		{
			for(std::size_t i = 0; i < deviceuptime_topics.size();i++)
			{
				if(deviceuptime_topics.at(i) == name)
				{
					return 0;
				}
			}
			deviceuptime_topics.push_back(name);
			return 1;
		}
		return -1;
	}
	eros::diagnostic new_truthpose(const eros::pose::ConstPtr& t_ptr);
	eros::diagnostic new_resourcemessage(const eros::resource::ConstPtr& t_ptr);
	eros::diagnostic new_resourceavailablemessage(const eros::resource::ConstPtr& t_ptr);
	eros::diagnostic new_loadfactormessage(const eros::loadfactor::ConstPtr& t_ptr);
	eros::diagnostic new_deviceuptime(const eros::uptime::ConstPtr& t_ptr);
	eros::diagnostic new_heartbeatmessage(const eros::heartbeat::ConstPtr& t_ptr);
	eros::diagnostic new_systemsnapshotstatemessage(const eros::systemsnapshot_state::ConstPtr& t_ptr);
	eros::diagnostic new_batterymessage(const eros::battery::ConstPtr& t_ptr);
	//Support Functions
	std::string get_powerinfo_string() { return powerinfo_string; }
	std::string get_truthposestring() { return truthpose_string; }
	uint8_t get_truthposestate() { return truthpose_state; }
	std::string get_taskheader();
	std::vector<std::string> get_modulelistheader();
	eros::heartbeat convert_fromptr(const eros::heartbeat::ConstPtr& t_ptr);
	eros::resource convert_fromptr(const eros::resource::ConstPtr& t_ptr);
	std::string map_taskstate_tostring(uint8_t state);
    //Printing Functions
protected:
private:
	std::string exec(const char* cmd,bool wait_for_result);
	std::string fixed_width(std::string item,uint16_t width);
	Task create_task(uint16_t id,std::string host,std::string name);
	std::string convert_uptime(double t);
	uint16_t mainwindow_width,mainwindow_height;
	eros::diagnostic diagnostic;
	std::string base_node_name,node_name;
	double run_time,ros_time;
	std::vector<Task> tasklist;
	std::vector<Module> modulelist;
	std::vector<std::string> resource_topics;
	std::vector<std::string> heartbeat_topics;
	std::vector<std::string> loadfactor_topics;
	std::vector<std::string> deviceuptime_topics;
	double uptime;
	PoseHelper pose_helper;
	SystemSnap snap;
	std::string truthpose_string;
	uint8_t truthpose_state;
	std::string powerinfo_string;

};
