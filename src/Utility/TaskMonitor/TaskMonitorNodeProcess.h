
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
//Project
#include "../../../include/eROS_Definitions.h"
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

#define KEY_Q 113
class TaskMonitorNodeProcess {
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
		return diagnostic;
	}
	bool read_nodelist(std::string node_list_path,std::vector<std::string> *hosts,std::vector<std::string> *nodes);
	eros::diagnostic init_nodelist(std::vector<std::string> hosts,std::vector<std::string> nodes);
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
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
	std::vector<std::string> get_taskbuffer();
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
		return -1;
	}
	eros::diagnostic new_resourcemessage(const eros::resource::ConstPtr& t_ptr);
	eros::diagnostic new_heartbeatmessage(const eros::heartbeat::ConstPtr& t_ptr);
	//Support Functions
	std::string get_taskheader();
	eros::heartbeat convert_fromptr(const eros::heartbeat::ConstPtr& t_ptr);
	eros::resource convert_fromptr(const eros::resource::ConstPtr& t_ptr);
	std::string map_taskstate_tostring(uint8_t state);
    //Printing Functions
protected:
private:
	std::string exec(const char* cmd,bool wait_for_result);
	std::string fixed_width(std::string item,uint16_t width);
	Task create_task(uint16_t id,std::string host,std::string name);
	uint16_t mainwindow_width,mainwindow_height;
	eros::diagnostic diagnostic;
	std::string base_node_name,node_name;
	double run_time,ros_time;
	std::vector<Task> tasklist;
	std::vector<std::string> resource_topics;
	std::vector<std::string> heartbeat_topics;
	double uptime;

};
