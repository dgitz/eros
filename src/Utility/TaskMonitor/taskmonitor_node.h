#include "TaskMonitorNodeProcess.cpp"
//C System Files
#include <unistd.h>
#include <csignal>
#include <curses.h>
#include <iostream>
#include <ctime>
#include<iostream>
#include<fstream>

//C++ System Files
#include <thread>
#include <iostream>

//ROS Base Functionality
#include "ros/ros.h"
#include "ros/time.h"

//ROS Messages


//Project
#include "../../../include/logger.h"
WINDOW *create_newwin(int height, int width, int starty, int startx);
class TaskMonitorNode {
public:

	~TaskMonitorNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	TaskMonitorNodeProcess* get_process() { return process; }
	bool update();
	void thread_loop();

	//Cleanup
	void cleanup();
private:
	//Initialization Functions
	bool init_screen();
	//Update Functions
	bool run_001hz();
	bool run_01hz();
	bool run_01hz_noisy();
	bool run_1hz();
	bool run_10hz();
	bool update_windowheader();
	bool update_windowtasklist();
	bool update_windowfooter();
	//Attribute Functions
	//Message Functions
	eros::diagnostic rescan_topics();
	void heartbeat_Callback(const eros::heartbeat::ConstPtr& msg);
	void resource_Callback(const eros::resource::ConstPtr& msg);
	void uptime_Callback(const std_msgs::Float32::ConstPtr& msg);
	//Utility Functions
	/*! \brief Measures time delay between 2 ros::Time timestamps.
	 *  Generally, if wanting to measure the time from now to a previous mark,
	 * the current timestamp should be the first parameter and the previous mark should be the 2nd parameter.
	 */
	double measure_time_diff(ros::Time t_timer_a, ros::Time t_timer_b)
	{
		double etime = t_timer_a.toSec() - t_timer_b.toSec();
		return etime;
	}
	boost::shared_ptr<ros::NodeHandle> get_nodehandle() { return n; }
	//Support Functions
	double ros_rate;
	boost::shared_ptr<ros::NodeHandle> n;

	TaskMonitorNodeProcess *process;
	Logger *logger;
	ros::Time boot_time;
	ros::Time current_time;
	ros::Time last_001hz_timer;
	ros::Time last_01hz_timer;
	ros::Time last_01hz_noisy_timer;
	ros::Time last_1hz_timer;
	ros::Time last_10hz_timer;
	time_t rawtime;
  	struct tm * timeinfo;
	double rand_delay_sec; 

	WINDOW *window_header;
	WINDOW *window_tasklist;
	WINDOW *window_footer;
	uint16_t mainwindow_width,mainwindow_height;
	std::string active_scenario;
	std::string base_node_name;
	std::string node_name;
	char host_name[1024];

	std::vector<ros::Subscriber> resource_subs;
	std::vector<ros::Subscriber> heartbeat_subs;
	ros::Subscriber uptime_sub;
	uint16_t start_node_index;

};