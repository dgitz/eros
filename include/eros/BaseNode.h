// Base class
//C System Files
#include <unistd.h>
#include <csignal>


//C++ System Files
#include <thread>
#include <iostream>

//ROS Base Functionality
#include "ros/ros.h"
#include "ros/time.h"

//ROS Messages


//Project
#include "../../../eROS/include/logger.h"
#include "../resourcemonitor.h"

/*! \class BaseNode BaseNode.h "BaseNode.h"
 *  \brief This is a BaseNode class.  All Nodes should be a derived class from this Base Class.*/
class BaseNode {
public:
	virtual ~BaseNode()
	{
	}
	//Constants
	const uint8_t MAX_SERIALPACKET_SIZE = 64;
	const uint16_t MAX_UDP_RX_BUFFER_SIZE = 2048;
	//Structs
	//Initialization Functions
	/*! \brief Set Node Base Name.  This will be the same for every instance of the node, and is independent on where the node is run.
	 * This value is equivelant to the "type" field in the launch file.
	 */
	void set_basenodename(std::string t_base_node_name);
	/*! \brief Initializes firmware based on Major, Minor, Build Number and Description. */
	void initialize_firmware(uint8_t t_major_version,uint8_t t_minor_version,uint8_t t_build_number,std::string t_description);
	/*! \brief Sets Node Diagnostic */
	void initialize_diagnostic(uint8_t t_system,uint8_t t_subsystem,uint8_t t_component);
	void set_nodename(std::string t_node_name)
	{
		node_name = t_node_name;
	}
	virtual bool start(int argc,char **argv) = 0;
	/*! \brief Pre-initialization of node.  This section will create the default pub/subs for the node, along with the logger. */
	eros::diagnostic preinitialize_basenode(int argc,char **argv);
	void set_loop1_rate(double t_rate) { loop1_rate = t_rate; loop1_enabled = true; }
	void set_loop2_rate(double t_rate) { loop2_rate = t_rate; loop2_enabled = true; }
	void set_loop3_rate(double t_rate) { loop3_rate = t_rate; loop3_enabled = true; }

	//Update Functions
	/*! \brief Main Node update section, will call all derived Node Loop Functions, and publish base pubs such as firmware, heartbeat, etc. */
	bool update(uint8_t task_state);
	/*! \brief Loop1 Must be Implemented in Derived Node.*/
	virtual bool run_loop1() = 0;
	/*! \brief Loop2 Must be Implemented in Derived Node.*/
	virtual bool run_loop2() = 0;
	/*! \brief Loop3 Must be Implemented in Derived Node.*/
	virtual bool run_loop3() = 0;
	/*! \brief Loop .01Hz Must be Implemented in Derived Node.*/
	virtual bool run_001hz() = 0;
		/*! \brief Loop .1Hz Must be Implemented in Derived Node.*/
	virtual bool run_01hz() = 0;
	/*! \brief Loop .1Hz Noisy Must be Implemented in Derived Node.*/
	virtual bool run_01hz_noisy() = 0;
	/*! \brief Loop 1Hz Must be Implemented in Derived Node.*/
	virtual bool run_1hz() = 0;
	/*! \brief Loop 10Hz Must be Implemented in Derived Node.*/
	virtual bool run_10hz() = 0;
	/*! \brief Thread Loop Must be Implemented in Derived Node.*/
	virtual void thread_loop() = 0;
	//Attribute Functions
	std::string get_basenodename() { return base_node_name; }
	std::string get_nodename() { return node_name; }
	std::string get_verbositylevel() { return verbosity_level; }
	eros::diagnostic get_diagnostic() { return diagnostic; }
	eros::diagnostic get_resource_diagnostic() { return resource_diagnostic; }
	std::string get_hostname() { return std::string(host_name); }

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
	Logger* get_logger() { return logger; }

	//Message Functions
	/*! \brief New Device Message Must be Implemented in Derived Node.*/
	virtual bool new_devicemsg(std::string t_query,eros::device t_device) = 0;
	/*! \brief Handles receiving the 1 PPS Msg. */
	void new_ppsmsg(const std_msgs::Bool::ConstPtr& t_msg);

	//Destructors
	virtual void cleanup() = 0;
	void base_cleanup();
	/*! \brief Publishes ReadyToArm By Default.  Call this to Disable the Publisher. */
	void disable_readytoarm_publisher()
	{
		publish_readytoarm = false;
	}

protected:
	/*! \brief Handles a new Command Message by the Node.
	 *  Typically all that is done here is to tell the Node
	 * what diag data to publish after the Base Node Process has already processed the command.
	 */
	void new_commandmsg_result(const eros::command::ConstPtr& t_msg,std::vector<eros::diagnostic> t_diaglist);
	/*! \brief Set my device info.
	 *  This will also initialize the resource monitor.
	 */
	eros::diagnostic set_mydevice(eros::device t_device);
	/*! \brief Get Base Launch parameters, which includes loop rates, verbosity, etc. */
	eros::diagnostic read_baselaunchparameters();

	boost::shared_ptr<ros::NodeHandle> n;
	eros::diagnostic diagnostic;
	eros::diagnostic resource_diagnostic;
	ros::Publisher diagnostic_pub;
	char host_name[1024];


	eros::heartbeat heartbeat;

	eros::firmware firmware;
	std::string base_node_name;
	std::string node_name;
	ros::Publisher state_pub;
	ros::Publisher firmware_pub;
	ros::Publisher resource_pub;
	ros::Publisher heartbeat_pub;
	ros::Publisher readytoarm_pub;
	Logger *logger;
	bool logger_initialized;
	ResourceMonitor *resourcemonitor;
	bool resourcemonitor_initialized;
	double ros_rate;

	ros::Time boot_time;
	ros::Time last_001hz_timer;
	ros::Time last_01hz_timer;
	ros::Time last_01hz_noisy_timer;
	ros::Time last_1hz_timer;
	ros::Time last_10hz_timer;
	bool loop1_enabled;
	ros::Time last_loop1_timer;
	double loop1_rate;

	bool loop2_enabled;
	ros::Time last_loop2_timer;
	double loop2_rate;

	bool loop3_enabled;
	double loop3_rate;
	ros::Time last_loop3_timer;

	std::string verbosity_level;
	eros::resource resources_used;
	bool require_pps_to_start,pps_received;
	bool ready_to_arm;
	bool publish_readytoarm;
	double rand_delay_sec; 

private:



};

