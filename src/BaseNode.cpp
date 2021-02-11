#include "BaseNode.h"

void BaseNode::set_basenodename(std::string t_base_node_name)
{
	base_node_name = t_base_node_name;
}
void BaseNode::initialize_firmware(uint8_t t_major_version,uint8_t t_minor_version,uint8_t t_build_number,std::string t_description)
{
	firmware.Major_Release = t_major_version;
	firmware.Minor_Release = t_minor_version;
	firmware.Build_Number = t_build_number;
	firmware.Description = t_description;
}

void BaseNode::initialize_diagnostic(uint8_t t_system,uint8_t t_subsystem,uint8_t t_component)
{
	diagnostic.System = t_system;
	diagnostic.SubSystem = t_subsystem;
	diagnostic.Component = t_component;

}
eros::diagnostic BaseNode::preinitialize_basenode(int argc,char **argv)
{
	resourcemonitor_initialized = false;
	logger_initialized = false;
	require_pps_to_start = false;
	pps_received = false;
	ready_to_arm = false;
	publish_readytoarm = true;
	ros::init(argc,argv,base_node_name);
	n.reset(new ros::NodeHandle);
	node_name = ros::this_node::getName();
	boot_time = ros::Time::now();
	firmware.Generic_Node_Name = base_node_name;
	firmware.Node_Name = node_name;
	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing.";
	heartbeat.BaseNode_Name = base_node_name;
	heartbeat.Node_Name = node_name;
	host_name[1023] = '\0';
	gethostname(host_name,1023);
	rand_delay_sec = (double)(rand() % 2000 - 1000)/1000.0;  

	diagnostic = read_baselaunchparameters();
	std::string firmware_topic = "/" + node_name + "/firmware";
	firmware_pub =  n->advertise<eros::firmware>(firmware_topic,1);
	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<eros::resource>(resource_topic,1);
	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = n->advertise<eros::heartbeat>(heartbeat_topic,1);
	heartbeat.stamp = ros::Time::now();
	heartbeat.TaskState = TASKSTATE_INITIALIZING;
	heartbeat_pub.publish(heartbeat);
	std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub = n->advertise<eros::diagnostic>(diagnostic_topic,30);
	std::string readytoarm_topic = "/" + node_name + "/readytoarm";
	readytoarm_pub = n->advertise<std_msgs::Bool>(readytoarm_topic,1);
	if(diagnostic.Level > WARN)
	{
		if(logger_initialized == true)
		{
			logger->log_diagnostic(diagnostic);
		}
		else
		{
			printf("[%s] Could not complete pre-initialization. Exiting.\n",node_name.c_str());
		}
	}
	return diagnostic;
}
eros::diagnostic BaseNode::set_mydevice(eros::device t_device)
{
	eros::diagnostic diag=diagnostic;
	diag.Diagnostic_Type = SYSTEM_RESOURCE;
	diag.Node_Name = node_name;
	diag.DeviceName = t_device.DeviceName;
	resourcemonitor = new ResourceMonitor(diag,t_device.Architecture,std::string(host_name),node_name);
	resourcemonitor_initialized = true;
	return diag;
}

eros::diagnostic BaseNode::read_baselaunchparameters()
{
	eros::diagnostic diag=diagnostic;
	loop1_enabled = false;
	loop2_enabled = false;
	loop3_enabled = false;
	std::string param_verbosity_level = node_name +"/verbosity_level";
	if(n->getParam(param_verbosity_level,verbosity_level) == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Missing Parameter: verbosity_level. Exiting.";
		diagnostic = diag;
		return diag;
	}
	else
	{
		logger = new Logger(verbosity_level,node_name);
		logger_initialized = true;
	}
	std::string param_startup_delay = node_name + "/startup_delay";
	double startup_delay = 0.0;
	if(n->getParam(param_startup_delay,startup_delay) == false)
	{
		logger->log_notice(__FILE__,__LINE__,"Missing Parameter: startup_delay.  Using Default: 0.0 sec.");
	}
	else
	{
		char tempstr[128];
		sprintf(tempstr,"Using Parameter: startup_delay = %4.2f sec.",startup_delay);
		logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
	}
	ros::Duration(startup_delay).sleep();
	std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
	if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Missing Parameter: require_pps_to_star. Exiting.";
		diagnostic = diag;
		logger->log_diagnostic(diag);
	}
	double max_rate = 0.0;
	last_01hz_timer = ros::Time::now();
	last_01hz_noisy_timer = ros::Time::now();
	last_1hz_timer = ros::Time::now();
	last_10hz_timer = ros::Time::now();
	std::string param_loop1_rate = node_name + "/loop1_rate";
	if(n->getParam(param_loop1_rate,loop1_rate) == false)
	{
		logger->log_warn(__FILE__,__LINE__,"Missing parameter: loop1_rate.  Not running loop1 code.");
		loop1_enabled = false;
	}
	else
	{
		last_loop1_timer = ros::Time::now();
		loop1_enabled = true;
		if(loop1_rate > max_rate) { max_rate = loop1_rate; }
	}

	std::string param_loop2_rate = node_name + "/loop2_rate";
	if(n->getParam(param_loop2_rate,loop2_rate) == false)
	{
		logger->log_warn(__FILE__,__LINE__,"Missing parameter: loop2_rate.  Not running loop2 code.");
		loop2_enabled = false;
	}
	else
	{
		last_loop2_timer = ros::Time::now();
		loop2_enabled = true;
		if(loop2_rate > max_rate) { max_rate = loop2_rate; }
	}

	std::string param_loop3_rate = node_name + "/loop3_rate";
	if(n->getParam(param_loop3_rate,loop3_rate) == false)
	{
		logger->log_warn(__FILE__,__LINE__,"Missing parameter: loop3_rate.  Not running loop3 code.");
		loop3_enabled = false;
	}
	else
	{
		last_loop3_timer = ros::Time::now();
		loop3_enabled = true;
		if(loop3_rate > max_rate) { max_rate = loop3_rate; }
	}
	ros_rate = max_rate * 4.0;
	if(ros_rate > 100.0) { ros_rate = 100.0; }
	if(ros_rate <= 1.0)
	{
		ros_rate = 20.0;
	}
	char tempstr[512];
	sprintf(tempstr,"Running Node at Rate: %4.2f Hz.",ros_rate);
	logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
	diagnostic = diag;
	return diag;
}

bool BaseNode::update(uint8_t task_state)
{
	bool ok_to_run = false;
	if(require_pps_to_start == false) { ok_to_run = true; }
	else if((require_pps_to_start == true) and (pps_received == true)) { ok_to_run = true; }
	if(ok_to_run == false)
	{
		ros::Duration d(1.0);
		d.sleep();
		ros::spinOnce();
		logger->log_notice(__FILE__,__LINE__,"Waiting on PPS To Start.");
		return true;
	}
	ros::Rate r(ros_rate);
	r.sleep();
	ros::spinOnce();
	double mtime;
	mtime = measure_time_diff(ros::Time::now(),last_001hz_timer);
	
	if(mtime >= 100.0)
	{
		run_001hz();
		last_001hz_timer = ros::Time::now();
	}
	mtime = measure_time_diff(ros::Time::now(),last_01hz_noisy_timer);
	if(mtime >= 10.0 + rand_delay_sec)
	{
		rand_delay_sec = (double)(rand() % 2000 - 1000)/1000.0;  
		run_01hz_noisy();
		last_01hz_noisy_timer = ros::Time::now();
		logger->log_info("",0,"Task State: " + std::to_string(task_state));
		
	}
	mtime = measure_time_diff(ros::Time::now(),last_01hz_timer);
	if(mtime >= 10.0)
	{
		run_01hz();
		last_01hz_timer = ros::Time::now();
		firmware_pub.publish(firmware);
		if(resourcemonitor_initialized == true)
		{
			eros::resource resource = resourcemonitor->get_resourceused();
			resource.stamp = ros::Time::now();
			resource_diagnostic = resourcemonitor->update();
			if(resource_diagnostic.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
			{
				logger->log_diagnostic(resource_diagnostic);
				diagnostic_pub.publish(resource_diagnostic);
			}
			else if(resource_diagnostic.Level >= WARN)
			{
				resource_pub.publish(resource);
				logger->log_diagnostic(resource_diagnostic);
				diagnostic_pub.publish(resource_diagnostic);
			}
			else
			{
				resource_pub.publish(resource);
			}
		}
		else
		{
			logger->log_warn(__FILE__,__LINE__,"Resource Monitor Not Initialized.");
		}
	}
	mtime = measure_time_diff(ros::Time::now(),last_1hz_timer);
	if(mtime >= 1.0)
	{
		run_1hz();
		last_1hz_timer = ros::Time::now();


	}
	mtime = measure_time_diff(ros::Time::now(),last_10hz_timer);
	if(mtime >= 0.1)
	{
		run_10hz();
		heartbeat.TaskState = task_state;
		heartbeat.stamp = ros::Time::now();
		heartbeat_pub.publish(heartbeat);
		if(publish_readytoarm == true)
		{
			std_msgs::Bool readytoarm_msg;
			readytoarm_msg.data = ready_to_arm;
			readytoarm_pub.publish(readytoarm_msg);
		}
		last_10hz_timer = ros::Time::now();
	}
	
	if(loop1_enabled == true)
	{
		mtime = measure_time_diff(ros::Time::now(),last_loop1_timer);
		if(mtime >= (1.0/loop1_rate))
		{
			run_loop1();
			last_loop1_timer = ros::Time::now();
		}
	}
	if(loop2_enabled == true)
	{
		mtime = measure_time_diff(ros::Time::now(),last_loop2_timer);
		if(mtime >= (1.0/loop2_rate))
		{
			run_loop2();
			last_loop2_timer = ros::Time::now();
		}
	}
	if(loop3_enabled == true)
	{
		mtime = measure_time_diff(ros::Time::now(),last_loop3_timer);
		if(mtime >= (1.0/loop3_rate))
		{
			run_loop3();
			last_loop3_timer = ros::Time::now();
		}
	}
	return ros::ok();
}
void BaseNode::new_ppsmsg(const std_msgs::Bool::ConstPtr& t_msg)
{
	if(t_msg->data == true)
	{
		pps_received = true;
	}
}

void BaseNode::new_commandmsg_result(const eros::command::ConstPtr& t_msg,std::vector<eros::diagnostic> t_diaglist)
{
	if (t_msg->Command == ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if((t_msg->Option1 >= LEVEL3) and (t_diaglist.size() == 1) and (t_diaglist.at(0).Diagnostic_Message == DIAGNOSTIC_PASSED))
		{
			get_logger()->log_diagnostic(t_diaglist.at(0));
			diagnostic_pub.publish(t_diaglist.at(0));

		}
		else
		{
			for(std::size_t i = 0; i < t_diaglist.size(); i++)
			{
				if(t_diaglist.at(i).Level >= NOTICE)
				{
					get_logger()->log_diagnostic(t_diaglist.at(i));
					diagnostic_pub.publish(t_diaglist.at(i));
				}
			}
		}
	}
	else if(t_msg->Command == ROVERCOMMAND_SETLOGLEVEL)
	{
		bool change_log_level = false;
		if(t_msg->Option1 == ENTIRE_SUBSYSTEM)
		{
			change_log_level = true;
		}
		else if(t_msg->Option1 == diagnostic.Component)
		{
			change_log_level = true;
		}
		else if(get_basenodename().find(t_msg->CommandText) != std::string::npos)
		{
			change_log_level = true;
		}
		else if(get_nodename().find(t_msg->CommandText) != std::string::npos)
		{
			change_log_level = true;
		}
		if(change_log_level)
		{
			get_logger()->set_logverbosity(t_msg->Option2);
		}
	}
}
void BaseNode::base_cleanup()
{
	heartbeat.TaskState = TASKSTATE_FINISHED;
	heartbeat.stamp = ros::Time::now();
	heartbeat_pub.publish(heartbeat);
}