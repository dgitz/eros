#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../TaskMonitorNodeProcess.h"

std::string Node_Name = "/unittest_taskmonitor_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
void print_taskinfo(std::vector<TaskMonitorNodeProcess::Task> list)
{
	for(std::size_t i = 0; i < list.size(); ++i)
	{
		printf("[%d] Host: %s Name: %s\n",list.at(i).id,list.at(i).host_device.c_str(),list.at(i).node_name.c_str());
	}
}
TaskMonitorNodeProcess *initializeprocess()
{
	uint16_t mainwindow_width = 200;
	uint16_t mainwindow_height = 50;
	TaskMonitorNodeProcess *process;
	process = new TaskMonitorNodeProcess;
	process->initialize("taskmonitor_node", Node_Name, Host_Name);
	bool status = process->set_mainwindow(200,50);
	if(status == false)
	{
		char tempstr[512];
		sprintf(tempstr,"Window: Width: %d Height: %d is too small.  Exiting.",mainwindow_width,mainwindow_height);
		printf("%s\n",tempstr);
	}
	EXPECT_TRUE(status);
	std::vector<std::string> hosts;
	std::vector<std::string> nodes;
	EXPECT_TRUE(process->read_nodelist("/home/robot/catkin_ws/src/eROS/src/Utility/TaskMonitor/unit_tests/UnitTest_AllNodeList",&hosts,&nodes) == true);
	eros::diagnostic diag = process->init_nodelist(hosts,nodes);
	EXPECT_TRUE(diag.Level <= NOTICE);
	
	return process;
}

TEST(Template, Process_Initialization)
{
	TaskMonitorNodeProcess *process = initializeprocess();
	
	
}
TEST(Execution,Process_Execution)
{
	TaskMonitorNodeProcess *process = initializeprocess();
	eros::diagnostic diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	print_taskinfo(process->get_alltasks());

	std::vector<TaskMonitorNodeProcess::Task> tasklist = process->get_alltasks();
	double current_time = 0.0;
	double dt = 0.1;
	double time_since_ping = 0.0;
	while(current_time <= 20.0)
	{
		for(std::size_t i = 0; i < tasklist.size(); ++i)
		{
			{
				eros::heartbeat msg;
				msg.Node_Name = "/" + tasklist.at(i).node_name;
				eros::heartbeat::ConstPtr msg_ptr(new eros::heartbeat(msg));
				diag = process->new_heartbeatmessage(msg_ptr);
				EXPECT_TRUE(diag.Level <= NOTICE);
			}

			{
				eros::resource msg;
				msg.Node_Name = "/" + tasklist.at(i).node_name;
				msg.PID = (i+10)*10;
				msg.RAM_MB = (i+20)*5;
				msg.CPU_Perc = (i+2)*2;
				eros::resource::ConstPtr msg_ptr(new eros::resource(msg));
				diag = process->new_resourcemessage(msg_ptr);
				EXPECT_TRUE(diag.Level <= NOTICE);
			}
		}
		eros::diagnostic diag;
		/*time_since_ping+=dt;
		if(time_since_ping > 1.0)
		{
			time_since_ping = 0.0;
			diag = process->ping_nodes();
			EXPECT_TRUE(diag.Level <= NOTICE);
		}
		*/
		diag = process->update(dt,current_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
		current_time += dt;
	
		usleep(dt*1000000.0);
	}
	std::vector<std::string> buffer = process->get_taskbuffer();
	for(std::size_t i = 0; i < buffer.size(); ++i)
	{
		printf("%s\n",buffer.at(i).c_str());
	}
	tasklist = process->get_alltasks();
	for(std::size_t i = 0; i < tasklist.size(); ++i)
	{
		if(tasklist.at(i).node_name == "rosout")
		{

		}
		else
		{
			EXPECT_TRUE(tasklist.at(i).initialized == true);
			EXPECT_TRUE(tasklist.at(i).type == TaskMonitorNodeProcess::TaskType::EROS);
			EXPECT_TRUE(tasklist.at(i).state == TASKSTATE_RUNNING);
		}
	}
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
