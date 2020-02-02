#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../../include/controlgroup.h"

std::string Node_Name = "/unittest_controlgroup";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
void print_diagnostic(uint8_t level,eros::diagnostic diagnostic)
{
	if(diagnostic.Level >= level)
	{
		printf("Type: %d Message: %d Level: %d Device: %s Desc: %s\n",diagnostic.Diagnostic_Type,diagnostic.Diagnostic_Message,
			  		diagnostic.Level,diagnostic.DeviceName.c_str(),diagnostic.Description.c_str());
	}
}
ControlGroup *initializeobject(ControlGroup::Mode mode)
{

	eros::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = GPIO_NODE;
	ControlGroup *object;
	object = new ControlGroup;
	diagnostic = object->init(diagnostic,mode);
	print_diagnostic(WARN,diagnostic);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	if(mode == ControlGroup::Mode::PID)
	{
		bool status = object->set_PIDGains(0.0,0.0,0.0);
		EXPECT_TRUE(status);
		return object;
	}
	else
	{
		return object;
	}
}
TEST(Template, Object_Initialization)
{
	ControlGroup *process = initializeobject(ControlGroup::Mode::PID);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);
	EXPECT_TRUE(process->get_diagnostic().Level <= NOTICE);
}
TEST(Template, PID_Calculation)
{
	ControlGroup *process = initializeobject(ControlGroup::Mode::PID);
	EXPECT_TRUE(process->set_PIDGains(91.0,200.0,-0.41) == true);
	process->set_outputlimits(-500.0,500.0);
	process->set_max_deltaoutput(50000.0);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->is_ready() == false);
	eros::diagnostic diag = process->get_diagnostic();
	EXPECT_TRUE(diag.Level <= NOTICE);

	double dt = 0.02;
	double elap_time = 0.0;
	double current_value = 0.0;
	eros::signal output;
	int counter = 0;
	while(elap_time <= (1.0+dt))
	{
		diag = process->set_commandvalue(5.0);
		EXPECT_TRUE(diag.Level <= NOTICE);
		diag = process->set_inputvalue(current_value);
		EXPECT_TRUE(diag.Level <= NOTICE);
		diag = process->update(dt);
		print_diagnostic(WARN,diag);
		EXPECT_TRUE(diag.Level <= NOTICE);
		EXPECT_TRUE(process->is_initialized() == true);
		EXPECT_TRUE(process->is_ready() == true);

		output = process->get_output();
		current_value = (current_value * 0.8) + (output.value*.03);
		elap_time += dt;
		counter++;
		
	}
	
	output = process->get_output();
	EXPECT_TRUE(fabs(output.value-31.1504125544823) < .0001);

}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
