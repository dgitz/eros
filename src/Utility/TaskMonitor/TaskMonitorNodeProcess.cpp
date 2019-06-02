#include "TaskMonitorNodeProcess.h"
eros::diagnostic TaskMonitorNodeProcess::update(double t_dt,double t_ros_time)
{
    eros::diagnostic diag = diagnostic;
    ros_time = t_ros_time;
    run_time += t_dt;
    for(std::size_t i = 0; i < tasklist.size(); ++i)
    {
        if(tasklist.at(i).initialized == false)
        {
            if((tasklist.at(i).pid > 0) and (tasklist.at(i).last_heartbeat > 0.0))
            {
                tasklist.at(i).initialized = true;
            }
        }
        if(tasklist.at(i).initialized == true)
        {
            //Other checks
            if(tasklist.at(i).type == TaskType::EROS)
            {
                tasklist.at(i).last_heartbeat_delta = fabs(ros_time - tasklist.at(i).last_heartbeat);
                double max_delay = std::max(tasklist.at(i).last_heartbeat_delta,tasklist.at(i).last_ping_time);
                tasklist.at(i).last_heartbeat_delta = max_delay;
                if((tasklist.at(i).last_heartbeat_delta >= 0.0) and (tasklist.at(i).last_heartbeat_delta < COMMTIMEOUT_THRESHOLD))
                {
                    tasklist.at(i).state = TASKSTATE_RUNNING;;
                }
                else
                {
                     tasklist.at(i).state = TASKSTATE_NODATA;
                }
            }
            else
            {
                tasklist.at(i).last_heartbeat_delta = tasklist.at(i).last_ping_time;
                if(tasklist.at(i).last_heartbeat_delta < COMMTIMEOUT_THRESHOLD)
                {
                    tasklist.at(i).state = TASKSTATE_RUNNING;;
                }
                else
                {
                     tasklist.at(i).state = TASKSTATE_NODATA;
                }
            }
        }
    }
    diagnostic = diag;
    return diagnostic; 
}
eros::heartbeat TaskMonitorNodeProcess::convert_fromptr(const eros::heartbeat::ConstPtr &t_ptr)
{
	eros::heartbeat msg;
	msg.stamp = t_ptr->stamp;
	msg.BaseNode_Name = t_ptr->BaseNode_Name;
    msg.Node_Name = t_ptr->Node_Name;
	return msg;
}
eros::resource TaskMonitorNodeProcess::convert_fromptr(const eros::resource::ConstPtr &t_ptr)
{
	eros::resource msg;
	msg.stamp = t_ptr->stamp;
    msg.Node_Name = t_ptr->Node_Name;
    msg.PID = t_ptr->PID;
    msg.RAM_MB = t_ptr->RAM_MB;
    msg.CPU_Perc = t_ptr->CPU_Perc;
	return msg;
}
bool TaskMonitorNodeProcess::read_nodelist(std::string node_list_path,std::vector<std::string> *hosts,std::vector<std::string> *nodes)
{
    bool status = false;
    std::string line;
	std::ifstream allnodelist_file(node_list_path.c_str());
    std::string list;
	if (allnodelist_file.is_open())
	{
		while (getline(allnodelist_file, line))
		{
			std::vector<std::string> items;
			boost::split(items, line, boost::is_any_of(":\t"));
			std::string host = items.at(1).substr(2, items.at(1).size());
            hosts->push_back(host);
            std::vector<std::string> items2;
            boost::split(items2, items.at(3), boost::is_any_of("/"));
            std::string node = items2.at(items2.size() - 1);
            nodes->push_back(node);
		}
		allnodelist_file.close();
	}
    if((hosts->size() == nodes->size()) and (nodes->size() > 0))
    {
        status = true;
    }
	return status;
}
eros::diagnostic TaskMonitorNodeProcess::init_nodelist(std::vector<std::string> hosts,std::vector<std::string> nodes)
{
    eros::diagnostic diag = diagnostic;
    for(std::size_t i = 0; i < hosts.size(); ++i)
    {
        if((nodes.at(i) != base_node_name) and 
           (nodes.at(i).find("rostopic") == std::string::npos))
        {
            Task task = create_task((uint16_t)i,hosts.at(i),nodes.at(i));
            tasklist.push_back(task);
        }
    }
    diag.Level = INFO;
    diag.Diagnostic_Message = INITIALIZING;
    char tempstr[512];
    sprintf(tempstr,"Initialized Node List: %d Nodes.",(int)tasklist.size());
    diag.Description = std::string(tempstr);
    diagnostic = diag;
    return diagnostic;
}
TaskMonitorNodeProcess::Task TaskMonitorNodeProcess::create_task(uint16_t id,std::string host,std::string name)
{
    Task task;
    task.id = id;
    task.initialized = false;
    task.host_device = host;
    task.node_name = name;
    task.type = TaskType::ROS;
    task.received_state = TASKSTATE_NODATA;
    task.state = TASKSTATE_NODATA;
    task.pid = -1;
    task.cpu_used_perc = -1;
    task.mem_used_perc = -1;
    task.last_heartbeat = -1.0;
    task.last_heartbeat_delta = 0.0;
    task.last_ping_time = -1.0;
    task.restart_count = 0;
    return task;
}
eros::diagnostic TaskMonitorNodeProcess::new_resourcemessage(const eros::resource::ConstPtr& t_ptr)
{
     eros::diagnostic diag = diagnostic;
    for(std::size_t i = 0; i < tasklist.size(); ++i)
    {
        if (t_ptr->Node_Name.find(tasklist.at(i).node_name) != std::string::npos) 
        {
            if((tasklist.at(i).initialized == true) and (tasklist.at(i).pid != -1))
            {
                if(tasklist.at(i).pid != t_ptr->PID)
                {
                    tasklist.at(i).restart_count++;
                }
            }
            tasklist.at(i).pid = t_ptr->PID;
            tasklist.at(i).cpu_used_perc = t_ptr->CPU_Perc;
            tasklist.at(i).mem_used_perc = t_ptr->RAM_MB;
            tasklist.at(i).type = TaskType::EROS;
            tasklist.at(i).initialized = true;
        }
    }
    diag.Level = INFO;
    diag.Diagnostic_Message = NOERROR;
    diag.Description = "Updated.";
    diagnostic = diag;
    return diagnostic;
}
eros::diagnostic TaskMonitorNodeProcess::new_heartbeatmessage(const eros::heartbeat::ConstPtr& t_ptr)
{
    eros::diagnostic diag = diagnostic;
    bool found = false;
    for(std::size_t i = 0; i < tasklist.size(); ++i)
    {
        if (t_ptr->Node_Name.find(tasklist.at(i).node_name) != std::string::npos) 
        {
            found = true;
            tasklist.at(i).last_heartbeat = t_ptr->stamp.toSec();
            tasklist.at(i).received_state = t_ptr->TaskState;
            tasklist.at(i).type = TaskType::EROS;
            tasklist.at(i).initialized = true;
        }
    }
    if(found == false)
    {
        diag.Level = WARN;
        diag.Diagnostic_Message = DROPPING_PACKETS;
        char tempstr[512];
        sprintf(tempstr,"Could not add Heartbeat topic: %s",t_ptr->Node_Name.c_str());
        diag.Description = std::string(tempstr);
    }
    else
    {
        diag.Level = INFO;
        diag.Diagnostic_Message = NOERROR;
        diag.Description = "Heartbeat Updated.";
    }
    diagnostic = diag;
    return diagnostic;
}
std::string TaskMonitorNodeProcess::get_taskheader()
{
    char buffer[mainwindow_width];
    sprintf(buffer,"ID\tDevice\t\t\tNode Name\t\t\t\t\t\tStatus\t\t      RESTARTS\tPID\tCPU\tRAM\tRx");
    return std::string(buffer);
}
std::vector<std::string> TaskMonitorNodeProcess::get_taskbuffer()
{
    uint16_t host_length = 20;
    uint16_t node_length = 40;
    uint16_t state_length = 20;
    std::vector<std::string> buffer;
    for(std::size_t i = 0; i < tasklist.size(); ++i)
    {
        char tempstr[mainwindow_width];
        sprintf(tempstr,"%d\t%s\t%s\t\t%s\t%lu\t%d\t%d\t%d\t%.2f",
            tasklist.at(i).id,
            fixed_width(tasklist.at(i).host_device,host_length).c_str(),
            fixed_width(tasklist.at(i).node_name,node_length).c_str(),
            fixed_width(map_taskstate_tostring(tasklist.at(i).state),state_length).c_str(),
            tasklist.at(i).restart_count,
            tasklist.at(i).pid,
            tasklist.at(i).cpu_used_perc,
            tasklist.at(i).mem_used_perc,
            tasklist.at(i).last_heartbeat_delta);
        buffer.push_back(std::string(tempstr));

        /*
        uint16_t nodename_width = 20;
			char nodename_buffer[nodename_width];
			if(tasklist.at(i).node_name.size() > nodename_width)
			{
				sprintf(nodename_buffer,"%.*s",nodename_width,tasklist.at(i).node_name.c_str());
			}
			else
			{
				sprintf(nodename_buffer,"%-*s",nodename_width,tasklist.at(i).node_name.c_str());
			}
			char buffer[256];
			sprintf(buffer,"%d\t%-.20s\t\t%s\t\tUNKNOWN\t\t%d\t\t%d\t\t%d\t\t%4.2f",
				tasklist.at(i).id,
				tasklist.at(i).host_device.c_str(),
				nodename_buffer,
				tasklist.at(i).pid,
				tasklist.at(i).cpu_used_perc,
				tasklist.at(i).mem_used_perc,
				tasklist.at(i).last_heartbeat);
			std::string str(buffer);
			
            */
    }
    return buffer;
}
std::string TaskMonitorNodeProcess::fixed_width(std::string item,uint16_t width)
{
    char buffer[width];
    if(item.size() > width)
    {
        sprintf(buffer,"%.*s",width,item.c_str());
    }
    else
    {
        sprintf(buffer,"%-*s",width,item.c_str());
    }
    return std::string(buffer);
}
std::string TaskMonitorNodeProcess::map_taskstate_tostring(uint8_t state)
{
    switch(state)
    {
        case TASKSTATE_UNDEFINED:
            return "UNDEFINED";
            break;
        case TASKSTATE_NODATA:
            return "NO DATA";
            break;
        case TASKSTATE_INITIALIZING:
            return "INITIALIZING";
            break;
        case TASKSTATE_RUNNING:
            return "RUNNING";
            break;
        case TASKSTATE_STOPPED:
            return "STOPPED";
            break;    
        default:
            return "UNDEFINED";
    }
}
eros::diagnostic TaskMonitorNodeProcess::ping_nodes()
{
    eros::diagnostic diag = diagnostic;
    uint16_t success_ping_count = 0;
    for(std::size_t i = 0; i < tasklist.size(); ++i)
    {
        if(tasklist.at(i).type == TaskType::ROS)
        {
        char tempstr[512];
        sprintf(tempstr,"rosnode ping -c 1 %s 2> /dev/null",tasklist.at(i).node_name.c_str());
        std::string result = exec(tempstr,true);
        std::size_t found1=result.find("time=");
        if (found1!=std::string::npos)
        {
            std::string substr1 = result.substr(found1+5);
            std::size_t found2 = substr1.find("ms");
            if(found2 != std::string::npos)
            {
                tasklist.at(i).initialized = true;
                success_ping_count++;
                std::string substr2 = substr1.substr(0,found2);
                tasklist.at(i).last_ping_time = std::atof(substr2.c_str())/1000.0;
            }
            else
            {
                tasklist.at(i).last_ping_time = -1.0;
            }
        }
        else
        {
            tasklist.at(i).last_ping_time = -1.0;
        }
        }
        else
        {
            success_ping_count++;
        }
        
    }
    if(success_ping_count == (uint16_t)tasklist.size())
    {
        diag.Level = INFO;
        diag.Diagnostic_Message = NOERROR;
        char tempstr[512];
        sprintf(tempstr,"All Nodes Pinged.");
        diag.Description = std::string(tempstr);
    }
    else
    {
        diag.Level = WARN;
        diag.Diagnostic_Message = DROPPING_PACKETS;
        char tempstr[512];
        sprintf(tempstr,"%d/%d Node Pings Unsuccessful.",(uint16_t)tasklist.size()-success_ping_count,(uint16_t)tasklist.size());
        diag.Description = std::string(tempstr);
    }
    return diag;

}
std::string TaskMonitorNodeProcess::exec(const char *cmd, bool wait_for_result)
{
	char buffer[512];
	std::string result = "";
	FILE *pipe = popen(cmd, "r");
    if (wait_for_result == false)
	{
		pclose(pipe);
        return "";
	}
	if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
	try
	{
		while (!feof(pipe))
		{
            if (fgets(buffer, 512, pipe) != NULL)
            {
               result += buffer;
            }
		}
	}
	catch (...)
	{
		pclose(pipe);
		throw;
	}
    return result;
}