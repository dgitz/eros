#include "SystemMonitorNodeProcess.h"
eros::diagnostic SystemMonitorNodeProcess::update(double t_dt,double t_ros_time)
{
    eros::diagnostic diag = diagnostic;
    ros_time = t_ros_time;
    run_time += t_dt;
    for(std::size_t i = 0; i < modulelist.size(); ++i)
    {
        modulelist.at(i).last_heartbeat_delta = fabs(ros_time - modulelist.at(i).last_heartbeat);
        if((modulelist.at(i).last_heartbeat_delta >= 0.0) and (modulelist.at(i).last_heartbeat_delta < (10.0*COMMTIMEOUT_THRESHOLD)))
        {
            modulelist.at(i).state = TASKSTATE_RUNNING;;
        }
        else
        {
            modulelist.at(i).state = TASKSTATE_NODATA;
        }
    }
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
eros::heartbeat SystemMonitorNodeProcess::convert_fromptr(const eros::heartbeat::ConstPtr &t_ptr)
{
	eros::heartbeat msg;
	msg.stamp = t_ptr->stamp;
	msg.BaseNode_Name = t_ptr->BaseNode_Name;
    msg.Node_Name = t_ptr->Node_Name;
	return msg;
}
eros::resource SystemMonitorNodeProcess::convert_fromptr(const eros::resource::ConstPtr &t_ptr)
{
	eros::resource msg;
	msg.stamp = t_ptr->stamp;
    msg.Node_Name = t_ptr->Node_Name;
    msg.PID = t_ptr->PID;
    msg.RAM_MB = t_ptr->RAM_MB;
    msg.CPU_Perc = t_ptr->CPU_Perc;
	return msg;
}
bool SystemMonitorNodeProcess::read_nodelist(std::string node_list_dir,std::vector<std::string> *hosts,std::vector<std::string> *nodes,
    std::vector<std::string> *resource_topics,
    std::vector<std::string> *heartbeat_topics,
    std::vector<std::string> *loadfactor_topics,
    std::vector<std::string> *resource_available_topics)
{
    bool status = false;
    std::string line;
    std::string node_list_path = node_list_dir + "NodeList.txt";
	std::ifstream nodelist_file(node_list_path.c_str());
    std::string list;
    uint16_t line_counter = -1;
    std::vector<std::string> instance_names;
	if (nodelist_file.is_open())
	{
		while (getline(nodelist_file, line))
		{
            line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
            line_counter++;
            boost::trim_left(line);
            if(line.size() > 0)
            {
                if(line.at(0) == '#')
                {
                    continue;
                }
            }
            else
            {
                continue;
            }

            if(line.find("=>") == std::string::npos)
            {
                continue;
            }
			std::vector<std::string> items;
			boost::split(items, line, boost::is_any_of("=#"));
            if(items.size() < 2)
            {
                printf("[ERROR] Unable to parse file: %s at line: %d\n",node_list_path.c_str(),line_counter);
            }
            std::string instance_name = items.at(0).substr(1,items.at(0).size()-2);
            boost::trim_left(instance_name);
            boost::trim_right(instance_name);
            std::size_t index = items.at(1).find("device[");
            std::string tempstr = items.at(1).substr(index+8);
            std::string host = tempstr.substr(0,tempstr.find("\""));
            instance_names.push_back(instance_name);
            hosts->push_back(host);
            line_counter++;
		}
		nodelist_file.close();
	}
    if(hosts->size() == 0)
    {
        printf("[ERROR]: Could not read any hosts in: %s\n",node_list_path.c_str());
        return false;
    }
    if(hosts->size() != instance_names.size())
    {
        printf("[ERROR]: Host/Instance Name Count Mismatch in file: %s\n",node_list_path.c_str());
        return false;
    }
    for(std::size_t i = 0; i < instance_names.size(); ++i)
    {
        std::string node_launch_file = node_list_dir + "launch/NodeLaunch/" + instance_names.at(i) + ".xml";
        std::ifstream file(node_launch_file.c_str());

        if (file.is_open())
        {
            std::string line;
            while (getline(file, line))
		    {
                std::size_t index = line.find("<node name=\"$(env ROS_HOSTNAME)_");
                if(index != std::string::npos)
                {
                    std::string tempstr = line.substr(index+32);
                    std::string name = tempstr.substr(0,tempstr.find("\""));
                    std::string node_name = "/" + hosts->at(i) + "_" + name;
                    nodes->push_back(node_name);
                    break;
                }
            }
        }
        else
        {
            printf("[ERROR]: Could not load: %s\n",node_launch_file.c_str());
            return false;
        }
        file.close();
        
    }
    for(std::size_t i = 0; i < nodes->size(); ++i)
    {
        {
            std::string tempstr = nodes->at(i) + "/resource";
            resource_topics->push_back(tempstr);
            push_topiclist("eros/resource",tempstr);
        }
        {
            std::string tempstr = nodes->at(i) + "/heartbeat";
            heartbeat_topics->push_back(tempstr);
            push_topiclist("eros/heartbeat",tempstr);
        }
    }
    for(std::size_t i = 0; i < hosts->size(); ++i)
    {
        bool add_me = true;
        for(std::size_t j = 0; j < modulelist.size(); ++j)
        {
            if(std::string::npos != modulelist.at(j).name.find(hosts->at(i)))
            {
                add_me = false;
            }
        }
        if(add_me == true)
        {
            SystemMonitorNodeProcess::Module module;
            module.name = hosts->at(i);
            module.initialized = true;
            module.RAMFree_Perc = -1;
            module.CPUFree_Perc = -1;
            module.DISKFree_Perc = -1;
            module.loadfactor_1min = -1.0;
            module.loadfactor_5min = -1.0;
            module.loadfactor_15min = -1.0;
            module.last_heartbeat = 0.0;
            module.last_heartbeat_delta = 0.0;
            module.state = TASKSTATE_NODATA;
            std::string tempstr = "/" + module.name + "/resource_available";
            resource_available_topics->push_back(tempstr);
            push_topiclist("eros/resource",tempstr);
            modulelist.push_back(module);
            std::string tempstr2 = "/" + module.name + "/loadfactor";
            push_topiclist("eros/loadfactor",tempstr2);
            loadfactor_topics->push_back(tempstr2);
        }
    }

    if((hosts->size() == nodes->size()) and (nodes->size() > 0))
    {
        status = true;
    }
    else
    {
        printf("[ERROR] Parsing Node List Directory: %s\n",node_list_dir.c_str());
    }
    
	return status;
}
eros::diagnostic SystemMonitorNodeProcess::init_nodelist(std::vector<std::string> hosts,std::vector<std::string> nodes)
{
    eros::diagnostic diag = diagnostic;
    for(std::size_t i = 0; i < hosts.size(); ++i)
    {
        if((nodes.at(i) != base_node_name) and 
           (nodes.at(i).find("rostopic") == std::string::npos) and 
           (nodes.at(i).find("rosout") == std::string::npos))
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
SystemMonitorNodeProcess::Task SystemMonitorNodeProcess::create_task(uint16_t id,std::string host,std::string name)
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
eros::diagnostic SystemMonitorNodeProcess::new_truthpose(const eros::pose::ConstPtr& t_ptr)
{
    eros::diagnostic diag = diagnostic;
    char tempstr[256];
    sprintf(tempstr,"Truth Pose: N: %4.2f(m) E: %4.2f(m) Z: %4.2f(m) Heading: %4.2f(deg)==%s",
        t_ptr->north.value,
        t_ptr->east.value,
        t_ptr->elev.value,
        t_ptr->yaw.value,
        pose_helper.compute_heading_simplestring(PoseHelper::HeadingReference::COMMON_YAW,t_ptr->yaw.value).c_str());
    truthpose_state = t_ptr->yaw.status; 
    truthpose_string = std::string(tempstr);
    return diag;
}
eros::diagnostic SystemMonitorNodeProcess::new_systemsnapshotstatemessage(const eros::systemsnapshot_state::ConstPtr& t_ptr)
{
     eros::diagnostic diag = diagnostic;
     snap.state.state = t_ptr->state;
     snap.state.percent_complete = t_ptr->percent_complete;
     snap.state.systemsnapshot_count = t_ptr->systemsnapshot_count;
     snap.state.source_device = t_ptr->source_device;
     snap.state.snapshot_path = t_ptr->snapshot_path;
     return diag;
}
eros::diagnostic SystemMonitorNodeProcess::new_loadfactormessage(const eros::loadfactor::ConstPtr& t_ptr)
{
    bool found = false;
    eros::diagnostic diag = diagnostic;
    for(std::size_t i = 0; i < modulelist.size(); ++i)
    {
        if(t_ptr->DeviceName.find(modulelist.at(i).name) != std::string::npos)
        {
            found = true;
            modulelist.at(i).last_heartbeat = t_ptr->stamp.toSec();
            modulelist.at(i).loadfactor_1min = t_ptr->loadfactor[0];
            modulelist.at(i).loadfactor_5min = t_ptr->loadfactor[1];
            modulelist.at(i).loadfactor_15min = t_ptr->loadfactor[2];
        }
    }
    if(found == false)
    {
        diag.Level = WARN;
        diag.Diagnostic_Message = DROPPING_PACKETS;
        diag.Description = "Did not find Module: " + t_ptr->DeviceName;
        return diag;
    }
    else
    {
        diag.Level = INFO;
        diag.Diagnostic_Message = NOERROR;
        diag.Description = "LoadFactor Updated.";
        return diag;
    }
   return diag;
}

eros::diagnostic SystemMonitorNodeProcess::new_resourceavailablemessage(const eros::resource::ConstPtr& t_ptr)
{
    bool found = false;
    eros::diagnostic diag = diagnostic;
    for(std::size_t i = 0; i < modulelist.size(); ++i)
    {
        if(t_ptr->Node_Name.find(modulelist.at(i).name) != std::string::npos)
        {
            found = true;
            modulelist.at(i).last_heartbeat = t_ptr->stamp.toSec();
            modulelist.at(i).RAMFree_Perc = t_ptr->RAM_Perc;
            modulelist.at(i).CPUFree_Perc = t_ptr->CPU_Perc;
            modulelist.at(i).DISKFree_Perc = t_ptr->DISK_Perc;
        }
    }
    if(found == false)
    {
        diag.Level = WARN;
        diag.Diagnostic_Message = DROPPING_PACKETS;
        diag.Description = "Did not find Module: " + t_ptr->Node_Name;
        return diag;
    }
    else
    {
        diag.Level = INFO;
        diag.Diagnostic_Message = NOERROR;
        diag.Description = "Resource Available Updated.";
        return diag;
    }
}
eros::diagnostic SystemMonitorNodeProcess::new_resourcemessage(const eros::resource::ConstPtr& t_ptr)
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
eros::diagnostic SystemMonitorNodeProcess::new_heartbeatmessage(const eros::heartbeat::ConstPtr& t_ptr)
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
std::string SystemMonitorNodeProcess::get_taskheader()
{
    char buffer[mainwindow_width];
    sprintf(buffer,"   ID\tDevice\t\t\tNode Name\t\t\t\t\t\tStatus\t\t      RESTARTS\tPID\tCPU(%%)\tRAM(MB)\tRx");
    return std::string(buffer);
}
std::vector<std::string> SystemMonitorNodeProcess::get_modulelistheader()
{
    std::vector<std::string> list;
/*    {
        char buffer[256];
        sprintf(buffer,"\t\t--- AVAILABLE RESOURCE ---");
        list.push_back(std::string(buffer));
    }
    */
    {
        char buffer[256];
        sprintf(buffer,"Device\t\tAv RAM(%%) Av CPU(%%) Av DISK(%%) Load Factor\tRx\t\t");
        list.push_back(std::string(buffer));
    }
    return list;
}
std::vector<std::string> SystemMonitorNodeProcess::get_modulebuffer()
{
    std::vector<std::string> list;
    for(std::size_t i = 0; i < modulelist.size(); ++i)
    {
        int ramfree_perc = modulelist.at(i).RAMFree_Perc;
        int cpufree_perc = modulelist.at(i).CPUFree_Perc;
        int diskfree_perc = modulelist.at(i).DISKFree_Perc;
        double loadfactor_1min = modulelist.at(i).loadfactor_1min;
        double loadfactor_5min = modulelist.at(i).loadfactor_5min;
        double loadfactor_15min = modulelist.at(i).loadfactor_15min;
        double dt = modulelist.at(i).last_heartbeat_delta;
        if(modulelist.at(i).state == TASKSTATE_NODATA)
        {
            ramfree_perc = -1;
            cpufree_perc = -1;
            diskfree_perc = -1;
            loadfactor_1min = -1.0;
            loadfactor_5min = -1.0;
            loadfactor_15min = -1.0;
            dt = -1.0;
        }
        char tempstr[128];
        sprintf(tempstr,"%s\t\t%d\t%d\t%d\t[%2.1f,%2.1f,%2.1f]\t%4.2f", 
            modulelist.at(i).name.c_str(),
            ramfree_perc,
            cpufree_perc,
            diskfree_perc,
            loadfactor_1min,
            loadfactor_5min,
            loadfactor_15min,
            dt);
        list.push_back(std::string(tempstr));
    }
    return list;
}
std::vector<std::string> SystemMonitorNodeProcess::get_taskbuffer()
{
    uint16_t host_length = 20;
    uint16_t node_length = 40;
    uint16_t state_length = 20;
    std::vector<std::string> buffer;
    for(std::size_t i = 0; i < tasklist.size(); ++i)
    {
        int pid = tasklist.at(i).pid;
        int cpu_used_perc = tasklist.at(i).cpu_used_perc;
        int mem_used_perc = tasklist.at(i).mem_used_perc;
        double dt = tasklist.at(i).last_heartbeat_delta;
        if(tasklist.at(i).state == TASKSTATE_NODATA)
        {
            pid = -1;
            cpu_used_perc = -1;
            mem_used_perc = -1;
            dt = -1.0;
        }
        char tempstr[mainwindow_width];
        sprintf(tempstr,"%d\t%s\t%s\t\t%s\t%lu\t%d\t%d\t%d\t%.2f",
            tasklist.at(i).id,
            fixed_width(tasklist.at(i).host_device,host_length).c_str(),
            fixed_width(tasklist.at(i).node_name,node_length).c_str(),
            fixed_width(map_taskstate_tostring(tasklist.at(i).state),state_length).c_str(),
            tasklist.at(i).restart_count,
            pid,
            cpu_used_perc,
            mem_used_perc,
            dt);
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
std::string SystemMonitorNodeProcess::fixed_width(std::string item,uint16_t width)
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
std::string SystemMonitorNodeProcess::map_taskstate_tostring(uint8_t state)
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
eros::diagnostic SystemMonitorNodeProcess::ping_nodes()
{
    eros::diagnostic diag = diagnostic;
    uint16_t success_ping_count = 0;
    for(std::size_t i = 0; i < tasklist.size(); ++i)
    {
        /*
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
        */
        
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
std::string SystemMonitorNodeProcess::exec(const char *cmd, bool wait_for_result)
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