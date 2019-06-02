#include "taskmonitor_node.h"
bool kill_node = false;
WINDOW *create_newwin(int height, int width, int starty, int startx)
{	WINDOW *local_win;

	local_win = newwin(height, width, starty, startx);
	box(local_win, 0 , 0);		/* 0, 0 gives default characters 
					 * for the vertical and horizontal
					 * lines			*/
	wrefresh(local_win);		/* Show that box 		*/

	return local_win;
}
bool TaskMonitorNode::start(int argc, char **argv)
{
	bool status = false;
	rand_delay_sec = (double)(rand() % 2000 - 1000)/1000.0; 
	process = new TaskMonitorNodeProcess();
	start_node_index = 0;
	ros_rate = 30.0;
	base_node_name = "task_monitor";
	ros::init(argc,argv,base_node_name);
	n.reset(new ros::NodeHandle);
	node_name = ros::this_node::getName();
	host_name[1023] = '\0';
	gethostname(host_name,1023);
	eros::diagnostic diag = process->initialize(base_node_name, node_name, host_name);
	uptime_sub = n->subscribe<std_msgs::Float32>("/Uptime",1,&TaskMonitorNode::uptime_Callback,this);
	logger = new Logger("debug",node_name);
	logger->disable_consoleprint();
	boot_time = ros::Time::now();
	current_time = ros::Time::now();
	system("rosnode list -ua > /home/robot/unsorted && sort /home/robot/unsorted > /home/robot/config/AllNodeList");
	std::vector<std::string> hosts;
	std::vector<std::string> nodes;
	process->read_nodelist("/home/robot/config/AllNodeList",&hosts,&nodes);
	diag = process->init_nodelist(hosts,nodes);
	logger->log_diagnostic(diag);
	std::vector<TaskMonitorNodeProcess::Task> tasks = process->get_alltasks();
	for(std::size_t i = 0; i < tasks.size(); ++i)
	{
		char tempstr[512];
		sprintf(tempstr,"%d %s",(int)i,tasks.at(i).node_name.c_str());
		logger->log_debug(std::string(tempstr));
	}
	
	time (&rawtime);
  	timeinfo = localtime(&rawtime);
	std::ifstream f("/home/robot/config/ActiveScenario");
  	getline( f, active_scenario, '\0');
 	f.close();
	status = init_screen();
	if(status == false)
	{
		return false;
	} 
	diag = rescan_topics();
	return true;
}
bool TaskMonitorNode::init_screen()
{
	initscr();
	if(has_colors() == FALSE)
	{	endwin();
		char tempstr[512];
		sprintf(tempstr,"Terminal does not support colors. Exiting.");
		printf("%s\n",tempstr);
		logger->log_error(std::string(tempstr));
		return false;
	}
	curs_set(0);
    noecho();
    cbreak();
	
	start_color();
	init_color(COLOR_BLACK, 0, 0, 0);
	init_color(COLOR_GREEN,0,600,0);
	init_pair(NO_COLOR, COLOR_WHITE, COLOR_BLACK);	
	init_pair(RED_COLOR, COLOR_WHITE, COLOR_RED);	
	init_pair(YELLOW_COLOR, COLOR_WHITE, COLOR_YELLOW);
	init_pair(GREEN_COLOR, COLOR_WHITE, COLOR_GREEN);
	init_pair(BLUE_COLOR, COLOR_WHITE, COLOR_BLUE);			
	
	getmaxyx(stdscr,mainwindow_height,mainwindow_width);
	bool status = process->set_mainwindow(mainwindow_width,mainwindow_height);
	if(status == false)
	{
		char tempstr[512];
		sprintf(tempstr,"Window: Width: %d Height: %d is too small.  Exiting.",mainwindow_width,mainwindow_height);
		printf("%s\n",tempstr);
		logger->log_error(std::string(tempstr));
		return false;
	}
	window_header = create_newwin(mainwindow_height/6, mainwindow_width, 0, 0);
	window_tasklist = create_newwin(TASKPAGE_COUNT+4, mainwindow_width, mainwindow_height/6, 0);
	window_footer = create_newwin(mainwindow_height/6, mainwindow_width, mainwindow_height/6+TASKPAGE_COUNT+4, 0);
	wbkgd(window_header,COLOR_PAIR(NO_COLOR));
	keypad(window_header, TRUE);
	keypad(window_tasklist, TRUE);
	wtimeout(window_header,10);
	wtimeout(window_tasklist,10);
	char tempstr[512];
	sprintf(tempstr,"Using Window Width: %d Height: %d",mainwindow_width,mainwindow_height);
	logger->log_info(std::string(tempstr));
	wattron(window_header,COLOR_PAIR(BLUE_COLOR));
	char buffer[80];
	sprintf(buffer,"Initializing...");
	std::string str(buffer);
	mvwprintw(window_header,ACTIVESCENARIO_Y,(int)(mainwindow_width-str.size())/2,buffer);
	wattroff(window_header,COLOR_PAIR(BLUE_COLOR));
	wrefresh(window_header);
	return true;
}
bool TaskMonitorNode::update_windowheader()
{
	{//Scenario
		wattron(window_header,COLOR_PAIR(BLUE_COLOR));
		char buffer[80];
		sprintf(buffer,"Scenario:%s",active_scenario.c_str());
		std::string str(buffer);
		mvwprintw(window_header,ACTIVESCENARIO_Y,(int)(mainwindow_width-str.size())/2,buffer);
		wattroff(window_header,COLOR_PAIR(BLUE_COLOR));
	}
	{//Unix Time
		mvwprintw(window_header,ROSTIME_COORD_Y,ROSTIME_COORD_X,"Unix:%4.2fs",current_time.toSec());
	}
	{//Date Time 
		char buffer[80];
		strftime(buffer,sizeof(buffer),"Time:%b/%d/%Y %H:%M:%S",timeinfo);
		std::string str(buffer);
		mvwprintw(window_header,DATETIME_COORD_Y,mainwindow_width-str.size()-DATETIME_COORD_X,buffer);
	}
	{//UpTime
		double uptime = process->get_uptime();
		char buffer[32];
		if(uptime < 0)
		{
			sprintf(buffer,"Uptime: --");
		}
		else
		{
			sprintf(buffer,"Uptime:%4.2fs",uptime);
		}
		std::string str(buffer);
		mvwprintw(window_header,RUNTIME_COORD_Y,(int)(mainwindow_width-str.size())/2,buffer);
	}
	wrefresh(window_header);
	return true;
}
bool TaskMonitorNode::update_windowtasklist()
{
	{//Column Names
		mvwprintw(window_tasklist,TASKSTART_COORD_Y,TASKSTART_COORD_X,process->get_taskheader().c_str());
	}
	{//Dashed line
		char buffer[mainwindow_width-2];
		strcpy(buffer, "");
		for(uint16_t i = 0; i < mainwindow_width-2; ++i)
		{
			sprintf(buffer,"%s-",buffer);
		}
		std::string str(buffer);
		mvwprintw(window_tasklist,TASKSTART_COORD_Y+1,TASKSTART_COORD_X,buffer);
	}
	{//Task List
		std::vector<std::string> taskbuffer = process->get_taskbuffer();
		std::vector<TaskMonitorNodeProcess::Task> tasklist = process->get_alltasks();
		if(taskbuffer.size() != tasklist.size())
		{
			return false;
		}
		int index = 0;
		for(std::size_t i = start_node_index; i < taskbuffer.size(); ++i)
		{
			if(index >= TASKPAGE_COUNT)
			{
				break;
			}
			uint8_t color = 0;
			switch(tasklist.at(i).state)
			{
				case TASKSTATE_UNDEFINED:
					color = RED_COLOR;
					break;
					case TASKSTATE_INITIALIZING:
					color = YELLOW_COLOR;
					break;
					case TASKSTATE_NODATA:
					color = RED_COLOR;
					break;
					case TASKSTATE_RUNNING:
					color = GREEN_COLOR;
					break;
					case TASKSTATE_STOPPED:
					color = YELLOW_COLOR;
					default:
					color = RED_COLOR;
					break;
			}
			wattron(window_tasklist,COLOR_PAIR(color));
			mvwprintw(window_tasklist,TASKSTART_COORD_Y+2+(int)index,TASKSTART_COORD_X,taskbuffer.at(i).c_str());
			wattroff(window_tasklist,COLOR_PAIR(color));
			index++;
		}
	}
	wrefresh(window_tasklist);
	return true;
}
bool TaskMonitorNode::update_windowfooter()
{
	//Help text
	{
		char tempstr[512];
		sprintf(tempstr,"HELP:\n\tq: Quit.\n\tUp: Page Up. Down: Page Down.");
		mvwprintw(window_footer,1,3,tempstr);
	}
	wrefresh(window_footer);
	return true;
}
bool TaskMonitorNode::run_001hz()
{
	return true;
}
bool TaskMonitorNode::run_01hz()
{
	return true;
}
bool TaskMonitorNode::run_01hz_noisy()
{
	eros::diagnostic diag = rescan_topics();
	if(diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
	return true;	
}
bool TaskMonitorNode::run_1hz()
{
	eros::diagnostic diag = process->ping_nodes();
	if(diag.Level >= NOTICE)
	{
		logger->log_diagnostic(diag);
	}	
	if(update_windowheader() == false)
	{
		return false;
	}
	if(update_windowtasklist() == false)
	{
		return false;
	}
	if(update_windowfooter() == false)
	{
		return false;
	}
	return true;
}
bool TaskMonitorNode::run_10hz()
{
	eros::diagnostic diagnostic = process->update(.1,ros::Time::now().toSec());
	if(diagnostic.Level > NOTICE)
	{
		logger->log_diagnostic(diagnostic);
	}
	std::vector<TaskMonitorNodeProcess::Task> tasklist = process->get_alltasks();
	int v = wgetch(window_tasklist);
	switch(v)
	{
		case KEY_UP:
			if(start_node_index > 0)
			{
				--start_node_index;
			}
			break;
		case KEY_DOWN:
			if(start_node_index < (tasklist.size()-TASKPAGE_COUNT))
			{
				++start_node_index;
			}
			break;
		case KEY_RESIZE:
			logger->log_debug("RESIZED");
		case KEY_Q:
			kill_node = true;
		default:
			break;
	}
	current_time = ros::Time::now();
	time (&rawtime);
  	timeinfo = localtime(&rawtime);
	return true;
}
eros::diagnostic TaskMonitorNode::rescan_topics()
{
	eros::diagnostic diag = process->get_diagnostic();
	int found_new_topics = 0;

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;
		if(info.datatype == "eros/resource")
		{
			int v = process->push_topiclist(info.datatype,info.name);
			if(v == 1)
			{
				found_new_topics++;
				char tempstr[255];
				sprintf(tempstr,"Subscribing to resource topic: %s",info.name.c_str());
				logger->log_info(tempstr);
				ros::Subscriber sub = n->subscribe<eros::resource>(info.name,20,&TaskMonitorNode::resource_Callback,this);
				resource_subs.push_back(sub);
			}
		}
		if(info.datatype == "eros/heartbeat")
		{
			int v = process->push_topiclist(info.datatype,info.name);
			if(v == 1)
			{
				found_new_topics++;
				char tempstr[255];
				sprintf(tempstr,"Subscribing to heartbeat topic: %s",info.name.c_str());
				logger->log_info(tempstr);
				ros::Subscriber sub = n->subscribe<eros::heartbeat>(info.name,20,&TaskMonitorNode::heartbeat_Callback,this);
				heartbeat_subs.push_back(sub);
			}
		}
	}
	char tempstr[255];
	if(found_new_topics > 0)
	{
		sprintf(tempstr,"Rescanned and found %d new topics.",found_new_topics);
	}
	else
	{
		sprintf(tempstr,"Rescanned and found no new topics.");
	}
	logger->log_info(tempstr);
	return diag;
}
void TaskMonitorNode::uptime_Callback(const std_msgs::Float32::ConstPtr& msg)
{
	process->set_uptime(msg->data);
}
void TaskMonitorNode::heartbeat_Callback(const eros::heartbeat::ConstPtr& msg)
{
	eros::diagnostic diag = process->new_heartbeatmessage(msg);
	if(diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
}
void TaskMonitorNode::resource_Callback(const eros::resource::ConstPtr& msg)
{
	eros::diagnostic diag = process->new_resourcemessage(msg);
	if(diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
}

void TaskMonitorNode::thread_loop()
{
	while (kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
bool TaskMonitorNode::update()
{
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
	}
	mtime = measure_time_diff(ros::Time::now(),last_01hz_timer);
	if(mtime >= 10.0)
	{
		run_01hz();
		last_01hz_timer = ros::Time::now();
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
		last_10hz_timer = ros::Time::now();
	}
	return ros::ok();
}

void TaskMonitorNode::cleanup()
{
	delwin(window_header);
	delwin(window_tasklist);
	delwin(window_footer);
	endwin();
	logger->log_notice("Node Safely Exited.");
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	printf("Killing Node with Signal: %d", sig);
	kill_node = true;
	exit(1);
}
int main(int argc, char **argv)
{
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	TaskMonitorNode *node = new TaskMonitorNode();
	bool status = node->start(argc, argv);
	if(status == false)
	{
		endwin();
		exit(0);
	}
	std::thread thread(&TaskMonitorNode::thread_loop, node);
	while ((status == true) and (kill_node == false))
	{
		status = node->update();
	}

	node->cleanup();
	thread.detach();
	return 0;
}
