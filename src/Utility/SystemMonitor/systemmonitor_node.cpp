#include "systemmonitor_node.h"
bool kill_node = false;
WINDOW *create_newwin(int height, int width, int starty, int startx)
{
	WINDOW *local_win;

	local_win = newwin(height, width, starty, startx);
	box(local_win, 0, 0); /* 0, 0 gives default characters 
					 * for the vertical and horizontal
					 * lines			*/
	wrefresh(local_win);  /* Show that box 		*/

	return local_win;
}
bool SystemMonitorNode::start(int argc, char **argv)
{
	selected_task_index = -1;
	bool status = false;
	select_task_mode = false;
	change_loglevel_mode = false;
	taskcommand_pause_mode = false;
	taskcommand_resume_mode = false;
	taskcommand_reset_mode = false;
	selected_loglevel_value = -1;
	rand_delay_sec = (double)(rand() % 2000 - 1000) / 1000.0;
	process = new SystemMonitorNodeProcess();
	start_node_index = 0;
	ros_rate = 30.0;
	base_node_name = "system_monitor";
	ros::init(argc, argv, base_node_name);
	n.reset(new ros::NodeHandle);
	node_name = ros::this_node::getName();
	host_name[1023] = '\0';
	gethostname(host_name, 1023);
	eros::diagnostic diag = process->initialize(base_node_name, node_name, host_name);
	uptime_sub = n->subscribe<std_msgs::Float32>("/Uptime", 1, &SystemMonitorNode::uptime_Callback, this);
	snapshotstate_sub = n->subscribe<eros::systemsnapshot_state>("/System/Snapshot/State", 5, &SystemMonitorNode::snapshotstate_Callback, this);
	command_pub = n->advertise<eros::command>("/command", 20);
	battery_sub = n->subscribe<eros::battery>("/battery",1,&SystemMonitorNode::battery_Callback,this);
	logger = new Logger("debug", node_name);
	logger->disable_consoleprint();
	boot_time = ros::Time::now();
	current_time = ros::Time::now();
	std::ifstream f("/home/robot/config/ActiveScenario");
	getline(f, active_scenario, '\0');
	f.close();
	boost::trim_left(active_scenario);
	boost::trim_right(active_scenario);
	//system("rosnode list -ua > /home/robot/unsorted && sort /home/robot/unsorted > /home/robot/config/AllNodeList");
	std::vector<std::string> hosts;
	std::vector<std::string> nodes;
	std::vector<std::string> resource_topics;
	std::vector<std::string> heartbeat_topics;
	std::vector<std::string> loadfactor_topics;
	std::vector<std::string> deviceuptime_topics;
	std::vector<std::string> resourceavailable_topics;
	status = process->read_nodelist("/home/robot/config/scenarios/" + active_scenario + "/", &hosts, &nodes,
									&resource_topics, &heartbeat_topics, &loadfactor_topics, &deviceuptime_topics, &resourceavailable_topics);
	if (status == false)
	{
		return false;
	}
	diag = process->init_nodelist(hosts, nodes);
	logger->log_diagnostic(diag);
	logger->log_info(__FILE__,__LINE__,"--- Subscribing to Resource Topics ---");
	for (std::size_t i = 0; i < resource_topics.size(); ++i)
	{
		logger->log_info("",0,"[" + std::to_string(i) + "] " + resource_topics.at(i));
		ros::Subscriber sub = n->subscribe<eros::resource>(resource_topics.at(i), 500, &SystemMonitorNode::resource_Callback, this);
		resource_subs.push_back(sub);
	}
	logger->log_info(__FILE__,__LINE__,"--- Subscribing to Heartbeat Topics ---");
	for (std::size_t i = 0; i < heartbeat_topics.size(); ++i)
	{
		logger->log_info("",0,"[" + std::to_string(i) + "] " + heartbeat_topics.at(i));
		ros::Subscriber sub = n->subscribe<eros::heartbeat>(heartbeat_topics.at(i), 500, &SystemMonitorNode::heartbeat_Callback, this);
		heartbeat_subs.push_back(sub);
	}
	logger->log_info(__FILE__,__LINE__,"--- Subscribing to LoadFactor Topics ---");
	for (std::size_t i = 0; i < loadfactor_topics.size(); ++i)
	{
		logger->log_info("",0,"[" + std::to_string(i) + "] " + loadfactor_topics.at(i));
		ros::Subscriber sub = n->subscribe<eros::loadfactor>(loadfactor_topics.at(i), 500, &SystemMonitorNode::loadfactor_Callback, this);
		loadfactor_subs.push_back(sub);
	}
	logger->log_info(__FILE__,__LINE__,"--- Subscribing to Device Uptime Topics ---");
	for (std::size_t i = 0; i < deviceuptime_topics.size(); ++i)
	{
		logger->log_info("",0,"[" + std::to_string(i) + "] " + deviceuptime_topics.at(i));
		ros::Subscriber sub = n->subscribe<eros::uptime>(deviceuptime_topics.at(i), 500, &SystemMonitorNode::deviceuptime_Callback, this);
		deviceuptime_subs.push_back(sub);
	}
	logger->log_info(__FILE__,__LINE__,"--- Subscribing to Resource Available Topics ---");
	for (std::size_t i = 0; i < resourceavailable_topics.size(); ++i)
	{
		logger->log_info("",0,"[" + std::to_string(i) + "] " + resourceavailable_topics.at(i));
		ros::Subscriber sub = n->subscribe<eros::resource>(resourceavailable_topics.at(i), 500, &SystemMonitorNode::resourceAvailable_Callback, this);
		resourceavailable_subs.push_back(sub);
	}
	logger->log_info(__FILE__,__LINE__,"--- Monitoring Tasks ---");
	std::vector<SystemMonitorNodeProcess::Task> tasks = process->get_alltasks();
	for (std::size_t i = 0; i < tasks.size(); ++i)
	{
		char tempstr[512];
		sprintf(tempstr, "[%d] %s", (int)i, tasks.at(i).node_name.c_str());
		logger->log_info("",0,std::string(tempstr));
	}
	truthpose_sub = n->subscribe<eros::pose>("/TruthPose_Simulated", 2, &SystemMonitorNode::truthpose_Callback, this);

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	status = init_screen();
	if (status == false)
	{
		return false;
	}
	//diag = rescan_topics();
	return true;
}
bool SystemMonitorNode::init_screen()
{
	mousemask(ALL_MOUSE_EVENTS, NULL);
	initscr();
	clear();
	if (has_colors() == FALSE)
	{
		endwin();
		char tempstr[512];
		sprintf(tempstr, "Terminal does not support colors. Exiting.");
		printf("%s\n", tempstr);
		logger->log_error(__FILE__,__LINE__,std::string(tempstr));
		return false;
	}
	curs_set(0);
	noecho();
	cbreak();

	start_color();
	init_color(COLOR_BLACK, 0, 0, 0);
	init_color(COLOR_GREEN, 0, 600, 0);
	init_color(COLOR_RED, 1000, 0, 150);
	init_pair(NO_COLOR, COLOR_WHITE, COLOR_BLACK);
	init_pair(RED_COLOR, COLOR_WHITE, COLOR_RED);
	init_pair(YELLOW_COLOR, COLOR_WHITE, COLOR_YELLOW);
	init_pair(GREEN_COLOR, COLOR_WHITE, COLOR_GREEN);
	init_pair(BLUE_COLOR, COLOR_WHITE, COLOR_BLUE);

	getmaxyx(stdscr, mainwindow_height, mainwindow_width);
	bool status = process->set_mainwindow(mainwindow_width, mainwindow_height);
	if (status == false)
	{
		char tempstr[512];
		sprintf(tempstr, "Window: Width: %d Height: %d is too small.  Exiting.", mainwindow_width, mainwindow_height);
		printf("%s\n", tempstr);
		logger->log_error(__FILE__,__LINE__,std::string(tempstr));
		return false;
	}
	window_header = create_newwin(mainwindow_height / 6, mainwindow_width, 0, 0);
	window_tasklist = create_newwin(TASKPAGE_COUNT + 4, mainwindow_width, mainwindow_height / 6, 0);
	window_footer_left = create_newwin((2 * (mainwindow_height / 6)) - 5, mainwindow_width / 6.0, mainwindow_height / 6 + TASKPAGE_COUNT + 4, 0);
	window_footer_center = create_newwin((mainwindow_height / 6) + 3, mainwindow_width / 3.0, mainwindow_height / 6 + TASKPAGE_COUNT + 4, mainwindow_width / 6.0);
	double remaining_width = (1 / 6.0) + (1 / 3.0);
	window_footer_right = create_newwin((2 * (mainwindow_height / 6)) - 5, (1.0 - remaining_width) * mainwindow_width, mainwindow_height / 6 + TASKPAGE_COUNT + 4, remaining_width * mainwindow_width);
	wbkgd(window_header, COLOR_PAIR(NO_COLOR));
	keypad(window_header, TRUE);
	keypad(window_tasklist, TRUE);
	keypad(window_footer_left, TRUE);
	wtimeout(window_header, 10);
	wtimeout(window_tasklist, 10);
	char tempstr[512];
	sprintf(tempstr, "Using Window Width: %d Height: %d", mainwindow_width, mainwindow_height);
	logger->log_info(__FILE__,__LINE__,std::string(tempstr));
	wattron(window_header, COLOR_PAIR(BLUE_COLOR));
	char buffer[80];
	sprintf(buffer, "Initializing...");
	std::string str(buffer);
	mvwprintw(window_header, ACTIVESCENARIO_Y, (int)(mainwindow_width - str.size()) / 2, buffer);
	wattroff(window_header, COLOR_PAIR(BLUE_COLOR));

	//Static Content -- Task Window
	{
		mvwprintw(window_tasklist, TASKSTART_COORD_Y, TASKSTART_COORD_X, process->get_taskheader().c_str());
	}
	{ //Dashed line
		char buffer[mainwindow_width - 2];
		strcpy(buffer, "");
		for (uint16_t i = 0; i < mainwindow_width - 2; ++i)
		{
			sprintf(buffer, "%s-", buffer);
		}
		std::string str(buffer);
		mvwprintw(window_tasklist, TASKSTART_COORD_Y + 1, TASKSTART_COORD_X, buffer);
	}
	//Static Content -- Device Window
	{
		std::vector<std::string> list = process->get_modulelistheader();
		for (std::size_t i = 0; i < list.size(); ++i)
		{
			mvwprintw(window_footer_right, i + 1, 1, list.at(i).c_str());
		}
	}
	{ //Dashed line
		char buffer[mainwindow_width - 2];
		strcpy(buffer, "");
		for (uint16_t i = 0; i < (mainwindow_width / 3) - 2; ++i)
		{
			sprintf(buffer, "%s-", buffer);
		}
		std::string str(buffer);
		mvwprintw(window_footer_right, 3, 1, buffer);
	}
	wrefresh(window_footer_right);

	wrefresh(window_header);
	wrefresh(window_tasklist);
	wrefresh(window_footer_left);
	wrefresh(window_footer_right);
	return true;
}
bool SystemMonitorNode::update_windowheader()
{
	wmove(window_header, 1, 1);
	wclrtoeol(window_header);
	{ //Unix Time
		mvwprintw(window_header, ROSTIME_COORD_Y, ROSTIME_COORD_X, "Unix:%4.2fs", current_time.toSec());
	}
	{ //Scenario
		wattron(window_header, COLOR_PAIR(BLUE_COLOR));
		char buffer[80];
		sprintf(buffer, "Scenario:%s", active_scenario.c_str());
		std::string str(buffer);
		mvwprintw(window_header, ACTIVESCENARIO_Y, (int)(mainwindow_width - str.size()) / 2, buffer);
		wattroff(window_header, COLOR_PAIR(BLUE_COLOR));
	}
	{ //Date Time
		char buffer[80];
		strftime(buffer, sizeof(buffer), "Time:%b/%d/%Y %H:%M:%S", timeinfo);
		std::string str(buffer);
		mvwprintw(window_header, DATETIME_COORD_Y, mainwindow_width - str.size() - DATETIME_COORD_X, buffer);
	}
	{ //UpTime
		double uptime = process->get_uptime();
		char buffer[32];
		if (uptime < 0)
		{
			sprintf(buffer, "Uptime: --");
		}
		else
		{
			sprintf(buffer, "Uptime:%4.2fs", uptime);
		}
		std::string str(buffer);
		mvwprintw(window_header, RUNTIME_COORD_Y, (int)(mainwindow_width - str.size()) / 2, buffer);
	}
	{//Power Info
		std::string powerinfo = process->get_powerinfo_string();
		mvwprintw(window_header,RUNTIME_COORD_Y,mainwindow_width-powerinfo.size()-1,powerinfo.c_str());
	}
	{ //Poses
		uint8_t truthpose_state = process->get_truthposestate();
		uint8_t color = 0;
		switch (truthpose_state)
		{
		case SIGNALSTATE_UNDEFINED:
			color = NO_COLOR;
			break;
		case SIGNALSTATE_INVALID:
			color = NO_COLOR;
			break;
		case SIGNALSTATE_INITIALIZING:
			color = YELLOW_COLOR;
			break;
		case SIGNALSTATE_UPDATED:
			color = BLUE_COLOR;
			break;
		case SIGNALSTATE_HOLD:
			color = GREEN_COLOR;
			break;
		case SIGNALSTATE_EXTRAPOLATED:
			color = GREEN_COLOR;
			break;
		case SIGNALSTATE_CALIBRATING:
			color = YELLOW_COLOR;
			break;
		default:
			color = NO_COLOR;
			break;
		}
		wattron(window_header, COLOR_PAIR(color));
		mvwprintw(window_header, ROSTIME_COORD_Y + 2, ROSTIME_COORD_X, "%s", process->get_truthposestring().c_str());
		wclrtoeol(window_header);
		wattroff(window_header, COLOR_PAIR(color));
	}
	box(window_header, 0, 0);
	wrefresh(window_header);
	return true;
}
bool SystemMonitorNode::update_windowtasklist()
{

	{ //Task List
		std::vector<std::string> taskbuffer = process->get_taskbuffer();
		std::vector<SystemMonitorNodeProcess::Task> tasklist = process->get_alltasks();
		if (taskbuffer.size() != tasklist.size())
		{
			return false;
		}
		int index = 0;
		for (std::size_t i = start_node_index; i < taskbuffer.size(); ++i)
		{
			if (index >= TASKPAGE_COUNT)
			{
				break;
			}
			uint8_t color = 0;
			switch (tasklist.at(i).state)
			{
			case TASKSTATE_UNDEFINED:
				color = RED_COLOR;
				break;
			case TASKSTATE_START:
				color = YELLOW_COLOR;
				break;
			case TASKSTATE_INITIALIZING:
				color = YELLOW_COLOR;
				break;
			case TASKSTATE_INITIALIZED:
				color = YELLOW_COLOR;
				break;
			case TASKSTATE_NODATA:
				color = RED_COLOR;
				break;
			case TASKSTATE_LOADING:
				color = YELLOW_COLOR;
				break;
			case TASKSTATE_RUNNING:
				color = BLUE_COLOR;
				break;
			case TASKSTATE_PAUSE:
				color = GREEN_COLOR;
				break;
			case TASKSTATE_RESET:
				color = YELLOW_COLOR;
				break;
			case TASKSTATE_FINISHED:
				color = YELLOW_COLOR;
				break;
			default:
				color = RED_COLOR;
				break;
			}
			if (i == (std::size_t)selected_task_index)
			{
				mvwprintw(window_tasklist, TASKSTART_COORD_Y + 2 + (int)index, TASKSTART_COORD_X, "***");
			}
			else
			{
				mvwprintw(window_tasklist, TASKSTART_COORD_Y + 2 + (int)index, TASKSTART_COORD_X, "   ");
			}

			wattron(window_tasklist, COLOR_PAIR(color));
			mvwprintw(window_tasklist, TASKSTART_COORD_Y + 2 + (int)index, TASKSTART_COORD_X + 3, taskbuffer.at(i).c_str());
			wclrtoeol(window_tasklist);
			wattroff(window_tasklist, COLOR_PAIR(color));
			index++;
		}
	}
	box(window_tasklist, 0, 0);
	wrefresh(window_tasklist);
	return true;
}
bool SystemMonitorNode::update_windowfooter()
{
	{//Left Footer
	 {
		 SystemMonitorNodeProcess::SystemSnap snap = process->get_systemsnapinfo();
	uint8_t color = 0;
	if (snap.state.state == "UNKNOWN")
	{
		color = RED_COLOR;
	}
	else if (snap.state.state == "NOT RUNNING")
	{
		color = NO_COLOR;
	}
	else if (snap.state.state == "RUNNING")
	{
		color = YELLOW_COLOR;
	}
	else if (snap.state.state == "READY")
	{
		color = GREEN_COLOR;
	}
	else if (snap.state.state == "COMPLETE")
	{
		color = GREEN_COLOR;
	}
	else if (snap.state.state == "INCOMPLETE")
	{
		color = YELLOW_COLOR;
	}
	else
	{
		color = RED_COLOR;
	}
	std::string tempstr = "Snap State: " + snap.state.state;
	if (snap.state.state == "RUNNING")
	{
		tempstr += ": " + std::to_string(snap.state.percent_complete) + "%%";
	}
	wattron(window_footer_left, COLOR_PAIR(color));
	mvwprintw(window_footer_left, 1, 1, tempstr.c_str());
	wclrtoeol(window_footer_left);
	wattroff(window_footer_left, COLOR_PAIR(color));
}
}
{//Center Footer
 //Generic Help Window
 {
	 int row = 1;
mvwprintw(window_footer_center, row++, 3, "HELP:");
mvwprintw(window_footer_center, row++, 3, "\tq: Quit.");
mvwprintw(window_footer_center, row++, 3, "\tg: Generate Snapshot.");
mvwprintw(window_footer_center, row++, 3, "\tc: Clear Snapshots.");

std::string tempstr1 = "\ts: Select Task Mode Toggle. (Currently: ";
if (select_task_mode == true)
{
	tempstr1 += "ON)";
}
else
{
	tempstr1 += "OFF)";
}
mvwprintw(window_footer_center, row++, 3, tempstr1.c_str());
wclrtoeol(window_footer_center);

if (select_task_mode == true)
{
	char buffer[(mainwindow_width / 3) - 2];
	strcpy(buffer, "");
	for (uint16_t i = 0; i < (mainwindow_width / 3) - 2; ++i)
	{
		sprintf(buffer, "%s-", buffer);
	}
	std::string str(buffer);
	mvwprintw(window_footer_center, row++, 1, buffer);
	std::string tempstr1 = "\tl: Change Log Level for the selected task.\n";
	std::string tempstr2 = "\tr: Reset selected task.\n";
	std::string tempstr3 = "\tp: Pause selected task.\n";
	std::string tempstr4 = "\tm: Resume selected task.\n";
	//change_loglevel_mode
	if(change_loglevel_mode == true)
	{
		if (selected_loglevel_value == -1)
		{
			tempstr1 = "\tl: Change Log Level for the selected task to: ";
		}
		else
		{
			tempstr1 = "\tl: Change Log Level for the selected task to: " + std::to_string(selected_loglevel_value);
		}
	}
	mvwprintw(window_footer_center, row++, 3, tempstr1.c_str());
	mvwprintw(window_footer_center, row++, 3, tempstr2.c_str());
	mvwprintw(window_footer_center, row++, 3, tempstr3.c_str());
	mvwprintw(window_footer_center, row++, 3, tempstr4.c_str());
	wclrtoeol(window_footer_center);
}
else
{
	int height, width = 0;
	getmaxyx(window_footer_center, height, width);
	for (int i = row; i < (height - 1); ++i)
	{
		wmove(window_footer_center, i, 1);
		wclrtoeol(window_footer_center);
	}
}
}
}
{ //Right Footer

	{
		std::vector<SystemMonitorNodeProcess::Module> modules = process->get_allmodules();
		std::vector<std::string> list = process->get_modulebuffer();
		if (list.size() != modules.size())
		{
			logger->log_error(__FILE__,__LINE__,"Module Count Mismatch.");
			return false;
		}
		for (std::size_t i = 0; i < list.size(); ++i)
		{
			uint8_t color = 0;
			switch (modules.at(i).state)
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
			case TASKSTATE_FINISHED:
				color = YELLOW_COLOR;
			default:
				color = RED_COLOR;
				break;
			}
			wattron(window_footer_right, COLOR_PAIR(color));
			mvwprintw(window_footer_right, 4 + i, 1, list.at(i).c_str());
			wclrtoeol(window_footer_right);
			wattroff(window_footer_right, COLOR_PAIR(color));
		}
	}
}
box(window_footer_left, 0, 0);
wrefresh(window_footer_left);
box(window_footer_center, 0, 0);
wrefresh(window_footer_center);
box(window_footer_right, 0, 0);
wrefresh(window_footer_right);

return true;
}
bool SystemMonitorNode::run_001hz()
{
	eros::diagnostic diag = rescan_topics();
	if (diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
	return true;
}
bool SystemMonitorNode::run_01hz()
{
	return true;
}
bool SystemMonitorNode::run_01hz_noisy()
{

	return true;
}
bool SystemMonitorNode::run_1hz()
{
	//eros::diagnostic diag = process->ping_nodes();
	//if(diag.Level >= NOTICE)
	//{
	//	logger->log_diagnostic(diag);
	//}
	if (update_windowheader() == false)
	{
		return false;
	}
	if (update_windowtasklist() == false)
	{
		return false;
	}
	if (update_windowfooter() == false)
	{
		return false;
	}
	return true;
}
bool SystemMonitorNode::run_10hz()
{
	eros::diagnostic diagnostic = process->update(.1, ros::Time::now().toSec());
	if (diagnostic.Level > NOTICE)
	{
		logger->log_diagnostic(diagnostic);
	}
	std::vector<SystemMonitorNodeProcess::Task> tasklist = process->get_alltasks();
	int v = wgetch(window_tasklist);
	bool generate_snapshot = false;
	bool clear_snapshots = false;
	bool enter_pressed = false;
	//logger->log_debug("",0,std::to_string(v));
	switch (v)
	{
	case ENTER_KEY:
		enter_pressed = true;
		break;
	case KEY_UP:
		if (select_task_mode == false)
		{
			if (start_node_index > 0)
			{
				--start_node_index;
			}
		}
		break;
	case KEY_DOWN:
		if (select_task_mode == false)
		{
			if (start_node_index < (tasklist.size() - TASKPAGE_COUNT))
			{
				++start_node_index;
			}
		}

		break;
	case KEY_RESIZE:
		logger->log_debug(__FILE__,__LINE__,"RESIZED");
		break;
	case KEY_q:
		kill_node = true;
		break;
	case KEY_Q:
		kill_node = true;
		break;
	case KEY_g:
		generate_snapshot = true;
		break;
	case KEY_G:
		generate_snapshot = true;
		break;
	case KEY_c:
		clear_snapshots = true;
		break;
	case KEY_C:
		clear_snapshots = true;
		break;
	case KEY_s:
		select_task_mode = !select_task_mode;
		break;
	case KEY_S:
		select_task_mode = !select_task_mode;
		break;
	case KEY_l:
		if (select_task_mode == true)
		{
			change_loglevel_mode = !change_loglevel_mode;
		}
		break;
	case KEY_L:
		if (select_task_mode == true)
		{
			change_loglevel_mode = !change_loglevel_mode;
		}
		break;
	case KEY_r:
		if (select_task_mode == true)
		{
			taskcommand_reset_mode = true;
		}
		break;
	case KEY_R:
		if (select_task_mode == true)
		{
			taskcommand_reset_mode = true;
		}
		break;
	case KEY_p:
		if (select_task_mode == true)
		{
			taskcommand_pause_mode = true;
		}
		break;
	case KEY_P:
		if (select_task_mode == true)
		{
			taskcommand_pause_mode = true;
		}
		break;
	case KEY_m:
		if (select_task_mode == true)
		{
			taskcommand_resume_mode = true;
		}
		break;
	case KEY_M:
		if (select_task_mode == true)
		{
			taskcommand_resume_mode = true;
		}
		break;
	default:
		if (v != -1)
		{
			if ((select_task_mode == true) and (change_loglevel_mode == true))
			{
				int d = v - 48;
				if ((d >= 0) and (d <= 5))
				{
					selected_loglevel_value = d;
				}
			}
		}
		break;
	}
	if (generate_snapshot == true)
	{
		eros::command cmd;
		cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
		cmd.Option1 = ROVERCOMMAND_SNAPSHOT_NEWSNAPSHOT;
		cmd.Option2 = ROVERCOMMAND_SNAPSHOTMODE_MANUAL;
		cmd.Option3 = 0;
		cmd.CommandText = "{}";
		cmd.Description = "Generate Snapshot Commanded by SystemMonitorNode on Host.";
		command_pub.publish(cmd);
	}
	if (clear_snapshots == true)
	{
		eros::command cmd;
		cmd.Command = ROVERCOMMAND_GENERATESNAPSHOT;
		cmd.Option1 = ROVERCOMMAND_SNAPSHOT_CLEARALL;
		cmd.Option2 = 0;
		cmd.Option3 = 0;
		cmd.CommandText = "{}";
		cmd.Description = "Clear Snapshots Commanded by SystemMonitorNode.";
		command_pub.publish(cmd);
	}
	if (select_task_mode == false)
	{
		selected_task_index = -1;
	}
	if ((select_task_mode == true) and (selected_task_index == -1))
	{
		selected_task_index = 0;
	}
	if (select_task_mode == true)
	{
		if (v == KEY_UP)
		{
			if (selected_task_index > 0)
			{
				--selected_task_index;
			}
			else
			{
				selected_task_index = tasklist.size() - 1;
			}
		}
		if (v == KEY_DOWN)
		{
			if ((std::size_t)selected_task_index < ((tasklist.size() - 1)))
			{
				++selected_task_index;
			}
			else
			{
				selected_task_index = 0;
			}
		}
	}
	if (select_task_mode == true)
	{
		if (enter_pressed == true)
		{
			if (change_loglevel_mode == true)
			{
				std::vector<SystemMonitorNodeProcess::Task> tasklist = process->get_alltasks();
				eros::command cmd;
				cmd.Command = ROVERCOMMAND_SETLOGLEVEL;
				cmd.Option1 = SUBSYSTEM_UNKNOWN;
				cmd.Option2 = selected_loglevel_value;
				cmd.Option3 = 0;
				cmd.CommandText = tasklist.at(selected_task_index).node_name;
				cmd.Description = "Log level change requested by SystemMonitorNode.";
				command_pub.publish(cmd);
				selected_loglevel_value = -1;
				change_loglevel_mode = false;
			}
		}
		if (taskcommand_pause_mode == true)
		{
			std::vector<SystemMonitorNodeProcess::Task> tasklist = process->get_alltasks();
			eros::command cmd;
			cmd.Command = ROVERCOMMAND_TASKCONTROL;
			cmd.Option1 = SUBSYSTEM_UNKNOWN;
			cmd.Option2 = TASKSTATE_PAUSE;
			cmd.Option3 = 0;
			cmd.CommandText = tasklist.at(selected_task_index).node_name;
			cmd.Description = "Pause Task requested SystemMonitorNode.";
			command_pub.publish(cmd);
			taskcommand_pause_mode = false;
		}
		else if (taskcommand_resume_mode == true)
		{
			std::vector<SystemMonitorNodeProcess::Task> tasklist = process->get_alltasks();
			eros::command cmd;
			cmd.Command = ROVERCOMMAND_TASKCONTROL;
			cmd.Option1 = SUBSYSTEM_UNKNOWN;
			cmd.Option2 = TASKSTATE_RUNNING;
			cmd.Option3 = 0;
			cmd.CommandText = tasklist.at(selected_task_index).node_name;
			cmd.Description = "Resume Task requested SystemMonitorNode.";
			command_pub.publish(cmd);
			taskcommand_resume_mode = false;
		}
		else if (taskcommand_reset_mode == true)
		{
			std::vector<SystemMonitorNodeProcess::Task> tasklist = process->get_alltasks();
			eros::command cmd;
			cmd.Command = ROVERCOMMAND_TASKCONTROL;
			cmd.Option1 = SUBSYSTEM_UNKNOWN;
			cmd.Option2 = TASKSTATE_RESET;
			cmd.Option3 = 0;
			cmd.CommandText = tasklist.at(selected_task_index).node_name;
			cmd.Description = "Reset Task requested SystemMonitorNode.";
			command_pub.publish(cmd);
			taskcommand_reset_mode = false;
		}
	}
	current_time = ros::Time::now();
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	return true;
}
eros::diagnostic SystemMonitorNode::rescan_topics()
{
	eros::diagnostic diag = process->get_diagnostic();
	int found_new_topics = 0;

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo &info = *it;
		if (info.datatype == "eros/resource")
		{
			int v = process->push_topiclist(info.datatype, info.name);
			if (v == 1)
			{
				found_new_topics++;
				char tempstr[255];
				sprintf(tempstr, "Subscribing to resource topic: %s", info.name.c_str());
				logger->log_info(__FILE__,__LINE__,tempstr);
				ros::Subscriber sub = n->subscribe<eros::resource>(info.name, 2, &SystemMonitorNode::resource_Callback, this);
				resource_subs.push_back(sub);
			}
		}
		if (info.datatype == "eros/loadfactor")
		{
			int v = process->push_topiclist(info.datatype, info.name);
			if (v == 1)
			{
				found_new_topics++;
				char tempstr[255];
				sprintf(tempstr, "Subscribing to loadfactor topic: %s", info.name.c_str());
				logger->log_info(__FILE__,__LINE__,tempstr);
				ros::Subscriber sub = n->subscribe<eros::loadfactor>(info.name, 2, &SystemMonitorNode::loadfactor_Callback, this);
				loadfactor_subs.push_back(sub);
			}
		}
		if (info.datatype == "eros/uptime")
		{
			int v = process->push_topiclist(info.datatype, info.name);
			if (v == 1)
			{
				found_new_topics++;
				char tempstr[255];
				sprintf(tempstr, "Subscribing to uptime topic: %s", info.name.c_str());
				logger->log_info(__FILE__,__LINE__,tempstr);
				ros::Subscriber sub = n->subscribe<eros::uptime>(info.name, 2, &SystemMonitorNode::deviceuptime_Callback, this);
				deviceuptime_subs.push_back(sub);
			}
		}
		if (info.datatype == "eros/heartbeat")
		{
			int v = process->push_topiclist(info.datatype, info.name);
			if (v == 1)
			{
				found_new_topics++;
				char tempstr[255];
				sprintf(tempstr, "Subscribing to heartbeat topic: %s", info.name.c_str());
				logger->log_info(__FILE__,__LINE__,tempstr);
				ros::Subscriber sub = n->subscribe<eros::heartbeat>(info.name, 2, &SystemMonitorNode::heartbeat_Callback, this);
				heartbeat_subs.push_back(sub);
			}
		}
	}
	char tempstr[255];
	if (found_new_topics > 0)
	{
		sprintf(tempstr, "Rescanned and found %d new topics.", found_new_topics);
	}
	else
	{
		sprintf(tempstr, "Rescanned and found no new topics.");
	}
	logger->log_info(__FILE__,__LINE__,tempstr);
	return diag;
}
void SystemMonitorNode::truthpose_Callback(const eros::pose::ConstPtr &msg)
{
	process->new_truthpose(msg);
}
void SystemMonitorNode::uptime_Callback(const std_msgs::Float32::ConstPtr &msg)
{
	process->set_uptime(msg->data);
}
void SystemMonitorNode::deviceuptime_Callback(const eros::uptime::ConstPtr &msg)
{
	process->new_deviceuptime(msg);
}
void SystemMonitorNode::heartbeat_Callback(const eros::heartbeat::ConstPtr &msg)
{
	eros::diagnostic diag = process->new_heartbeatmessage(msg);
	if (diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
}
void SystemMonitorNode::resource_Callback(const eros::resource::ConstPtr &msg)
{
	eros::diagnostic diag = process->new_resourcemessage(msg);
	if (diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
}
void SystemMonitorNode::loadfactor_Callback(const eros::loadfactor::ConstPtr &msg)
{
	eros::diagnostic diag = process->new_loadfactormessage(msg);
	if (diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
}
void SystemMonitorNode::resourceAvailable_Callback(const eros::resource::ConstPtr &msg)
{
	eros::diagnostic diag = process->new_resourceavailablemessage(msg);
	if (diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
}
void SystemMonitorNode::snapshotstate_Callback(const eros::systemsnapshot_state::ConstPtr &msg)
{
	eros::diagnostic diag = process->new_systemsnapshotstatemessage(msg);
	if(diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
}
void SystemMonitorNode::battery_Callback(const eros::battery::ConstPtr& msg)
{
	eros::diagnostic diag = process->new_batterymessage(msg);
	if (diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
}
void SystemMonitorNode::thread_loop()
{
	while (kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
bool SystemMonitorNode::update()
{
	ros::Rate r(ros_rate);
	r.sleep();
	ros::spinOnce();
	double mtime;
	mtime = measure_time_diff(ros::Time::now(), last_001hz_timer);
	if (mtime >= 100.0)
	{
		run_001hz();
		last_001hz_timer = ros::Time::now();
	}
	mtime = measure_time_diff(ros::Time::now(), last_01hz_noisy_timer);
	if (mtime >= 10.0 + rand_delay_sec)
	{
		rand_delay_sec = (double)(rand() % 2000 - 1000) / 1000.0;
		run_01hz_noisy();
		last_01hz_noisy_timer = ros::Time::now();
	}
	mtime = measure_time_diff(ros::Time::now(), last_01hz_timer);
	if (mtime >= 10.0)
	{
		run_01hz();
		last_01hz_timer = ros::Time::now();
	}
	mtime = measure_time_diff(ros::Time::now(), last_1hz_timer);
	if (mtime >= 1.0)
	{
		run_1hz();
		last_1hz_timer = ros::Time::now();
	}
	mtime = measure_time_diff(ros::Time::now(), last_10hz_timer);
	if (mtime >= 0.1)
	{
		run_10hz();
		last_10hz_timer = ros::Time::now();
	}
	return ros::ok();
}

void SystemMonitorNode::cleanup()
{
	delwin(window_header);
	delwin(window_tasklist);
	delwin(window_footer_left);
	delwin(window_footer_center);
	delwin(window_footer_right);
	endwin();
	logger->log_notice(__FILE__,__LINE__,"Node Safely Exited.");
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
	SystemMonitorNode *node = new SystemMonitorNode();
	bool status = node->start(argc, argv);
	if (status == false)
	{
		endwin();
		exit(0);
	}
	std::thread thread(&SystemMonitorNode::thread_loop, node);
	while ((status == true) and (kill_node == false))
	{
		status = node->update();
	}

	node->cleanup();
	thread.detach();
	return 0;
}
