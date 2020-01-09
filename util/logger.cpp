#include "../include/logger.h"

Logger::Logger()
{
}

Logger::Logger(std::string level,std::string modpath,std::string name)
{
	console_print = true;
	verbosity = map_logverbosity_toint(level);
	line_counter = 0;
	name.erase(name.begin());
	replace(name.begin(),name.end(),'/','_');
	char buffer[100];
	sprintf(buffer,"%s.out",name.c_str());

	sprintf(file_path,"/var/log/output/%s/%s",modpath.c_str(),buffer);
	ofstream log_file;
	log_file.open(file_path); //Overwrite file.
	log_file.close();
}
Logger::Logger(std::string level,std::string name)
{
	console_print = true;
	verbosity = map_logverbosity_toint(level);
	line_counter = 0;
	name.erase(name.begin());
	replace(name.begin(),name.end(),'/','_');
	node_name = name;
	char buffer[100];
	sprintf(buffer,"%s.out",name.c_str());

	sprintf(file_path,"/var/log/output/%s",buffer);
	ofstream log_file;
	log_file.open(file_path); //Overwrite file.
	log_file.close();
}
Logger::~Logger()
{
}
void Logger::set_logverbosity(int v)
{
	if(v == verbosity)
	{
		return;
	}
	if((v < 0) or (v > FATAL))
	{
		return;
	}
	std::string tempstr = "Changing Log Level from " + map_logverbosity_tostring(verbosity) + " to " + map_logverbosity_tostring(v) + ".";
	verbosity = v;
	print_log("",0,verbosity,tempstr);
}
std::string Logger::map_logverbosity_tostring(int v)
{
	switch(v)
	{
		case DEBUG:
			return "DEBUG";
			break;
		case INFO: 
			return "INFO";
			break;
		case NOTICE:
			return "NOTICE";
			break;
		case WARN:
			return "WARN";
			break;
		case ERROR:
			return "ERROR";
			break;
		case FATAL:
			return "FATAL";
			break;
		default:
			return "UNKNOWN";
			break;
	}
	return "UNKNOWN";
}
int Logger::map_logverbosity_toint(std::string level)
{
	if     (level=="DEBUG"){    return DEBUG;}
	else if(level=="INFO"){     return INFO; }
	else if(level=="NOTICE"){   return NOTICE; }
	else if(level=="WARN"){     return WARN; }
	else if(level=="ERROR"){    return ERROR; }
	else if(level=="FATAL"){    return FATAL; }
	else{                       return DEBUG; }
}
void Logger::log_debug(std::string filename,uint64_t linenumber,std::string tempstr)
{
	print_log(filename,linenumber,DEBUG,tempstr);
}
void Logger::log_info(std::string filename,uint64_t linenumber,std::string tempstr)
{
	print_log(filename,linenumber,INFO,tempstr);
}
void Logger::log_notice(std::string filename,uint64_t linenumber,std::string tempstr)
{
	print_log(filename,linenumber,NOTICE,tempstr);
}
void Logger::log_warn(std::string filename,uint64_t linenumber,std::string tempstr)
{
	print_log(filename,linenumber,WARN,tempstr);
}
void Logger::log_error(std::string filename,uint64_t linenumber,std::string tempstr)
{
	print_log(filename,linenumber,ERROR,tempstr);
}
void Logger::log_fatal(std::string filename,uint64_t linenumber,std::string tempstr)
{
	print_log(filename,linenumber,FATAL,tempstr);
}
void Logger::log_diagnostic(eros::diagnostic diagnostic)
{

	char tempstr[2048];

	sprintf(tempstr,"Device: %s System: %s Subsystem: %s Component: %s Type: %s Message: %s Description: %s",
			diagnostic.DeviceName.c_str(),
			diagclass.get_DiagSystemString(diagnostic.System).c_str(),
			diagclass.get_DiagSubSystemString(diagnostic.SubSystem).c_str(),
			diagclass.get_DiagComponentString(diagnostic.Component).c_str(),
			diagclass.get_DiagTypeString(diagnostic.Diagnostic_Type).c_str(),
			diagclass.get_DiagMessageString(diagnostic.Diagnostic_Message).c_str(),
			diagnostic.Description.c_str());

	switch(diagnostic.Level)
	{
	case DEBUG:
		log_debug("",0,std::string(tempstr));
		break;
	case INFO:
		log_info("",0,std::string(tempstr));
		break;
	case NOTICE:
		log_notice("",0,std::string(tempstr));
		break;
	case WARN:
		log_warn("",0,std::string(tempstr));
		break;
	case ERROR:
		log_error("",0,std::string(tempstr));
		break;
	case FATAL:
		log_fatal("",0,std::string(tempstr));
		break;
	}
	/*
	switch(diagnostic.Level)
		{
			case DEBUG:
				log_debug(diagnostic.Description);
				break;
			case INFO:
				log_info(diagnostic.Description);
				break;
			case NOTICE:
				log_notice(diagnostic.Description);
				break;
			case WARN:
				log_warn(diagnostic.Description);
				break;
			case ERROR:
				log_error(diagnostic.Description);
				break;
			case FATAL:
				log_fatal(diagnostic.Description);
				break;
		}
	 */
}
void Logger::print_log(std::string filename,uint64_t linenumber,int level,std::string tempstr)
{
	time_t rawtime;
	struct tm * timeinfo;
	char datebuffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(datebuffer,80,"%d/%m/%Y %I:%M:%S",timeinfo);
	std::string str(datebuffer);
	log_file.open(file_path,ios::out | ios::app | ios::binary|std::ios::ate);
	std::string swcode_info = "";
	if(linenumber > 0)
	{
		swcode_info = filename + "(" + std::to_string(linenumber) + ")";
	}
	if(log_file.is_open() && level >= verbosity)
	{
		line_counter++;
		switch (level)
		{
		case DEBUG:
			log_file << "[" << datebuffer << "]: DEBUG: " << swcode_info << " " << tempstr << endl;
			if(console_print) { printf("[%s %s]: DEBUG: %s\n",datebuffer,node_name.c_str(),tempstr.c_str()); }
			break;
		case INFO:
			log_file << "[" << datebuffer << "]: INFO: " << swcode_info << " " << tempstr << endl;
			if(console_print) {printf("[%s %s]: INFO: %s\n",datebuffer,node_name.c_str(),tempstr.c_str()); }
			break;
		case NOTICE:
			log_file << "[" << datebuffer << "]: NOTICE: " << swcode_info << " " << tempstr << endl;
			if(console_print) {printf("%s[%s %s]: NOTICE: %s%s\n",GREEN_FOREGROUND,datebuffer,node_name.c_str(),tempstr.c_str(),END_COLOR); }
			break;
		case WARN:
			log_file << "[" << datebuffer << "]: WARN: " << swcode_info << " " << tempstr << endl;
			if(console_print) {printf("%s[%s %s]: WARN: %s%s\n",YELLOW_FOREGROUND,datebuffer,node_name.c_str(),tempstr.c_str(),END_COLOR); }
			break;
		case ERROR:
			log_file << "[" << datebuffer << "]: ERROR: " << swcode_info << " " << tempstr << endl;
			if(console_print) {printf("%s[%s %s]: ERROR: %s%s\n",RED_FOREGROUND,datebuffer,node_name.c_str(),tempstr.c_str(),END_COLOR); }
			break;
		case FATAL:
			log_file << "[" << datebuffer << "]: FATAL: " << swcode_info << " " << tempstr << endl;
			if(console_print) {printf("%s[%s %s]: FATAL: %s%s\n",RED_FOREGROUND,datebuffer,node_name.c_str(),tempstr.c_str(),END_COLOR); }
			break;
		default:
			break;
		}
	}
	log_file.close();
	if(line_counter > 5000)
	{
		log_file.open(file_path); //Overwrite file.
		log_file.close();
		line_counter = 0;
	}


}
