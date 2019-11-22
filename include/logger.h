#ifndef LOGGER_H
#define LOGGER_H

#include "ros/ros.h"
#include "eROS_Definitions.h"
#include <eros/diagnostic.h>
#include "DiagnosticClass.h"
#include "ros/time.h"
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#define GREEN_FOREGROUND "\033[1;32m"
#define YELLOW_FOREGROUND "\033[1;33m"
#define RED_FOREGROUND "\033[1;31m"
#define END_COLOR "\033[0m"

using std::string;
using namespace std;
class Logger
{
public:
    Logger();
    Logger(std::string level, std::string name);
    Logger(std::string level,std::string modpath,std::string name);
    int get_logverbosity() { return verbosity; }
    
    void set_logverbosity(int v);
    ~Logger();

    void log_debug(std::string filename,uint64_t linenumber,std::string tempstr);
    void log_info(std::string filename,uint64_t linenumber,std::string tempstr);
    void log_notice(std::string filename,uint64_t linenumber,std::string tempstr);
    void log_warn(std::string filename,uint64_t linenumber,std::string tempstr);
    void log_error(std::string filename,uint64_t linenumber,std::string tempstr);
    void log_fatal(std::string filename,uint64_t linenumber,std::string tempstr);
    void log_diagnostic(eros::diagnostic diagnostic);
    void disable_consoleprint() { console_print = false; }
private:
    int line_counter;
    int verbosity;
    ofstream log_file;
    std::string node_name;
    char file_path[120];
    std::string map_logverbosity_tostring(int v);
    int map_logverbosity_toint(std::string level);
    void print_log(std::string filename,uint64_t linenumber,int level,std::string tempstr);
    bool console_print;

    DiagnosticClass diagclass;

};
#endif
