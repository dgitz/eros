/*! \file ResourceMonitor.h
 */
#ifndef RESOURCEMONITOR_h
#define RESOURCEMONITOR_H
#include <eros/Diagnostic.h>
#include <eros/Logger.h>
#include <eros/eROS_Definitions.h>
#include <stdio.h>
#include <unistd.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
/*! \class ResourceMonitor
    \brief ResourceMonitor class
    ResourceMonitor class used to collect resource information on a process or device.
*/
class ResourceMonitor
{
   public:
    struct ResourceInfo {
        std::string process_name;
        uint16_t pid;
        double cpu_perc;
        double ram_perc;
        double disk_perc;
    };

    enum class Mode { UNKNOWN = 0, PROCESS = 1, DEVICE = 2, END_OF_LIST = 3 };
    ResourceMonitor();
    ~ResourceMonitor();
    ResourceMonitor(Mode _mode, Diagnostic::DiagnosticDefinition _diag, Logger* _logger);

    std::string pretty(ResourceInfo info);
    Diagnostic::DiagnosticDefinition init();
    ResourceInfo get_resourceinfo() {
        return resourceInfo;
    }
    Architecture::Type get_architecture() {
        return architecture;
    }
    Diagnostic::DiagnosticDefinition update(double t_dt);

    std::string exec(const char* cmd, bool wait_for_result);

   private:
    Diagnostic::DiagnosticDefinition read_process_resource_usage();
    Diagnostic::DiagnosticDefinition read_device_resource_availability();
    Architecture::Type read_device_architecture();

    Mode mode;
    Architecture::Type architecture;
    Diagnostic::DiagnosticDefinition diagnostic;
    Logger* logger;
    ResourceInfo resourceInfo;
    bool initialized;
    double run_time;
    uint16_t processor_count;
};
#endif  // RESOURCEMONITOR_H