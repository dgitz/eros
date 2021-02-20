/*! \file ResourceMonitor.h
 */
#ifndef RESOURCEMONITOR_h
#define RESOURCEMONITOR_H
#include <eros/Diagnostic.h>
#include <eros/eROS_Definitions.h>
#include <stdio.h>
#include <unistd.h>
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
        double ram_mb;
        double disk_perc;
    };

    enum class Mode { UNKNOWN = 0, PROCESS = 1, DEVICE = 2, END_OF_LIST = 3 };
    ResourceMonitor();
    ~ResourceMonitor();
    ResourceMonitor(Architecture::Type _architecture,
                    Mode _mode,
                    Diagnostic::DiagnosticDefinition _diag);

    std::string pretty(ResourceInfo info);
    Diagnostic::DiagnosticDefinition init();
    ResourceInfo get_resourceinfo() {
        return resourceInfo;
    }
    Diagnostic::DiagnosticDefinition update(double t_dt);

   private:
    Diagnostic::DiagnosticDefinition read_process_resource_usage();
    Diagnostic::DiagnosticDefinition read_device_resource_availability();
    Architecture::Type architecture;
    Mode mode;
    Diagnostic::DiagnosticDefinition diagnostic;
    ResourceInfo resourceInfo;
    bool initialized;
    double run_time;
};
#endif  // RESOURCEMONITOR_H