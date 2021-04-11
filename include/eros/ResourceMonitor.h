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
namespace eros {
/*! \class ResourceMonitor
    \brief ResourceMonitor class
    ResourceMonitor class used to collect resource information on a process or device.
*/
class ResourceMonitor
{
   public:
    /*! \struct ResourceInfo
        \brief ResourceInfo Information:
        Holds information about a Node's Resource Usage.
    */
    struct ResourceInfo {
        /*@{*/
        std::string process_name; /**< The name of the process. */
        uint16_t pid;             /**< The PID of the Process.  0 is Invalid. */
        double cpu_perc; /**< CPU Usage of a Process in Percentage.  100% would indicate the process
                            is fully utilizing 1 CPU. */
        double ram_perc; /**< RAM Usage of a Process in Percentage.  100% would indicate the process
                            is using 100% of the avaialble RAM. */
        double disk_perc; /**< Disk Usage of a Process in Percentage.  100% would indicate the
                             process is using 100% of the available disk space. */
        /*@}*/
    };

    enum class Mode {
        UNKNOWN = 0, /*!< Uninitialized value. */
        PROCESS = 1, /*!< This Mode is used when checking Process Resource Usage Information. */
        DEVICE =
            2, /*!< This Mode is used when checking Device Resource Availability Information. */
        END_OF_LIST = 3 /*!< Last item of list. Used for Range Checks. */
    };
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

    std::vector<double> get_load_factor() {
        return load_factor;
    }
    void reset();

   private:
    Diagnostic::DiagnosticDefinition read_process_resource_usage();
    Diagnostic::DiagnosticDefinition read_device_resource_availability();
    Diagnostic::DiagnosticDefinition read_device_loadfactor();
    Architecture::Type read_device_architecture();

    Mode mode;
    Architecture::Type architecture;
    Diagnostic::DiagnosticDefinition diagnostic;
    Logger* logger;
    ResourceInfo resourceInfo;
    bool initialized;
    double run_time;
    uint16_t processor_count;
    std::vector<double> load_factor;
};
}  // namespace eros
#endif  // RESOURCEMONITOR_H