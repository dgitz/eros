/*! \file ResourceMonitor.h
 */
#ifndef RESOURCEMONITOR_h
#define RESOURCEMONITOR_H
#include <eros/Logger.h>
#include <eros/Utility.h>
#include <eros/eROS_Definitions.h>
#include <eros_diagnostic/Diagnostic.h>
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
        uint64_t pid;             /**< The PID of the Process.  0 is Invalid. */
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
    ~ResourceMonitor();
    ResourceMonitor(Mode _mode, eros_diagnostic::Diagnostic _diag, Logger* _logger);
    bool is_initialized() {
        return initialized;
    }

    std::string pretty(ResourceInfo info);
    eros_diagnostic::Diagnostic init();
    ResourceInfo get_resourceinfo() {
        return resourceInfo;
    }
    Architecture::Type get_architecture() {
        return architecture;
    }
    eros_diagnostic::Diagnostic update(double t_dt);

    std::vector<double> get_load_factor() {
        return load_factor;
    }
    bool reset();

   private:
    eros_diagnostic::Diagnostic read_process_resource_usage();
    eros_diagnostic::Diagnostic read_device_resource_availability();
    eros_diagnostic::Diagnostic read_device_loadfactor();
    Architecture::Type read_device_architecture();

    Mode mode;
    Architecture::Type architecture;
    eros_diagnostic::Diagnostic diagnostic;
    Logger* logger;
    ResourceInfo resourceInfo;
    bool initialized;
    double run_time;
    uint16_t processor_count;
    std::vector<double> load_factor;
};
}  // namespace eros
#endif  // RESOURCEMONITOR_H