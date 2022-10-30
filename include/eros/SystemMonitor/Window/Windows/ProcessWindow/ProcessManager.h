#ifndef SYSTEMMONITOR_PROCESSMANAGER_H
#define SYSTEMMONITOR_PROCESSMANAGER_H
#include <eros/Logger.h>
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/IProcess.h>
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/Processes/EROSProcess.h>
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/Processes/GenericProcess.h>
#include <eros/heartbeat.h>
#include <eros/resource.h>

#include <map>
#include <memory>
#include <mutex>
namespace eros {
class ProcessManager
{
   public:
    ProcessManager(eros::Logger* logger, double commTimeout_s)
        : logger(logger), commTimeout_s(commTimeout_s) {
    }
    bool new_resourceUsed(eros::resource msg);
    bool new_heartbeat(eros::heartbeat msg);
    bool new_nodeAlive(std::string nodeName, double currentTime_s);
    bool update(double currentTime_s);
    std::map<std::string, std::shared_ptr<IProcess>> getProcesses() {
        return processList;
    }
    std::string pretty(const std::string& pre = "", const std::string& post = "");

   private:
    eros::Logger* logger;
    double commTimeout_s;

    std::mutex processListMutex;
    std::map<std::string, std::shared_ptr<IProcess>> processList;
};
}  // namespace eros
#endif