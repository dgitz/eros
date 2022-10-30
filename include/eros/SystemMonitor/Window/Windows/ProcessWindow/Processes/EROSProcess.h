#ifndef SYSTEMMONITOR_EROSPROCESS_H
#define SYSTEMMONITOR_EROSPROCESS_H

#include <eros/SystemMonitor/Window/Windows/ProcessWindow/BaseProcess.h>
#include <eros/heartbeat.h>
#include <eros/resource.h>
namespace eros {
class EROSProcess : public BaseProcess
{
   public:
    EROSProcess(eros::Logger* logger,
                std::string hostName,
                std::string nodeName,
                double commTimeout_s)
        : BaseProcess(logger, ProcessType::EROS, hostName, nodeName, commTimeout_s) {
    }
    bool new_resourceused(eros::resource resourceUsed);
    bool new_heartbeat(eros::heartbeat heartbeat);
    std::string pretty(const std::string& pre = "", const std::string& post = "");

   private:
    eros::resource resourceInfo;
};
}  // namespace eros
#endif