#ifndef SYSTEMMONITOR_GENERICPROCESS_H
#define SYSTEMMONITOR_GENERICPROCESS_H

#include <eros/SystemMonitor/Window/Windows/ProcessWindow/BaseProcess.h>
namespace eros {
class GenericProcess : public BaseProcess
{
   public:
    GenericProcess(eros::Logger* logger,
                   std::string hostName,
                   std::string nodeName,
                   double commTimeout_s)
        : BaseProcess(logger, ProcessType::GENERIC, hostName, nodeName, commTimeout_s) {
    }
    std::string pretty(const std::string& pre = "", const std::string& post = "");
    bool setNodeAlive(double currentTime_s);
    bool update(double currentTime_s) override;
};
}  // namespace eros
#endif