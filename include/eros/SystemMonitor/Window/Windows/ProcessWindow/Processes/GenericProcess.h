#ifndef SYSTEMMONITOR_GENERICPROCESS_H
#define SYSTEMMONITOR_GENERICPROCESS_H

#include <eros/SystemMonitor/Window/Windows/ProcessWindow/BaseProcess.h>
namespace eros {
class GenericProcess : public BaseProcess
{
   public:
    GenericProcess(eros::Logger* logger, std::string nodeName, double commTimeout_s)
        : BaseProcess(logger, ProcessType::GENERIC, nodeName, commTimeout_s) {
    }
    std::string pretty(const std::string& pre = "", const std::string& post = "");
    bool setNodeAlive(double currentTime_s);
};
}  // namespace eros
#endif