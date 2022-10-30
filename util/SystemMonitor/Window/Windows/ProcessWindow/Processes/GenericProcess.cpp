#include <eros/SystemMonitor/Window/Windows/ProcessWindow/Processes/GenericProcess.h>
namespace eros {
bool GenericProcess::setNodeAlive(double currentTime_s) {
    return BaseProcess::setNodeAlive(currentTime_s);
}
std::string GenericProcess::pretty(const std::string& pre, const std::string& post) {
    std::string str = BaseProcess::pretty(pre, post) + "\n";
    return str;
}
}  // namespace eros