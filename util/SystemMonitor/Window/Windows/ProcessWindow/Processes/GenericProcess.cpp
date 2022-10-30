#include <eros/SystemMonitor/Window/Windows/ProcessWindow/Processes/GenericProcess.h>
namespace eros {
bool GenericProcess::setNodeAlive(double currentTime_s) {
    return BaseProcess::setNodeAlive(currentTime_s);
}
std::string GenericProcess::pretty(const std::string& pre, const std::string& post) {
    std::string str = BaseProcess::pretty(pre, post) + "\n";
    return str;
}
bool GenericProcess::update(double currentTime_s) {
    lastHeartbeat = currentTime_s;
    lastHeartbeatDelta = 0.0;
    level = Level::Type::INFO;
    state = Node::State::RUNNING;
    return true;
}
}  // namespace eros