#include <eros/SystemMonitor/Window/Windows/ProcessWindow/BaseProcess.h>
namespace eros {
std::string BaseProcess::pretty(const std::string& pre, const std::string& post) {
    std::string str = "";
    str += pre + "Process: " + nodeName +
           " Type: " + IProcess::ProcessTypeString(getProcessType()) + "\n";
    str += pre + pre + "Level: " + Level::LevelString(getLevel()) +
           " State: " + Node::NodeStateString(getState()) + "\n";
    str += pre + pre + "Current HB: " + std::to_string(lastHeartbeat) +
           " Last Delta: " + std::to_string(lastHeartbeatDelta);
    str += post;
    return str;
}
bool BaseProcess::update(double currentTime_s) {
    lastHeartbeatDelta = currentTime_s - lastHeartbeat;
    if (lastHeartbeatDelta > (2.0 * commTimeout_s)) {
        level = Level::Type::ERROR;
    }
    else if (lastHeartbeatDelta > commTimeout_s) {
        level = Level::Type::WARN;
    }
    else {
        level = Level::Type::INFO;
    }
    return true;
}
bool BaseProcess::setNodeAlive(double currentTime_s) {
    lastHeartbeat = currentTime_s;
    lastHeartbeatDelta = 0.0;
    aliveCount++;
    return true;
}
}  // namespace eros