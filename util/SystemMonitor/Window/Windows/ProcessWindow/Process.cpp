#include <eros/SystemMonitor/Window/Windows/ProcessWindow/Process.h>
namespace eros {
bool Process::new_resourceused(eros::resource resourceUsed) {
    if (resourceInfo.PID != 0) {
        if (resourceInfo.PID != resourceUsed.PID) {
            restartCount++;
        }
    }
    lastHeartbeat = resourceUsed.stamp.toSec();
    lastHeartbeatDelta = 0.0;
    resourceInfo = resourceUsed;

    return false;
}
std::string Process::pretty(std::string pre, std::string post) {
    std::string str = pre;
    str += "Process: " + nodeName + " PID: " + std::to_string(resourceInfo.PID) +
           " State: " + Level::LevelString(getStatus()) + "\n";
    str += "\tLast Heartbeat: " + std::to_string(lastHeartbeatDelta) + "\n";
    str += post;
    return str;
}
bool Process::update(double currentTime_s) {
    lastHeartbeatDelta = currentTime_s - lastHeartbeat;
    if (lastHeartbeatDelta > (2.0 * commTimeout_s)) {
        status = Level::Type::ERROR;
    }
    else if (lastHeartbeatDelta > commTimeout_s) {
        status = Level::Type::WARN;
    }
    else {
        status = Level::Type::INFO;
    }
    return true;
}

}  // namespace eros