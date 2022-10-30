#include <eros/SystemMonitor/Window/Windows/ProcessWindow/Processes/EROSProcess.h>
namespace eros {
bool EROSProcess::new_resourceused(eros::resource resourceUsed) {
    if (resourceUsed.Name == "") {
        return false;
    }
    if (resourceUsed.Name != nodeName) {
        return false;
    }
    if (resourceInfo.PID != 0) {
        if (resourceInfo.PID != resourceUsed.PID) {
            restartCount++;
        }
    }
    PID = resourceUsed.PID;
    CPUUsed = resourceUsed.CPU_Perc;
    RAMUsed = resourceUsed.RAM_Perc;

    resourceInfo = resourceUsed;
    return setNodeAlive(resourceInfo.stamp.toSec());
}
bool EROSProcess::new_heartbeat(eros::heartbeat heartbeat) {
    if (heartbeat.NodeName == "") {
        return false;
    }
    if (heartbeat.NodeName != nodeName) {
        return false;
    }
    hostName = heartbeat.HostName;
    state = (Node::State)heartbeat.NodeState;
    return setNodeAlive(heartbeat.stamp.toSec());
}
std::string EROSProcess::pretty(const std::string& pre, const std::string& post) {
    std::string str = BaseProcess::pretty(pre, post) + "\n";
    str += "\tPID: " + std::to_string(resourceInfo.PID) + "\n";
    str += "\tRestarts: " + std::to_string(restartCount) + "\n";
    str += "\tPID: " + std::to_string(resourceInfo.PID) +
           " CPU Used: " + std::to_string(resourceInfo.CPU_Perc) +
           "% RAM Used: " + std::to_string(resourceInfo.RAM_Perc) +
           "% Disk Used: " + std::to_string(resourceInfo.DISK_Perc) + "%\n";
    str += post;
    return str;
}
}  // namespace eros