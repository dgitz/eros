#include <eros/SystemMonitor/Window/Windows/DeviceWindow/Device.h>
namespace eros {
std::string Device::pretty(std::string pre, std::string post) {
    std::string str =
        pre + " Device Name: " + getName() + "[" + Level::LevelString(getStatus()) + "] ";
    str += " Heartbeat dT: " + std::to_string(last_heartbeat_delta) + post;
    return str;
}
bool Device::new_resourceavailable(eros::resource v) {
    if (v.Name != getName()) {
        return false;
    }
    resourceAvailable = v;
    last_heartbeat = v.stamp.toSec();
    last_heartbeat_delta = 0.0;
    status = Level::Type::INFO;
    return true;
}
bool Device::new_loadfactor(eros::loadfactor v) {
    if (v.DeviceName != getName()) {
        return false;
    }
    if (v.loadfactor.size() != 3) {
        return false;
    }
    loadFactor = v;
    last_heartbeat = v.stamp.toSec();
    last_heartbeat_delta = 0.0;
    status = Level::Type::INFO;
    return true;
}
bool Device::update(double currentTime_s) {
    last_heartbeat_delta = currentTime_s - last_heartbeat;
    if (last_heartbeat_delta > (2.0 * commTimeout_s)) {
        status = Level::Type::ERROR;
    }
    else if (last_heartbeat_delta > commTimeout_s) {
        status = Level::Type::WARN;
    }
    return true;
}
}  // namespace eros