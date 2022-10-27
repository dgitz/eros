#include <eros/SystemMonitor/Window/Windows/DeviceWindow/DeviceManager.h>
namespace eros {
std::string DeviceManager::sanitizeDeviceName(std::string deviceName) {
    if (deviceName.length() > 0) {
        if (deviceName.at(0) == '/') {
            deviceName = deviceName.substr(1, deviceName.length());
        }
    }
    return deviceName;
}
std::string DeviceManager::pretty(std::string pre, std::string post) {
    std::string str = pre;
    str += "--- Device Manager --- \n";
    uint16_t index = 0;
    for (auto device : devices) {
        str += pre + "[" + std::to_string(index + 1) + "/" + std::to_string(devices.size()) + "] " +
               device.second.pretty() + "\n";
        index++;
    }
    str += post;
    return str;
}
bool DeviceManager::new_loadfactor(eros::loadfactor msg) {
    msg.DeviceName = sanitizeDeviceName(msg.DeviceName);
    std::map<std::string, Device>::iterator it = devices.find(msg.DeviceName);
    if (it == devices.end()) {
        Device newDevice(logger, msg.DeviceName, commTimeout_s);
        if (newDevice.new_loadfactor(msg) == false) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            return false;
            // LCOV_EXCL_STOP
        }
        devices.insert(std::pair<std::string, Device>(newDevice.getName(), newDevice));
        return true;
    }
    return it->second.new_loadfactor(msg);
}
bool DeviceManager::new_resourceavailable(eros::resource msg) {
    msg.Name = sanitizeDeviceName(msg.Name);
    std::map<std::string, Device>::iterator it = devices.find(msg.Name);
    if (it == devices.end()) {
        Device newDevice(logger, msg.Name, commTimeout_s);
        if (newDevice.new_resourceavailable(msg) == false) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            return false;
            // LCOV_EXCL_STOP
        }
        devices.insert(std::pair<std::string, Device>(newDevice.getName(), newDevice));
        return true;
    }
    return it->second.new_resourceavailable(msg);
}
bool DeviceManager::update(double currentTime_s) {
    bool anyFail = false;
    for (std::map<std::string, Device>::iterator it = devices.begin(); it != devices.end(); ++it) {
        if (it->second.update(currentTime_s) == false) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            anyFail = true;
            // LCOV_EXCL_STOP
        }
    }

    if (anyFail == false) {
        return true;
    }
    // No Practical way to Unit Test
    // LCOV_EXCL_START
    return false;
    // LCOV_EXCL_STOP
}
}  // namespace eros