#ifndef SYSTEMMONITOR_DEVICEMANAGER_H
#define SYSTEMMONITOR_DEVICEMANAGER_H
#include <eros/Logger.h>
#include <eros/SystemMonitor/Window/Windows/DeviceWindow/Device.h>
#include <eros/loadfactor.h>
#include <eros/resource.h>

#include <map>
namespace eros {
class DeviceManager
{
   public:
    static std::string sanitizeDeviceName(std::string deviceName);
    DeviceManager(eros::Logger* logger, double commTimeout_s)
        : logger(logger), commTimeout_s(commTimeout_s) {
    }
    virtual ~DeviceManager() {
    }
    bool new_loadfactor(eros::loadfactor msg);
    bool new_resourceavailable(eros::resource msg);
    std::map<std::string, Device> getDevices() {
        return devices;
    }
    bool update(double currentTime_s);
    std::string pretty(std::string pre = "", std::string post = "");

   private:
    eros::Logger* logger;
    double commTimeout_s;
    std::map<std::string, Device> devices;
};
}  // namespace eros
#endif
