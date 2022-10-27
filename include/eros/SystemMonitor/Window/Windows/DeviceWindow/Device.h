#ifndef SYSTEMMONITOR_DEVICE_H
#define SYSTEMMONITOR_DEVICE_H
#include <eros/Logger.h>
#include <eros/eROS_Definitions.h>
#include <eros/loadfactor.h>
#include <eros/resource.h>
namespace eros {

class Device
{
   public:
    Device(Logger* logger, std::string name, double commTimeout_s)
        : logger(logger),
          name(name),
          commTimeout_s(commTimeout_s),
          status(Level::Type::UNKNOWN),
          last_heartbeat(0.0),
          last_heartbeat_delta(0.0) {
    }
    virtual ~Device() {
    }
    std::string getName() {
        return name;
    }
    bool new_resourceavailable(eros::resource resourceAvailable);
    eros::resource getResourceAvailable() {
        return resourceAvailable;
    }
    bool new_loadfactor(eros::loadfactor loadFactor);
    eros::loadfactor getLoadFactor() {
        return loadFactor;
    }
    Level::Type getStatus() {
        return status;
    }
    bool update(double currentTime_s);
    double getLastHeartbeatDelta() {
        return last_heartbeat_delta;
    }
    std::string pretty(std::string pre = "", std::string post = "");

   private:
    Logger* logger;
    std::string name;
    double commTimeout_s;
    eros::Level::Type status;
    double last_heartbeat;
    double last_heartbeat_delta;

    eros::resource resourceAvailable;
    eros::loadfactor loadFactor;
};
}  // namespace eros

#endif