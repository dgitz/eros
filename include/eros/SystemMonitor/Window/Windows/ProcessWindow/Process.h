#ifndef SYSTEMMONITOR_PROCESS_H
#define SYSTEMMONITOR_PROCESS_H
#include <eros/Logger.h>
#include <eros/eROS_Definitions.h>
#include <eros/resource.h>
namespace eros {
class Process
{
   public:
    enum class ProcessType { UNKNOWN = 0, EROS = 1, NON_EROS = 2 };
    Process(eros::Logger* logger,
            std::string nodeName,
            std::string baseNodeName,
            std::string hostDevice,
            double commTimeout_s)
        : logger(logger),
          processType(ProcessType::NON_EROS),
          nodeName(nodeName),
          baseNodeName(baseNodeName),
          hostDevice(hostDevice),
          commTimeout_s(commTimeout_s),
          state(eros::Node::State::UNKNOWN),
          status(eros::Level::Type::UNKNOWN),
          lastHeartbeat(0.0),
          lastHeartbeatDelta(0.0),
          restartCount(0) {
    }
    Process(eros::Logger* logger,
            ProcessType processType,
            std::string nodeName,
            std::string baseNodeName,
            std::string hostDevice,
            double commTimeout_s,
            eros::resource resourceInfo)
        : logger(logger),
          processType(processType),
          nodeName(nodeName),
          baseNodeName(baseNodeName),
          hostDevice(hostDevice),
          commTimeout_s(commTimeout_s),
          state(eros::Node::State::UNKNOWN),
          status(eros::Level::Type::UNKNOWN),
          lastHeartbeat(0.0),
          lastHeartbeatDelta(0.0),
          restartCount(0) {
        this->resourceInfo = resourceInfo;
        lastHeartbeat = resourceInfo.stamp.toSec();
        lastHeartbeatDelta = 0.0;
    }
    bool new_resourceused(eros::resource resourceUsed);
    std::string pretty(std::string pre = "", std::string post = "");
    bool update(double currentTime_s);
    Level::Type getStatus() {
        return status;
    }

   private:
    eros::Logger* logger;
    ProcessType processType;
    std::string nodeName;
    std::string hostDevice;
    std::string baseNodeName;
    double commTimeout_s;

    eros::resource resourceInfo;

    eros::Node::State state;
    eros::Level::Type status;

    double lastHeartbeat;
    double lastHeartbeatDelta;
    uint64_t restartCount;
};
}  // namespace eros
#endif