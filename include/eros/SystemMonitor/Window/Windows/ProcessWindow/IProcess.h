#ifndef SYSTEMMONITOR_IPROCESS_H
#define SYSTEMMONITOR_IPROCESS_H
#include <eros/Logger.h>
#include <eros/eROS_Definitions.h>
namespace eros {
class IProcess
{
   public:
    enum class ProcessType { UNKNOWN = 0, EROS = 1, GENERIC = 2, END_OF_LIST = 3 };
    static std::string ProcessTypeString(ProcessType v) {
        switch (v) {
            case ProcessType::UNKNOWN: return "UNKNOWN";
            case ProcessType::EROS: return "EROS";
            case ProcessType::GENERIC: return "GENERIC";
            default: return ProcessTypeString(ProcessType::UNKNOWN);
        }
    }
    static ProcessType ProcessTypeEnum(std::string type) {
        if (type == "EROS") {
            return ProcessType::EROS;
        }
        if (type == "GENERIC") {
            return ProcessType::GENERIC;
        }
        return ProcessType::UNKNOWN;
    }
    virtual std::string pretty(const std::string& pre = "", const std::string& post = "") = 0;
    virtual bool update(double currentTime_s) = 0;
    virtual Level::Type getLevel() = 0;
    virtual Node::State getState() = 0;
    virtual ProcessType getProcessType() = 0;
    virtual std::string getNodeName() = 0;
    virtual uint64_t getAliveCount() = 0;
    /*
     enum class ProcessType { UNKNOWN = 0, EROS = 1, NON_EROS = 2 };
     Process(eros::Logger* logger,
             ProcessType processType,
             std::string nodeName,
             std::string baseNodeName,
             std::string hostDevice,
             double commTimeout_s)
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


     uint64_t getRestartCount() {
         return restartCount;
     }
     bool setNodeAlive(double currentTime_s);

    private:
     eros::Logger* logger;
     ProcessType processType;
     std::string nodeName;

     std::string baseNodeName;
     std::string hostDevice;
     double commTimeout_s;

     eros::resource resourceInfo;

     eros::Node::State state;
     eros::Level::Type status;

     double lastHeartbeat;
     double lastHeartbeatDelta;
     uint64_t restartCount;
     */
};
}  // namespace eros
#endif