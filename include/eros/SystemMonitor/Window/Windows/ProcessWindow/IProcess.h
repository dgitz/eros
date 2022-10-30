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
    virtual std::string getHostName() = 0;
    virtual uint64_t getAliveCount() = 0;
    virtual uint64_t getRestartCount() = 0;
    virtual uint64_t getPID() = 0;
    virtual double getCPUUsed() = 0;
    virtual double getRAMUsed() = 0;
    virtual double getLastHeartbeatDelta() = 0;
};
}  // namespace eros
#endif