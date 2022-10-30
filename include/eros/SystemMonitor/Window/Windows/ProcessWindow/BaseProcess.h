#ifndef SYSTEMMONITOR_BASEPROCESS_H
#define SYSTEMMONITOR_BASEPROCESS_H
#include "IProcess.h"
namespace eros {
class BaseProcess : public IProcess
{
   public:
    BaseProcess(eros::Logger* logger,
                ProcessType processType,
                std::string hostName,
                std::string nodeName,
                double commTimeout_s)
        : logger(logger),
          processType(processType),
          hostName(hostName),
          nodeName(nodeName),
          commTimeout_s(commTimeout_s),
          level(Level::Type::INFO),
          state(Node::State::RUNNING),
          lastHeartbeatDelta(0.0),
          lastHeartbeat(0.0),
          PID(0),
          CPUUsed(0.0),
          RAMUsed(0.0),
          restartCount(0),
          aliveCount(0) {
    }
    ProcessType getProcessType() {
        return processType;
    }
    std::string pretty(const std::string& pre = "", const std::string& post = "");
    bool update(double currentTime_s);
    Level::Type getLevel() {
        return level;
    }
    Node::State getState() {
        return state;
    }

    std::string getNodeName() {
        return nodeName;
    }
    std::string getHostName() {
        return hostName;
    }
    uint64_t getAliveCount() {
        return aliveCount;
    }
    uint64_t getPID() {
        return PID;
    }
    double getCPUUsed() {
        return CPUUsed;
    }
    double getRAMUsed() {
        return RAMUsed;
    }
    uint64_t getRestartCount() {
        return restartCount;
    }
    double getLastHeartbeatDelta() {
        return lastHeartbeatDelta;
    }

   protected:
    bool setNodeAlive(double currentTime_s);
    eros::Logger* logger;
    ProcessType processType;
    std::string hostName;
    std::string nodeName;
    double commTimeout_s;
    eros::Level::Type level;
    Node::State state;
    double lastHeartbeatDelta;
    double lastHeartbeat;
    uint64_t PID;
    double CPUUsed;
    double RAMUsed;
    uint64_t restartCount;

   private:
    uint64_t aliveCount;
};
}  // namespace eros
#endif