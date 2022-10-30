#ifndef SYSTEMMONITOR_BASEPROCESS_H
#define SYSTEMMONITOR_BASEPROCESS_H
#include "IProcess.h"
namespace eros {
class BaseProcess : public IProcess
{
   public:
    BaseProcess(eros::Logger* logger,
                ProcessType processType,
                std::string nodeName,
                double commTimeout_s)
        : logger(logger),
          processType(processType),
          nodeName(nodeName),
          commTimeout_s(commTimeout_s),
          level(Level::Type::INFO),
          state(Node::State::RUNNING),
          lastHeartbeatDelta(0.0),
          lastHeartbeat(0.0),
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
    uint64_t getAliveCount() {
        return aliveCount;
    }

   protected:
    bool setNodeAlive(double currentTime_s);
    eros::Logger* logger;
    ProcessType processType;
    std::string nodeName;
    double commTimeout_s;
    eros::Level::Type level;
    Node::State state;
    double lastHeartbeatDelta;
    double lastHeartbeat;

   private:
    uint64_t aliveCount;
};
}  // namespace eros
#endif