#ifndef SYSTEMMONITOR_PROCESSMANAGER_H
#define SYSTEMMONITOR_PROCESSMANAGER_H
#include <eros/Logger.h>
namespace eros {
class ProcessManager
{
   public:
    ProcessManager(eros::Logger* logger) : logger(logger) {
    }

   private:
    eros::Logger* logger;
};
}  // namespace eros
#endif