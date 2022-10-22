#ifndef IWINDOW_H
#define IWINDOW_H
#include <eros/SystemMonitor/DataStructures.h>

#include <string>
namespace eros {
class IWindow
{
   public:
    virtual std::string getData() = 0;
    virtual WindowSize getWindowSize() = 0;

   private:
};
}  // namespace eros
#endif