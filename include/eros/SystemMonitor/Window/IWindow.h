#ifndef IWINDOW_H
#define IWINDOW_H
#include <eros/SystemMonitor/DataStructures.h>

#include <string>
#include <vector>
namespace eros {
class IWindow
{
   public:
    virtual WindowSize getWindowSize() = 0;
    virtual std::vector<RenderData> getData() = 0;

   private:
};
}  // namespace eros
#endif