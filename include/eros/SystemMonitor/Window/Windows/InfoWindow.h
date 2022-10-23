#ifndef INFOWINDOW_H
#define INFOWINDOW_H
#include <eros/SystemMonitor/Field/GenericField.h>
#include <eros/SystemMonitor/Record/GenericRecord.h>
#include <eros/SystemMonitor/Window/WindowText.h>
namespace eros {
class InfoWindow : public WindowText
{
   public:
    InfoWindow(eros::Logger* logger) : WindowText(logger) {
    }
    virtual ~InfoWindow() {
    }
    WindowSize getWindowSize();
    std::vector<IRecord*> getRecords();
    bool keyPressed(KeyMap key);

   private:
};
}  // namespace eros
#endif