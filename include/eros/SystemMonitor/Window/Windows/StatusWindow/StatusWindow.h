#ifndef STATUSWINDOW_H
#define STATUSWINDOW_H
#include <eros/SystemMonitor/Field/GenericField.h>
#include <eros/SystemMonitor/Record/GenericRecord.h>
#include <eros/SystemMonitor/Window/WindowText.h>
namespace eros {
class StatusWindow : public WindowText
{
   public:
    StatusWindow(eros::Logger* logger) : WindowText(logger) {
    }
    virtual ~StatusWindow() {
    }
    WindowSize getWindowSize();
    std::vector<std::shared_ptr<IRecord>> getRecords();
    bool keyPressed(KeyMap key);
    // Not allowed to set records independently.
    bool setRecords(std::vector<std::shared_ptr<IRecord>> records) {
        (void)records;
        return false;
    }

   private:
};
}  // namespace eros
#endif