#ifndef STATUSWINDOW_H
#define STATUSWINDOW_H
#include <eros/SystemMonitor/Field/GenericField.h>
#include <eros/SystemMonitor/Record/GenericRecord.h>
#include <eros/SystemMonitor/Window/WindowText.h>
#include <eros/armed_state.h>
#include <eros/eROS_Definitions.h>
#include <std_msgs/Time.h>
#include <time.h>
namespace eros {
class StatusWindow : public WindowText
{
   public:
    StatusWindow(eros::Logger* logger)
        : WindowText(logger, IWindow::WindowType::STATUS),
          armedState(ArmDisarm::Type::UNKNOWN),
          currentTime_s(0.0) {
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
    bool newArmedState(eros::armed_state armedState);
    Color convertArmedStateColor(ArmDisarm::Type state);
    bool set_currentROSTime(double currentTime_s) {
        this->currentTime_s = currentTime_s;
        return true;
    }

   private:
    const std::string getCurrentDateTime();
    ArmDisarm::Type armedState;
    double currentTime_s;
};
}  // namespace eros
#endif