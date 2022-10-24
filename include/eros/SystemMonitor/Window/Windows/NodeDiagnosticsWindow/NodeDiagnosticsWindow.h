#ifndef NODEDIAGNOSTICSWINDOW_H
#define NODEDIAGNOSTICSWINDOW_H
#include <eros/SystemMonitor/Field/GenericField.h>
#include <eros/SystemMonitor/Record/GenericRecord.h>
#include <eros/SystemMonitor/Window/WindowTable.h>
namespace eros {
class NodeDiagnosticsWindow : public WindowTable
{
   public:
    NodeDiagnosticsWindow(eros::Logger* logger) : WindowTable(logger) {
    }
    virtual ~NodeDiagnosticsWindow() {
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