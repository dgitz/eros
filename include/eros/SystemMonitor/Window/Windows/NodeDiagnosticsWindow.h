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
    std::vector<IRecord*> getRecords();
    bool keyPressed(KeyMap key);

   private:
};
}  // namespace eros
#endif