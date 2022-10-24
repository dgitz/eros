/*! \file SystemMonitorProcess.h
 */
#ifndef SystemMonitorProcess_H
#define SystemMonitorProcess_H
#include <eros/BaseNodeProcess.h>
#include <eros/SystemMonitor/RenderEngine/RenderEngine.h>
#include <eros/SystemMonitor/Window/IWindow.h>
#include <eros/SystemMonitor/Window/Windows/DeviceWindow/DeviceWindow.h>
#include <eros/SystemMonitor/Window/Windows/InfoWindow/InfoWindow.h>
#include <eros/SystemMonitor/Window/Windows/NodeDiagnosticsWindow/NodeDiagnosticsWindow.h>
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/ProcessWindow.h>
#include <eros/SystemMonitor/Window/Windows/StatusWindow/StatusWindow.h>
/*! \class SystemMonitorProcess SystemMonitorProcess.h "SystemMonitorProcess.h"
 *  \brief */
class SystemMonitorProcess : public eros::BaseNodeProcess
{
   public:
    SystemMonitorProcess();
    ~SystemMonitorProcess();
    eros::Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    eros::Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<eros::Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg);
    std::vector<eros::Diagnostic::DiagnosticDefinition> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }
    bool shouldKill() {
        return renderEngine->shouldKill();
    }
    eros::Diagnostic::DiagnosticDefinition new_heartbeatmessage(
        const eros::heartbeat::ConstPtr& t_msg);

   private:
    bool initializeWindows();
    std::map<eros::IWindow::WindowType, eros::IWindow*> windows;
    eros::RenderEngine* renderEngine;
};
#endif  // SystemMonitorProcess_H
