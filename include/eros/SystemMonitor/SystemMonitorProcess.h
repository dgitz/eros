/*! \file SystemMonitorProcess.h
 */
// No practical way to this file due to screen rendering.
// LCOV_EXCL_START
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
    eros::Diagnostic::DiagnosticDefinition new_heartbeatmessage(eros::heartbeat msg);

    eros::Diagnostic::DiagnosticDefinition new_resourceavailablemessage(
        const eros::resource::ConstPtr& t_msg);
    eros::Diagnostic::DiagnosticDefinition new_resourceavailablemessage(eros::resource msg);

    eros::Diagnostic::DiagnosticDefinition new_resourceusedmessage(
        const eros::resource::ConstPtr& t_msg);
    eros::Diagnostic::DiagnosticDefinition new_resourceusedmessage(eros::resource msg);

    eros::Diagnostic::DiagnosticDefinition new_loadfactormessage(
        const eros::loadfactor::ConstPtr& t_msg);
    eros::Diagnostic::DiagnosticDefinition new_loadfactormessage(eros::loadfactor msg);

    eros::Diagnostic::DiagnosticDefinition update_genericNode(std::string hostName,
                                                              std::string nodeName,
                                                              double currentTime_s);

   private:
    bool initializeWindows();
    std::map<eros::IWindow::WindowType, eros::IWindow*> windows;
    eros::RenderEngine* renderEngine;
};
#endif  // SystemMonitorProcess_H
        // LCOV_EXCL_STOP