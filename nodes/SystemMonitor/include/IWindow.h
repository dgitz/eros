#pragma once
namespace eros_nodes::SystemMonitor {

class IWindow
{
   public:
    virtual ~IWindow() {
    }
    virtual std::string get_name() = 0;
    virtual eros::Diagnostic::DiagnosticDefinition update(double dt, double t_ros_time) = 0;
    virtual bool new_msg(eros::ArmDisarm::State armed_state) = 0;
    virtual bool new_msg(eros::heartbeat heartbeat_msg) = 0;
};
}  // namespace eros_nodes::SystemMonitor