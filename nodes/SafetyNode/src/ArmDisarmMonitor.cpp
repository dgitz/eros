#include "ArmDisarmMonitor.h"
namespace eros_nodes {
ArmDisarmMonitor::ArmDisarmMonitor(std::string _name, Type _type) : name(_name), type(_type) {
    ready_to_arm.ready_to_arm = false;
    ready_to_arm.diag.Description = "Nothing Received";
    update_count = 0;
    last_delta_update_time = 0.0;
}
ArmDisarmMonitor::~ArmDisarmMonitor() {
}
}  // namespace eros_nodes