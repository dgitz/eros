#include <eros/SystemMonitor/Window/Windows/ProcessWindow/ProcessManager.h>
namespace eros {
bool ProcessManager::new_resourceUsed(eros::resource msg) {
    std::lock_guard<std::mutex> lock(processListMutex);
    std::string nodeName = msg.Name;
    if (processList.find(nodeName) == processList.end()) {
        std::shared_ptr<EROSProcess> process(new EROSProcess(logger, nodeName, commTimeout_s));
        processList.insert(
            std::pair<std::string, std::shared_ptr<IProcess>>(nodeName, std::move(process)));
    }
    std::map<std::string, std::shared_ptr<IProcess>>::iterator it = processList.find(nodeName);
    if (it == processList.end()) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        logger->log_error("Process failed to add to Map.");
        return false;
        // LCOV_EXCL_STOP
    }
    std::shared_ptr<EROSProcess> p = std::dynamic_pointer_cast<EROSProcess>(it->second);
    if (p == nullptr) {
        std::shared_ptr<GenericProcess> genP =
            std::dynamic_pointer_cast<GenericProcess>(it->second);
        if (genP == nullptr) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            logger->log_error("Unable to figure out what type " + nodeName + " is...");
            return false;
            // LCOV_EXCL_STOP
        }
        std::shared_ptr<EROSProcess> newProcess(new EROSProcess(logger, nodeName, commTimeout_s));
        if (newProcess->new_resourceused(msg) == false) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            logger->log_error("Unable to process heartbeat.");
            return false;
            // LCOV_EXCL_STOP
        }
        processList.erase(nodeName);
        processList.insert(
            std::pair<std::string, std::shared_ptr<IProcess>>(nodeName, std::move(newProcess)));
        return true;
    }
    return p->new_resourceused(msg);
}
bool ProcessManager::new_heartbeat(eros::heartbeat msg) {
    std::lock_guard<std::mutex> lock(processListMutex);
    std::string nodeName = msg.NodeName;
    if (processList.find(nodeName) == processList.end()) {
        std::shared_ptr<EROSProcess> process(new EROSProcess(logger, nodeName, commTimeout_s));
        processList.insert(
            std::pair<std::string, std::shared_ptr<IProcess>>(nodeName, std::move(process)));
    }
    std::map<std::string, std::shared_ptr<IProcess>>::iterator it = processList.find(nodeName);
    if (it == processList.end()) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        logger->log_error("Process failed to add to Map.");
        return false;
        // LCOV_EXCL_STOP
    }
    std::shared_ptr<EROSProcess> p = std::dynamic_pointer_cast<EROSProcess>(it->second);
    if (p == nullptr) {
        std::shared_ptr<GenericProcess> genP =
            std::dynamic_pointer_cast<GenericProcess>(it->second);
        if (genP == nullptr) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            logger->log_error("Unable to figure out what type " + nodeName + " is...");
            return false;
            // LCOV_EXCL_STOP
        }
        std::shared_ptr<EROSProcess> newProcess(new EROSProcess(logger, nodeName, commTimeout_s));
        if (newProcess->new_heartbeat(msg) == false) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            logger->log_error("Unable to process heartbeat.");
            return false;
            // LCOV_EXCL_STOP
        }
        processList.erase(nodeName);
        processList.insert(
            std::pair<std::string, std::shared_ptr<IProcess>>(nodeName, std::move(newProcess)));
        return true;
    }
    return p->new_heartbeat(msg);
}
bool ProcessManager::new_nodeAlive(std::string nodeName, double currentTime_s) {
    std::lock_guard<std::mutex> lock(processListMutex);
    if (processList.find(nodeName) == processList.end()) {
        std::shared_ptr<GenericProcess> process(
            new GenericProcess(logger, nodeName, commTimeout_s));
        processList.insert(
            std::pair<std::string, std::shared_ptr<IProcess>>(nodeName, std::move(process)));
    }
    std::map<std::string, std::shared_ptr<IProcess>>::iterator it = processList.find(nodeName);
    if (it == processList.end()) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        logger->log_error("Process failed to add to Map.");
        return false;
        // LCOV_EXCL_STOP
    }
    std::shared_ptr<GenericProcess> p = std::dynamic_pointer_cast<GenericProcess>(it->second);
    if (p == nullptr) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        logger->log_error("Unable to cast: " + nodeName + " to Generic Process.");
        return false;
        // LCOV_EXCL_STOP
    }
    return p->setNodeAlive(currentTime_s);
}
bool ProcessManager::update(double currentTime_s) {
    std::lock_guard<std::mutex> lock(processListMutex);
    std::map<std::string, std::shared_ptr<IProcess>>::iterator it;
    bool allOk = true;
    for (it = processList.begin(); it != processList.end(); ++it) {
        if (it->second->update(currentTime_s) == false) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            logger->log_warn("Failed to update Process: " + it->second->getNodeName());
            allOk = false;
            // LCOV_EXCL_STOP
        }
    }
    return allOk;
}
std::string ProcessManager::pretty(const std::string& pre, const std::string& post) {
    std::lock_guard<std::mutex> lock(processListMutex);
    std::string str = pre + "--- Process Manager ---\n";
    str += "\tProcess Count: " + std::to_string(getProcesses().size()) + "\n";
    for (auto proc : getProcesses()) { str += proc.second->pretty(pre, post); }
    return str;
}

}  // namespace eros