#include <eros/SystemMonitor/Window/Windows/ProcessWindow/ProcessWindow.h>
using namespace eros;
WindowSize ProcessWindow::getWindowSize() {
    WindowSize size;
    ScreenCoordinatePerc coord(CoordinateReference::GLOBAL, 0.0, 15.0, 70.0, 60.0);
    size.coordinate = coord;
    size.min_height_pixel = 8;
    size.min_width_pixel = 60;
    return size;
}
bool ProcessWindow::update(double currentTime_s) {
    bool v = BaseWindow::update(currentTime_s);
    if (v == false) {
        return false;
    }
    return processManager.update(currentTime_s);
}
std::vector<std::shared_ptr<IRecord>> ProcessWindow::getRecords() {
    std::vector<std::shared_ptr<IRecord>> records;
    uint16_t processIndex = 0;
    uint16_t selectedIndex = 0;
    for (auto process : processManager.getProcesses()) {
        Color color = BaseWindow::convertLevelToColor(process.second->getLevel());
        std::vector<std::shared_ptr<IField>> fields;
        std::shared_ptr<GenericRecord> record(new GenericRecord);
        uint16_t xValue = 0;
        uint16_t index = 0;
        {  // Selected?
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.color = color;
            if (selectedIndex == processIndex) {
                data.data = "***";
            }
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = processIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // ID
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.data = std::to_string(processIndex);
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = processIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // HostName
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.data = smallifyROSName(process.second->getHostName());
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = processIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // NodeName
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.data = smallifyROSName(process.second->getNodeName());
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = processIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // State
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.data = Node::NodeStateString(process.second->getState());
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = processIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // Restarts
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.data = std::to_string(process.second->getRestartCount());
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = processIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {
            // PID
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.data = std::to_string(process.second->getPID());
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = processIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {
            // CPU Used
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            char tempstr[128];
            sprintf(tempstr, "%3.2f", process.second->getCPUUsed());
            data.data = std::string(tempstr);
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = processIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {
            // RAM Used
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            char tempstr[128];
            sprintf(tempstr, "%3.2f", process.second->getRAMUsed());
            data.data = std::string(tempstr);
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = processIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // Heartbeat
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            double lastHeartbeat = process.second->getLastHeartbeatDelta();
            if (lastHeartbeat > 99.9) {
                lastHeartbeat = 99.9;
            }
            char tempstr[128];
            sprintf(tempstr, "%2.2f", lastHeartbeat);
            data.data = std::string(tempstr);
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = processIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }

        processIndex++;
        record->setFields(fields);
        records.push_back(std::move(record));
    }
    return records;
}
bool ProcessWindow::new_heartbeat(eros::heartbeat msg) {
    return processManager.new_heartbeat(msg);
}
bool ProcessWindow::new_resource(eros::resource msg) {
    return processManager.new_resourceUsed(msg);
}
bool ProcessWindow::new_nodeAlive(std::string nodeName, double currentTime_s) {
    return processManager.new_nodeAlive(nodeName, currentTime_s);
}
bool ProcessWindow::keyPressed(KeyMap key) {
    (void)key;
    logger->log_warn("NOT SUPPORTED YET.");
    return true;
}
std::string ProcessWindow::smallifyROSName(std::string v) {
    uint16_t slashCount = boost::count(v, '/');
    if (slashCount <= 1) {
        return v;
    }

    std::size_t lastSlashIndex = v.find_last_of('/');
    std::string newStr = "/...";
    return newStr + v.substr(lastSlashIndex);
}