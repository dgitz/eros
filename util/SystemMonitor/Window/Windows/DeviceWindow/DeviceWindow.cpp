#include <eros/SystemMonitor/Window/Windows/DeviceWindow/DeviceWindow.h>
using namespace eros;
std::string DeviceWindow::pretty(std::string pre, std::string post) {
    std::string str = pre;
    str += "--- Device Window ---\n";
    str += deviceManager.pretty(pre, post);
    return str;
}
WindowSize DeviceWindow::getWindowSize() {
    WindowSize size;
    uint64_t minWidth = 0;
    for (auto column : getColumnLabels()) { minWidth += column.minWidth; }
    ScreenCoordinatePerc coord(CoordinateReference::GLOBAL, 50.0, 75.0, 50.0, 25.0);
    size.coordinate = coord;
    size.min_height_pixel = 4;
    size.min_width_pixel = minWidth;
    return size;
}
bool DeviceWindow::update(double currentTime_s) {
    if (BaseWindow::update(currentTime_s) == false) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
    bool v = deviceManager.update(currentTime_s);
    if (v == false) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
    return true;
}
std::vector<std::shared_ptr<IRecord>> DeviceWindow::getRecords() {
    std::vector<std::shared_ptr<IRecord>> records;
    uint16_t deviceIndex = 0;
    for (auto device : deviceManager.getDevices()) {
        Color color = BaseWindow::convertLevelToColor(device.second.getStatus());
        bool useNoValues = false;
        if (device.second.getStatus() > Level::Type::WARN) {
            useNoValues = true;
        }
        std::vector<std::shared_ptr<IField>> fields;
        std::shared_ptr<GenericRecord> record(new GenericRecord);
        uint16_t xValue = 0;
        uint16_t index = 0;
        {  // ID
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.color = color;
            data.data = std::to_string(deviceIndex);
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = deviceIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // DEVICENAME
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.data = device.second.getName();
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = deviceIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // CPU AVAILABLE
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            char tempstr[128];
            if (useNoValues == false) {
                sprintf(tempstr, "%2.2f", device.second.getResourceAvailable().CPU_Perc);
            }
            else {
                sprintf(tempstr, "---");
            }
            data.data = std::string(tempstr);
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = deviceIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // RAM AVAILABLE
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            char tempstr[128];
            if (useNoValues == false) {
                sprintf(tempstr, "%2.2f", device.second.getResourceAvailable().RAM_Perc);
            }
            else {
                sprintf(tempstr, "---");
            }
            data.data = std::string(tempstr);
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = deviceIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // DISK AVAILABLE
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            char tempstr[128];
            if (useNoValues == false) {
                sprintf(tempstr, "%2.2f", device.second.getResourceAvailable().DISK_Perc);
            }
            else {
                sprintf(tempstr, "---");
            }
            data.data = std::string(tempstr);
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = deviceIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // Load Factor
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            char tempstr[128];
            if (useNoValues == false) {
                if (device.second.getLoadFactor().loadfactor.size() == 3) {
                    sprintf(tempstr,
                            "[%2.2f,%2.2f,%2.2f]",
                            device.second.getLoadFactor().loadfactor.at(0),
                            device.second.getLoadFactor().loadfactor.at(1),
                            device.second.getLoadFactor().loadfactor.at(2));
                }
                else {
                    logger->log_warn("Load Factor Ill-Formed!");
                }
            }
            else {
                sprintf(tempstr, "---");
            }

            data.data = std::string(tempstr);
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = deviceIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        {  // Heartbeat
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            double lastHeartbeat = device.second.getLastHeartbeatDelta();
            if (lastHeartbeat > 99.9) {
                lastHeartbeat = 99.9;
            }
            char tempstr[128];
            sprintf(tempstr, "%2.2f", lastHeartbeat);
            data.data = std::string(tempstr);
            data.startCoordinate.start_x_pixel = xValue;
            data.startCoordinate.start_y_pixel = deviceIndex;
            field->setData(data);
            fields.push_back(std::move(field));
            xValue += getColumnLabels().at(index).minWidth;
            index++;
        }
        deviceIndex++;
        record->setFields(fields);
        records.push_back(std::move(record));
    }
    return records;
}
bool DeviceWindow::keyPressed(KeyMap key) {
    (void)key;
    logger->log_warn("NOT SUPPORTED YET.");
    return true;
}