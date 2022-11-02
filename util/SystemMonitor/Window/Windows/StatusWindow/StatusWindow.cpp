#include <eros/SystemMonitor/Window/Windows/StatusWindow/StatusWindow.h>
using namespace eros;
WindowSize StatusWindow::getWindowSize() {
    WindowSize size;
    ScreenCoordinatePerc coord(CoordinateReference::GLOBAL, 0.0, 0.0, 100.0, 15.0);
    size.coordinate = coord;
    size.min_height_pixel = 5;
    size.min_width_pixel = 30;
    return size;
}
std::vector<std::shared_ptr<IRecord>> StatusWindow::getRecords() {
    std::vector<std::shared_ptr<IRecord>> records;
    {  // System Time
        std::vector<std::shared_ptr<IField>> fields;
        std::shared_ptr<GenericRecord> record(new GenericRecord);
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = getCurrentDateTime();
        data.startCoordinate.start_x_pixel = 0;
        data.startCoordinate.start_y_pixel = 0;
        field->setData(data);
        fields.push_back(std::move(field));

        record->setFields(fields);
        records.push_back(std::move(record));
    }
    {
        // ROS Time
        std::vector<std::shared_ptr<IField>> fields;
        std::shared_ptr<GenericRecord> record(new GenericRecord);
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        char tempstr[128];
        sprintf(tempstr, "%12.4f", currentTime_s);
        data.data = std::string(tempstr);
        data.startCoordinate.start_x_pixel = actualWindowSize.width_pixel - data.data.length() - 2;
        data.startCoordinate.start_y_pixel = 0;
        field->setData(data);
        fields.push_back(std::move(field));

        record->setFields(fields);
        records.push_back(std::move(record));
    }
    {  // Armed State
        std::vector<std::shared_ptr<IField>> fields;
        std::shared_ptr<GenericRecord> record(new GenericRecord);
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.color = convertArmedStateColor((ArmDisarm::Type)armedState);
        data.data = "Armed State: " + ArmDisarm::ArmDisarmString(armedState);
        data.startCoordinate.start_x_pixel = 0;
        // Center String
        uint16_t leftBound = 0;
        uint16_t rightBound = actualWindowSize.width_pixel;
        uint16_t widthText = data.data.length();
        uint16_t leftOffset = (rightBound - leftBound) / 2 - widthText / 2;
        data.startCoordinate.start_x_pixel = leftOffset;
        data.startCoordinate.start_y_pixel = 1;
        field->setData(data);
        fields.push_back(std::move(field));

        record->setFields(fields);
        records.push_back(std::move(record));
    }

    return records;
}
bool StatusWindow::keyPressed(KeyMap key) {
    (void)key;
    return true;
}
Color StatusWindow::convertArmedStateColor(ArmDisarm::Type state) {
    switch (state) {
        case ArmDisarm::Type::ARMED: return Color::GREEN;
        case ArmDisarm::Type::DISARMED_CANNOTARM: return Color::RED;
        case ArmDisarm::Type::DISARMED: return Color::GREEN;
        case ArmDisarm::Type::DISARMING: return Color::GREEN;
        case ArmDisarm::Type::ARMING: return Color::GREEN;
        default: return Color::BLACK;
    }
}
const std::string StatusWindow::getCurrentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
    return buf;
}
bool StatusWindow::newArmedState(eros::armed_state _armedState) {
    armedState = (ArmDisarm::Type)_armedState.armed_state;
    return true;
}