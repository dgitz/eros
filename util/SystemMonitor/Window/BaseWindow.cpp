#include <eros/SystemMonitor/Window/BaseWindow.h>
namespace eros {
bool BaseWindow::update(double currentTime_s) {
    (void)currentTime_s;
    return true;
}
Color BaseWindow::convertLevelToColor(Level::Type level) {
    switch (level) {
        case Level::Type::UNKNOWN: return Color::RED;
        case Level::Type::DEBUG: return Color::BLUE;
        case Level::Type::INFO: return Color::BLUE;
        case Level::Type::NOTICE: return Color::BLUE;
        case Level::Type::WARN: return Color::YELLOW;
        case Level::Type::ERROR: return Color::RED;
        case Level::Type::FATAL: return Color::RED;
        default: return convertLevelToColor(Level::Type::UNKNOWN);
    }
}
Color BaseWindow::convertNodeStateToColor(Node::State state) {
    switch (state) {
        case Node::State::UNKNOWN: return Color::RED;
        case Node::State::START: return Color::GREEN;
        case Node::State::INITIALIZING: return Color::GREEN;
        case Node::State::INITIALIZED: return Color::GREEN;
        case Node::State::RUNNING: return Color::BLUE;
        case Node::State::PAUSED: return Color::GREEN;
        case Node::State::RESET: return Color::GREEN;
        case Node::State::FINISHED: return Color::GREEN;
        default: return Color::RED;
    }
}

}  // namespace eros