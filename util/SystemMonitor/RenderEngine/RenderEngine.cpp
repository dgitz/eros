#include <eros/SystemMonitor/RenderEngine/RenderEngine.h>
namespace eros {

bool RenderEngine::initScreen() {
    // setlocale(LC_ALL, "");
    mousemask(ALL_MOUSE_EVENTS, NULL);
    initscr();
    timeout(1);
    clear();
    if (has_colors() == FALSE) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        endwin();
        return false;
        // LCOV_EXCL_STOP
    }

    curs_set(0);
    noecho();
    raw();

    /*
    start_color();
    init_color(COLOR_BLACK, 0, 0, 0);
    init_color(COLOR_GREEN, 0, 600, 0);
    init_color(10, 500, 0, 500);
    init_pair((uint8_t)SystemMonitorProcess::Color::NO_COLOR, COLOR_WHITE, COLOR_BLACK);
    init_pair((uint8_t)SystemMonitorProcess::Color::RED_COLOR, COLOR_WHITE, COLOR_RED);
    init_pair((uint8_t)SystemMonitorProcess::Color::YELLOW_COLOR, COLOR_WHITE, COLOR_YELLOW);
    init_pair((uint8_t)SystemMonitorProcess::Color::GREEN_COLOR, COLOR_WHITE, COLOR_GREEN);
    init_pair((uint8_t)SystemMonitorProcess::Color::BLUE_COLOR, COLOR_WHITE, COLOR_BLUE);
    init_pair((uint8_t)SystemMonitorProcess::Color::PURPLE_COLOR, COLOR_WHITE, 10);
    */
    uint16_t mainwindow_width, mainwindow_height;
    getmaxyx(stdscr, mainwindow_height, mainwindow_width);

    logger->log_notice(std::to_string(mainwindow_width) + " " + std::to_string(mainwindow_height));
    for (auto window : dataWindows) {
        RenderWindow* renderWindow = new RenderWindow(
            logger, window.second->getWindowSize(), mainwindow_width, mainwindow_height);
        if (renderWindow->init() == false) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            return false;
            // LCOV_EXCL_STOP
        }
        if (window.first == IWindow::WindowType::PROCESS) {
            renderWindow->setFocused(true);
        }
        Window win(window.second, renderWindow);
        windows.insert(std::pair<IWindow::WindowType, Window>(window.first, win));
    }
    return true;
}
bool RenderEngine::update(double dt, std::map<IWindow::WindowType, IWindow*> _windows) {
    (void)dt;
    for (auto window : _windows) { windows.find(window.first)->second.windowData = window.second; }
    uint16_t key = getch();
    KeyMap keyPressed = (KeyMap)key;
    bool validKey = false;
    switch (keyPressed) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        case KeyMap::KEY_Q:
            killMe = true;
            validKey = true;
            break;
        case KeyMap::KEY_q:
            killMe = true;
            validKey = true;
            break;
        case KeyMap::KEY_TAB:
            incrementFocus();
            validKey = true;
            break;
        default:
            if (key != USHRT_MAX) {
                validKey = true;
                logger->log_debug("Key: " + std::to_string(key));
            }
            break;
            // LCOV_EXCL_STOP
    }
    if (validKey == true) {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        if (windows.find(IWindow::WindowType::INFO)->second.windowData->keyPressed(keyPressed) ==
            false) {
            return false;
        }
        // LCOV_EXCL_STOP
    }
    for (std::map<IWindow::WindowType, Window>::iterator it = windows.begin(); it != windows.end();
         it++) {
        renderWindow(it->second.windowData, it->second.windowRender);
        if (it->second.windowRender->isFocused()) {
            if (validKey == true) {
                // No Practical way to Unit Test
                // LCOV_EXCL_START
                if (it->second.windowData->keyPressed(keyPressed) == false) {
                    return false;
                }
                // LCOV_EXCL_STOP
            }
        }
    }
    return true;
}
bool RenderEngine::renderWindow(IWindow* windowData, RenderWindow* renderWindow) {
    for (auto record : windowData->getRecords()) {
        for (auto field : record->getFields()) {
            mvwprintw(renderWindow->get_window_reference(),
                      field->getData().x + 1,
                      field->getData().y + 1,
                      field->getData().data.c_str());
        }
    }

    if (renderWindow->isFocused() == true) {
        box(renderWindow->get_window_reference(), '|', '-');
    }
    else {
        box(renderWindow->get_window_reference(), 0, 0);
    }
    wrefresh(renderWindow->get_window_reference());
    return true;
}
bool RenderEngine::incrementFocus() {
    IWindow::WindowType currentFocusedWindow = IWindow::WindowType::UNKNOWN;
    // Get current window that's focused
    for (std::map<IWindow::WindowType, Window>::iterator it = windows.begin(); it != windows.end();
         it++) {
        if (it->second.windowRender->isFocused() == true) {
            currentFocusedWindow = it->first;
        }
    }
    IWindow::WindowType newFocusedWindow = (IWindow::WindowType)((uint8_t)currentFocusedWindow + 1);
    if ((newFocusedWindow == IWindow::WindowType::END_OF_LIST) ||
        (newFocusedWindow == IWindow::WindowType::INFO)) {
        newFocusedWindow = (IWindow::WindowType)((uint8_t)IWindow::WindowType::UNKNOWN + 1);
    }
    windows.find(currentFocusedWindow)->second.windowRender->setFocused(false);
    windows.find(newFocusedWindow)->second.windowRender->setFocused(true);
    if (currentFocusedWindow != newFocusedWindow) {
        return true;
    }
    else {
        // No Practical way to Unit Test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
}
}  // namespace eros