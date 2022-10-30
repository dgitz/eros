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
        logger->log_error("No Colors for Terminal.");
        endwin();
        return false;
        // LCOV_EXCL_STOP
    }
    curs_set(0);
    noecho();
    raw();

    start_color();
    init_color(COLOR_BLACK, 0, 0, 0);
    init_color(COLOR_GREEN, 0, 600, 0);
    init_color(10, 500, 0, 500);

    init_pair((uint8_t)Color::BLACK, COLOR_BLACK, COLOR_WHITE);
    init_pair((uint8_t)Color::WHITE, COLOR_WHITE, COLOR_BLACK);
    init_pair((uint8_t)Color::RED, COLOR_WHITE, COLOR_RED);
    init_pair((uint8_t)Color::YELLOW, COLOR_WHITE, COLOR_YELLOW);
    init_pair((uint8_t)Color::GREEN, COLOR_WHITE, COLOR_GREEN);
    init_pair((uint8_t)Color::BLUE, COLOR_WHITE, COLOR_BLUE);
    init_pair((uint8_t)Color::PURPLE, COLOR_WHITE, 10);
    uint16_t mainwindow_width, mainwindow_height;
    getmaxyx(stdscr, mainwindow_height, mainwindow_width);
    logger->log_notice(std::to_string(mainwindow_width) + " " + std::to_string(mainwindow_height));
    for (auto window : dataWindows) {
        RenderWindow* renderWindow = new RenderWindow(
            logger, window.second->getWindowSize(), mainwindow_width, mainwindow_height);
        if (renderWindow->init() == false) {
            // No Practical way to Unit Test
            // LCOV_EXCL_START
            logger->log_error("Unable to initialize Render Window.");
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
        case KeyMap::KEY_down: validKey = true; break;
        case KeyMap::KEY_up: validKey = true; break;
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
    if (renderWindow->isFocused() == true) {
        box(renderWindow->get_window_reference(), '|', '-');
    }
    else {
        box(renderWindow->get_window_reference(), 0, 0);
    }
    {
        // WindowTable Render

        WindowTable* win = dynamic_cast<WindowTable*>(windowData);
        if (win != nullptr) {
            uint16_t xValue = 1;
            std::vector<WindowTable::ColumnLabel> labels = win->getColumnLabels();
            for (auto label : labels) {
                mvwprintw(renderWindow->get_window_reference(), 1, xValue, label.label.c_str());
                xValue += label.minWidth;
            }

            std::string dashed(renderWindow->getActualSize().width_pixel - 2, '-');
            mvwprintw(renderWindow->get_window_reference(), 2, 1, dashed.c_str());

            uint16_t recordsAllowedToShow = renderWindow->getActualSize().height_pixel - 4;
            uint16_t startRecordShow = 0;
            uint16_t stopRecordShow = recordsAllowedToShow;
            uint16_t selectedRecordIndex = win->getSelectedRecordIndex();
            uint16_t shiftDown = 0;
            if (win->getRecords().size() > recordsAllowedToShow) {
                if (selectedRecordIndex >= recordsAllowedToShow) {
                    shiftDown = selectedRecordIndex - stopRecordShow + 1;
                }
            }

            startRecordShow += shiftDown;
            stopRecordShow += shiftDown;

            uint16_t recordIndex = 0;
            for (auto record : win->getRecords()) {
                if ((recordIndex < startRecordShow) || (recordIndex >= stopRecordShow)) {
                    recordIndex++;
                    continue;
                }

                if (labels.size() < record->getFields().size()) {
                    logger->log_error("No label defined for all Record Fields: Win: " +
                                      std::to_string((uint8_t)win->getWindowType()) + " " +
                                      std::to_string(labels.size()) + " < " +
                                      std::to_string(record->getFields().size()));
                    return false;
                }
                bool useFirstColor = false;
                Color firstColor = Color::UNKNOWN;
                if (record->getFields().size() > 1) {
                    if (record->getFields().at(1)->getData().color == Color::UNKNOWN) {
                        useFirstColor = true;
                        firstColor = record->getFields().at(0)->getData().color;
                    }
                }
                uint16_t labelIndex = 0;
                for (auto field : record->getFields()) {
                    std::string data = field->getData().data;
                    if (data.length() > labels.at(labelIndex).minWidth) {
                        data = data.substr(0, labels.at(labelIndex).minWidth - 3);
                        data += "...";
                    }
                    else {
                        data += std::string(labels.at(labelIndex).minWidth - data.size(), ' ');
                    }
                    if (useFirstColor == false) {
                        wattron(renderWindow->get_window_reference(),
                                COLOR_PAIR(field->getData().color));
                    }
                    else {
                        wattron(renderWindow->get_window_reference(), COLOR_PAIR(firstColor));
                    }
                    mvwprintw(renderWindow->get_window_reference(),
                              field->getData().startCoordinate.start_y_pixel + 3,
                              field->getData().startCoordinate.start_x_pixel + 2,
                              data.c_str());
                    labelIndex++;
                    if (useFirstColor == false) {
                        wattroff(renderWindow->get_window_reference(),
                                 COLOR_PAIR(field->getData().color));
                    }
                    else {
                        wattroff(renderWindow->get_window_reference(), COLOR_PAIR(firstColor));
                    }
                }
                recordIndex++;
            }
        }
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