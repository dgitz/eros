#include <eros/SystemMonitor/RenderEngine/RenderEngine.h>
namespace eros {
WINDOW *create_newwin(int height, int width, int starty, int startx) {
    WINDOW *local_win;

    local_win = newwin(height, width, starty, startx);
    box(local_win, 0, 0); /* 0, 0 gives default characters
                           * for the vertical and horizontal
                           * lines			*/
    wrefresh(local_win);  /* Show that box 		*/

    return local_win;
}
bool RenderEngine::initScreen() {
    // setlocale(LC_ALL, "");
    mousemask(ALL_MOUSE_EVENTS, NULL);
    initscr();
    timeout(1);
    clear();
    if (has_colors() == FALSE) {
        endwin();
        return false;
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
    for (auto window : dataWindows) {
        RenderWindow *renderWindow = new RenderWindow(logger, window.second->getWindowSize());
        Window win(window.second, renderWindow);
        windows.insert(std::pair<std::string, Window>(window.first, win));
        // renderWindows.insert(std::pair<std::string, RenderWindow *>(window.first, renderWindow));
    }

    /*bool status = process->set_mainwindow(mainwindow_width, mainwindow_height);
    if (status == false) {
        logger->enable_consoleprint();
        logger->log_error("Window: Width: " + std::to_string(mainwindow_width) + " Height: " +
                          std::to_string(mainwindow_height) + " is too small. Exiting.");
        return false;
    }
    status = process->initialize_windows();
    if (status == false) {
        return false;
    }
    */
    return true;
}
bool RenderEngine::update(double dt, std::map<std::string, IWindow *> _windows) {
    KeyMap keyPressed = (KeyMap)getch();
    switch (keyPressed) {
        case KeyMap::KEY_Q: killMe = true; break;
        case KeyMap::KEY_q: killMe = true; break;
        default: break;
    }

    for (std::map<std::string, Window>::iterator it = windows.begin(); it != windows.end(); it++) {
        renderWindow(it->second.windowData, it->second.windowRender);
        // wprintw(it->second->get_window_reference(), 2, 1, "Dumb");
        //  box(it->second->get_window_reference(), 0, 0);
        //  wrefresh();
        //   renderWindow(/*it->second->windowData*/ it->second);
        //    box(it->secondwindowRender->get_window_reference(), 0, 0);
        // refresh();
        // wrefresh(it->second->get_window_reference());
    }
    return true;
}
bool RenderEngine::renderWindow(IWindow *windowData, RenderWindow *renderWindow) {
    mvwprintw(renderWindow->get_window_reference(), 5, 5, windowData->getData().c_str());
    box(renderWindow->get_window_reference(), 0, 0);
    wrefresh(renderWindow->get_window_reference());
    return true;
}
}  // namespace eros