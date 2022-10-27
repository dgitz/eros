#ifndef RENDERENGINE_H
#define RENDERENGINE_H
#include <curses.h>
#include <eros/Logger.h>
#include <eros/SystemMonitor/DataStructures.h>
#include <eros/SystemMonitor/RenderEngine/RenderWindow.h>
#include <eros/SystemMonitor/Window/IWindow.h>

#include <map>
#include <string>
namespace eros {

class RenderEngine
{
    struct Window {
        Window(IWindow* windowData, RenderWindow* windowRender)
            : windowData(windowData), windowRender(windowRender) {
        }
        IWindow* windowData;
        RenderWindow* windowRender;
    };

   public:
    RenderEngine(eros::Logger* logger, std::map<IWindow::WindowType, IWindow*> dataWindows)
        : logger(logger), dataWindows(dataWindows), killMe(false) {
    }
    virtual ~RenderEngine() {
        std::map<IWindow::WindowType, Window>::iterator win_it = windows.begin();
        while (win_it != windows.end()) {
            delwin(win_it->second.windowRender->get_window_reference());
            ++win_it;
        }

        endwin();
    }
    bool initScreen();
    bool update(double dt, std::map<IWindow::WindowType, IWindow*> windows);
    // No practical way to unit test
    // LCOV_EXCL_START
    bool shouldKill() {
        return killMe;
    }
    // LCOV_EXCL_STOP
    bool incrementFocus();
    bool renderWindow(IWindow* windowData, RenderWindow* renderWindow);
    std::map<IWindow::WindowType, Window> getWindows() {
        return windows;
    }

   private:
    eros::Logger* logger;
    std::map<IWindow::WindowType, IWindow*> dataWindows;
    std::map<IWindow::WindowType, Window> windows;
    bool killMe;
};
}  // namespace eros
#endif