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
    enum class KeyMap { KEY_q = 113, KEY_Q = 81, KEY_TAB = 9 };
    struct Window {
        Window(IWindow* windowData, RenderWindow* windowRender)
            : windowData(windowData), windowRender(windowRender) {
        }
        IWindow* windowData;
        RenderWindow* windowRender;
    };

   public:
    RenderEngine(eros::Logger* logger, std::map<IWindow::WindowType, IWindow*> windowData)
        : logger(logger), dataWindows(windowData), killMe(false) {
    }
    virtual ~RenderEngine() {
        endwin();
    }
    bool initScreen();
    bool update(double dt, std::map<IWindow::WindowType, IWindow*> windows);
    bool shouldKill() {
        return killMe;
    }

   private:
    bool renderWindow(IWindow* windowData, RenderWindow* renderWindow);

   private:
    void incrementFocus();
    eros::Logger* logger;
    std::map<IWindow::WindowType, IWindow*> dataWindows;
    std::map<IWindow::WindowType, RenderWindow*> renderWindows;
    std::map<IWindow::WindowType, Window> windows;
    bool killMe;
};
}  // namespace eros
#endif