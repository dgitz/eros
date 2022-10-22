#ifndef RENDERENGINE_H
#define RENDERENGINE_H
#include <curses.h>
#include <eros/Logger.h>
#include <eros/SystemMonitor/DataStructures.h>
#include <eros/SystemMonitor/Window/IWindow.h>

#include <map>
#include <string>
namespace eros {
WINDOW* create_newwin(int height, int width, int starty, int startx);
class RenderWindow
{
   public:
    RenderWindow(eros::Logger* logger, WindowSize size) : logger(logger) {
        logger->log_notice("A");
        WINDOW* win = create_newwin(10, 10, 1, 1);
        logger->log_notice("B");
        set_window_reference(win);
    }
    virtual ~RenderWindow() {
        if (window_reference != nullptr) {
            delete window_reference;
            window_reference = nullptr;
        }
    }
    void set_window_reference(WINDOW* win) {
        window_reference = win;
    }
    WINDOW* get_window_reference() {
        return window_reference;
    }

   private:
    eros::Logger* logger;
    WINDOW* window_reference;
};
class RenderEngine
{
    enum class KeyMap { KEY_q = 113, KEY_Q = 81 };
    struct Window {
        Window(IWindow* windowData, RenderWindow* windowRender)
            : windowData(windowData), windowRender(windowRender) {
        }
        IWindow* windowData;
        RenderWindow* windowRender;
    };

   public:
    RenderEngine(eros::Logger* logger, std::map<std::string, IWindow*> windowData)
        : logger(logger), dataWindows(windowData), killMe(false) {
    }
    virtual ~RenderEngine() {
        endwin();
    }
    bool initScreen();
    bool update(double dt, std::map<std::string, IWindow*> windows);
    bool shouldKill() {
        return killMe;
    }

   private:
    bool renderWindow(IWindow* windowData, RenderWindow* renderWindow);

   private:
    eros::Logger* logger;
    std::map<std::string, IWindow*> dataWindows;
    std::map<std::string, RenderWindow*> renderWindows;
    std::map<std::string, Window> windows;
    bool killMe;
};
}  // namespace eros
#endif