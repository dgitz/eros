#pragma once
namespace eros_nodes::SystemMonitor {
const double COMMTIMEOUT_THRESHOLD = 5.0f;
enum class WindowCommandType {
    UNKNOWN = 0,
    VIEW_DIAGNOSTICS_NODE = 1,
    VIEW_DIAGNOSTICS_SYSTEM = 2,
    END_OF_LIST = 3
};
struct WindowCommand {
    WindowCommand() : type(WindowCommandType::UNKNOWN) {
    }
    WindowCommandType type;
    std::string option;
};
/*! \struct ScreenCoordinatePerc
    \brief ScreenCoordinatePerc container.
    */
struct ScreenCoordinatePerc {
    ScreenCoordinatePerc(double start_x, double start_y, double width, double height)
        : start_x_perc(start_x), start_y_perc(start_y), width_perc(width), height_perc(height) {
    }
    double start_x_perc;
    double start_y_perc;
    double width_perc;
    double height_perc;
};  // namespace eros_nodes::SystemMonitorstructScreenCoordinatePerc
/*! \struct ScreenCoordinatePixel
\brief ScreenCoordinatePixel container.
*/
struct ScreenCoordinatePixel {
    ScreenCoordinatePixel(double start_x, double start_y, double width, double height)
        : start_x_pix(start_x), start_y_pix(start_y), width_pix(width), height_pix(height) {
    }
    uint16_t start_x_pix;
    uint16_t start_y_pix;
    uint16_t width_pix;
    uint16_t height_pix;
};
enum class Color {
    UNKNOWN = 0,
    NO_COLOR = 1,
    RED_COLOR = 2,
    YELLOW_COLOR = 3,
    GREEN_COLOR = 4,
    BLUE_COLOR = 5,
    GRAY_COLOR = 6,
    PURPLE_COLOR = 7,
    END_OF_LIST = 8
};
/*! \struct Field
    \brief Field container, used for holding Field attributes.
    */
struct Field {
    Field(std::string _text, uint16_t _width) : text(_text), width(_width) {
    }
    std::string text;
    std::size_t width;
};

struct MessageText {
    MessageText(std::string text, eros::Level::Type level) : text(text), level(level) {
    }
    MessageText() : text(""), level(eros::Level::Type::DEBUG) {
    }
    std::string text;
    eros::Level::Type level;
};
struct KeyEventContainer {
    WindowCommand command;
    MessageText message;
};
// Keys
static constexpr int KEY_q = 113;
static constexpr int KEY_Q = 81;
static constexpr int KEY_s = 83;
static constexpr int KEY_S = 115;
static constexpr int KEY_c = 99;
static constexpr int KEY_C = 67;
static constexpr int KEY_f = 102;
static constexpr int KEY_F = 70;
static constexpr int KEY_g = 103;
static constexpr int KEY_G = 71;
static constexpr int KEY_l = 108;
static constexpr int KEY_L = 76;
static constexpr int KEY_d = 100;
static constexpr int KEY_D = 68;
static constexpr int KEY_r = 114;
static constexpr int KEY_R = 82;
static constexpr int KEY_p = 112;
static constexpr int KEY_P = 80;
static constexpr int KEY_m = 109;
static constexpr int KEY_M = 77;
static constexpr int KEY_n = 110;
static constexpr int KEY_N = 78;

static constexpr int KEY_1 = 49;
static constexpr int KEY_2 = 50;
static constexpr int KEY_3 = 51;
static constexpr int KEY_4 = 52;
static constexpr int KEY_5 = 53;
static constexpr int KEY_6 = 54;
static constexpr int KEY_7 = 55;
static constexpr int KEY_8 = 56;
static constexpr int KEY_9 = 57;

static constexpr int KEY_tab = 9;
static constexpr int KEY_space = 32;
}  // namespace eros_nodes::SystemMonitor