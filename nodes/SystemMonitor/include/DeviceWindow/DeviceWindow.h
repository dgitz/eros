#pragma once
#include <mutex>

#include "BaseWindow.h"
namespace eros_nodes::SystemMonitor {
enum class DeviceFieldColumn {
    MARKER = 0,
    ID = 1,
    NAME = 2,
    CPU = 3,
    RAM = 4,
    DISK = 5,
    LOADFACTOR = 6,
    RX = 7
};
struct DeviceData {
    DeviceData(uint16_t _id, std::string _name)
        : id(_id),
          name(_name),
          state(eros::Node::State::INITIALIZING),
          last_heartbeat_delta(0.0),
          cpu_av_perc(0.0),
          ram_av_perc(0.0),
          disk_av_perc(0.0),
          load_factor({0.0, 0.0, 0.0}) {
    }
    uint16_t id;
    bool initialized;
    std::string name;
    eros::Node::State state;
    double last_heartbeat_delta;
    double cpu_av_perc;
    double ram_av_perc;
    double disk_av_perc;
    std::vector<double> load_factor;
};
/**
 * @brief A Window to render Device Information
 *
 */
class DeviceWindow : public BaseWindow
{
   public:
    static constexpr double START_X_PERC =
        55.0; /*!< What percentage of the screen to put top left corner (X) of window. */
    static constexpr double START_Y_PERC =
        80.0; /*!< What percentage of the screen to put top left corner (Y) of window. */
    static constexpr double WIDTH_PERC =
        45.0; /*!< What percentage of the screen (Width) to draw the window. */
    static constexpr double HEIGHT_PERC =
        20.0; /*!< What percentage of the screen (Height) to draw the window. */
    DeviceWindow(ros::NodeHandle* nodeHandle,
                 std::string robot_namespace,
                 eros::Logger* logger,
                 int16_t tab_order,
                 uint16_t mainwindow_height,
                 uint16_t mainwindow_width)
        : BaseWindow("device_window",
                     tab_order,
                     START_X_PERC,
                     START_Y_PERC,
                     WIDTH_PERC,
                     HEIGHT_PERC,
                     nodeHandle,
                     robot_namespace,
                     logger,
                     mainwindow_height,
                     mainwindow_width) {
        // NO Supported Keys

        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::MARKER, Field("", 3)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::ID, Field("ID", 3)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::NAME, Field(" Device ", 25)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::CPU, Field(" CPU Av ", 8)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::RAM, Field(" RAM Av ", 8)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::DISK, Field(" DISK Av ", 8)));
        device_window_fields.insert(std::pair<DeviceFieldColumn, Field>(
            DeviceFieldColumn::LOADFACTOR, Field(" LOAD FACTOR ", 18)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::RX, Field(" Rx ", 6)));

        ScreenCoordinatePixel coord_pix = SystemMonitorUtility::convertCoordinate(
            get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);
        WINDOW* win = SystemMonitorUtility::create_newwin(coord_pix.height_pix,
                                                          coord_pix.width_pix,
                                                          coord_pix.start_y_pix,
                                                          coord_pix.start_x_pix);
        set_screen_coordinates_pix(coord_pix);
        set_window(win);
        wrefresh(win);
    }
    virtual ~DeviceWindow();
    bool is_selectable() override {
        return true;
    }
    bool update(double dt, double t_ros_time) override;
    bool new_msg(eros::ArmDisarm::State /* armed_state */) override {  // Not Used
        return true;
    }
    bool new_msg(eros::heartbeat /*heartbeat_msg*/) override {  // Not Used
        return true;
    }
    bool new_msg(eros::command_state /* command_state_msg */) override {  // Not Used
        return true;
    }
    bool new_msg(eros::resource resource_msg) override;
    bool new_msg(eros::loadfactor loadfactor_msg) override;
    KeyEventContainer new_keyevent(int /* key */) override {  // Not Used
        KeyEventContainer output;
        return output;
    }
    bool new_command(std::vector<WindowCommand> /* commands*/) override {  // Not Used
        return true;
    }
    std::string pretty();

   private:
    bool insertDevice(eros::resource resource_data);
    bool insertDevice(eros::loadfactor loadfactor_data);
    std::string get_device_info(DeviceData device, bool selected);
    bool update_window();
    std::string get_deviceheader();
    std::mutex device_list_mutex;
    std::map<DeviceFieldColumn, Field> device_window_fields;
    std::map<std::string, DeviceData> device_list;
};
}  // namespace eros_nodes::SystemMonitor