#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <serial/serial.h>
#include <string>
#include "hoverboard_driver/HoverboardConfig.h"
#include "hoverboard_driver/pid.h"
#include <hoverboard_driver/HoverboardStateStamped.h>
#include "protocol.h"

// #define DEBUG_BYTES_STATS
// #define ENABLE_PID

class HoverboardAPI;

class Hoverboard : public hardware_interface::RobotHW {
public:
    Hoverboard();
    ~Hoverboard();
    
    bool read(hoverboard_driver::HoverboardStateStamped& state_msg);
    void write(const ros::Time& time, const ros::Duration& period);
    void tick();

    ros::NodeHandle nh;
    ros::NodeHandle paramNh = ros::NodeHandle("~");
    std::string port;

 private:
    bool protocol_recv (char c,hoverboard_driver::HoverboardStateStamped& state_msg
        #ifdef DEBUG_BYTES_STATS
            , int& validBytes,int& invalidBytes,int& inProgressBytes
        #endif
    );
    void on_encoder_update (int16_t right, int16_t left,hoverboard_driver::HoverboardStateStamped& state_msg);
    void openSerial();
 
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    // The units for wheels are radians (pos), radians per second (vel,cmd), and Netwton metres (eff)
    struct Joint {
        std_msgs::Float64 pos;
        std_msgs::Float64 vel;
        std_msgs::Float64 eff;
        std_msgs::Float64 cmd;
    } joints[2];

#ifdef ENABLE_PID
    double wheel_radius;
    double max_velocity = 0.0;
#endif
    int direction_correction = 1;

    // Last known encoder values
    int16_t last_wheelcountR;
    int16_t last_wheelcountL;
    // Count of full encoder wraps
    int multR;
    int multL;
    // Thresholds for calculating the wrap
    int low_wrap;
    int high_wrap;

    // Hoverboard protocol
    serial::Serial serial_port;
    int msg_len = 0;
    char prev_byte = 0;
    uint16_t start_frame = 0;
    char* p;
    SerialFeedback msg, prev_msg;

#ifdef ENABLE_PID
    PID pids[2];
#endif
};
