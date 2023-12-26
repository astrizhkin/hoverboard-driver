#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <dynamic_reconfigure/server.h>
#include "hoverboard.h"
#include <hoverboard_driver/HoverboardStateStamped.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "hoverboard_driver");
    ROS_INFO("[hoverboard_driver] Starting");

    Hoverboard hoverboard;
    controller_manager::ControllerManager cm(&hoverboard);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(50.0);

    ros::Publisher state_pub = hoverboard.paramNh.advertise<hoverboard_driver::HoverboardStateStamped>("state", 3);
    hoverboard_driver::HoverboardStateStamped state_msg;
    //state_msg.header.frame_id = hoverboard.paramNh.getNamespace();
    while (ros::ok()) {
        if(hoverboard.read(state_msg)) {
            state_msg.state.connection_state = hoverboard_driver::HoverboardState::HOVERBOARD_CONNECTION_STATE_CONNECTED;
            //check connection timout on hoverboard side
            if(state_msg.state.status & hoverboard_driver::HoverboardState::STATUS_CONN_TIMEOUT > 0) {
                state_msg.state.connection_state = hoverboard_driver::HoverboardState::HOVERBOARD_CONNECTION_STATE_DISCONNECTED;
            }
            state_msg.header.stamp = ros::Time::now();
            state_msg.header.seq++;
            state_pub.publish(state_msg);
        } else {
            if ((ros::Time::now() - state_msg.header.stamp).toSec() > 1) {
                ROS_ERROR("[hoverboard_driver] No serial %s valid data available for a long period", hoverboard.port.c_str());
                //reset all feild values
                state_msg.state = hoverboard_driver::HoverboardState{};
                //publish DISCONNECTED when not receiving serial data
                state_msg.state.connection_state = hoverboard_driver::HoverboardState::HOVERBOARD_CONNECTION_STATE_DISCONNECTED;
                state_msg.header.stamp = ros::Time::now();
                state_msg.header.seq++;
                state_pub.publish(state_msg);
            }
        }

        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;
        cm.update(time, period);
        hoverboard.write(time, period);

        rate.sleep();
    }
    spinner.stop();
    ROS_INFO("[hoverboard_driver] Exiting");

    return 0;
}
