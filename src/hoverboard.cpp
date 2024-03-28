#include "config.h"
#include "hoverboard.h"

#include <unistd.h>
#include <fcntl.h>
#include <dynamic_reconfigure/server.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#define RPM_TO_RADS_MULTIPLIER 0.104719755

Hoverboard::Hoverboard() {
        hardware_interface::JointStateHandle left_wheel_state_handle("left_wheel",
								 &joints[0].pos.data,
								 &joints[0].vel.data,
								 &joints[0].eff.data);
    hardware_interface::JointStateHandle right_wheel_state_handle("right_wheel",
								  &joints[1].pos.data,
								  &joints[1].vel.data,
								  &joints[1].eff.data);
        joint_state_interface.registerHandle (left_wheel_state_handle);
    joint_state_interface.registerHandle (right_wheel_state_handle);
        registerInterface(&joint_state_interface);

        hardware_interface::JointHandle left_wheel_vel_handle(
        joint_state_interface.getHandle("left_wheel"),
        &joints[0].cmd.data);
    hardware_interface::JointHandle right_wheel_vel_handle(
        joint_state_interface.getHandle("right_wheel"),
        &joints[1].cmd.data);
        velocity_joint_interface.registerHandle (left_wheel_vel_handle);
    velocity_joint_interface.registerHandle (right_wheel_vel_handle);
    registerInterface(&velocity_joint_interface);

    ROS_INFO("[hoverboard_driver] get params");
    std::size_t error = 0;
#ifdef ENABLE_PID
    error += !rosparam_shortcuts::get("hoverboard_driver", paramNh, "hoverboard_velocity_controller/wheel_radius", wheel_radius);
    error += !rosparam_shortcuts::get("hoverboard_driver", paramNh, "hoverboard_velocity_controller/linear/x/max_velocity", max_velocity);
#endif
    error += !rosparam_shortcuts::get("hoverboard_driver", paramNh, "direction", direction_correction);
    rosparam_shortcuts::shutdownIfError("hoverboard_driver", error);
    ROS_INFO("[hoverboard_driver] init other");

    if (!rosparam_shortcuts::get("hoverboard_driver", paramNh, "port", port)) {
        port = DEFAULT_PORT;
        ROS_WARN("[hoverboard_driver] Port is not set in config, using default %s", port.c_str());
    } else {
        ROS_INFO("[hoverboard_driver] Using port %s", port.c_str());
    }

    // Convert m/s to rad/s
#ifdef ENABLE_PID
    max_velocity /= wheel_radius;
#endif
    low_wrap = ENCODER_LOW_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
    high_wrap = ENCODER_HIGH_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
    last_wheelcountR = last_wheelcountL = 0;
    multR = multL = 0;

#ifdef ENABLE_PID
    ros::NodeHandle nh_left(nh, "pid/left");
    ros::NodeHandle nh_right(nh, "pid/right");
    // Init PID controller
    pids[0].init(nh_left, 1.0, 0.0, 0.0, 0.01, 1.5, -1.5, true, max_velocity, -max_velocity);
    pids[0].setOutputLimits(-max_velocity, max_velocity);
    pids[1].init(nh_right, 1.0, 0.0, 0.0, 0.01, 1.5, -1.5, true, max_velocity, -max_velocity);
    pids[1].setOutputLimits(-max_velocity, max_velocity);
#endif
    openSerial();
}

Hoverboard::~Hoverboard() {
    if (serial_port.isOpen()) {
        serial_port.close();
    }
}

void Hoverboard::openSerial(){
    if(!serial_port.isOpen()){
        try {
            serial_port.setPort(port);
            serial_port.setBaudrate(115200);
            auto to = serial::Timeout::simpleTimeout(10);
            serial_port.setTimeout(to);
            serial_port.open();
        } catch (std::exception &e) {
            ROS_FATAL("[hoverboard_driver] Cannot open serial port %s to hoverboard", port.c_str());
        }
    }
}

bool Hoverboard::read(hoverboard_driver::HoverboardStateStamped& state_msg) {
    bool gotPacket = false;
    openSerial();

    if (serial_port.isOpen()) {
        unsigned char c;
        size_t r = 0;
        
        #ifdef DEBUG_BYTES_STATS
        int validBytes=0,invalidBytes=0,inProgressBytes=0;
        int totalReadCalls=0, totalReadBytes=0;
        #endif
        try {
            while(serial_port.available() > 0) {
                r = serial_port.read(&c, 1);
                #ifdef DEBUG_BYTES_STATS
                totalReadCalls++;
                totalReadBytes+=r;
                #endif
                if (r>0) {
                    if(protocol_recv(c,state_msg 
                        #ifdef DEBUG_BYTES_STATS 
                        ,validBytes,invalidBytes,inProgressBytes 
                        #endif
                        )) {
                        //mark we have packet but read stream to the end
                        gotPacket = true;
                    }
                } else {
                    break;
                }
            }
            #ifdef DEBUG_BYTES_STATS 
                ROS_INFO("[hoverboard_driver] bytes stats: valid %d, invalid %d, read bytes %d, read calls %d",validBytes,invalidBytes,totalReadBytes,totalReadCalls);
            #endif
        } catch (std::exception &e) {
            ROS_ERROR("[hoverboard_driver] Reading from serial %s failed. Closing Connection.", port.c_str());
            serial_port.close();
        }
    }

    return gotPacket;
}

bool Hoverboard::protocol_recv(char byte,hoverboard_driver::HoverboardStateStamped& state_msg
    #ifdef DEBUG_BYTES_STATS
        ,int& validBytes,int& invalidBytes,int& inProgressBytes
    #endif
    ) {
    bool returnCode = false;
    start_frame = ((uint16_t)(byte) << 8) | (uint8_t)prev_byte;

    // Read the start frame
    if (start_frame == START_FRAME) {
        p = (char*)&msg;
        *p++ = prev_byte;
        *p++ = byte;
        msg_len = 2;
        #ifdef DEBUG_BYTES_STATS
            inProgressBytes+=2;
            invalidBytes--;
        #endif
    } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
        // Otherwise just read the message content until the end
        *p++ = byte;
        msg_len++;
        #ifdef DEBUG_BYTES_STATS
            inProgressBytes++;
        #endif
    } else {
        #ifdef DEBUG_BYTES_STATS
            invalidBytes++;
        #endif
    }

    if (msg_len == sizeof(SerialFeedback)) {
        uint16_t checksum = (uint16_t)(
            msg.start ^ msg.cmd1 ^ msg.cmd2 ^ msg.speedR_meas ^ msg.speedL_meas ^
	        msg.wheelR_cnt ^ msg.wheelL_cnt ^
            msg.currR_meas ^ msg.currL_meas ^ 
            msg.motorL_temp ^ msg.motorR_temp ^ 
            msg.status ^
            msg.batVoltage ^ msg.boardTemp ^ msg.cmdLed);

        if (msg.start == START_FRAME && msg.checksum == checksum) {
            state_msg.state.cmdL = ((double)msg.cmd1) * RPM_TO_RADS_MULTIPLIER;
            state_msg.state.cmdR = ((double)msg.cmd2) * RPM_TO_RADS_MULTIPLIER;
            state_msg.state.batVoltage = ((double)msg.batVoltage)/100.0;
            state_msg.state.boardTemp  = ((double)msg.boardTemp)/10.0;
            state_msg.state.currL_meas = ((double)msg.currL_meas)/10.0;
            state_msg.state.currR_meas = ((double)msg.currR_meas)/10.0;
            state_msg.state.motorL_temp = (double)msg.motorL_temp/10.0;
            state_msg.state.motorR_temp = (double)msg.motorR_temp/10.0;
            state_msg.state.status=msg.status;

            // Convert RPM to RAD/S
            joints[0].vel.data = direction_correction * ((double)msg.speedL_meas) * RPM_TO_RADS_MULTIPLIER;
            joints[1].vel.data = direction_correction * ((double)msg.speedR_meas) * RPM_TO_RADS_MULTIPLIER;
            //why we need abs? we loose direction
            //joints[0].vel.data = direction_correction * (abs(msg.speedL_meas) * RPM_TO_RADS_MULTIPLIER);
            //joints[1].vel.data = direction_correction * (abs(msg.speedR_meas) * RPM_TO_RADS_MULTIPLIER);
            state_msg.state.speedL_meas=joints[0].vel.data;
            state_msg.state.speedR_meas=joints[1].vel.data;

            // Process encoder values and update odometry
            on_encoder_update (msg.wheelR_cnt, msg.wheelL_cnt,state_msg);
            
            //It is important to update stamp here because on_encoder_update uses previous stamp to reset wheel ticks
            #ifdef DEBUG_BYTES_STATS
                validBytes+=inProgressBytes;
            #endif
            returnCode = true;
        } else {
            #ifdef DEBUG_BYTES_STATS
                invalidBytes+=inProgressBytes;
            #endif
            ROS_WARN("[hoverboard_driver] Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
        }
        #ifdef DEBUG_BYTES_STATS
            inProgressBytes=0;
        #endif
        msg_len = 0;
    }
    prev_byte = byte;
    return returnCode;
}

void Hoverboard::write(const ros::Time& time, const ros::Duration& period) {
    if (!serial_port.isOpen()) {
        ROS_ERROR("[hoverboard_driver] Attempt to write on closed serial");
        return;
    }

#ifdef ENABLE_PID
    double pid_outputs[2];
    pid_outputs[0] = pids[0](joints[0].vel.data, joints[0].cmd.data, period);
    pid_outputs[1] = pids[1](joints[1].vel.data, joints[1].cmd.data, period);
#endif
    // Convert PID outputs in RAD/S to RPM
    double set_speed[2] = {
#ifdef ENABLE_PID
        pid_outputs[0] / RPM_TO_RADS_MULTIPLIER,
        pid_outputs[1] / RPM_TO_RADS_MULTIPLIER
#else
        joints[0].cmd.data / RPM_TO_RADS_MULTIPLIER,
        joints[1].cmd.data / RPM_TO_RADS_MULTIPLIER
#endif        
    };

    // Calculate steering from difference of left and right
    //const double speed = (set_speed[0] + set_speed[1])/2.0;
    //const double steer = (set_speed[0] - speed)*2.0;

    SerialCommand command;
    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t)set_speed[0];//steer
    command.speed = (int16_t)set_speed[1];//speed
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    try {
        size_t rc = serial_port.write((const uint8_t*)&command, sizeof(command));
        if(rc != sizeof(command)) {
            ROS_ERROR("[hoverboard_driver] Actual bytes wrote don't match requested");
        }
    } catch (std::exception &e) {
        ROS_ERROR("[hoverboard_driver] Error writing to hoverboard serial port");
    }
}

void Hoverboard::on_encoder_update (int16_t right, int16_t left,hoverboard_driver::HoverboardStateStamped& state_msg) {
    double posL = 0.0, posR = 0.0;

    // Calculate wheel position in ticks, factoring in encoder wraps
    if (right < low_wrap && last_wheelcountR > high_wrap)
        multR++;
    else if (right > high_wrap && last_wheelcountR < low_wrap)
        multR--;
    posR = right + multR*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountR = right;

    if (left < low_wrap && last_wheelcountL > high_wrap)
        multL++;
    else if (left > high_wrap && last_wheelcountL < low_wrap)
        multL--;
    posL = left + multL*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountL = left;

    // When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddently lost
    // This section accumulates ticks even if board shuts down and is restarted   
    static double lastPosL = 0.0, lastPosR = 0.0;
    static double lastPubPosL = 0.0, lastPubPosR = 0.0;
    static bool nodeStartFlag = true;
    
    //IF there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restard
    //(the board seems to often report 1-3 ticks on startup instead of zero)
    //reset the last read ticks to the startup values
    if((ros::Time::now() - state_msg.header.stamp).toSec() > 0.2
		&& abs(posL) < 5 && abs(posR) < 5){
            lastPosL = posL;
            lastPosR = posR;
	}
    double posLDiff = 0;
    double posRDiff = 0;

    //if node is just starting keep odom at zeros
	if(nodeStartFlag){
		nodeStartFlag = false;
	}else{
            posLDiff = posL - lastPosL;
            posRDiff = posR - lastPosR;
	}

    lastPubPosL += posLDiff;
    lastPubPosR += posRDiff;
    lastPosL = posL;
    lastPosR = posR;
    
    // Convert position in accumulated ticks to position in radians
    joints[0].pos.data = 2.0*M_PI * lastPubPosL/(double)TICKS_PER_ROTATION;
    joints[1].pos.data = 2.0*M_PI * lastPubPosR/(double)TICKS_PER_ROTATION;
    state_msg.state.wheelL_cnt=joints[0].pos.data;
    state_msg.state.wheelR_cnt=joints[1].pos.data;
}
