#ifndef _DIFF_DRIVE_ROBOT_HARDWARE_INTERFACE_H_
#define _DIFF_DRIVE_ROBOT_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace robot_base
{
    class robotHardwareInterface : public hardware_interface::RobotHW
    {
    public:
        robotHardwareInterface(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
        void registerControlInterface();
        void updateJointsFromHardware(const ros::Duration &period);
        void writeCommandsToHardware();
        
    private:
        void wheelLeftCallback(const std_msgs::Int32::ConstPtr& msg);
        void wheelRightCallback(const std_msgs::Int32::ConstPtr& msg);
        void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right);
        double linearToAngular(const double &travel) const;
        double angularToLinear(const double &angle) const;

    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _private_nh;
        ros::Publisher _wheel_control_publisher;
        ros::Subscriber _wheel_left_subscriber;
        ros::Subscriber _wheel_right_subscriber;
        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::VelocityJointInterface _velocity_joint_interface;
        double _wheel_diameter;
        int _wheel_encoder_pulses;
        double _max_speed;
        double _cmd[2];
        double _pos[2];
        double _vel[2];
        double _eff[2];
        int32_t ticks[2];
    };
}

#endif // _DIFF_DRIVE_ROBOT_HARDWARE_INTERFACE_H_