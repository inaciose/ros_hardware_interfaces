#ifndef _DIFF_DRIVE_ROBOT_HARDWARE_INTERFACE_H_
#define _DIFF_DRIVE_ROBOT_HARDWARE_INTERFACE_H_

//
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

// ros control minimal include required
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

// used by messages to topic to send encoder ticks
#include <std_msgs/Int32.h>

// used by messages to topic to send motors commands
#include <std_msgs/Float32MultiArray.h>

class robotHardwareInterface : public hardware_interface::RobotHW 
{
    public:
        // constructor & destructor
        robotHardwareInterface(ros::NodeHandle& nh, ros::NodeHandle &private_nh);
        ~robotHardwareInterface();

        // hardware_interface minimal methods
        void registerControlInterface();
        void update(const ros::TimerEvent& e);
        void read(const ros::Duration &period);
        void write();
        
    protected:
        void wheelLeftCallback(const std_msgs::Int32::ConstPtr& msg);
        void wheelRightCallback(const std_msgs::Int32::ConstPtr& msg);
        void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right);
        double linearToAngular(const double &travel) const;
        double angularToLinear(const double &angle) const;

        ros::NodeHandle _nh;
        ros::NodeHandle _private_nh;

        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::VelocityJointInterface _velocity_joint_interface;
        
        ros::Publisher _wheel_control_publisher;
        ros::Subscriber _wheel_left_subscriber;
        ros::Subscriber _wheel_right_subscriber;

        double _max_speed;
        double _wheel_diameter;
        int32_t _wheel_encoder_pulses;
        int32_t _wheel_encoder_ticks[2];

        double _cmd[2];
        double _pos[2];
        double _vel[2];
        double _eff[2];

        double _control_frequency;
        ros::Timer _non_realtime_loop;
        ros::Duration _elapsed_time;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

#endif // _DIFF_DRIVE_ROBOT_HARDWARE_INTERFACE_H_