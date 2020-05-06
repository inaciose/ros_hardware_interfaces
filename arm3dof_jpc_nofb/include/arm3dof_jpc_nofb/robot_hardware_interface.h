//
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

// ros control minimal include required 
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

// ros control joint_limits_interface
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// used by messages to topic to send servo commands
#include <std_msgs/UInt16MultiArray.h>
// used to convert degrees and radian
#include <angles/angles.h>

class robotHardwareInterface : public hardware_interface::RobotHW 
{
    public:
        // constructor & destructor
        robotHardwareInterface(ros::NodeHandle& nh);
        ~robotHardwareInterface();
        
        // hardware_interface minimal methods
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        
        ros::Publisher pub;
        std_msgs::UInt16MultiArray joints_pub;

    protected:
        ros::NodeHandle nh_;

        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
        //joint_limits_interface::PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;

        int num_joints_;
        std::string joint_names_[4];
        double joint_position_[4];
        double joint_velocity_[4];
        double joint_effort_[4];
        double joint_position_command_[4];

        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};
