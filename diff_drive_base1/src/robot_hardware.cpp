#include <std_msgs/Int32.h>
#include <diff_drive_base1/WheelControl.h>
#include <diff_drive_base1/robot_hardware_interface.h>

robot_base::robotHardwareInterface::robotHardwareInterface(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : _nh(nh)
    , _private_nh(private_nh)
{
    _private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.31);
    _private_nh.param<int>("wheel_encoder_pulses", _wheel_encoder_pulses, 740);
    _private_nh.param<double>("max_speed", _max_speed, 1.0);

    _wheel_control_publisher = _nh.advertise<diff_drive_base1::WheelControl>("wheel_control", 10);
    _wheel_left_subscriber = _nh.subscribe("wheel_left", 10, &robotHardwareInterface::wheelLeftCallback, this);
    _wheel_right_subscriber = _nh.subscribe("wheel_right", 10, &robotHardwareInterface::wheelRightCallback, this);

    registerControlInterface();
}

void robot_base::robotHardwareInterface::registerControlInterface()
{
    hardware_interface::JointStateHandle hWheelStateFL("front_left_wheel_joint",  &_pos[0], &_vel[0], &_eff[0]);
    hardware_interface::JointStateHandle hWheelStateFR("front_right_wheel_joint", &_pos[1], &_vel[1], &_eff[1]);
    _joint_state_interface.registerHandle(hWheelStateFL);
    _joint_state_interface.registerHandle(hWheelStateFR);

    hardware_interface::JointHandle hWheelFL(hWheelStateFL, &_cmd[0]);
    hardware_interface::JointHandle hWheelFR(hWheelStateFR, &_cmd[1]);
    _velocity_joint_interface.registerHandle(hWheelFL);
    _velocity_joint_interface.registerHandle(hWheelFR);

    registerInterface(&_joint_state_interface);
    registerInterface(&_velocity_joint_interface);
}

void robot_base::robotHardwareInterface::updateJointsFromHardware(const ros::Duration &period)
{    
    // calculate traveled distance
    double distance_left = (ticks[0] * ((_wheel_diameter * M_PI) / _wheel_encoder_pulses));
    double distance_right = (ticks[1] * ((_wheel_diameter * M_PI) / _wheel_encoder_pulses));
    
    // update joint states position
    _pos[0] += linearToAngular(distance_left);
    _pos[1] += linearToAngular(distance_right);
    // update joint states velocity
    _vel[0] = linearToAngular(distance_left / period.toSec());
    _vel[1] = linearToAngular(distance_right / period.toSec());
}

void robot_base::robotHardwareInterface::writeCommandsToHardware()
{
    double diff_speed_left = angularToLinear(_cmd[0]);
    double diff_speed_right = angularToLinear(_cmd[1]);
    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    diff_drive_base1::WheelControl wheel_control_msg;
    wheel_control_msg.left_wheel_speed = (float)diff_speed_left;
    wheel_control_msg.right_wheel_speed = (float)diff_speed_right;
    _wheel_control_publisher.publish(wheel_control_msg);
}

void robot_base::robotHardwareInterface::wheelLeftCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ticks[0] = (int32_t)msg->data;
}

void robot_base::robotHardwareInterface::wheelRightCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ticks[1] = (int32_t)msg->data;
}

void robot_base::robotHardwareInterface::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
{
    double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
    if (speed > _max_speed)
    {
        diff_speed_left *= _max_speed / speed;
        diff_speed_right *= _max_speed / speed;
    }
}

double robot_base::robotHardwareInterface::linearToAngular(const double &travel) const
{
    return travel / _wheel_diameter * 2;
}

double robot_base::robotHardwareInterface::angularToLinear(const double &angle) const
{
    return angle * _wheel_diameter / 2;
}