// required class declarations
#include <diff_drive_base2/robot_hardware_interface.h>

robotHardwareInterface::robotHardwareInterface(ros::NodeHandle& nh, ros::NodeHandle& private_nh) 
    : _nh(nh)
    , _private_nh(private_nh)
{
    // zero controller variables
    _cmd[0] = 0;
    _cmd[1] = 0;
    _pos[0] = 0;
    _pos[1] = 0;
    _vel[0] = 0;
    _vel[1] = 0;
    _eff[0] = 0;
    _eff[1] = 0;
    // zero encoder variables
    _wheel_encoder_ticks[0] = 0;
    _wheel_encoder_ticks[1] = 0;
    
    // load parameters 
    _private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.31);
    _private_nh.param<int>("wheel_encoder_pulses", _wheel_encoder_pulses, 740);
    _private_nh.param<double>("max_speed", _max_speed, 1.0);
    _private_nh.param<double>("control_frequency", _control_frequency, 10.0);

    // setup the controller interfaces for the joints
    registerControlInterface();

    controller_manager_.reset(new controller_manager::ControllerManager(this, _nh));

    // advertise topic publisher and declare subscribers
    _wheel_control_publisher = _nh.advertise<std_msgs::Float32MultiArray>("wheel_control", 10);    
    _wheel_left_subscriber = _nh.subscribe("wheel_left", 10, &robotHardwareInterface::wheelLeftCallback, this);
    _wheel_right_subscriber = _nh.subscribe("wheel_right", 10, &robotHardwareInterface::wheelRightCallback, this);

    // create a timer for updates   
    _non_realtime_loop = _nh.createTimer(ros::Duration(1.0/_control_frequency), &robotHardwareInterface::update, this);   
}

robotHardwareInterface::~robotHardwareInterface() {}

void robotHardwareInterface::registerControlInterface() {

    hardware_interface::JointStateHandle hWheelStateFL("front_left_wheel_joint", &_pos[0], &_vel[0], &_eff[0]);
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

void robotHardwareInterface::update(const ros::TimerEvent& e) {
    // read the hardware sensors and write to actuatores
    _elapsed_time = ros::Duration(e.current_real - e.last_real);
    read(_elapsed_time);
    controller_manager_->update(ros::Time::now(), _elapsed_time);
    write();
}

void robotHardwareInterface::read(const ros::Duration &period) {

    // calculate traveled distance
    double distance_left = (_wheel_encoder_ticks[0] * ((_wheel_diameter * M_PI) / _wheel_encoder_pulses));
    double distance_right = (_wheel_encoder_ticks[1] * ((_wheel_diameter * M_PI) / _wheel_encoder_pulses));

    // update joint states position
    _pos[0] += linearToAngular(distance_left);
    _pos[1] += linearToAngular(distance_right);
    
    // update joint states velocity
    _vel[0] = linearToAngular(distance_left / period.toSec());
    _vel[1] = linearToAngular(distance_right / period.toSec());
}

void robotHardwareInterface::write() {
    double diff_speed_left = angularToLinear(_cmd[0]);
    double diff_speed_right = angularToLinear(_cmd[1]);
    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    //wheel_control_msg.data.clear();
    std_msgs::Float32MultiArray wheel_control_msg;
    wheel_control_msg.data.push_back((float)diff_speed_left);
    wheel_control_msg.data.push_back((float)diff_speed_right);
    _wheel_control_publisher.publish(wheel_control_msg);
}

void robotHardwareInterface::wheelLeftCallback(const std_msgs::Int32::ConstPtr& msg)
{
    _wheel_encoder_ticks[0] = (int32_t)msg->data;
}

void robotHardwareInterface::wheelRightCallback(const std_msgs::Int32::ConstPtr& msg)
{
    _wheel_encoder_ticks[1] = (int32_t)msg->data;
}

void robotHardwareInterface::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
{
    double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
    if (speed > _max_speed)
    {
        diff_speed_left *= _max_speed / speed;
        diff_speed_right *= _max_speed / speed;
    }
}

double robotHardwareInterface::linearToAngular(const double &travel) const
{
    return travel / _wheel_diameter * 2;
}

double robotHardwareInterface::angularToLinear(const double &angle) const
{
    return angle * _wheel_diameter / 2;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_hardware_interface");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::MultiThreadedSpinner spinner(2); 
    robotHardwareInterface robot(nh, private_nh);
    spinner.spin();
    return 0;
}
