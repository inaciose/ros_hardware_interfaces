#include <arm3dof_jpc_nofb/robot_hardware_interface.h>

robotHardwareInterface::robotHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    // setup the controller interfaces for the joints
    init();

    // reset controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

    // advertise topic publisher used to write to actuators mcu
    pub = nh_.advertise<std_msgs::UInt16MultiArray>("/motors", 10);

    // create a timer for updates
    loop_hz_ = 10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    non_realtime_loop_ = nh_.createTimer(update_freq, &robotHardwareInterface::update, this);
}

robotHardwareInterface::~robotHardwareInterface() {}

void robotHardwareInterface::init() {
    num_joints_ = 4;
    joint_names_[0] = "joint_1";
    joint_names_[1] = "joint_2";
    joint_names_[2] = "joint_3";
    joint_names_[3] = "joint_4";

    for (int i = 0; i < num_joints_; ++i) {
        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
        
        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        position_joint_interface_.registerHandle(jointPositionHandle);

        // Create joint limits interface from the ros parameter server
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits);
        joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle(jointPositionHandle, limits);
        position_joint_saturation_interface_.registerHandle(jointLimitsHandle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    registerInterface(&position_joint_saturation_interface_);
}

void robotHardwareInterface::update(const ros::TimerEvent& e) {
    // read the hardware sensors and write to actuatores
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void robotHardwareInterface::read() {
    // fake servo feedback, just use desired position
    joint_position_[0] = joint_position_command_[0];
    joint_position_[1] = joint_position_command_[1];
    joint_position_[2] = joint_position_command_[2];
    joint_position_[3] = joint_position_command_[3];
}

void robotHardwareInterface::write(ros::Duration elapsed_time) {

    // apply limits loaded on init()
    position_joint_saturation_interface_.enforceLimits(elapsed_time); 

    // publish the received values for joint_position_command
    joints_pub.data.clear();
    joints_pub.data.push_back(90-(angles::to_degrees(joint_position_command_[0])));
    joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[1])));
    joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[2])));
    joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[3])));
    pub.publish(joints_pub);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_hardware_interface");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2); 
    robotHardwareInterface robot(nh);
    spinner.spin();
    return 0;
}
