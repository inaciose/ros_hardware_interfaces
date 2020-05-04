#include <robotpos1/robot_hardware_interface.h>

robotHardwareInterface::robotHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

    pub = nh_.advertise<robotpos1::Floats>("/joints_to_arduino",10);
    client = nh_.serviceClient<robotpos1::Floats_array>("/read_joint_state");

    non_realtime_loop_ = nh_.createTimer(update_freq, &robotHardwareInterface::update, this);
}

robotHardwareInterface::~robotHardwareInterface() {}

void robotHardwareInterface::init() {
    num_joints_ = 1;
    joint_names_[0] = "joint_1";

    for (int i = 0; i < num_joints_; ++i) {
        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        position_joint_interface_.registerHandle(jointPositionHandle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
}

void robotHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void robotHardwareInterface::read() {
    joint_read.request.req = 1.0;

    if(client.call(joint_read)) {
        joint_position_[0]=angles::from_degrees(90-joint_read.response.res[0]);
        //ROS_INFO("Receiving  j1: %.2f", joint_read.response.res[0]);
    } else {
    	joint_position_[0] = 0;
        //ROS_INFO("Service not found ");
    }
}

void robotHardwareInterface::write(ros::Duration elapsed_time) {

    joints_pub.data.clear();
    joints_pub.data.push_back(90-(angles::to_degrees(joint_position_command_[0])));
    joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[1])));
    joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[2])));
    //ROS_INFO("Publishing j1: %.2f", joints_pub.data[0]);
    pub.publish(joints_pub);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_hardware_interface");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2);// 2 threads for controller service and for the Service client used to get the feedback from ardiuno

    robotHardwareInterface robot(nh);
    spinner.spin();

    return 0;
}
