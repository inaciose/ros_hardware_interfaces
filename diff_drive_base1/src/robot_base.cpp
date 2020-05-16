#include <chrono>
#include <functional>
#include <ros/callback_queue.h>
#include <diff_drive_base1/robot_hardware_interface.h>

void controlLoop(robot_base::robotHardwareInterface &hw, controller_manager::ControllerManager &cm, std::chrono::system_clock::time_point &last_time)
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw.updateJointsFromHardware(elapsed);
    cm.update(ros::Time::now(), elapsed);
    hw.writeCommandsToHardware();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_base");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double control_frequency;
    private_nh.param<double>("control_frequency", control_frequency, 10.0);

    robot_base::robotHardwareInterface hw(nh, private_nh);
    controller_manager::ControllerManager cm(&hw, nh);

    ros::CallbackQueue robot_queue;
    ros::AsyncSpinner robot_spinner(1, &robot_queue);

    std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();
    ros::TimerOptions control_timer(ros::Duration(1 / control_frequency), std::bind(controlLoop, std::ref(hw), std::ref(cm), std::ref(last_time)), &robot_queue);
    ros::Timer control_loop = nh.createTimer(control_timer);

    robot_spinner.start();
    ros::spin();

    return 0;
}