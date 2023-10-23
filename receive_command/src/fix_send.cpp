#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include <termio.h>
#include <stdio.h>
#include <unistd.h>

#include "robot_control_msgs/msg/pose.hpp"


using std::placeholders::_1;
using namespace std;


class ReceiveCommand : public rclcpp::Node
{
public:

    rclcpp::Publisher<robot_control_msgs::msg::Pose>::SharedPtr pose_pub;

    int scanKeyboard();
    void getKeyboardInput();
    void timer_callback();


    rclcpp::TimerBase::SharedPtr timer_;
    int rate=1000;

    ReceiveCommand()
    : Node("receive_command")
    {
        pose_pub = this -> create_publisher<robot_control_msgs::msg::Pose>("/arm_1/pose",10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/rate), std::bind(&ReceiveCommand::timer_callback, this));
    }
    
};



void ReceiveCommand::timer_callback()
{

    robot_control_msgs::msg::Pose control_value;

    control_value.qx = 0;
    control_value.qy = 0;
    control_value.qz = 0;
    control_value.qw = 1;

    control_value.dx = -0.02;
    pose_pub -> publish(control_value);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReceiveCommand>());
    rclcpp::shutdown();
    return 0;
}
