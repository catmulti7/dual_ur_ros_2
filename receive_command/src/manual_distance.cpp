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

    rclcpp::Publisher<robot_control_msgs::msg::Pose>::SharedPtr pose_pub_0;
    rclcpp::Publisher<robot_control_msgs::msg::Pose>::SharedPtr pose_pub_1;

    int scanKeyboard();
    void getKeyboardInput();
    void timer_callback();

    float dx, dy, dz;
    int arm;
    string confirm;


    rclcpp::TimerBase::SharedPtr timer_;
    int rate=1;

    ReceiveCommand()
    : Node("receive_command")
    {
        pose_pub_0 = this -> create_publisher<robot_control_msgs::msg::Pose>("/arm_0/pose",10);
        pose_pub_1 = this -> create_publisher<robot_control_msgs::msg::Pose>("/arm_1/pose",10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/rate), std::bind(&ReceiveCommand::timer_callback, this));
    }
    
};



void ReceiveCommand::timer_callback()
{
    cout << "Arm which recevice command, 0 for right, 1 for left"<<endl;
    cin >> arm;
    if(arm != 0 && arm != 1)
    {
        cout << "invaild input!" <<endl;;
        return;
    }
    cout << "distance increment in x-axis (m)" << endl;
    cin >> dx;
    cout << "distance increment in y-axis (m)" << endl;
    cin >> dy;
    cout << "distance increment in z-axis (m)" << endl;
    cin >> dz;
    cout << "x: "<< dx <<" y: "<< dy <<" z: "<< dz << endl;
    cout << "press y for confirm, n for abort" << endl;
    cin >> confirm;
    if(confirm == "y")
    {
        confirm = "n";
        robot_control_msgs::msg::Pose control_value;
        control_value.qx = 0;
        control_value.qy = 0;
        control_value.qz = 0;
        control_value.qw = 1;
        control_value.dx = dx;
        control_value.dy = dy;
        control_value.dz = dz;
        if(arm == 0)
            pose_pub_0 -> publish(control_value);
        else
            pose_pub_1 -> publish(control_value);
    }
    
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReceiveCommand>());
    rclcpp::shutdown();
    return 0;
}
