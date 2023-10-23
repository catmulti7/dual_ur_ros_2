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

    rclcpp::Publisher<robot_control_msgs::msg::Pose>::SharedPtr control_command_left;
    rclcpp::Publisher<robot_control_msgs::msg::Pose>::SharedPtr control_command_right;

    int scanKeyboard();
    void getKeyboardInput();
    void timer_callback();


    rclcpp::TimerBase::SharedPtr timer_;
    int rate=50;

    ReceiveCommand()
    : Node("receive_command")
    {
        control_command_left = this -> create_publisher<robot_control_msgs::msg::Pose>("/arm_1/pose",10);
        control_command_right = this -> create_publisher<robot_control_msgs::msg::Pose>("/arm_0/pose",10);
        timer_ = this -> create_wall_timer(
            std::chrono::milliseconds(1000/rate), std::bind(&ReceiveCommand::timer_callback, this));
    }
    
};

int ReceiveCommand::scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
    new_settings = stored_settings;           //
    new_settings.c_lflag &= (~ICANON);        //
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO,TCSANOW,&new_settings); //

    in = getchar();

    tcsetattr(STDIN_FILENO,TCSANOW,&stored_settings);
    return in;
}

void ReceiveCommand::getKeyboardInput()
{
    int key = scanKeyboard();
    std::cout<<"key is"<<key<<std::endl;
    robot_control_msgs::msg::Pose control_value_left;
    robot_control_msgs::msg::Pose control_value_right;

    control_value_left.qx = 0;
    control_value_left.qy = 0;
    control_value_left.qz = 0;
    control_value_left.qw = 1;

    control_value_right.qx = 0;
    control_value_right.qy = 0;
    control_value_right.qz = 0;
    control_value_right.qw = 1;

    //left arm
    if(key == 119)//W
    {
        control_value_left.dx = 0.02;
        control_command_left -> publish(control_value_left);
        return;
    }
    else if(key == 115)//S
    {
        control_value_left.dx = -0.02;
        control_command_left -> publish(control_value_left);
        return;
    }
    else if(key == 97)//A
    {
        control_value_left.dy = 0.02;
        control_command_left -> publish(control_value_left);
        return;
    }
    else if(key == 100)//D
    {
        control_value_left.dy = -0.02;
        control_command_left -> publish(control_value_left);
        return;
    }
    else if(key == 113)//Q
    {
        control_value_left.dz = 0.02;
        control_command_left -> publish(control_value_left);
        return;
    }
    else if(key == 101)//E
    {
        control_value_left.dz = -0.02;
        control_command_left -> publish(control_value_left);
        return;
    }


    //right arm
    else if(key==106)//J
    {
        control_value_right.dy = 0.02;
        control_command_right -> publish(control_value_right);
        return;
    }
    else if(key == 105)//I
    {
        control_value_right.dx = 0.02;
        control_command_right -> publish(control_value_right);
        return;
    }
    else if(key == 107)//K
    {
        control_value_right.dx = -0.02;
        control_command_right -> publish(control_value_right);
        return;
    }
    else if(key == 108)//L
    {
        control_value_right.dy = -0.02;
        control_command_right -> publish(control_value_right);
        return;
    }
    else if(key == 117)//U
    {
        control_value_right.dz = 0.02;
        control_command_right -> publish(control_value_right);
        return;
    }
    else if(key == 111)//O
    {
        control_value_right.dz = -0.02;
        control_command_right -> publish(control_value_right);
        return;
    }
    else
    {
        cout<<"use a valid control key!"<<endl;
        return;
    }


}


void ReceiveCommand::timer_callback()
{
    getKeyboardInput();
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReceiveCommand>());
    rclcpp::shutdown();
    return 0;
}
