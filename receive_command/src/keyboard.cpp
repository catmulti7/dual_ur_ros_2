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
    int rate=50;

    ReceiveCommand()
    : Node("receive_command")
    {
        pose_pub = this -> create_publisher<robot_control_msgs::msg::Pose>("/pose",10);
        timer_ = this->create_wall_timer(
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
    robot_control_msgs::msg::Pose control_value;

    control_value.qx = 0;
    control_value.qy = 0;
    control_value.qz = 0;
    control_value.qw = 1;

    if(key==119)//W
    {
        control_value.dx = 0.02;
    }
    else if(key == 115)//S
    {
        control_value.dx = -0.02;
    }
    else if(key == 97)//A
    {
        control_value.dy = 0.02;
    }
    else if(key == 100)//D
    {
        control_value.dy = -0.02;
    }
    else if(key == 113)//Q
    {
        control_value.dz = 0.02;
    }
    else if(key == 101)//E
    {
        control_value.dz = -0.02;
    }
    else
    {
        
        cout<<"use a valid control key!"<<endl;
        return;
    }

    pose_pub -> publish(control_value);
    return;

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
