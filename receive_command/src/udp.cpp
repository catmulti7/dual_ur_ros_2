#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <thread>
#include <atomic>
#include <std_msgs/msg/char.hpp>

#include "robot_control_msgs/msg/pose.hpp"
#include "robot_control_msgs/msg/moblie.hpp"
#include "robot_control_msgs/msg/joint.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include <fstream>

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;
using std::string;
using namespace std;
typedef boost::shared_ptr<udp::socket> socket_ptr;

ofstream joint_file;

robot_control_msgs::msg::Pose pose_control;
rclcpp::Publisher<robot_control_msgs::msg::Pose>::SharedPtr control_command_0;
rclcpp::Publisher<robot_control_msgs::msg::Pose>::SharedPtr control_command_1;
atomic<bool> pgmEnd(false);

struct receiveInfo
{
    uint16_t header1;//0XFA04
    // uint8_t header2;
    uint16_t length;
    short dX;
    short dY;
    short dZ;
    short qw;
    short qx;
    short qy;
    //short qz;
    uint8_t lb_desire;
    uint8_t status; 
    uint8_t tool;
    uint8_t mission;
    uint8_t op_status;
    uint16_t tail;//0XFFFF
}__attribute__((packed)) desire_pose;

struct sendInfo
{
    uint16_t header1;//0XEA04
    uint16_t length;
    short elbow_joint_0;
    short shoulder_lift_joint_0;
    short shoulder_pan_joint_0;
    short wrist_1_joint_0;
    short wrist_2_joint_0;
    short wrist_3_joint_0;
    short elbow_joint_1;
    short shoulder_lift_joint_1;
    short shoulder_pan_joint_1;
    short wrist_1_joint_1;
    short wrist_2_joint_1;
    short wrist_3_joint_1;
    short dx;
    short dw;
    uint16_t tail;//0XFFFF
}__attribute__((packed));

uint16_t receiveCRC16(char *data)
{
    int len = sizeof (receiveInfo) - 2; // 需要校验的数据长度
    uint16_t wcrc=0XFFFF;
    uint8_t temp;
    int i=0,j=0;
    for(i=0;i<len;i++)  // 需要校验的数据位置
    {
        temp=data[i];
        wcrc^=temp;
        for(j=0; j<8; j++)
        {
            if(wcrc & 0X0001)
            {
                wcrc >>= 1;
                wcrc^= 0XA001;
            }
            else
                wcrc >>= 1;
        }
    }
    temp=wcrc;
    return wcrc;
}

uint16_t sendCRC16(char *data)
{
    int len = sizeof (sendInfo) - 2; // 需要校验的数据长度
    uint16_t wcrc=0XFFFF;
    uint8_t temp;
    int i=0,j=0;
    for(i=0;i<len;i++)  // 需要校验的数据位置
    {
        temp=data[i];
        wcrc^=temp;
        for(j=0; j<8; j++)
        {
            if(wcrc & 0X0001)
            {
                wcrc >>= 1;
                wcrc^= 0XA001;
            }
            else
                wcrc >>= 1;
        }
    }
    temp=wcrc;
    return wcrc;
}

void print_data(const receiveInfo& data_)
{
    cout<<hex;
    cout<<"header1 is "<<(int)data_.header1<<endl;
    cout<<"lb_desire is "<<(int)data_.lb_desire<<endl;
    cout<<"state is "<<(int)data_.status<<endl;
    cout<<dec;
    cout<<"dX is "<<data_.dX<<endl;
    cout<<"dY is "<<data_.dY<<endl;
    cout<<"dZ is "<<data_.dZ<<endl;
    cout<<"dr is "<<data_.qw<<endl;
    cout<<"dp is "<<data_.qx<<endl;
    cout<<"dy is "<<data_.qy<<endl;
    cout<<hex;
    
    cout<<"tail is"<<data_.tail<<endl;
    //cout<<"dz is "<<data_.qz<<endl;
}

int select_mission(uint8_t mission, uint8_t tool)
{

}

void read_session(socket_ptr sock, ip::udp::endpoint ep) 
{
    while(!pgmEnd) 
    {
        boost::array<char, sizeof(receiveInfo)> buf;
        boost::system::error_code error;
        sock->receive_from(boost::asio::buffer(buf), ep);

        if (error == boost::asio::error::eof)
            break; // Connection closed cleanly by peer.
        else if (error)
            throw boost::system::system_error(error); // Some other error.

        //check

        desire_pose = *(receiveInfo*)buf.data();
        if(desire_pose.header1 != 0XFA04)
        {
            cout<<"header not match!"<<endl;
            continue;
        }
            
        if(desire_pose.length != sizeof(receiveInfo))
        {
            cout<<"length not match!"<<endl;
            continue;
        }
        if(desire_pose.tail != 0xFFFF)
        {
            cout<<"tail not match!"<<endl;
            continue;
        }
        //print_data(desire_pose);
    
        pose_control.dx = desire_pose.dX/1000.0;
        pose_control.dy = desire_pose.dY/1000.0;
        pose_control.dz = desire_pose.dZ/1000.0;
        pose_control.qw = desire_pose.qw/1000.0;
        pose_control.qx = desire_pose.qx/1000.0;
        pose_control.qy = desire_pose.qy/1000.0;

        if(desire_pose.mission == 0XFF)
        {
            pose_control.mission = 0; //free move
        }
        else
        {
            if(desire_pose.lb_desire == 0)
            {
                if(desire_pose.mission < 4)
                    pose_control.mission = (int)desire_pose.mission; //layser
                else
                    pose_control.mission = 0;
            } 
            else
            {   
                if(desire_pose.mission > 3)
                    pose_control.mission = (int)desire_pose.mission;
                else
                    pose_control.mission = 0;
            }
            cout<<"mission is"<<(int)desire_pose.mission<<" "<<pose_control.mission<<endl;
        }

        pose_control.op_status = (int)desire_pose.op_status;

        if(desire_pose.lb_desire == 0)
        {
            control_command_1 -> publish(pose_control);
        }
        else
        {
            control_command_0 -> publish(pose_control);
        }

        
    }
}

float joint_value_0[6];
void joint_callback_0(sensor_msgs::msg::JointState joint)
{
    for(int i=0; i<6; i++)
    {
        joint_value_0[i] = joint.position[i];
    }
}

float joint_value_1[6];
void joint_callback_1(sensor_msgs::msg::JointState joint)
{
    for(int i=0; i<6; i++)
    {
        joint_value_1[i] = joint.position[i];
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("udp");
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_0_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_1_joint_state;
    arm_0_joint_state = node -> create_subscription<sensor_msgs::msg::JointState>("arm_0/joint_states",1,&joint_callback_0);
    arm_1_joint_state = node -> create_subscription<sensor_msgs::msg::JointState>("arm_1/joint_states",1,&joint_callback_0);
    

    control_command_0 = node -> create_publisher<robot_control_msgs::msg::Pose>("arm_0/control_command", 1);
    control_command_1 = node -> create_publisher<robot_control_msgs::msg::Pose>("arm_1/control_command", 1);

    
    // string file_name = "/home/catmulti7/joint.txt";
    // joint_file.open(file_name, ios::out);
    
    asio::io_service io_service;
    ip::udp::endpoint ep(ip::address::from_string("192.168.71.15"), 8004);

    socket_ptr sock(new ip::udp::socket(io_service, ip::udp::endpoint(ip::udp::v4(), 8004)));
    pgmEnd = false;
    thread read_thread(std::bind(read_session, sock, ep));
    cout<<"connection establish"<<endl;
    rclcpp::Rate loop_rate(5);
    while(rclcpp::ok())
    {
        
        boost::system::error_code error;
        sendInfo send;
        send.header1 = 0XEA04;
        send.length = sizeof(sendInfo);
        //arm_0
        send.elbow_joint_0 = joint_value_1[0]*1000;
        send.shoulder_lift_joint_0 = joint_value_1[1]*1000;
        send.shoulder_pan_joint_0 = joint_value_1[2]*1000;
        send.wrist_1_joint_0 = joint_value_1[3]*1000;
        send.wrist_2_joint_0 = joint_value_1[4]*1000;
        send.wrist_3_joint_0 = joint_value_1[5]*1000;

        // for(int i = 0;i<6;i++)
        // {
        //     joint_file<<joint_value_0[i]<<" ";
        // }

        //arm_1
        send.elbow_joint_1 = joint_value_0[0]*1000;
        send.shoulder_lift_joint_1 = joint_value_0[1]*1000;
        send.shoulder_pan_joint_1 = joint_value_0[2]*1000;
        send.wrist_1_joint_1 = joint_value_0[3]*1000;
        send.wrist_2_joint_1 = joint_value_0[4]*1000;
        send.wrist_3_joint_1 = joint_value_0[5]*1000;
        // for(int i = 0;i<6;i++)
        // {
        //     joint_file<<joint_value_1[i]<<" ";
        // }
        // joint_file<<endl;

        sock->send_to(asio::buffer((uint8_t*)&send, sizeof(sendInfo)), ep);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    pgmEnd = true;
    read_thread.join();

    
    return 0;
}