#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <std_msgs/msg/char.hpp>
#include <thread>
#include <atomic>

#include "robot_control_msgs/msg/pose.hpp"
#include "robot_control_msgs/msg/moblie.hpp"
#include "robot_control_msgs/msg/joint.hpp"

using namespace boost;
using namespace boost::asio;
using namespace boost::asio::ip;
using std::string;
using namespace std;
typedef boost::shared_ptr<tcp::socket> socket_ptr;

robot_control_msgs::msg::Pose pose_control;
rclcpp::Publisher<robot_control_msgs::msg::Pose>::SharedPtr pose_control_pub_0;
rclcpp::Publisher<robot_control_msgs::msg::Pose>::SharedPtr pose_control_pub_1;

robot_control_msgs::msg::Moblie mobile_control;
rclcpp::Publisher<robot_control_msgs::msg::Moblie>::SharedPtr mobile_control_pub;

robot_control_msgs::msg::Joint joint_control;
rclcpp::Publisher<robot_control_msgs::msg::Joint>::SharedPtr joint_control_pub_0;
rclcpp::Publisher<robot_control_msgs::msg::Joint>::SharedPtr joint_control_pub_1;

std_msgs::msg::Char mode_control;
rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr mode_control_pub;

atomic<bool> pgmEnd(false);

asio::io_service service;

float scale = 1;

struct pose_info
{
    uint8_t header1;
    uint8_t header2;
    uint8_t instruct;
    uint8_t lb_desire;
    float dX;
    float dY;
    float dZ;
    float qw;
    float qx;
    float qy;
    float qz;
    uint8_t crc1;
    uint8_t crc2;
    friend pose_info operator - (pose_info A, pose_info B)
    {
        pose_info diff;
        diff.dX = A.dX - B.dX;
        diff.dY = A.dY - B.dY;
        diff.dZ = A.dZ - B.dZ;
        return diff;
    }
}__attribute__((packed)) current_pose;

pose_info last_pose, d_pose;

struct mobile_info
{
    uint8_t header1;
    uint8_t header2;
    uint8_t instruct;
    int16_t vx;
    int16_t vw;
    uint8_t crc1;
    uint8_t crc2;
}__attribute__((packed)) mobile_desire;

struct joint_info
{
    uint8_t header1;
    uint8_t header2;
    uint8_t instruct;
    uint8_t lb_desire;
    float shoulder_pan;
    float shoulder_lift;
    float elbow;
    float wrist_1;
    float wrist_2;
    float wrist_3;
    uint8_t crc1;
    uint8_t crc2;
}__attribute__((packed)) joint_desire;

struct mode_info
{
    uint8_t header1;
    uint8_t header2;
    uint8_t instruct;
    uint8_t mode;
    uint8_t crc1;
    uint8_t crc2;
}__attribute__((packed)) mode_desire;


template <typename T>
uint16_t receiveCRC16(char *data)
{
    int len = sizeof (T) - 2; // 需要校验的数据长度
    uint16_t wcrc = 0XFFFF;
    uint8_t temp;
    int i=0, j=0;
    for(i=0; i<len; i++)  // 需要校验的数据位置
    {
        temp=data[i];
        wcrc^=temp;
        for(j=0; j<8; j++)
        {
            if(wcrc & 0X0001)
            {
                wcrc >>= 1;
                wcrc ^= 0XA001;
            }
            else
                wcrc >>= 1;
        }
    }
    temp=wcrc;
    return wcrc;
}

void print_data(const pose_info& data_)
{
    cout<<hex;
    cout<<"header1 is "<<(int)data_.header1<<endl;
    cout<<"header2 is "<<(int)data_.header2<<endl;
    cout<<"instruct is "<<(int)data_.instruct<<endl;
    cout<<"lb_desire is "<<(int)data_.lb_desire<<endl;
    cout<<dec;
    cout<<"dX is "<<data_.dX<<endl;
    cout<<"dY is "<<data_.dY<<endl;
    cout<<"dZ is "<<data_.dZ<<endl;
    cout<<"dr is "<<data_.qw<<endl;
    cout<<"dp is "<<data_.qx<<endl;
    cout<<"dy is "<<data_.qy<<endl;
    cout<<"dy is "<<data_.qz<<endl;
    cout<<hex;
    cout<<"crc1 is "<<(int)data_.crc1<<endl;
    cout<<"crc2 is "<<(int)data_.crc2<<endl;
    cout<<dec;
}

void print_data(const mobile_info& data_)
{
    cout<<hex;
    cout<<"header1 is "<<(int)data_.header1<<endl;
    cout<<"header2 is "<<(int)data_.header2<<endl;
    cout<<"instruct is "<<(int)data_.instruct<<endl;
    cout<<dec;
    cout<<"dX is "<<data_.vx<<endl;
    cout<<"dY is "<<data_.vw<<endl;
    cout<<hex;
    cout<<"crc1 is "<<(int)data_.crc1<<endl;
    cout<<"crc2 is "<<(int)data_.crc2<<endl;
    cout<<dec;
}

void print_data(const joint_info& data_)
{
    cout<<hex;
    cout<<"header1 is "<<(int)data_.header1<<endl;
    cout<<"header2 is "<<(int)data_.header2<<endl;
    cout<<"instruct is "<<(int)data_.instruct<<endl;
    cout<<"lb_desire is "<<(int)data_.lb_desire<<endl;
    cout<<dec;
    cout<<"dX is "<<data_.shoulder_pan<<endl;
    cout<<"dY is "<<data_.shoulder_lift<<endl;
    cout<<"dZ is "<<data_.elbow<<endl;
    cout<<"dr is "<<data_.wrist_1<<endl;
    cout<<"dp is "<<data_.wrist_2<<endl;
    cout<<"dy is "<<data_.wrist_3<<endl;
    cout<<hex;
    cout<<"crc1 is "<<(int)data_.crc1<<endl;
    cout<<"crc2 is "<<(int)data_.crc2<<endl;
    cout<<dec;
}


void stop_handle(socket_ptr sock)
{
    if(sock -> is_open())
        sock -> close();
    service.stop();
}



void read_session(socket_ptr sock) 
{
    while(!pgmEnd)
    {
        boost::array<char, 36> buf;
        boost::system::error_code error;
        sock->read_some(boost::asio::buffer(buf), error);

        if (error == boost::asio::error::eof)
            break; // Connection closed cleanly by peer.
        else if (error)
            //throw boost::system::system_error(error); // Some other error.
            break;

        if(buf[2] == 0x00)
        {
            current_pose = *(pose_info*)buf.data();

            //check crc
            uint16_t crc = receiveCRC16<pose_info>(buf.data());
            uint8_t crc1 = crc>>8;
            uint8_t crc2 = crc;
            cout<<hex;
            cout<<"crc1"<<(int)crc1<<endl;
            cout<<"crc2"<<(int)crc2<<endl;

            d_pose = current_pose - last_pose;
            last_pose = current_pose;
            cout<<"================1 frame===================="<<endl;
            print_data(current_pose);
            cout<<"-----------------diff----------------"<<endl;
            print_data(d_pose);
            if(abs(d_pose.dX)>100 || abs(d_pose.dY)>100 || abs(d_pose.dZ)>100)
            {
                cout<<"INPUT DIFF LIMIT EXCEEDED!!!";
                continue;
            }

            if(current_pose.crc1 == crc1 && current_pose.crc2 == crc2) // CRC校验码
            {
                cout<<"check pass"<<endl;
                pose_control.dx = d_pose.dX;
                pose_control.dy = d_pose.dY;
                pose_control.dz = d_pose.dZ;
                pose_control.qw = d_pose.qw;
                pose_control.qx = d_pose.qx;
                pose_control.qy = d_pose.qy;
                pose_control.qz = d_pose.qz;

                if(current_pose.lb_desire == 0xFE)
                //temp change to arm 1
                    pose_control_pub_1 -> publish(pose_control);
                else if(current_pose.lb_desire == 0xEF)
                    pose_control_pub_1 -> publish(pose_control);
            }
            else
            {
                cout<<"CRC check failed"<<endl;
            } 

        }
        else if(buf[2] == 0x01)
        {
            joint_desire = *(joint_info*)buf.data();

            //check crc
            uint16_t crc = receiveCRC16<joint_info>(buf.data());
            uint8_t crc1 = crc>>8;
            uint8_t crc2 = crc;
            cout<<hex;
            cout<<"crc1"<<(int)crc1<<endl;
            cout<<"crc2"<<(int)crc2<<endl;

            print_data(joint_desire);

            if(joint_desire.crc1 == crc1 && joint_desire.crc2 == crc2)
            {
                joint_control.shoulder_pan = joint_desire.shoulder_pan;
                joint_control.shoulder_lift = joint_desire.shoulder_lift;
                joint_control.elbow = joint_desire.elbow;
                joint_control.wrist_1 = joint_desire.wrist_1;
                joint_control.wrist_2 = joint_desire.wrist_2;
                joint_control.wrist_3 = joint_desire.wrist_3;

                if(joint_desire.lb_desire == 0xFE)
                    joint_control_pub_0 -> publish(joint_control);
                else if(joint_desire.lb_desire == 0xEF)
                    joint_control_pub_1 -> publish(joint_control);
            }
            
        }
        else if(buf[2] == 0x02)
        {
            mobile_desire = *(mobile_info*)buf.data();

            //check crc
            uint16_t crc = receiveCRC16<mobile_info>(buf.data());
            uint8_t crc1 = crc>>8;
            uint8_t crc2 = crc;
            cout<<hex;
            cout<<"crc1"<<(int)crc1<<endl;
            cout<<"crc2"<<(int)crc2<<endl;

            print_data(mobile_desire);

            if(mobile_desire.crc1 == crc1 && mobile_desire.crc2 == crc2)
            {
                cout<<"CRC check pass"<<endl;
                mobile_control.vx = mobile_desire.vx * scale;
                mobile_control.vw = mobile_desire.vw * scale;
                if(mobile_control.vx > 10)
                    mobile_control.vx = 10;
                mobile_control_pub -> publish(mobile_control);
            }
        }
        else if(buf[2] == 0x03)
        {
            mode_desire = *(mode_info*)buf.data();

            //check crc
            uint16_t crc = receiveCRC16<mobile_info>(buf.data());
            uint8_t crc1 = crc>>8;
            uint8_t crc2 = crc;
            cout<<hex;
            cout<<"crc1"<<(int)crc1<<endl;
            cout<<"crc2"<<(int)crc2<<endl;

            cout << "mode switch to " << (int)mode_desire.mode << endl;
            mode_control.data = (int)mode_desire.mode;
            mode_control_pub -> publish(mode_control);
        }

    }
    sock -> close();
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("tcp");

    pose_control_pub_0 = node -> create_publisher<robot_control_msgs::msg::Pose>("/arm_0/control_command", 1);
    pose_control_pub_1 = node -> create_publisher<robot_control_msgs::msg::Pose>("/arm_1/control_command", 1);

    joint_control_pub_0 = node -> create_publisher<robot_control_msgs::msg::Joint>("/arm_0/joint_control_command", 1);
    joint_control_pub_1 = node -> create_publisher<robot_control_msgs::msg::Joint>("/arm_1/joint_control_command", 1);

    mobile_control_pub = node -> create_publisher<robot_control_msgs::msg::Moblie>("/mobile_control_command",1);

    mode_control_pub = node -> create_publisher<std_msgs::msg::Char>("/mode_control_command",1);

    asio::signal_set signals(service);

    tcp::acceptor acceptor(service, tcp::endpoint(tcp::v4(), 5001));

    signals.add(SIGINT);
    


    socket_ptr sock(new ip::tcp::socket(service));
    signals.async_wait(std::bind(stop_handle,sock));
   

    pgmEnd = false;

    vector<std::thread> tcp_threads;
    thread* tmp_thread;

    while(rclcpp::ok())
    {
        socket_ptr sock(new ip::tcp::socket(service));
        acceptor.accept(*sock);
        tmp_thread =new thread(std::bind(read_session,sock));
        cout<<"connection estabished"<<endl;
        tcp_threads.push_back(std::move(*tmp_thread));
        


        // boost::system::error_code error;

        // pose_info send;
        // send.header1 = 0X88;
        // send.header2 = 0XA4;
        // send.instruct = 0X10;
        // send.lb_desire = 0XEF;
        // send.dX = 0.1;
        // send.dY = 0.2;
        // send.dZ = 0.3;
        // send.qw = 0.4;
        // send.qx = 0.5;
        // send.qy = 0.6;
        // send.qz = 0.6;
        // uint16_t crc_ = receiveCRC16((char*)&send);
        // uint8_t crc1_ = crc_>>8;
        // uint8_t crc2_ = crc_;
        // send.crc1 = crc1_;
        // sock->write_some(asio::buffer((uint8_t*)&send,31), error);
    }

    pgmEnd = true;
    for(auto& t: tcp_threads)
    {
        t.join();
    }

    
    return 0;
}