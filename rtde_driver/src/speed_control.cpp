//
// Created by catmulti7 on 14/07/23.
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <boost/lockfree/queue.hpp>
#include <queue>
#include <algorithm>
#include <fstream>

//sensor_msgs/msg/JointState

#include <thread>
#include <chrono>
#include <memory>



using namespace ur_rtde;
using namespace std::chrono;
using std::cout, std::endl;
using std::placeholders::_1;
using std::vector, std::queue, std::string, std::fstream;


sensor_msgs::msg::JointState joint_state;


template<typename T>
class sharedObj
{
private:
    std::mutex mtx;
    T obj;

public:
    void operator()(T& other)
    {
        mtx.lock();
        obj.swap(other);
        mtx.unlock();
    }
};




class RTDEDriver : public rclcpp::Node
{
public:
    string robot_ip;
    string robot_name;
    std::unique_ptr<RTDEControlInterface> rtde_control;
    std::unique_ptr<RTDEReceiveInterface> rtde_receive;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_status_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_torque_pub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ik_sub;
    

    std::thread* control_thread;
    sharedObj<vector<double>> d_j_share;


    std::chrono::time_point<steady_clock> start_time;


    double vel = 0.1;
    double acc = 0.5; //1
    double rtde_frequency = 500; // Hz
    double dt = 1.0 / rtde_frequency; // 2ms
    float step_threshold = 0.05;

    std::atomic<bool> new_pos, ready, pgm_end;

    vector<double> d_j_position, d_speed, pos_err, last_pos_err, dif_pos_err, r_ik;
    double kp = 1; //3
    double kd = 0.2; //1
    std::mutex j_pos_lock;
    
    vector<double> position, velocity, effort, torque;
    vector<double> r_position, r_velocity, r_effort;

    vector<string> name = {"shoulder_pan_joint",
                            "shoulder_lift_joint", 
                            "elbow_joint", 
                            "wrist_1_joint", 
                            "wrist_2_joint", 
                            "wrist_3_joint"
                            };

    vector<string> name_with_prefix;


    RTDEDriver() : Node("rtde_driver")
    {
        this -> declare_parameter<string>("ip_address");
        this -> declare_parameter<string>("robot_name");
        this -> get_parameter("ip_address", robot_ip);
        this -> get_parameter("robot_name", robot_name);

        cout << robot_ip << endl;

        d_j_position = vector<double>(6);
        d_speed = vector<double>(6);
        dif_pos_err = vector<double>(6);
        pos_err = vector<double>(6);


        for(auto joint: name)
        {
            name_with_prefix.push_back(robot_name + "_" + joint);
        }

        rclcpp::on_shutdown(std::bind(&RTDEDriver::closeConnect, this));

        rtde_control = std::make_unique<RTDEControlInterface>(robot_ip, rtde_frequency);
        rtde_receive = std::make_unique<RTDEReceiveInterface>(robot_ip, 200);

        joint_status_pub = this -> create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        joint_torque_pub = this -> create_publisher<std_msgs::msg::Float32MultiArray>("joint_torque", 10);
        ik_sub = this -> create_subscription<std_msgs::msg::Float64MultiArray>
                            ("ik_results", 1, std::bind(&RTDEDriver::IKResCallback, this,_1));
                            
        
        // control_thread = new std::thread(&RTDEDriver::controlArm, this);

        timer_ = this->create_wall_timer(
                    2ms, std::bind(&RTDEDriver::controlArm, this));

        start_time = std::chrono::steady_clock::now();

    }

    ~RTDEDriver()
    {
        pgm_end = true;
    }

    void controlArm();
    void receiveCallback();
    void print_vector(vector<double>& v);
    void swap_idx(vector<double>& v);
    void IKResCallback(const std_msgs::msg::Float64MultiArray& ik_res);
    float calc_distance(vector<double>& p_a, vector<double>& p_b);
    void start_from_current(vector<double>& current_p, queue<vector<double>>& trajectory);
    void verify(vector<double>& current_p, vector<double>& desire_p);
    void closeConnect();
    
};

void RTDEDriver::closeConnect()
{
    rtde_control -> speedStop();
    rtde_control -> stopScript();
    cout << "RTDE Control Quit" << endl;
    // RCLCPP_WARN(this -> get_logger(), "RTDE Control Quit");
    return;
}

float RTDEDriver::calc_distance(vector<double>& p_a, vector<double>& p_b)
{
    float distance = 0;
    for(unsigned int i = 0; i < p_a.size(); i++)
    {
        distance += (p_a[i] - p_b[i]) * (p_a[i] - p_b[i]);
    }
    return distance;

}

void RTDEDriver::start_from_current(vector<double>& current_p, queue<vector<double>>& trajectory)
{
    float last_distance = 100000000;
    float distance;
    while(trajectory.size() > 1)
    {
        auto p = trajectory.front();
        
        distance = calc_distance(p, current_p);
        if(distance > last_distance)
        {
            //cout<<"distance is "<<distance<<endl;
            break;
        }
        else
        {
            last_distance = distance;
        }
        trajectory.pop();
    }

}

void RTDEDriver::verify(vector<double>& current_p, vector<double>& desire_p)
{
    //cout<<"distance is "<<calc_distance(current_p, desire_p)<<endl;
    if(calc_distance(current_p, desire_p) > step_threshold)
    {
        rtde_control -> triggerProtectiveStop();

    }
}


void RTDEDriver::controlArm()
{
    if(pgm_end)
    {
        rtde_control -> speedStop();
        rtde_control -> stopScript();
        RCLCPP_WARN(this -> get_logger(), "RTDE Control Quit");
        return;
    }
    
    position = rtde_receive -> getActualQ();
    velocity = rtde_receive -> getActualQd();
    effort = rtde_receive -> getTargetCurrent();

    for(auto v:velocity)
    {
        if(fabs(v) > 3)
            rtde_control -> triggerProtectiveStop();
    }

    if(new_pos)
    {
        d_j_share(d_j_position);
        new_pos = false;
    }

    //speed control
    
    if(ready)
    {
        for(int i = 0; i < 6; i++)
        {
            pos_err[i] = d_j_position[i] - position[i];

            if(!last_pos_err.empty())
                dif_pos_err[i] = pos_err[i] - last_pos_err[i];
            
            d_speed[i] = kp * pos_err[i] + kd * dif_pos_err[i];
            if(fabs(d_speed[i]) > 1)
            {
                RCLCPP_WARN_STREAM(this -> get_logger(), "SPEED LIMIT EXCEED AT JOINT " << i);
                d_speed[i] = d_speed[i] > 0 ? 0.5 : -0.5;
            }
        }
        

        rtde_control -> speedJ(d_speed, acc, dt);
        last_pos_err = pos_err;

    }
    

    joint_state.header.stamp = this -> get_clock()-> now();
    joint_state.name = name_with_prefix;
    joint_state.position = position;
    joint_state.velocity = velocity;

    joint_status_pub -> publish(joint_state);

    
}

// NOT USED
void RTDEDriver::receiveCallback()
{
    r_position = rtde_receive -> getActualQ();
    r_velocity = rtde_receive -> getActualQd();
    r_effort = rtde_receive -> getTargetCurrent();



    joint_state.header.stamp = this -> get_clock()-> now();
    joint_state.name = name_with_prefix;
    joint_state.position = r_position;
    joint_state.velocity = r_velocity;

    joint_status_pub -> publish(joint_state);
    
   
}

void RTDEDriver::IKResCallback(const std_msgs::msg::Float64MultiArray& ik_res)
{
    r_ik = ik_res.data;
    d_j_share(r_ik);
    new_pos = true;
    ready = true;

}


void RTDEDriver::print_vector(vector<double>& v)
{
    for(auto it: v)
    {
        cout<<"- "<<it<<endl;
    }
}

void RTDEDriver::swap_idx(vector<double>& v)
{
    vector<double> tmp;
    tmp.push_back(v[5]);
    tmp.push_back(v[0]);
    tmp.push_back(v[1]);
    tmp.push_back(v[2]);
    tmp.push_back(v[3]);
    tmp.push_back(v[4]);
    v = tmp;
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RTDEDriver>());
  
    /*
    shoulder_pan
    shoulder_lift
    elbow
    wrist 1
    wrist 2
    wrist 3
    */

    rclcpp::shutdown();
    return 0;
}
