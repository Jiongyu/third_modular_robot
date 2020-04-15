#define PI_RAD  0.0174532925199 // 角度转换为弧度参数
#define PI_DEG 57.2957795130823 // 弧度转换为角度参数
#include "./../kinematics/Kine.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <iomanip>

std_msgs::Float64MultiArray request_descartes_point;
std_msgs::Float64MultiArray robot_joint_data;

int main(int argc, char **argv)
{
    void subCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
    void subCallback_jointData(const std_msgs::Float64MultiArrayConstPtr &msg);

    double Robot_Link_Len[6] = {0.1764,0.2568,0.2932,0.2932,0.2568,0.1764}; //robot link length

    Kine_CR_FiveDoF_G1 Biped5d; // robot based on the gripper0 to get inverse solution
    Biped5d.Set_Length(Robot_Link_Len);
    
    ros::init(argc, argv, "biped5d_command");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/low_level/biped5d_joint_command", 10);
    ros::Subscriber sub = nh.subscribe("/low_level/biped5d_descart_command", 10, subCallback);
    while(ros::ok() && sub.getNumPublishers()<1){
        ROS_INFO("topic /low_level/biped5d_descart_command not exist!!");
        sleep(1);
    }

    ros::Subscriber sub_jointData = nh.subscribe("/low_level/biped5d_joint_point", 30, subCallback_jointData);
    while(ros::ok() && sub_jointData.getNumPublishers()<1){
        ROS_INFO("topic /low_level/biped5d_joint_point not exist!!");
        sleep(1);
    }

    std_msgs::Float64MultiArray transmit_joint_list; // I1,T2,T3,T4,I5
    transmit_joint_list.data.resize(10);
    transmit_joint_list.data.clear();
    
    robot_joint_data.data.resize(5);
    robot_joint_data.data.clear();
    robot_joint_data.data = {0,0,0,0,0};

    request_descartes_point.data.resize(11);  //descartes points + joint points

    double current_joint_value[5] = {0,0,0,0,0};  //unit:degree
    double current_top_velocity[6]; //m/s,degree/s
    double new_joint_velocity[5]; //degree/s
    double new_joint_value[5]; //rad
    double new_decartes_point[6] = {0.5864,0,0,0,0,180}; //new cartesian point (xyzwpr) unit:(meter,degree)
    boost::shared_ptr<std_msgs::Float64MultiArray const> shared;

    ros::Rate timer(5);
    while (ros::ok())
    {
        shared = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/low_level/biped5d_descart_command",nh);
        if(shared != NULL){
            request_descartes_point = *shared;

            // ROS_INFO("%.2f",request_descartes_point.data[0]);
            
            new_decartes_point[0] = request_descartes_point.data[0];    //X   
            new_decartes_point[1] = request_descartes_point.data[1];    //Y   
            new_decartes_point[2] = request_descartes_point.data[2];    //Z
            new_decartes_point[3] = request_descartes_point.data[3];    //RX
            new_decartes_point[4] = request_descartes_point.data[4];    //RY   
            new_decartes_point[5] = request_descartes_point.data[5];    //RZ

            // current_joint_value[0] = request_descartes_point.data[6];   //I1
            // current_joint_value[1] = request_descartes_point.data[7];   //T2
            // current_joint_value[2] = request_descartes_point.data[8];   //T3
            // current_joint_value[3] = request_descartes_point.data[9];   //T4
            // current_joint_value[4] = request_descartes_point.data[10];  //I5

            current_joint_value[0] = robot_joint_data.data[0] * PI_DEG;   //I1
            current_joint_value[1] = robot_joint_data.data[1] * PI_DEG;   //T2
            current_joint_value[2] = robot_joint_data.data[2] * PI_DEG;   //T3
            current_joint_value[3] = robot_joint_data.data[3] * PI_DEG;   //T4
            current_joint_value[4] = robot_joint_data.data[4] * PI_DEG;  //I5

            current_top_velocity[0] = request_descartes_point.data[6]; // X_v
            current_top_velocity[1] = request_descartes_point.data[7]; // Y_v
            current_top_velocity[2] = request_descartes_point.data[8]; // Z_v
            current_top_velocity[3] = request_descartes_point.data[9]; // RX_v
            current_top_velocity[4] = request_descartes_point.data[10]; // RY_v
            current_top_velocity[5] = request_descartes_point.data[11]; // RZ_v

            Biped5d.IKine(new_decartes_point,current_joint_value,new_joint_value);

            transmit_joint_list.data.push_back(new_joint_value[0] * PI_RAD);  //I1
            transmit_joint_list.data.push_back(new_joint_value[1] * PI_RAD);  //T2
            transmit_joint_list.data.push_back(new_joint_value[2] * PI_RAD);  //T3
            transmit_joint_list.data.push_back(new_joint_value[3] * PI_RAD);  //T4
            transmit_joint_list.data.push_back(new_joint_value[4] * PI_RAD);  //I5

            ROS_INFO("-----------");
            ROS_INFO("pos:");
            ROS_INFO("%.2f", transmit_joint_list.data[0]);
            ROS_INFO("%.2f", transmit_joint_list.data[1]);
            ROS_INFO("%.2f", transmit_joint_list.data[2]);
            ROS_INFO("%.2f", transmit_joint_list.data[3]);
            ROS_INFO("%.2f", transmit_joint_list.data[4]);

            Biped5d.Vel_IKine(new_joint_value,current_top_velocity,new_joint_velocity);

            transmit_joint_list.data.push_back(fabs(new_joint_velocity[0]) * PI_RAD); //I1 velocity
            transmit_joint_list.data.push_back(fabs(new_joint_velocity[1]) * PI_RAD); //T2 velocity
            transmit_joint_list.data.push_back(fabs(new_joint_velocity[2]) * PI_RAD); //T3 velocity
            transmit_joint_list.data.push_back(fabs(new_joint_velocity[3]) * PI_RAD); //T4 velocity
            transmit_joint_list.data.push_back(fabs(new_joint_velocity[4]) * PI_RAD); //I5 velocity

            ROS_INFO("-----------");
            ROS_INFO("vel:");
            ROS_INFO("%.2f", transmit_joint_list.data[5]);
            ROS_INFO("%.2f", transmit_joint_list.data[6]);
            ROS_INFO("%.2f", transmit_joint_list.data[7]);
            ROS_INFO("%.2f", transmit_joint_list.data[8]);
            ROS_INFO("%.2f", transmit_joint_list.data[9]);

            pub.publish(transmit_joint_list);    
            transmit_joint_list.data.clear(); 
        
        }
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}    

void subCallback(const std_msgs::Float64MultiArrayConstPtr &msg){
    request_descartes_point = *msg;
}

void subCallback_jointData(const std_msgs::Float64MultiArrayConstPtr &msg){
    robot_joint_data = *msg;

    // ROS_INFO("-----------");
    // ROS_INFO("pos:");
    // ROS_INFO("%.2f", robot_joint_data.data[0]);
    // ROS_INFO("%.2f", robot_joint_data.data[1]);
    // ROS_INFO("%.2f", robot_joint_data.data[2]);
    // ROS_INFO("%.2f", robot_joint_data.data[3]);
    // ROS_INFO("%.2f", robot_joint_data.data[4]);
}