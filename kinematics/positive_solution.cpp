#define PI_RAD  0.0174532925199 // 角度转换为弧度参数
#define PI_DEG 57.2957795130823 // 弧度转换为角度参数
#include "./../kinematics/Kine.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <birl_module_robot/descartes_point.h>


std_msgs::Float64MultiArray request_joint_point;

int main(int argc, char **argv)
{
    void subCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
    double Robot_Link_Len[6] = {0.1764,0.2568,0.2932,0.2932,0.2568,0.1764}; //robot link length

    Kine_CR_FiveDoF_G1 Biped5d; 
    Biped5d.Set_Length(Robot_Link_Len);

    double actual_joint_value[5] = {1,1,1,1,1};
    double descartes_point[6];
    
    ros::init(argc, argv, "biped5d_feedback");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<birl_module_robot::descartes_point>("/low_level/biped5d_descartes_point", 5);
    // ros::Subscriber sub = nh.subscribe("/low_level/biped5d_joint_point", 10, subCallback);
    ros::Subscriber sub = nh.subscribe("/low_level/joints_point_feedback", 10, subCallback);


    while(ros::ok() && sub.getNumPublishers()<1){
        ROS_INFO("Topic /low_level/biped5d_joint_point not exist!!");
        sleep(1);
    }
    birl_module_robot::descartes_point transmit_descartes_point;

    transmit_descartes_point.data.data.resize(6);
    request_joint_point.data.resize(6);
    ros::Rate timer(50);

    boost::shared_ptr<std_msgs::Float64MultiArray const> shared;


    while(ros::ok()){

        shared =  ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/low_level/biped5d_joint_point",nh);
        if(shared != NULL){

            request_joint_point = *shared;
            actual_joint_value[0] = request_joint_point.data[0]*PI_DEG;
            actual_joint_value[1] = request_joint_point.data[1]*PI_DEG;
            actual_joint_value[2] = request_joint_point.data[2]*PI_DEG;
            actual_joint_value[3] = request_joint_point.data[3]*PI_DEG;
            actual_joint_value[4] = request_joint_point.data[4]*PI_DEG;
            
            Biped5d.FKine(actual_joint_value,descartes_point);
            
            transmit_descartes_point.header.stamp = ros::Time::now();
            transmit_descartes_point.data.data[0] = descartes_point[0];
            transmit_descartes_point.data.data[1] = descartes_point[1];
            transmit_descartes_point.data.data[2] = descartes_point[2];
            transmit_descartes_point.data.data[3] = descartes_point[3];
            transmit_descartes_point.data.data[4] = descartes_point[4];
            transmit_descartes_point.data.data[5] = descartes_point[5];

            // ROS_INFO("-----------");
            // ROS_INFO("%.2f", transmit_descartes_point.data[0]);
            // ROS_INFO("%.2f", transmit_descartes_point.data[1]);
            // ROS_INFO("%.2f", transmit_descartes_point.data[2]);
            // ROS_INFO("%.2f", transmit_descartes_point.data[3]);
            // ROS_INFO("%.2f", transmit_descartes_point.data[4]);
            // ROS_INFO("%.2f", transmit_descartes_point.data[5]);

            pub.publish(transmit_descartes_point);
            
        }
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}

void subCallback(const std_msgs::Float64MultiArrayConstPtr &msg){
    request_joint_point = *msg;
}