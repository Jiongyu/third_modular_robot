/**
 * @file inverse_solution_server.cpp
 * @author your name (you@domain.com)
 * @brief   基于ros服务获取运动学逆解---服务器端
 *          输入输出参考srv/inverse_solution.srv
 * @version 0.1
 * @date 2020-07-25
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <ros/ros.h>
#include "./../kinematics/Kine.h"
#include "birl_module_robot/inverse_solution.h"


bool handle_function(   birl_module_robot::inverse_solution::Request &req,
                        birl_module_robot::inverse_solution::Response &res){

    ROS_INFO_STREAM("Inverse Solution Server Get New Request.");

    static double Robot_Link_Len[6]; 
    if(req.which_robot == 0){
        //climbing robot link length mm
        Robot_Link_Len[0] = 176.4; 
        Robot_Link_Len[1] = 256.8; 
        Robot_Link_Len[2] = 293.2; 
        Robot_Link_Len[3] = 293.2; 
        Robot_Link_Len[4] = 256.8; 
        Robot_Link_Len[5] = 176.4; 

    }                 
    else if(req.which_robot == 1){
        // biped robot link length
        Robot_Link_Len[0] = 72.2; 
        Robot_Link_Len[1] = 256.8; 
        Robot_Link_Len[2] = 293.2; 
        Robot_Link_Len[3] = 293.2; 
        Robot_Link_Len[4] = 256.8; 
        Robot_Link_Len[5] = 72.2; 
    }
    else{
        ROS_INFO_STREAM("Server Request about which_robot must be 0 or 1, else error!");  
        return false;
    }

    static Kine_CR_FiveDoF_G1 robot5d_G0; // robot based on the gripper0 to get inverse solution
    static Kine_CR_FiveDoF_G2 robot5d_G6;
    robot5d_G0.Set_Length(Robot_Link_Len);
    robot5d_G6.Set_Length(Robot_Link_Len);

    static double new_decartes_point[6] = {586.4,0,0,0,0,180}; //new cartesian point (xyzwpr) unit:(mm,degree)
    static double current_top_velocity[6]; //mm/s,degree/s
    static double current_joint_value[5] = {0,0,0,0,0};  //unit:degree

    static double new_joint_velocity[5]; //degree/s
    static double new_joint_value[5]; //degree
            
    new_decartes_point[0] = req.descartes_pos_commands[0];    //X   
    new_decartes_point[1] = req.descartes_pos_commands[1];    //Y   
    new_decartes_point[2] = req.descartes_pos_commands[2];    //Z
    new_decartes_point[3] = req.descartes_pos_commands[3];    //RX
    new_decartes_point[4] = req.descartes_pos_commands[4];    //RY   
    new_decartes_point[5] = req.descartes_pos_commands[5];    //RZ

    current_joint_value[0] = req.current_joint_state[0];   //I1
    current_joint_value[1] = req.current_joint_state[1];   //T2
    current_joint_value[2] = req.current_joint_state[2];   //T3
    current_joint_value[3] = req.current_joint_state[3];   //T4
    current_joint_value[4] = req.current_joint_state[4];   //I5

    current_top_velocity[0] = req.descartes_vel_commands[0]; // X_v
    current_top_velocity[1] = req.descartes_vel_commands[1]; // Y_v
    current_top_velocity[2] = req.descartes_vel_commands[2]; // Z_v
    current_top_velocity[3] = req.descartes_vel_commands[3]; // RX_v
    current_top_velocity[4] = req.descartes_vel_commands[4]; // RY_v
    current_top_velocity[5] = req.descartes_vel_commands[5]; // RZ_v
    // ROS_INFO_STREAM("convert request data success.");                   

    if(req.base)
        if(! robot5d_G0.IKine(new_decartes_point,current_joint_value,new_joint_value))
            ROS_INFO_STREAM("G0 IKine success");  
    else
        if(! robot5d_G6.IKine(new_decartes_point,current_joint_value,new_joint_value))
            ROS_INFO_STREAM("G6 IKine success");  

    res.joint_pos_commands.clear();
    res.joint_pos_commands.push_back(new_joint_value[0]);  // I1
    res.joint_pos_commands.push_back(new_joint_value[1]);  // T2
    res.joint_pos_commands.push_back(new_joint_value[2]);  // T3
    res.joint_pos_commands.push_back(new_joint_value[3]);  // T4
    res.joint_pos_commands.push_back(new_joint_value[4]);  // I5
    
    if(req.base)
        if(! robot5d_G0.Vel_IKine(new_joint_value,current_top_velocity,new_joint_velocity))
            ROS_INFO_STREAM("G0 Vel_IKine success");  
    else
        if(! robot5d_G6.Vel_IKine(new_joint_value,current_top_velocity,new_joint_velocity))
            ROS_INFO_STREAM("G6 Vel_IKine success");  

    res.joint_vel_commands.clear();
    res.joint_vel_commands.push_back(new_joint_velocity[0]);
    res.joint_vel_commands.push_back(new_joint_velocity[1]);
    res.joint_vel_commands.push_back(new_joint_velocity[2]);
    res.joint_vel_commands.push_back(new_joint_velocity[3]);
    res.joint_vel_commands.push_back(new_joint_velocity[4]);
    
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "inverse_solution_server");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("inverse_solution", handle_function);

    ROS_INFO_STREAM("Inverse Solution Server Already start.");  
    ros::spin();
    
    return 0;
}
