/**
 * @file positive_solution_server.cpp
 * @author your name (you@domain.com)
 * @brief   基于ros服务获取运动学正解---服务器端
 *          输入输出参考 srv/positive_solution.srv
 * @version 0.1
 * @date 2020-07-25
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <ros/ros.h>
#include "./../kinematics/Kine.h"
#include "birl_module_robot/positive_solution.h"

bool handle_function(   birl_module_robot::positive_solution::Request &req,
                        birl_module_robot::positive_solution::Response &res){

    ROS_INFO_STREAM("Positive Solution Server Get New Request.");  

    static double Robot_Link_Len[6]; 
    if(req.which_robot == 0){
<<<<<<< HEAD
        //climbing robot link length mm
=======
        //climbing robot link length
>>>>>>> master
        Robot_Link_Len[0] = 176.4; 
        Robot_Link_Len[1] = 256.8; 
        Robot_Link_Len[2] = 293.2; 
        Robot_Link_Len[3] = 293.2; 
        Robot_Link_Len[4] = 256.8; 
        Robot_Link_Len[5] = 176.4; 

    }                 
    else if(req.which_robot == 1){
<<<<<<< HEAD
        // biped robot link length mm
=======
        // biped robot link length
>>>>>>> master
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

    static double decartes_position_value[6] ; // (xyzwpr) unit:(mm,degree)
    static double current_joint_value[5];  //unit:degree

    current_joint_value[0] = req.current_joint_state[0];
    current_joint_value[1] = req.current_joint_state[1];
    current_joint_value[2] = req.current_joint_state[2];
    current_joint_value[3] = req.current_joint_state[3];
    current_joint_value[4] = req.current_joint_state[4];

    if(req.base){
        if( ! robot5d_G0.FKine(current_joint_value, decartes_position_value)){
            ROS_INFO_STREAM("G0 FKine success");  
            res.ifGetSolve = true;
        }
        else
        {
            res.ifGetSolve = false;
        }
        
    }
    else
    {
        if( ! robot5d_G6.FKine(current_joint_value, decartes_position_value)){
            ROS_INFO_STREAM("G6 FKine success");  
            res.ifGetSolve = true;
        }
        else
        {
            res.ifGetSolve = false;
        } 
    }

    res.descartes_pos_state.clear();
    res.descartes_pos_state.push_back( decartes_position_value[0]);
    res.descartes_pos_state.push_back( decartes_position_value[1]);
    res.descartes_pos_state.push_back( decartes_position_value[2]);
    res.descartes_pos_state.push_back( decartes_position_value[3]);
    res.descartes_pos_state.push_back( decartes_position_value[4]);
    res.descartes_pos_state.push_back( decartes_position_value[5]);

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "positive_solution_server");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("positive_solution", handle_function);

    ROS_INFO_STREAM("Positive Solution Server Already start.");  
    ros::spin();
    
    return 0;
}
