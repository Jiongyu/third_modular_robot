/**
 * @file robot_increTransTool.cpp
 * @author your name (you@domain.com)
 * @brief  基于ros服务，根据笛卡尔当前位姿，和位姿增量计算新的笛卡尔空间命令
 * @version 0.1
 * @date 2020-12-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <ros/ros.h>
#include "./../kinematics/Kine.h"
#include "birl_module_robot/robot_increTransTool.h"

bool handle_function(birl_module_robot::robot_increTransTool::Request &req,
                    birl_module_robot::robot_increTransTool::Response &res)
{
    ROS_INFO_STREAM("IncreTransTool Server Get New Request."); 
    static double current_descrates_position[6];
    static double incre_descrates_position_command[6]; 
    static double new_descrates_position_command[6];

    // xyz mm
    current_descrates_position[0] = req.current_descartes_postion[0];
    current_descrates_position[1] = req.current_descartes_postion[1];
    current_descrates_position[2] = req.current_descartes_postion[2];
    // rx ry rz deg
    current_descrates_position[3] = req.current_descartes_postion[3];
    current_descrates_position[4] = req.current_descartes_postion[4];
    current_descrates_position[5] = req.current_descartes_postion[5];
    
    // xyz mm
    incre_descrates_position_command[0] = req.incre_descartes_command[0];
    incre_descrates_position_command[1] = req.incre_descartes_command[1];
    incre_descrates_position_command[2] = req.incre_descartes_command[2];
    // rx ry rz deg
    incre_descrates_position_command[3] = req.incre_descartes_command[3];
    incre_descrates_position_command[4] = req.incre_descartes_command[4];
    incre_descrates_position_command[5] = req.incre_descartes_command[5];
    
    // 计算新的笛卡尔命令
    Robot_IncreTransTool(current_descrates_position, incre_descrates_position_command, new_descrates_position_command);

    // xyz mm
    res.descartes_command.clear();
    res.descartes_command.push_back( new_descrates_position_command[0] );
    res.descartes_command.push_back( new_descrates_position_command[1] );
    res.descartes_command.push_back( new_descrates_position_command[2] );
    // rxryrz deg
    res.descartes_command.push_back( new_descrates_position_command[3] );
    res.descartes_command.push_back( new_descrates_position_command[4] );
    res.descartes_command.push_back( new_descrates_position_command[5] );

    return true;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_increransTool_server");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("robot_increransTool", handle_function);
    ROS_INFO_STREAM("IncreTransTool Server Already start.");  
    ros::spin();

    return 0;
}

