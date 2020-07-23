#include <ros/ros.h>
#include "./../kinematics/Kine.h"
#include "birl_module_robot/positive_solution.h"

bool handle_function(   birl_module_robot::positive_solution::Request &req,
                        birl_module_robot::positive_solution::Response &res){

    ROS_INFO_STREAM("Positive Solution Server Get New Request.");                   
    static double Robot_Link_Len[6] = {0.1764,0.2568,0.2932,0.2932,0.2568,0.1764}; //robot link length
    static Kine_CR_FiveDoF_G1 robot5d_G0; // robot based on the gripper0 to get inverse solution
    static Kine_CR_FiveDoF_G2 robot5d_G6;
    robot5d_G0.Set_Length(Robot_Link_Len);
    robot5d_G6.Set_Length(Robot_Link_Len);

    static double decartes_position_value[6] ; // (xyzwpr) unit:(meter,degree)
    static double current_joint_value[5];  //unit:degree

    current_joint_value[0] = req.current_joint_state[0];
    current_joint_value[1] = req.current_joint_state[1];
    current_joint_value[2] = req.current_joint_state[2];
    current_joint_value[3] = req.current_joint_state[3];
    current_joint_value[4] = req.current_joint_state[4];

    if(req.base)
        robot5d_G0.FKine(current_joint_value, decartes_position_value);
    else
        robot5d_G6.FKine(current_joint_value, decartes_position_value);

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
