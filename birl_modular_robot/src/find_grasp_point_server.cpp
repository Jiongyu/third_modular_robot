/**
 * @file find_grasp_point_server.cpp
 * @author your name (you@domain.com)
 * @brief  通过ros服务，根据机器人tcp当前位置姿态（相对于base）、
 *         杆件两端点（相对于carmera)、机器人夹持器基座，求解夹持
 *         点位置姿态（相对于base）。
 * @version 0.1
 * @date 2020-12-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <ros/ros.h>
#include "birl_module_robot/grasp_point.h"
#include "./grasp_intelligent.h"

#define CALIBRATION_DATA

#define POLE_WIDTH 3
#define POINT_WIDTH 6
#define PI_DEG 57.295779
#define PI_RAD 0.0174533

bool handle_function(   birl_module_robot::grasp_point::Request &req,
                        birl_module_robot::grasp_point::Response &res)
{
    ROS_INFO_STREAM("Find Grasp Point Server Get Request.");  

    if(req.p1.size() != POLE_WIDTH || req.p2.size() != POLE_WIDTH || 
        req.current_descartes_postion.size() != POINT_WIDTH)
    {
        ROS_WARN_STREAM("find_grasp_point 输入错误！");
        return false;
    }
    // ROS_INFO_STREAM("1.---------------.");  
    std::cout << req.p1[0] << " " << req.p1[1] << " "<< req.p1[2] << std::endl;
    std::cout << req.p2[0] << " " << req.p2[1] << " "<< req.p2[2] << std::endl;

    for(size_t i =0 ; i < req.current_descartes_postion.size() ; ++i){
        std::cout << req.current_descartes_postion[i] << " ";
    }
    std::cout << std::endl;
    GraspIntelligent Strategy;

#ifdef CALIBRATION_DATA
       // ur3 --> tcp
    Eigen::Matrix3d rotate_matrix_g0;
    Eigen::Matrix3d rotate_matrix_g6;
    // rotate_matrix_g0 << 0, 1, 0, 
    //                     1, 0, 0, 
    //                     0, 0, -1;
    rotate_matrix_g0 << 0, -1, 0, 
                        1, 0, 0, 
                        0, 0, 1;
    rotate_matrix_g6 << 0, -1, 0, 
                        1, 0, 0, 
                        0, 0, 1;
    // carmera --> ur3
    const double incre_z = 0;
    // const double incre_z = 222 - 40.2 - 176.4;

    const std::vector<double>calibrationData_g0{-13.759, -163.739, -261.080 + incre_z, 
                                                358.347 * PI_RAD, 0.151 * PI_RAD, 1.526 * PI_RAD};
    const std::vector<double>calibrationData_g6{-13.759, -177.911, -249.306 + incre_z, 
                                                356.999 * PI_RAD, 359.797 * PI_RAD, 4.036 * PI_RAD};
        
#else
    // ur3 --> tcp
    Eigen::Matrix3d rotate_matrix_g0;
    Eigen::Matrix3d rotate_matrix_g6;
    rotate_matrix_g0 << 1, 0, 0, 
                        0, 1, 0, 
                        0, 0, 1;
    rotate_matrix_g6 << 1, 0, 0, 
                        0, 1, 0, 
                        0, 0, 1;
    // carmera --> ur3
    const std::vector<double>calibrationData_g0{0, 0, 0, 0, 0, 0};
    const std::vector<double>calibrationData_g6{0, 0, 0, 0, 0, 0};
#endif

    // 设置ur3 --> tcp 标定结果
    Strategy.setHandEyeCalibrationBridge(rotate_matrix_g0, GraspIntelligent::G0_GRIPPER);
    Strategy.setHandEyeCalibrationBridge(rotate_matrix_g6, GraspIntelligent::G6_GRIPPER);

    // 设置carmera --> ur3标定结果
    Strategy.setHandEyeCalibrationConsq(calibrationData_g0, GraspIntelligent::G0_GRIPPER);
    Strategy.setHandEyeCalibrationConsq(calibrationData_g6, GraspIntelligent::G6_GRIPPER);

    std::vector<double> grasp_point;
    std::vector<double> p1;
    std::vector<double> p2;
    std::vector<double> tcp;

    grasp_point.resize(POINT_WIDTH);
    p1.resize(POLE_WIDTH);
    p2.resize(POLE_WIDTH);
    tcp.resize(POINT_WIDTH);

    int ret;
    for(size_t i = 0; i < POLE_WIDTH; ++ i){
        p1[i] = req.p1[i];
        p2[i] = req.p2[i];
    }
     for(size_t i = 0; i < POINT_WIDTH; ++ i){
        tcp[i] = req.current_descartes_postion[i];

    }
    // ROS_INFO_STREAM("2.---------------.");  

    // 取反 : base 与 tcp 切换
    if(! req.base){
        ret = Strategy.findGraspPointByLine(p1, p2, tcp, GraspIntelligent::G0_GRIPPER, &grasp_point);
        ROS_INFO_STREAM("G0 tcp");  

    }else{
        ret = Strategy.findGraspPointByLine(p1, p2, tcp, GraspIntelligent::G6_GRIPPER, &grasp_point);
        ROS_INFO_STREAM("G6 tcp");  
    }
    // ROS_INFO_STREAM("3.---------------.");  
    if(!ret){
        ROS_WARN_STREAM("夹持点求解错误！");
        return false;
    }

    res.grasp_point.resize(POINT_WIDTH);
    for(size_t i = 0; i < POINT_WIDTH; ++i)
    {
        if(i < 3){
            res.grasp_point[i] = grasp_point[i];
        }else{
            res.grasp_point[i] = grasp_point[i] * PI_DEG;
        }
        std::cout << grasp_point[i] << std::endl;
    }
    // ROS_INFO_STREAM("4.---------------.");  

    return true;
}

int main(int argc, char *argv[])
{

    

    ros::init(argc, argv, "find_grasp_point_server");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("find_grasp_point", handle_function);

    ROS_INFO_STREAM("Find Grasp Point Server Already start.");  
    ros::spin();
    
    return 0;
}
