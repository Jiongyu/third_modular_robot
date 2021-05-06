/**
 * @file find_grasp_point_server.cpp
 * @author your name (you@domain.com)
 * @brief  通过ros服务，根据机器人tcp当前位置姿态（相对于base）、
 *         杆件两端点（相对于camera)、机器人夹持器基座，求解夹持
 *         点位置姿态（相对于base）。
 * @version 0.1
 * @date 2020-12-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <jsoncpp/json/json.h>
#include <fstream>

#include "birl_module_robot/grasp_point.h"
#include "./grasp_intelligent.h"
#include "./grasp_point_error.h"

// 是否采用标定结果
#define CALIBRATION_DATA

// 是否记录数据
// #define ERROR_NOTE

#define POLE_WIDTH 3
#define POINT_WIDTH 6
#define JOINT_WIDTH 5
#define PI_DEG_ 57.295779
#define PI_RAD_ 0.0174533

std::vector<double> grasp_point;
std::vector<double> p1;
std::vector<double> p2;
std::vector<double> tcp;
std::vector<double> current_joint_pos;

#ifdef ERROR_NOTE
    std::unique_ptr<GraspPointErrorNote>Note = std::make_unique<GraspPointErrorNote>();
#endif

bool set_eye_hand_calibration_data(std::unique_ptr<GraspIntelligent>& t_strategy)
{

#ifdef CALIBRATION_DATA
    /**
     * @brief 通过json文件导入实际标定结果
     */

    Json::Value root;
    Json::Reader reader;
    std::ifstream ifs;

    const static std::string json_file_path = ros::package::getPath("birl_module_robot") + "/file/eye_hand_calibration_consquence.json";

    ifs.open(json_file_path);

    if (! ifs.is_open())
    {
        ROS_WARN_STREAM("Can not open eye_hand_calibration_consquence.json.");  
        return false;
    }

    if (! reader.parse(ifs, root))
    {
        ROS_WARN_STREAM("Can not parse eye_hand_calibration_consquence.json file!!!");
        return false;
    }

   // ur3 --> tcp
    static Eigen::Matrix3d rotate_matrix_g0;
    static Eigen::Matrix3d rotate_matrix_g6;

    rotate_matrix_g0 << root["g0_bridge_to_tcp_transformation"]["r11"].asDouble(), 
                        root["g0_bridge_to_tcp_transformation"]["r12"].asDouble(),
                        root["g0_bridge_to_tcp_transformation"]["r13"].asDouble(),
                        root["g0_bridge_to_tcp_transformation"]["r21"].asDouble(),
                        root["g0_bridge_to_tcp_transformation"]["r22"].asDouble(),
                        root["g0_bridge_to_tcp_transformation"]["r23"].asDouble(),
                        root["g0_bridge_to_tcp_transformation"]["r31"].asDouble(),
                        root["g0_bridge_to_tcp_transformation"]["r32"].asDouble(),
                        root["g0_bridge_to_tcp_transformation"]["r33"].asDouble();

    rotate_matrix_g6 << root["g6_bridge_to_tcp_transformation"]["r11"].asDouble(), 
                        root["g6_bridge_to_tcp_transformation"]["r12"].asDouble(),
                        root["g6_bridge_to_tcp_transformation"]["r13"].asDouble(),
                        root["g6_bridge_to_tcp_transformation"]["r21"].asDouble(),
                        root["g6_bridge_to_tcp_transformation"]["r22"].asDouble(),
                        root["g6_bridge_to_tcp_transformation"]["r23"].asDouble(),
                        root["g6_bridge_to_tcp_transformation"]["r31"].asDouble(),
                        root["g6_bridge_to_tcp_transformation"]["r32"].asDouble(),
                        root["g6_bridge_to_tcp_transformation"]["r33"].asDouble();

    // camera --> ur3
    const static std::vector<double>calibrationData_g0{
                        root["g0_camera_to_bridge_transformation"]["x"].asDouble(), 
                        root["g0_camera_to_bridge_transformation"]["y"].asDouble(),
                        root["g0_camera_to_bridge_transformation"]["z"].asDouble(), 
                        root["g0_camera_to_bridge_transformation"]["rx"].asDouble() * PI_RAD_, 
                        root["g0_camera_to_bridge_transformation"]["ry"].asDouble() * PI_RAD_, 
                        root["g0_camera_to_bridge_transformation"]["rz"].asDouble() * PI_RAD_,  
    };

    const static std::vector<double>calibrationData_g6{
                        root["g6_camera_to_bridge_transformation"]["x"].asDouble(), 
                        root["g6_camera_to_bridge_transformation"]["y"].asDouble(),
                        root["g6_camera_to_bridge_transformation"]["z"].asDouble(), 
                        root["g6_camera_to_bridge_transformation"]["rx"].asDouble() * PI_RAD_, 
                        root["g6_camera_to_bridge_transformation"]["ry"].asDouble() * PI_RAD_, 
                        root["g6_camera_to_bridge_transformation"]["rz"].asDouble() * PI_RAD_,  
    };

#else
    /**
     * @brief 理想标定结果，测试使用
     */

    // ur3 --> tcp
    static Eigen::Matrix3d rotate_matrix_g0;
    static Eigen::Matrix3d rotate_matrix_g6;
    rotate_matrix_g0 << 1, 0, 0, 
                        0, 1, 0, 
                        0, 0, 1;
    rotate_matrix_g6 << 1, 0, 0, 
                        0, 1, 0, 
                        0, 0, 1;
    // camera --> ur3
    const static std::vector<double>calibrationData_g0{0, 0, 0, 0, 0, 0};
    const static std::vector<double>calibrationData_g6{0, 0, 0, 0, 0, 0};
#endif

    bool ret = true;
    // 设置ur3 --> tcp 标定结果
    ret &= t_strategy->setHandEyeCalibrationBridge(rotate_matrix_g0, GraspIntelligent::G0_GRIPPER);
    ret &= t_strategy->setHandEyeCalibrationBridge(rotate_matrix_g6, GraspIntelligent::G6_GRIPPER);

    // 设置camera --> ur3标定结果
    ret &= t_strategy->setHandEyeCalibrationConsq(calibrationData_g0, GraspIntelligent::G0_GRIPPER);
    ret &= t_strategy->setHandEyeCalibrationConsq(calibrationData_g6, GraspIntelligent::G6_GRIPPER);

    if(!ret){return false;}

    return true;
}

bool handle_function(   birl_module_robot::grasp_point::Request &req,
                        birl_module_robot::grasp_point::Response &res)
{
    ROS_INFO_STREAM("Find Grasp Point Server Get Request.");  

    if(req.p1.size() != POLE_WIDTH || req.p2.size() != POLE_WIDTH || 
        req.current_descartes_postion.size() != POINT_WIDTH || 
        req.current_joint_postion.size() != JOINT_WIDTH)
    {
        ROS_WARN_STREAM("find_grasp_point 输入错误！");
        return false;
    }

    current_joint_pos.resize(JOINT_WIDTH);
    for(int i : req.current_joint_postion)
    {
        current_joint_pos[i] = static_cast<double>(req.current_joint_postion[i]);
        // ROS_WARN_STREAM("[ " << current_joint_pos[i] << " ]");
    }

    // ROS_INFO_STREAM("1.---------------.");  
    // std::cout << req.p1[0] << " " << req.p1[1] << " "<< req.p1[2] << std::endl;
    // std::cout << req.p2[0] << " " << req.p2[1] << " "<< req.p2[2] << std::endl;

    // for(size_t i =0 ; i < req.current_descartes_postion.size() ; ++i){
    //     std::cout << req.current_descartes_postion[i] << " ";
    // }
    // std::cout << std::endl;
    std::unique_ptr<GraspIntelligent>Strategy = std::make_unique<GraspIntelligent>();

    if (!set_eye_hand_calibration_data(Strategy)){
        return false;
    }

    int ret;
    for(size_t i = 0; i < POLE_WIDTH; ++ i){
        p1[i] = req.p1[i];
        p2[i] = req.p2[i];
        // std::cout << p1[i] << "  " << p2[i] << std::endl;
    }
     for(size_t i = 0; i < POINT_WIDTH; ++ i){
        tcp[i] = req.current_descartes_postion[i];

    }
    // ROS_INFO_STREAM("2.---------------.");  

    // 取反 : base 与 tcp 切换
    if(! req.base){
        ret = Strategy->findGraspPointBySearch(p1, p2, tcp, current_joint_pos, GraspIntelligent::G0_GRIPPER, &grasp_point);
        // ret = Strategy->findGraspPointByLine(p1, p2, tcp, GraspIntelligent::G0_GRIPPER, &grasp_point);
        ROS_INFO_STREAM("G0 tcp");  

    }else{
        ret = Strategy->findGraspPointBySearch(p1, p2, tcp, current_joint_pos, GraspIntelligent::G6_GRIPPER, &grasp_point);
        // ret = Strategy->findGraspPointByLine(p1, p2, tcp, GraspIntelligent::G6_GRIPPER, &grasp_point);
        ROS_INFO_STREAM("G6 tcp");  
    }
    // ROS_INFO_STREAM("3.---------------.");  
    if(!ret){
        ROS_WARN_STREAM("夹持点求解错误！");
        return false;
    }

#ifdef ERROR_NOTE
    Note->setMocapLabelName("/robot_mocap_label");
    ret = Note->noteData(Strategy->getTransformCarmeraToBase(), grasp_point, Strategy->getPolePosition());
    if(ret < 0){
        ROS_WARN_STREAM("夹持点位姿误差记录错误");
    }
#endif

    res.grasp_point.resize(POINT_WIDTH);
    for(size_t i = 0; i < POINT_WIDTH; ++i)
    {
        if(i < 3){
            res.grasp_point[i] = grasp_point[i];
        }else{
            res.grasp_point[i] = grasp_point[i] * PI_DEG_;
        }
        // std::cout << grasp_point[i] << std::endl;
    }
    // ROS_INFO_STREAM("4.---------------.");  

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "find_grasp_point_server");
    ros::NodeHandle nh;

#ifdef ERROR_NOTE
    Note->setRosNodeHandle(nh);
#endif
    grasp_point.resize(POINT_WIDTH);
    p1.resize(POLE_WIDTH);
    p2.resize(POLE_WIDTH);
    tcp.resize(POINT_WIDTH);

    ros::ServiceServer server = nh.advertiseService("find_grasp_point", handle_function);

    ROS_INFO_STREAM("Find Grasp Point Server Already start.");  
    ros::spin();
    
    return 0;
}
