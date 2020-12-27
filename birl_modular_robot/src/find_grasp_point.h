#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define DEBUG_FIND_GRASP_POINT 

class FindGraspPoint
{

public:

    enum GRIPPER{
        G0_GRIPPER = 0,
        G6_GRIPPER,
    };

    FindGraspPoint(/* args */);

    ~FindGraspPoint(){};

    /**
     * @brief 设置机器人手眼标定结果
     * 
     * @param calibrationData : 机器人手眼标定(x,y,z --> mm; rpy --> rad)
     * @param which_gripper : 夹持器编号(0: 夹持器G0； 6：夹持器G6)
     * @return true 
     * @return false 
     */
    bool setHandEyeCalibrationConsq(const std::vector<double>& calibrationData, const enum GRIPPER which_gripper);

    /**
     * @brief 根据杆件检测到的两点位置和机器人末端姿态，计算机器人夹持点位姿
     * 
     * @param point1 : 杆件位置点1 (xyz --> mm) 相对相机坐标系
     * @param point2 : 杆件位置点2 (xyz --> mm) 相对相机坐标系
     * @param robot_tcp : 机器人当前tcp （xyz-->mm,  rpy --> rad） 相对机器人base端
     * @param which_gripper : 夹持器编号(0: 夹持器G0； 6：夹持器G6)
     * @param grasp_point : 机器人求解夹持点 （xyz-->mm,  rpy --> rad）相对机器人base端
     * @return true 求解成功
     * @return false 求解失败
     */
    bool findGraspPointByLine(  const std::vector<double>& point1, const std::vector<double>& point2, 
                                const std::vector<double>& robot_tcp, const enum GRIPPER which_gripper, 
                                std::vector<double> *grasp_point);


private:

    /**
     * @brief 夹持器编号(0: 夹持器G0； 6：夹持器G6)
     * 
     */
    enum GRIPPER _which_gripper;

    /**
     * @brief 手眼标定变换矩阵，相机端相对机器人tcp端
     * 
     */
    Eigen::Isometry3d   _transform_carmera_to_tcp_gripper0,
                        _transform_carmera_to_tcp_gripper6;

    /**
     * @brief 机器人tcp端相对于base端变换矩阵
     * 
     */
    Eigen::Isometry3d _transform_tcp_to_base;


private:

    /**
     * @brief 计算机器人tcp端向量至变换矩阵
     * 
     * @param tvector : 向量(x,y,z --> mm; Rx, Ry, Rz --> rad)
     */
    void calculateRobotTcpToBase(const std::vector<double>& tvector);

    /**
     * @brief 欧拉角转旋转矩阵
     * 
     * @param theta 
     * @return Eigen::Matrix3d 
     */
    Eigen::Matrix3d eulerAnglesToRotationMatrix(const Eigen::Vector3d &theta);

    /**
     * @brief 转换机器人向量 至 变换矩阵
     * 
     * @param tvector : 向量(x,y,z --> mm; Rx, Ry, Rz --> rad)
     */
    void convertVectorToTransMatrix(const std::vector<double>& tvector, Eigen::Isometry3d *matrix);

    /**
     * @brief 计算夹持点位姿
     * 
     * @param point1 : 杆件位置点1 (xyz --> mm) 相对相机坐标系
     * @param point2 : 杆件位置点2 (xyz --> mm) 相对相机坐标系
     * @param grasp_point grasp_point : 夹持点 (xyz --> mm， rpy --> rad) 相对tcp
     */
    void calculateGraspPointPosture(const std::vector<double>& point1, const std::vector<double>& point2, 
                                            std::vector<double> *grasp_point);

    /**
     * @brief 根据点到直线距离计算夹持点
     * 
     * @param point1 : 杆件位置点1 (xyz --> mm) 相对相机坐标系
     * @param point2 : 杆件位置点2 (xyz --> mm) 相对相机坐标系
     * @param grasp_point : 夹持点 (xyz --> mm) 相对base
     * @return true 
     * @return false 
     */
    void calculateGaspPointPosition(const std::vector<double>& point1, const std::vector<double>& point2, 
                                            std::vector<double> *grasp_point);

    /**
     * @brief 计算夹持点姿态 
     * 
     * @param rotate_matrix : 杆件姿态旋转矩阵
     * @param grasp_point : 夹持点 (xyz --> mm) 相对tcp
     */
    void calculateGaspPointPostureBetweenPoleTcp(const Eigen::Matrix3d &rotate_matrix,std::vector<double> *grasp_point);
};
