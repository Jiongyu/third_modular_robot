#ifndef GRASP_POINT_ERROR_NOTE_H
#define GRASP_POINT_ERROR_NOTE_H

#include <string>
#include <memory>
#include <atomic>
#include <vector>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class GraspPointErrorNote
{
public:
    GraspPointErrorNote();
    ~GraspPointErrorNote();
    

    void setRosNodeHandle(ros::NodeHandle &nh);

    /**
     * @brief 设置mocap标记话题名称
     * 
     * @param name mocap 标记话题名称 
     */
    void setMocapLabelName(const std::string& name);

    /**
     * @brief 记录数据
     * @param transform  相机相对与机器人基座端坐标系
     * @param grasp_point  抓夹点， 相对于机器人基座标系
     * @return int (0:success, -1 error)
     */
    int noteData(const Eigen::Isometry3d &transform, const std::vector<double> &grasp_point, const std::vector<Eigen::Vector3d>& pole_position);

private:

    std::unique_ptr<ros::NodeHandle> nh_;

    // mocap 标记节点记录
    std::unique_ptr<ros::Subscriber> sub_mocap_label_;

    // 记录文件句柄
    std::ofstream file_;

    // 文件名
    std::string note_file_name_;

    static geometry_msgs::PoseStamped robot_lable_msg_;

    static bool if_get_new_mocap_robot_label_info;

    Eigen::Isometry3d _transform_carmera_to_mocap_label;
    Eigen::Isometry3d _transform_mocap_label_to_world;

    // 世界坐标系下抓夹点
    Eigen::Isometry3d grasp_point_;
    // 抓夹点位置误差
    Eigen::Vector3d grasp_point_error;

    // 世界坐标系下杆件两端点
    std::vector<Eigen::Vector3d> pole_position;
    // 机器人坐标系下杆件两端点
    std::vector<Eigen::Vector3d> detection_pole_position;

    /**
     * @brief Set the Carmera To Mocap Lable Trans object
     * 
     * @return true :success
     * @return false :failed
     */
    bool setCarmeraToMocapLableTrans();


    /**
     * @brief 循环等待接收mocap label 消息
     * 
     * @return int (0:success, -1 error)
     */
    int getNewMocapLabelInfo();

    /**
     * @brief 接收mocap  label  回调函数
     * 
     */
    static void subMocapLabelCallback(  \
        const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief 
     * 
     * @param transform  相机相对与机器人基座端坐标系
     * @param grasp_point  抓夹点， 相对于机器人基座标系
     */
    void caculateGraspPoint(const Eigen::Isometry3d &transform,     \
                            const std::vector<double> &grasp_point, \
                            const std::vector<Eigen::Vector3d>& pole_position);


    void caculateGraspPointError();

    /**
     * @brief 写入文件夹持点误差
     * 
     */
    int writeGraspPointErrorToFile();

};

#endif