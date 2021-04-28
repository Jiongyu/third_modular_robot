#include <ros/package.h>
#include <jsoncpp/json/json.h>

#include "./grasp_point_error.h"
#include <math.h>

namespace{

    /**
     * @brief 计算点到直线的垂足点位置
     * 
     * @param pt 
     * @param begin 
     * @param end 
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d GetFootOfPerpendicular(
        const Eigen::Vector3d &pt,     // 直线外一点
        const Eigen::Vector3d &begin,  // 直线开始点
        const Eigen::Vector3d &end)   // 直线结束点
    {
        Eigen::Vector3d retVal;
    
        double dx = begin.x() - end.x();
        double dy = begin.y() - end.y();
        double dz = begin.z() - end.z();

        if(abs(dx) < 0.00000001 && abs(dy) < 0.00000001 && abs(dz) < 0.00000001 )
        {
            retVal = begin;
            return retVal;
        }

        double u =  (pt.x() - begin.x())*(begin.x() - end.x()) +    \
                    (pt.y() - begin.y())*(begin.y() - end.y()) +    \
                    (pt.z() - begin.z())*(begin.z() - end.z());
        
        u = u / ( (dx * dx) + (dy * dy) + (dz * dz) );

        retVal.x() = begin.x() + u * dx;
        retVal.y() = begin.y() + u * dy;
        retVal.z() = begin.z() + u * dz;

        return retVal;
    }

}

geometry_msgs::PoseStamped GraspPointErrorNote::robot_lable_msg_ =  \
    geometry_msgs::PoseStamped();

bool GraspPointErrorNote::if_get_new_mocap_robot_label_info = false;

GraspPointErrorNote::GraspPointErrorNote()
{
    note_file_name_ = ros::package::getPath("birl_module_robot") + "/file/grasp_point_error.txt";
    
    _transform_carmera_to_mocap_label = Eigen::Isometry3d::Identity();
    _transform_mocap_label_to_world = Eigen::Isometry3d::Identity();
    grasp_point_ = Eigen::Isometry3d::Identity();

    pole_position.clear();
    detection_pole_position.clear();
    setCarmeraToMocapLableTrans();

}

GraspPointErrorNote::~GraspPointErrorNote()
{

}

void GraspPointErrorNote::setRosNodeHandle(ros::NodeHandle &nh)
{
    nh_ = std::make_unique<ros::NodeHandle>(nh);
}


void GraspPointErrorNote::setMocapLabelName(const std::string& name)
{
    sub_mocap_label_ = std::make_unique<ros::Subscriber>(std::move( \
        nh_->subscribe<geometry_msgs::PoseStamped>(name,     \
        1,subMocapLabelCallback)));       
}

int GraspPointErrorNote::noteData(const Eigen::Isometry3d &transform, const std::vector<double> &grasp_point,const std::vector<Eigen::Vector3d>& pole_position)
{

    int ret;
    ret = getNewMocapLabelInfo();
    if(ret < 0){
        ROS_WARN_STREAM("Can not ceceive mocap topic[/robot_mocap_label] info.");  
        return ret;
    }

    // _transform_mocap_label_to_world(0,0) = 1;
    // _transform_mocap_label_to_world(0,1) = 0;
    // _transform_mocap_label_to_world(0,2) = 0;
    // _transform_mocap_label_to_world(0,3) = 0;

    // _transform_mocap_label_to_world(1,0) = 0;
    // _transform_mocap_label_to_world(1,1) = 1;
    // _transform_mocap_label_to_world(1,2) = 0;
    // _transform_mocap_label_to_world(1,3) = 0;

    // _transform_mocap_label_to_world(2,0) = 0;
    // _transform_mocap_label_to_world(2,1) = 0;
    // _transform_mocap_label_to_world(2,2) = 1;
    // _transform_mocap_label_to_world(2,3) = 0;

    // _transform_mocap_label_to_world(3,0) = 0;
    // _transform_mocap_label_to_world(3,1) = 0;
    // _transform_mocap_label_to_world(3,2) = 0;
    // _transform_mocap_label_to_world(3,3) = 1;

    caculateGraspPoint(transform, grasp_point, pole_position);
    
    // caculateGraspPointError();

    writeGraspPointErrorToFile();

}

bool GraspPointErrorNote::setCarmeraToMocapLableTrans()
{
    Json::Value root;
    Json::Reader reader;
    std::ifstream ifs;

    const static std::string json_file_path = ros::package::getPath("birl_module_robot") + "/file/pole_position.json";

    ifs.open(json_file_path);

    if (! ifs.is_open())
    {
        ROS_WARN_STREAM("Can not open pole_position.json.");  
        return false;
    }

    if (! reader.parse(ifs, root))
    {
        ROS_WARN_STREAM("Can not parse pole_position.json file!!!");
        return false;
    }

    _transform_carmera_to_mocap_label(0,0) = root["carmear_to_mocap_marker_transformation"]["r11"].asDouble(); 
    _transform_carmera_to_mocap_label(0,1) = root["carmear_to_mocap_marker_transformation"]["r12"].asDouble();
    _transform_carmera_to_mocap_label(0,2) = root["carmear_to_mocap_marker_transformation"]["r13"].asDouble();
    _transform_carmera_to_mocap_label(0,3) = root["carmear_to_mocap_marker_transformation"]["r14"].asDouble();

    _transform_carmera_to_mocap_label(1,0) = root["carmear_to_mocap_marker_transformation"]["r21"].asDouble();
    _transform_carmera_to_mocap_label(1,1) = root["carmear_to_mocap_marker_transformation"]["r22"].asDouble();
    _transform_carmera_to_mocap_label(1,2) = root["carmear_to_mocap_marker_transformation"]["r23"].asDouble();
    _transform_carmera_to_mocap_label(1,3) = root["carmear_to_mocap_marker_transformation"]["r24"].asDouble();

    _transform_carmera_to_mocap_label(2,0) = root["carmear_to_mocap_marker_transformation"]["r31"].asDouble();
    _transform_carmera_to_mocap_label(2,1) = root["carmear_to_mocap_marker_transformation"]["r32"].asDouble();
    _transform_carmera_to_mocap_label(2,2) = root["carmear_to_mocap_marker_transformation"]["r33"].asDouble();
    _transform_carmera_to_mocap_label(2,3) = root["carmear_to_mocap_marker_transformation"]["r34"].asDouble();

    _transform_carmera_to_mocap_label(3,0) = root["carmear_to_mocap_marker_transformation"]["r41"].asDouble();
    _transform_carmera_to_mocap_label(3,1) = root["carmear_to_mocap_marker_transformation"]["r42"].asDouble();
    _transform_carmera_to_mocap_label(3,2) = root["carmear_to_mocap_marker_transformation"]["r43"].asDouble();
    _transform_carmera_to_mocap_label(3,3) = root["carmear_to_mocap_marker_transformation"]["r44"].asDouble();

    pole_position.push_back(    Eigen::Vector3d(root["real_pole_position"]["p_11"]["x"].asDouble(),
                                                root["real_pole_position"]["p_11"]["y"].asDouble(),
                                                root["real_pole_position"]["p_11"]["z"].asDouble())
                            );

    pole_position.push_back(    Eigen::Vector3d(root["real_pole_position"]["p_12"]["x"].asDouble(),
                                                root["real_pole_position"]["p_12"]["y"].asDouble(),
                                                root["real_pole_position"]["p_12"]["z"].asDouble())
                            );

    pole_position.push_back(    Eigen::Vector3d(root["real_pole_position"]["p_21"]["x"].asDouble(),
                                                root["real_pole_position"]["p_21"]["y"].asDouble(),
                                                root["real_pole_position"]["p_21"]["z"].asDouble())
                            );

    pole_position.push_back(    Eigen::Vector3d(root["real_pole_position"]["p_22"]["x"].asDouble(),
                                                root["real_pole_position"]["p_22"]["y"].asDouble(),
                                                root["real_pole_position"]["p_22"]["z"].asDouble())
                            );

    // 真实杆件环境向量
    pole_position[0] = (pole_position[0] + pole_position[1]) / 2;
    pole_position[1] = (pole_position[2] + pole_position[3]) / 2;

    pole_position.pop_back();
    pole_position.pop_back();

    std::cout << "1.-----------------------\n"; 
    std::cout << _transform_carmera_to_mocap_label.matrix() << std::endl;
    std::cout << "2.-----------------------\n";
    std::cout << pole_position[0].matrix() << std::endl;
    std::cout << "3.-----------------------\n"; 
    std::cout << pole_position[1].matrix() << std::endl;

    return true;
}


int GraspPointErrorNote::getNewMocapLabelInfo()
{
    // 超时 5s
    ros::Rate rate(5);
    int index = 0;
    while (index < 25)
    {
        if(if_get_new_mocap_robot_label_info)
        {
            break;
        }
        ros::spinOnce();    
        rate.sleep();
        index ++;
    }
    if_get_new_mocap_robot_label_info = false;
    if(index >= 25)
    {
        return -1;
    }

    tf::Quaternion quat;
    tf::quaternionMsgToTF(robot_lable_msg_.pose.orientation, quat);
    tf::Matrix3x3 temp(quat);

    _transform_mocap_label_to_world(0,0) = temp.getRow(0)[0];
    _transform_mocap_label_to_world(0,1) = temp.getRow(0)[1];
    _transform_mocap_label_to_world(0,2) = temp.getRow(0)[2];
    _transform_mocap_label_to_world(0,3) = robot_lable_msg_.pose.position.x;

    _transform_mocap_label_to_world(1,0) = temp.getRow(1)[0];
    _transform_mocap_label_to_world(1,1) = temp.getRow(1)[1];
    _transform_mocap_label_to_world(1,2) = temp.getRow(1)[2];
    _transform_mocap_label_to_world(1,3) = robot_lable_msg_.pose.position.y;

    _transform_mocap_label_to_world(2,0) = temp.getRow(1)[0];
    _transform_mocap_label_to_world(2,1) = temp.getRow(1)[1];
    _transform_mocap_label_to_world(2,2) = temp.getRow(1)[2];
    _transform_mocap_label_to_world(2,3) = robot_lable_msg_.pose.position.z;

    _transform_mocap_label_to_world(3,0) = 0;
    _transform_mocap_label_to_world(3,1) = 0;
    _transform_mocap_label_to_world(3,2) = 0;
    _transform_mocap_label_to_world(3,3) = 1;

    return 0;
}

void GraspPointErrorNote::subMocapLabelCallback(  \
        const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        robot_lable_msg_ = *msg;
        if_get_new_mocap_robot_label_info = true;
}

void GraspPointErrorNote::caculateGraspPoint(   const Eigen::Isometry3d &transform,     \
                                                const std::vector<double> &grasp_point, \
                                                const std::vector<Eigen::Vector3d>& pole_position)
{
    Eigen::Vector3d translation (grasp_point[0], grasp_point[1], grasp_point[2]);

    // 欧拉角
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix  = Eigen::AngleAxisd(grasp_point[3], Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(grasp_point[4], Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(grasp_point[5], Eigen::Vector3d::UnitX());

    grasp_point_.rotate(rotationMatrix);
    grasp_point_.pretranslate(translation);

    std::cout << "4.-----------------------\n"; 
    std::cout << grasp_point_.matrix() << std::endl;
    grasp_point_ = transform.inverse() * _transform_carmera_to_mocap_label * _transform_mocap_label_to_world * grasp_point_;
    detection_pole_position.push_back(transform.inverse() * _transform_carmera_to_mocap_label * _transform_mocap_label_to_world * pole_position[0]);
    detection_pole_position.push_back(transform.inverse() * _transform_carmera_to_mocap_label * _transform_mocap_label_to_world * pole_position[1]);
    std::cout << "5.-----------------------\n"; 
    std::cout << grasp_point_.matrix() << std::endl;

}

void GraspPointErrorNote::caculateGraspPointError()
{
    // grasp_point_error = GetFootOfPerpendicular(grasp_point_, pole_position[0], pole_position[1]);
    // std::cout << "6.-----------------------\n"; 
    // std::cout << grasp_point_error.matrix() << std::endl;
    // grasp_point_error = grasp_point_error - grasp_point_;
    // std::cout << "7.-----------------------\n"; 
    // std::cout << grasp_point_error.matrix() << std::endl;
}


int GraspPointErrorNote::writeGraspPointErrorToFile()
{
    file_.open(note_file_name_, std::ios::app);

    if (! file_.is_open())
    {
        ROS_WARN_STREAM("Can not open grasp_point_error.txt.");  
        return -1;
    }

    file_   << detection_pole_position[0].x() << " " \
            << detection_pole_position[0].y() << " " \
            << detection_pole_position[0].z() << std::endl;

    file_   << detection_pole_position[1].x() << " " \
            << detection_pole_position[1].y() << " " \
            << detection_pole_position[1].z() << std::endl;

    Eigen::Matrix3d rotate_matrix;
    rotate_matrix <<    grasp_point_(0,0), grasp_point_(0,1), grasp_point_(0,2),
                        grasp_point_(1,0), grasp_point_(1,1), grasp_point_(1,2),
                        grasp_point_(2,0), grasp_point_(2,1), grasp_point_(2,2);
    Eigen::Vector3d rpy_v = rotate_matrix.eulerAngles(2,1,0);

    file_   << grasp_point_(0,3) << " " \
            << grasp_point_(1,3) << " " \
            << grasp_point_(2,3) << " " \
            << rpy_v.x() << " " \
            << rpy_v.y() << " " \
            << rpy_v.z() << std::endl << std::endl;

    grasp_point_ = Eigen::Isometry3d::Identity();

    file_.close();
}
