#include "find_grasp_point.h"


FindGraspPoint::FindGraspPoint(/* args */){

    _transform_carmera_to_tcp_gripper0 = Eigen::Isometry3d::Identity();
    _transform_carmera_to_tcp_gripper6 = Eigen::Isometry3d::Identity();
    _transform_tcp_to_base = Eigen::Isometry3d::Identity();
};


bool FindGraspPoint::setHandEyeCalibrationConsq(const std::vector<double>& calibrationData, 
                                                const enum GRIPPER which_gripper)
{
    if(calibrationData.size() != 6)
    {
        ROS_WARN_STREAM("设置标定结果输入错误!");  
        return false;
    }

    if(which_gripper == G0_GRIPPER)
    {
        convertVectorToTransMatrix(calibrationData, &_transform_carmera_to_tcp_gripper0);
    }
    else if(which_gripper == G6_GRIPPER)
    {
        convertVectorToTransMatrix(calibrationData, &_transform_carmera_to_tcp_gripper6);
    }else
    {
        return false;
    }
    
    #ifdef DEBUG_FIND_GRASP_POINT
        std::cout << "1.变换矩阵：carmera --> tcp:" << std::endl << (which_gripper == G0_GRIPPER ? 
                                                                    _transform_carmera_to_tcp_gripper0.matrix() : 
                                                                    _transform_carmera_to_tcp_gripper6.matrix())<< std::endl;
    #endif

    return true;
}


bool FindGraspPoint::findGraspPointByLine(  const std::vector<double>& point1, const std::vector<double>& point2, 
                                                const std::vector<double>& robot_tcp, const enum GRIPPER which_gripper, 
                                                std::vector<double> *grasp_point)
{
    if(point1.size() != 3 || point2.size() != 3 || robot_tcp.size() != 6 || grasp_point->size() != 6)
    {
        std::cout <<"求解夹持点输入错误!" << std::endl;  
        return false;
    }
    _which_gripper = which_gripper;

    // 1.计算转换矩阵 tcp --> base 
    calculateRobotTcpToBase(robot_tcp);

    #ifdef DEBUG_FIND_GRASP_POINT
        std::cout << "2. 变换矩阵: tcp --> base:" << std::endl << _transform_tcp_to_base.matrix() << std::endl;
    #endif

    // 2. 计算夹持点位置 相对base坐标系
    calculateGaspPointPosition(point1, point2, grasp_point);
    // 3. 计算夹持点姿态 相对base坐标系
    calculateGraspPointPosture(point1, point2, grasp_point);


    return true;
};

void FindGraspPoint::calculateRobotTcpToBase(const std::vector<double>& tvector){

    convertVectorToTransMatrix(tvector, &_transform_tcp_to_base);
}

void FindGraspPoint::convertVectorToTransMatrix(const std::vector<double>& tvector, Eigen::Isometry3d *matrix){

    // 平移向量
    Eigen::Vector3d translation (tvector[0], tvector[1], tvector[2]);

    // 欧拉角
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix  = Eigen::AngleAxisd(tvector[5], Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(tvector[4], Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(tvector[3], Eigen::Vector3d::UnitX());
    matrix->rotate(rotationMatrix);
    matrix->pretranslate(translation);
}

void FindGraspPoint::calculateGraspPointPosture(const std::vector<double>& point1, const std::vector<double>& point2, 
                                                std::vector<double> *grasp_point)
{
    // 计算杆件向量ng
    // 将杆件位置转换base坐标系
    Eigen::Vector3d ng (point2[0] - point1[0], 
                        point2[1] - point1[1], 
                        point2[2] - point1[2]);
    if(_which_gripper == G0_GRIPPER)
    {
        ng = _transform_tcp_to_base * _transform_carmera_to_tcp_gripper0 * ng;
    }
    else
    {
        ng = _transform_tcp_to_base * _transform_carmera_to_tcp_gripper6 * ng;
    }
    // x
    ng = ng.normalized();

    // 夹持点向量
    Eigen::Vector3d q(grasp_point->at(0),grasp_point->at(1),grasp_point->at(2));
    q = q.normalized();

    Eigen::Vector3d m = q.cross(ng);

    // ag(ag_x, ag_y, ag_z) z
    Eigen::Vector3d ag = ng.cross(m);
    // og y 
    Eigen::Vector3d og = ag.cross(ng);

    // rpy
    // x
    grasp_point->at(3) = atan2(ag(1), ag(0));
    // y
    grasp_point->at(4) = atan2( 
                                -ag(2), 
                                (ag(0) * cos(grasp_point->at(3)) + ag(1) * sin(grasp_point->at(3)))
                                );
    // z
    grasp_point->at(5) = atan2(
                                (-ng(1) * cos(grasp_point->at(3)) + ng(0) * sin(grasp_point->at(3))),
                                (og(1) * cos(grasp_point->at(3)) - og(0) * sin(grasp_point->at(3)))
                                );

}


void FindGraspPoint::calculateGaspPointPosition(const std::vector<double>& point1, const std::vector<double>& point2, 
                                                        std::vector<double> *grasp_point)
{
    // 将杆件位置转换tcp坐标系
    Eigen::Vector3d p1 (point1[0], point1[1], point1[2]);
    Eigen::Vector3d p2 (point2[0], point2[1], point2[2]);
    Eigen::Isometry3d temp_p1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d temp_p2 = Eigen::Isometry3d::Identity();
    temp_p1.pretranslate(p1);
    temp_p2.pretranslate(p2);

    if(_which_gripper == G0_GRIPPER)
    {
        temp_p1 = _transform_carmera_to_tcp_gripper0 * temp_p1;
        temp_p2 = _transform_carmera_to_tcp_gripper0 * temp_p2;
    }
    else
    {
        temp_p1 = _transform_carmera_to_tcp_gripper6 * temp_p1;
        temp_p2 = _transform_carmera_to_tcp_gripper6 * temp_p2;
    }
    p1 << temp_p1(0,3), temp_p1(1,3), temp_p1(2, 3);
    p2 << temp_p2(0,3), temp_p2(1,3), temp_p2(2, 3);

    #ifdef DEBUG_FIND_GRASP_POINT
        std::cout << "4. p1 杆件点位置tcp坐标系:" << std::endl << p1 << std::endl;
        std::cout << "5. p2 杆件点位置tcp坐标系:" << std::endl << p2 << std::endl;
    #endif
    
    // 参数法计算
    double a = p2(0) - p1(0);
    double b = p2(1) - p1(1);
    double c = p2(2) - p1(2);
    double t =  - ( p1(0) * a + 
                    p1(1) * b + 
                    p1(2) * c ) 
                / (pow(a, 2) + pow(b, 2) + pow(c, 2));

    grasp_point->at(0) = a * t + p1(0);
    grasp_point->at(1) = b * t + p1(1);
    grasp_point->at(2) = c * t + p1(2);

    //  转至base坐标系 
    Eigen::Vector3d translation (grasp_point->at(0), grasp_point->at(1), grasp_point->at(2));
    translation = _transform_tcp_to_base * translation;
    grasp_point->at(0) = translation(0);
    grasp_point->at(1) = translation(1);
    grasp_point->at(2) = translation(2);

}
