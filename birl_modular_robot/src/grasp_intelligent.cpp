#include "grasp_intelligent.h"


GraspIntelligent::GraspIntelligent(/* args */){

    _transform_carmera_to_tcp_gripper0 = Eigen::Isometry3d::Identity();
    _transform_carmera_to_tcp_gripper6 = Eigen::Isometry3d::Identity();

    _transform_carmera_to_birdge_gripper0 = Eigen::Isometry3d::Identity();
    _transform_carmera_to_birdge_gripper6 = Eigen::Isometry3d::Identity();

    _transform_birdge_to_tcp_gripper0 = Eigen::Isometry3d::Identity(),
    _transform_birdge_to_tcp_gripper6 = Eigen::Isometry3d::Identity();

    _transform_tcp_to_base = Eigen::Isometry3d::Identity();
};

bool GraspIntelligent::setHandEyeCalibrationBridge(const Eigen::Matrix3d& bridge ,const enum GRIPPER which_gripper)
{
    if(bridge.determinant() != 1)
    {
        std::cout  << __PRETTY_FUNCTION__ << "设置标定结果输入错误!" << std::endl;  
        return false;
    }

    if(which_gripper == G0_GRIPPER)
    {
        _transform_birdge_to_tcp_gripper0.rotate(bridge);
    }
    else if(which_gripper == G6_GRIPPER)
    {
        _transform_birdge_to_tcp_gripper6.rotate(bridge);
    }else
    {
        return false;
    }
    
    #ifdef DEBUG_GRASP_INTELLIGENT
        std::cout << "1.变换矩阵：carmera --> birdge:" << std::endl << (which_gripper == G0_GRIPPER ? 
                                                                    _transform_birdge_to_tcp_gripper0.matrix() : 
                                                                    _transform_birdge_to_tcp_gripper6.matrix())<< std::endl;
    #endif

    return true; 
}


bool GraspIntelligent::setHandEyeCalibrationConsq(const std::vector<double>& calibrationData, 
                                                const enum GRIPPER which_gripper)
{
    if(calibrationData.size() != 6)
    {
        std::cout  << __PRETTY_FUNCTION__ << "设置标定结果输入错误!" << std::endl;  
        return false;
    }

    if(which_gripper == G0_GRIPPER)
    {
        convertVectorToTransMatrix(calibrationData, &_transform_carmera_to_birdge_gripper0);
        _transform_carmera_to_tcp_gripper0 = _transform_birdge_to_tcp_gripper0 * _transform_carmera_to_birdge_gripper0;
    }
    else if(which_gripper == G6_GRIPPER)
    {
        convertVectorToTransMatrix(calibrationData, &_transform_carmera_to_birdge_gripper6);
        _transform_carmera_to_tcp_gripper6 = _transform_birdge_to_tcp_gripper6 * _transform_carmera_to_birdge_gripper6;
        // std::cout << _transform_carmera_to_tcp_gripper6.matrix() << std::endl;

    }else
    {
        return false;
    }
    
    #ifdef DEBUG_GRASP_INTELLIGENT
        std::cout << "1.变换矩阵：carmera --> tcp:" << std::endl << (which_gripper == G0_GRIPPER ? 
                                                                    _transform_carmera_to_tcp_gripper0.matrix() : 
                                                                    _transform_carmera_to_tcp_gripper6.matrix())<< std::endl;
    #endif

    return true;
}

bool GraspIntelligent::findGraspPointByLine(  const std::vector<double>& point1, const std::vector<double>& point2, 
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

    #ifdef DEBUG_GRASP_INTELLIGENT
        std::cout << "2. 变换矩阵: tcp --> base:" << std::endl << _transform_tcp_to_base.matrix() << std::endl;
    #endif

    // 2. 计算夹持点位置 相对base坐标系
    calculateGraspPointPosition(point1, point2, robot_tcp, grasp_point);
    // 3. 计算夹持点姿态 相对base坐标系
    calculateGraspPointPosture(point1, point2, grasp_point);

    return true;
};

void GraspIntelligent::calculateRobotTcpToBase(const std::vector<double>& tvector){

    convertVectorToTransMatrix(tvector, &_transform_tcp_to_base);
}

void GraspIntelligent::convertVectorToTransMatrix(const std::vector<double>& tvector, Eigen::Isometry3d *matrix){

    // 平移向量
    Eigen::Vector3d translation (tvector[0], tvector[1], tvector[2]);

    // 欧拉角
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix  = Eigen::AngleAxisd(tvector[3], Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(tvector[4], Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(tvector[5], Eigen::Vector3d::UnitX());

    matrix->rotate(rotationMatrix);
    matrix->pretranslate(translation);
}

void GraspIntelligent::calculateGraspPointPosture(const std::vector<double>& point1, const std::vector<double>& point2, 
                                                std::vector<double> *grasp_point)
{
    // 计算杆件向量ng
    // 将杆件位置转换base坐标系

    Eigen::Vector3d ng (point2[0] - point1[0], point2[1] - point1[1], point2[2] - point1[2]);
    // x
    ng = ng.normalized();
    if(_which_gripper == G0_GRIPPER)
    {
        ng = _transform_tcp_to_base.rotation() * _transform_carmera_to_tcp_gripper0.rotation() * ng;
    }
    else
    {
        ng = _transform_tcp_to_base.rotation() * _transform_carmera_to_tcp_gripper6.rotation() * ng;
    }

    
    //根据5DOF机器人和目标点，求出其余a和o; 其中 m = [-TagGripP[1],TagGripP[0],0]
    Eigen::Vector3d ag( grasp_point->at(0) * ng(2),
                        grasp_point->at(1) * ng(2),
                        - grasp_point->at(0) * ng(0) - grasp_point->at(1) * ng(1));
    
    ag = ag.normalized();

    //条件成立说明n与夹子的n夹角大于90° (ng * gripper{n})
    if(ng(0) * _transform_tcp_to_base(0,0) + ng(1) * _transform_tcp_to_base(1,0) + ng(2) * _transform_tcp_to_base(2,0) < 0)
    {
        for(size_t i = 0; i < 3; ++ i){
            ng(i) = -ng(i);
        }
    }
    //条件成立说明a与夹子的a夹角大于90°
    if(ag(0) * _transform_tcp_to_base(0,2) + ag(1) * _transform_tcp_to_base(1,2) + ag(2) * _transform_tcp_to_base(2,2) < 0 )
    {
        for(size_t i = 0; i < 3; ++ i){
            ag(i) = -ag(i);
        }
    }

    // og
    Eigen::Vector3d og( ag(1) * ng(2) - ag(2) * ng(1),
                        ng(0) * ag(2) - ng(2) * ag(0),
                        ag(0) * ng(1) - ag(1) * ng(0) );
    og = og.normalized();

    //条件成立说明o与夹子的o夹角大于90°
    if(og(0) * _transform_tcp_to_base(0,1) + og(1) * _transform_tcp_to_base(1,1) + og(2) * _transform_tcp_to_base(2,1) < 0)
    {
        for(size_t i = 0; i < 3; ++ i){
            og(i) = -og(i);
        }
    }
    
    Eigen::Matrix3d matrix;
    matrix <<   ng(0), og(0), ag(0),
                ng(1), og(1), ag(1),
                ng(2), og(2), ag(2);
    #ifdef DEBUG_GRASP_INTELLIGENT
        std::cout << "2. 夹持点姿态:" << std::endl << matrix.eulerAngles(2,1,0) << std::endl;
    #endif

    convertPostureMatrixToEuler(matrix, grasp_point);

}


void GraspIntelligent::calculateGraspPointPosition(const std::vector<double>& point1, const std::vector<double>& point2, 
                                                const std::vector<double>& robot_tcp, std::vector<double> *grasp_point)
{
    // 将杆件位置转换base坐标系
    Eigen::Vector3d p1 (point1[0], point1[1], point1[2]);
    Eigen::Vector3d p2 (point2[0], point2[1], point2[2]);

    // std::cout << "----------------"<< std::endl;
    // std::cout << p1.transpose() << std::endl;
    // std::cout << p2.transpose() << std::endl;

    Eigen::Isometry3d temp_p1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d temp_p2 = Eigen::Isometry3d::Identity();
    temp_p1.pretranslate(p1);
    temp_p2.pretranslate(p2);

    // std::cout << "----------------"<< std::endl;
    // std::cout << temp_p1.matrix() << std::endl;
    // std::cout << temp_p2.matrix() << std::endl;

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
    // std::cout << "----------------"<< std::endl;
    // std::cout << temp_p1.matrix() << std::endl;
    // std::cout << temp_p2.matrix() << std::endl;

    temp_p1 = _transform_tcp_to_base * temp_p1;
    temp_p2 = _transform_tcp_to_base * temp_p2;

    p1 << temp_p1(0,3), temp_p1(1,3), temp_p1(2, 3);
    p2 << temp_p2(0,3), temp_p2(1,3), temp_p2(2, 3);

    #ifdef DEBUG_GRASP_INTELLIGENT
        std::cout << "4. p1 杆件点位置base坐标系:" << std::endl << p1 << std::endl;
        std::cout << "5. p2 杆件点位置base坐标系:" << std::endl << p2 << std::endl;
    #endif

    // 斜率
    p1 = p2 - p1;
    double k_ = (   
                    p1(0) * (robot_tcp[0] - p2(0)) 
                    + p1(1) * (robot_tcp[1] - p2(1))
                    + p1(2) * (robot_tcp[2] - p2(2))
                ) / 
                (
                    pow(p1(0), 2) + pow(p1(1), 2) + pow(p1(2), 2)
                );
    for(size_t i = 0 ; i < 3; ++ i)
    {   
        grasp_point->at(i) = p2(i) + k_ * p1(i);
        if( abs(grasp_point->at(i)) < CALCULATE_THRESHOLD){
            grasp_point->at(i) = 0;
        }
    }

    // grasp_point->at(0) = (p2(0) + p1(0)) / 2;
    // grasp_point->at(1) = (p2(1) + p1(1)) / 2;
    // grasp_point->at(2) = (p2(2) + p1(2)) / 2;
}


void GraspIntelligent::convertPostureMatrixToEuler(const Eigen::Matrix3d &matrix, std::vector<double> *grasp_point)
{
    double THRESHOLD = 0.0001;
	double ld_temp[6];
    //--------------------- 输出位姿 -------------------------//
    // 计算RPY角 - rad //
	ld_temp[4] = atan2( -  matrix(2,0), 
		sqrt(matrix(0,0) * matrix(0,0) + matrix(1,0) * matrix(1,0)));
	
    if (fabs(ld_temp[4] - M_PI / 2) < THRESHOLD)
    {
        ld_temp[3] = 0;
        ld_temp[5] = atan2(matrix(0,1), matrix(1,1));
    }
    else if (fabs(ld_temp[4] + M_PI / 2) < THRESHOLD)
    {
        ld_temp[3] = 0;
        ld_temp[5] = -atan2(matrix(0,1), matrix(1,1));
    }
    else
    {
        ld_temp[0] = 1 / cos(ld_temp[4]);
		
		ld_temp[1] = matrix(1,0) * ld_temp[0];
		ld_temp[2] = matrix(0,0) * ld_temp[0];
		if(fabs(ld_temp[1]) < THRESHOLD)
		{
			ld_temp[1] = 0;
		}
		if(fabs(ld_temp[2]) < THRESHOLD)
		{
			ld_temp[2] = 0;
		}
        ld_temp[3] = atan2(ld_temp[1], ld_temp[2]);
		
		ld_temp[1] = matrix(2,1) * ld_temp[0];
		ld_temp[2] = matrix(2,2) * ld_temp[0];
		if(fabs(ld_temp[1]) < THRESHOLD)
		{
			ld_temp[1] = 0;
		}
		if(fabs(ld_temp[2]) < THRESHOLD)
		{
			ld_temp[2] = 0;
		}
        ld_temp[5] = atan2(ld_temp[1], ld_temp[2]);
    }
	
    for(size_t i = 3 ; i < 6; ++ i)
    {   
        grasp_point->at(i) = ld_temp[i];
        if( abs(grasp_point->at(i)) < CALCULATE_THRESHOLD){
            grasp_point->at(i) = 0;
        }
    }

}
