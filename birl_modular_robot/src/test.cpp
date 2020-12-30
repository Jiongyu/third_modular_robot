#include "grasp_intelligent.h"

#define PI_DEG 57.295779
#define PI_RAD 0.0174533

#define CALIBRATION_DATA

int main(int argc, char const *argv[])
{
    GraspIntelligent demo;

#ifdef CALIBRATION_DATA
    // ur3 --> tcp
    Eigen::Matrix3d rotate_matrix_g0;
    Eigen::Matrix3d rotate_matrix_g6;
    rotate_matrix_g0 << 0, 1, 0, 
                        1, 0, 0, 
                        0, 0, -1;
    rotate_matrix_g6 << 0, -1, 0, 
                        1, 0, 0, 
                        0, 0, 1;
    // carmera --> ur3
    const double incre_z = 222 - 40.2 - 176.4;
    const std::vector<double>calibrationData_g0{-28.280, -163.739, -261.080 + incre_z, 
                                                358.347 * PI_RAD, 0.151 * PI_RAD, 1.526 * PI_RAD};
    const std::vector<double>calibrationData_g6{-22.262, -177.911, -249.306 + incre_z, 
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
    demo.setHandEyeCalibrationBridge(rotate_matrix_g0, GraspIntelligent::G0_GRIPPER);
    demo.setHandEyeCalibrationBridge(rotate_matrix_g6, GraspIntelligent::G6_GRIPPER);

    // 设置carmera --> ur3标定结果
    demo.setHandEyeCalibrationConsq(calibrationData_g0, GraspIntelligent::G0_GRIPPER);
    demo.setHandEyeCalibrationConsq(calibrationData_g6, GraspIntelligent::G6_GRIPPER);


    // 求解夹持点相对于carmera
    std::vector<double>p1{19.06,-97.22,275.31};
    std::vector<double>p2{26.699, 99.51, 267.234};
    // std::vector<double>p1{-50,0,50};
    // std::vector<double>p2{50,0,50};
 
    // 机器人tcp相对于base
    std::vector<double>tcp{556.8,0,50,0,0,3.141592};
    // 夹持点相对于base
    std::vector<double>grasp_point;
    grasp_point.resize(6);
    demo.findGraspPointByLine(p1, p2, tcp, GraspIntelligent::G6_GRIPPER, &grasp_point);

    // 打印夹持点数据
    std::cout << "夹持点位姿 相对与base:" << std::endl;
    for(size_t i = 0 ; i < grasp_point.size() ; ++ i){
        if(i < 3)
            std::cout << grasp_point.at(i) << " ";
        else
            std::cout << grasp_point.at(i) *  PI_DEG << " ";
    }
    std::cout << std::endl;
    return 0;
}