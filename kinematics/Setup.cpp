/*****************************************************************************
 *        Robot公共变量及Robot系统文件操作类                                 *
 *        SCUT, 2011                                                         *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2011-04-22                                       *
 *****************************************************************************/

#include "Setup.h"
//#include "File.h"

///////////////////////////////////////////////////////////////// 静态变量预定义


unchar Robot::axisNum = 5;
unchar Robot::robotMode = ROBOT_MODE_CR_P; // 机器人构型 - 爬杆
unchar Robot::actualTool = GRIP_FIRST;     // 当前工具 - #1
unchar Robot::runMode = RUNMODE_TEACH;     // 当前示教坐标系 - 示教
unchar Robot::teachMode = COORDINATE_JOINT;// 当前示教模式 - 关节
bool Robot::ifMonitor = true;              // 开机监控
bool Robot::ifTool = true;
bool Robot::ifMaster = true;
bool Robot::ifWheel = true;

//double Robot::robotLen[7] = { 250, 375, 200, 175, 100, 100, 100};	// 杆长
//double Robot::motorRat[MAX_AXIS_NUM] = {-225.0, 375, 375, 375, -450.0, 375};// 各轴减速比
//double Robot::posLim[MAX_AXIS_NUM] = { 270, 110,  110, 270,  110}; // 正极限
//double Robot::negLim[MAX_AXIS_NUM] = {-270, -110, -110, -270, -110}; // 负极限
//double Robot::teachLim[6] = {10, 30, 20, 60, 5, 20};           // 示教限制参数
//double Robot::autoLim[7] = {30, 50, 100, 150, 10, 40, 500};

/////////////////////////////////////////////////////////////////////////////////////////
double Robot::robotLen[7] = { 210.4, 93.4, 293.2, 254, 131, 248.5, 100};	// 杆长
double Robot::motorRat[MAX_AXIS_NUM] = {457.14, 480, 480, 188.24, 200, 375};// 各轴减速比
double Robot::posLim[MAX_AXIS_NUM] = { 360, 210,  210, 360,  120}; // 正极限
double Robot::negLim[MAX_AXIS_NUM] = {-360, -30, -30, -360, -120}; // 负极限
double Robot::teachLim[6] = {10, 30, 20, 60, 5, 20};           // 示教限制参数
double Robot::autoLim[7] = {30, 5, 100, 150, 10, 40, 500};
//////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////// 静态变量预定义

