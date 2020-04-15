#ifndef __SPEEDPROCESS_H__
#define __SPEEDPROCESS_H__
/*****************************************************************************
 *        加减速处理结构定义                                                 *
 *        Copyright (c) GSK Inc., 2010                                       *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2010-04-15                                       *
 *****************************************************************************/
#include "Robot.h"



// 轨迹点参数类
class TrajPointPara
{
public:
	double Acc; // 加速度
	double Vel; // 速度
	double Len; // 距离
};


// 轨迹输入数据类
class SpdPTrajInputData
{
public:
	double Len;  // 轨迹段长度
	double Vs;   // 初始速度
	double V;    // 期望速度
	double Ve;   // 终点速度
	double Acc;  // 最大加速度
	double Jerk; // 最大加加速度
	double T_Acc;// 加速时间
};

// 轨迹输出数据类
class SpdPTrajData
{
public:
	double Len;  // 轨迹段长度
	double Vs;   // 初始速度
	double V;    // 期望速度
	double Ve;   // 终点速度

	double Jacc;
	double Jdec;

	double T1;   // 各段时间
	double T2;
	double T3;
	double T4;
	double T5;
	double T6;
	double T7;
	
	double T;    // 规划总时间

	double V1;   // 各段端点速度
	double V2;
	double V3;
	double V4;
	double V5;
	double V6;

	double L1;  // 各段端点位移
	double L2;
	double L3;
	double L4;
	double L5;
	double L6;
	double L7;
};


class SpeedProcess
{
public:
	SpeedProcess();
	~SpeedProcess();

	// S加减速规划
	int SpeedPlanningS(IN SpdPTrajInputData* pIn, OUT SpdPTrajData* pOut);
	// 获取轨迹
	void Get_SpeedProcessData(IN SpdPTrajData* pIn, IN double dTime, OUT TrajPointPara* pOut);
	// 简化S加减速规划
	int SpeedPlanningS1(IN SpdPTrajInputData* pIn, OUT SpdPTrajData* pOut);
	// 简化S加减速规划 - 运动时间固定
	int SpeedPlanningS1ByTime(IN SpdPTrajInputData* pIn, IN double dTime, OUT SpdPTrajData* pOut);

private:
	// 
	double Solve_Equation3(double a, double b, double c, double d, double val);
	int Solve_Equation2(double a, double b, double c,
						   double* res1, double* res2);
	void Get_AccDecLen(SpdPTrajData *out, double* sacc, double *sdec);
};

#endif
