#ifndef __INTERPOLATION_H__
#define __INTERPOLATION_H__
/*****************************************************************************
 *        Robot插补结构定义                                                  *
 *        SCUT, 2010                                                         *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2010-12-08                                       *
 *****************************************************************************/
#include "Robot.h"
#include "MyPath.h"
#include "Kine.h"
#include <deque>
#include <vector>


// 系统输入数据类
class InterpStruct
{
public:
	int Line;    // 行号

	int Mode;    // 插补方式 - MOVJ 1 | MOVL 2 | MOVC 3
	             // MOVJ - 多于2个点的联动(J空间)
	             // MOVL|MOVC - 单段轨迹运动(C空间)
	int IfLinkage;
	
	double JStart[MAX_AXIS_NUM]; // 当前各关节位置, deg
	double JMid[MAX_AXIS_NUM];   // 中间点位置, deg
	double JEnd[MAX_AXIS_NUM];   // 终点位置, deg

	double Acc;   // 加速度,     deg/s*s(J) | mm/s*s(C)
	double Jerk;  // 加加速度, deg/s*s*s(J) | mm/s*s*s(C)

	double Vel;   // 期望速度,     deg/s(J) | mm/s(C)
	double G_V;   // 姿态期望速度, deg/s(C)

	double T_Acc; // 加速时间, 
};

class JointInterpStruct
{
public:
	int Line;

	double JStart[MAX_AXIS_NUM];
	double JEnd[MAX_AXIS_NUM];
	double Vel[MAX_AXIS_NUM];

	double T_Acc;
	double T;
};

class SplineStruct
{
public:
	int Line;
	double P[MAX_AXIS_NUM];
	double V[MAX_AXIS_NUM];
	double A[MAX_AXIS_NUM];
	double T;
};

// 向量类
class VectorStruct
{
public:
	double X;   // 向量x
	double Y;   // 向量y
	double Z;   // 向量z
	double Con; // 常量
};

// 五角星顶点
#define  MAX_STAR_POINT  50
class StarPointStruct
{
public:
	double Point[MAX_STAR_POINT][MAX_AXIS_NUM];
	int Mode[MAX_STAR_POINT];
	int Count; // 数目
};

// 笛卡尔插补类
class Interpolation
{
public:
	Interpolation();
	~Interpolation();
	
	void Set_RobotMode(IN int mode);           // 设置机器人模式
	void Set_Rat(IN double* rat);              // 设置各轴减速比
	void Set_Length(IN double len[]);          // 设置杆长
	void Set_Lim(IN double* pos, IN double* neg); // 设置各轴极限
	void Set_Tool(IN double tool[]);           // 设置工具参数
	void Set_Grip(IN int grip);                // 设置当前手爪编号
	
	int Plan_Joint(IN InterpStruct* in);
	int Plan_JointS(IN InterpStruct* in, OUT JointInterpStruct* out); // 单点PTP规划
	int Plan_JointM(IN std::deque<JointInterpStruct> *in);        // 多点PTP运动规划

	int Plan_Line(IN InterpStruct* in);   // Line规划
	int Plan_Circle(IN InterpStruct* in); // Circle规划

public:
	MyPath m_iPath;               // MyPath类变量, 全局变量
	std::vector<PointPVT> outputdata;

	StarPointStruct m_iStarPoint; // 由圆弧获取的五角星顶点
	bool m_bStarPoint;            // 已获取五角星顶点标志位

private:
	int m_nMode;
	int m_nGrip;
	Kine_IR_FiveDoF m_iKine_IR_5;      // 5dof机械手运动学类实例
	Kine_IR_SixDoF  m_iKine_IR_6;      // 6dof机械手运动学类实例
	Kine_CR_FiveDoF_G1 m_iKine_CR_G1;  // 攀爬G1固定运动学实例
	Kine_CR_FiveDoF_G2 m_iKine_CR_G2;  // 攀爬G2固定运动学实例

	int I_Kine(int mode, double gdCPos[], double gdJCurr[], double gdJPos[]);

private:	
	double m_dPosLim[MAX_AXIS_NUM];  // 正极限
	double m_dNegLim[MAX_AXIS_NUM];  // 负极限

	// 使两位姿之间的姿态转动量最小
	void Modify_RadRot(IN double* start, IN OUT double* end);
public:
	int If_PosLimit(IN double* pos, OUT int* flag); // 判断极限
};

// 关节插补类
class JointInterp
{
public:
	JointInterp(IN double* pos, IN int degree); // 初始化当前位置和维数
	~JointInterp(void);

	int Get_Num(); // 每次插补点数

	void InsertNewPoint(IN double* pos); // 插入新点
	// 获取关节插补数据
	int Get_JointInterpData(IN int count, OUT double* outp, OUT double* outv);

private:
	
	int m_nDegree;     // 插补拟合曲线的最高次幂
	int m_nNum;        // 每周期插补点数

	double m_dPos0[MAX_AXIS_NUM]; // 第一点数据
	double m_dPos1[MAX_AXIS_NUM]; // 第二点
	double m_dPos2[MAX_AXIS_NUM]; // 第三点
	double m_dPos3[MAX_AXIS_NUM]; // 第四点
	double m_dPos4[MAX_AXIS_NUM]; // 第五点 
};

#endif