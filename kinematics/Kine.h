#ifndef __KINE_H__
#define __KINE_H__
/*****************************************************************************
 *        Robot运动学定义                                                    *
 *        SCUT, 2010                                                         *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2010-12-08                                       *
 *****************************************************************************/
#include "Robot.h"
#include "Matrix.h"

/*****************************************************************************
 *  运动学基类
 *****************************************************************************/
class Kine
{
public:
	Kine();
	~Kine();

	/******************************************************************************
	 * 函数：FKine()
	 * 功能：正解
	 *
	 * 输入：double* gdJPos - 关节转角, 5/6关节
	 * 输出：double* gdCPos - 正解位姿, (x,y,z,w,p,r)
	 *
	 * 返回：int - 0成功,
	 ******************************************************************************/
	virtual int FKine(IN double gdJPos[], OUT double gdCPos[]) = 0;


	/******************************************************************************
	 * 函数：IKine()
	 * 功能：逆解
	 *
	 * 输入：double* gdCPos  - 位姿数组, (x,y,z,w,p,r)
	 *       double* gdJCurr - 当前关节转角, 5/6关节
	 * 输出：double* id_jPos  - 逆解关节转角, 5/6关节
	 *
	 * 返回：int - 0成功, 其他错误
	 ******************************************************************************/
	virtual int IKine(IN double gdCPos[], IN double gdJCurr[], OUT double gdJPos[]) = 0;
	
	/******************************************************************************
	 * 函数：Vel_FKine()
	 * 功能：速度逆解, 工具坐标系速度
	 *
	 * 输入：double* gdJPos - 当前关节转角, 5/6关节, deg
	 *       double* gdJVel - 当前关节速度, 5/6关节, deg/s
	 * 输出：double* gdCVel - 末端速度, [vx,vy,vz,wx,wy,wz], mm/s, deg/s
	 *
	 * 返回：int - 0成功, 其他错误
	 ******************************************************************************/
	virtual int Vel_FKine(IN double gdJPos[], IN double gdJVel[], OUT double gdCVel[]) = 0;
	
	/******************************************************************************
	 * 函数：Vel_IKine()
	 * 功能：速度逆解
	 *
	 * 输入：double gdJPos[] - 当前关节转角, 5/6关节, deg
	 *       double gdCVel[] - 当前末端速度, [vx,vy,vz,wx,wy,wz], mm/s, deg/s
	 * 输出：double gdJVel[] - 关节速度, 5/6关节, deg/s
	 *
	 * 返回：int - 0成功, 其他错误
	 ******************************************************************************/
	virtual int Vel_IKine(IN double gdJPos[], IN double gdCVel[], OUT double gdJVel[]) = 0;
protected:
	// 将角度变换为(-360,0)或(0,+360)范围内
	void RadInRange(double* pdRad, double* pdDeg);
};

/*****************************************************************************
 *  五关节攀爬机器人运动学 - G1
 *****************************************************************************/
class Kine_CR_FiveDoF_G1: public Kine
{
public:
	// 初始化杆长 
	void Set_Length(IN double gdLen[5]);
	// 正解
	int FKine(IN double gdJPos[5], OUT double gdCPos[6]);
	// 逆解
	int IKine(IN double gdCPos[6], IN double gdJCurr[5], OUT double gdJPos[5]);
	// 速度正解
	int Vel_FKine(IN double gdJPos[5], IN double gdJVel[5], OUT double gdCVel[6]);
	// 速度逆解
	int Vel_IKine(IN double gdJPos[5], IN double gdCVel[6], OUT double gdJVel[5]);

	int FKine_Inc(IN double gdJPos[5], IN double inc[3], OUT double gdCPos[3]);

private:
	double m_dL1;
	double m_dL2;
	double m_dL3;
	double m_dL4;
	double m_dL5;

};

/*****************************************************************************
 *  五关节攀爬机器人运动学-G2
 *****************************************************************************/
class Kine_CR_FiveDoF_G2: public Kine
{
public:
	// 初始化杆长 
	void Set_Length(IN double gdLen[5]);
	// 正解
	int FKine(IN double gdJPos[5], OUT double gdCPos[6]);
	// 逆解
	int IKine(IN double gdCPos[6], IN double gdJCurr[5], OUT double gdJPos[5]);
	// 速度正解
	int Vel_FKine(IN double gdJPos[5], IN double gdJVel[5], OUT double gdCVel[6]);
	// 速度逆解
	int Vel_IKine(IN double gdJPos[5], IN double gdCVel[6], OUT double gdJVel[5]);

	int FKine_Inc(IN double gdJPos[5], IN double inc[3], OUT double gdCPos[3]);
	
private:
	Kine_CR_FiveDoF_G1 kine;
};

/*****************************************************************************
 *  五关节串联机械手运动学类
 *****************************************************************************/
class Kine_IR_FiveDoF: public Kine
{
public:
	// 初始化杆长 
	void Set_Length(IN double gdLen[5]);
	void Set_Tool(IN double gdTool[6]);
	//记录变换矩阵
	void FKineMatrix(IN double gdJPos[6]);

	// 正解
	int FKine(IN double gdJPos[5], OUT double gdCPos[6]);
	// 逆解
	int IKine(IN double gdCPos[6], IN double gdJCurr[5], OUT double gdJPos[5]);
	// 速度正解
	int Vel_FKine(IN double gdJPos[5], IN double gdJVel[5], OUT double gdCVel[6]);
	// 速度逆解
	int Vel_IKine(IN double gdJPos[5], IN double gdCVel[6], OUT double gdJVel[5]);
	// 拧螺母的特殊逆解
	int IKine_s(double gdCPos[], double gdJCurr[], double gdJPos[]);

	double m_dL_GetMatrix[6];

private:
	double m_dL1;
	double m_dL2;
	double m_dL3;
	double m_dL4;
	double m_dL5;
	double m_dTool[6];

	Kine_CR_FiveDoF_G1 kine;
};

/*****************************************************************************
 *  六关节串联机械手运动学类
 *****************************************************************************/
class Kine_IR_SixDoF: public Kine
{
public:
	// 初始化杆长 
	void Set_Length(IN double gdLen[6]);
	void Set_Tool(IN double gdTool[6]);

	// 正解
	int FKine(IN double gdJPos[6], OUT double gdgdCPos[6]);
	// 逆解
	int IKine(IN double gdCPos[6], IN double gdJCurr[6], OUT double gdJPos[6]);
	// 速度正解
	int Vel_FKine(IN double gdJPos[6], IN double gdJVel[6], OUT double gdCVel[6]);
	// 速度逆解
	int Vel_IKine(IN double gdJPos[6], IN double gdCVel[6], OUT double gdJVel[6]);

private:
	double m_dL1;
	double m_dL2;
	double m_dL3;
	double m_dL4;
	double m_dL5;
	double m_dL6;
	double m_dTool[6];
};


void Trans_PosToMtx(double* pos, MtxKine* output, int inv);
void Trans_MtxToPos(MtxKine* input, double* outpos);
void Mtx_Multiply(MtxKine* input, MtxKine* middle, MtxKine* output, int inv);
void Robot_IncreTransTool(IN double* currpos, IN double* increpos, OUT double* outpos);

#endif
