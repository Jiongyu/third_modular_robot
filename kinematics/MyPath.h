#ifndef __MYPATH_H__
#define __MYPATH_H__
/*****************************************************************************
 *        轨迹序列定义                                                       *
 *        SCUT, 2010                                                         *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2010-12-08                                       *
 *****************************************************************************/
#include "Robot.h"
#include "CML_Trajectory.h"

#include <deque>

CML_NAMESPACE_USE();

class PointPVT
{
public:
	int Line;  // 所在程序行号

	double P[MAX_AXIS_NUM];  // 伺服位置，单位: deg
	double V[MAX_AXIS_NUM];  // 伺服速度，单位: deg/s
	double A[MAX_AXIS_NUM];
	double T;     // 插补周期，单位: s
};

class MyPath : public LinkTrajectory 
{
public:
   MyPath(); 
   virtual ~MyPath( void );
   
   virtual void Clear();          // 清除
   void Set_Rat(IN double* rat);  // 设置各轴减速比
   virtual int GetDim( void );    // 获取维数
   int GetSize();                 // 获取插补点数

   // 插入点
   virtual const Error *AddPoint(IN PointPVT &p);
   // 获取点, 若队列中无数据, 返回错误
   //virtual const Error *NextSegment(OUT uunit pos[], OUT uunit vel[], OUT uint8 &time );

private:
	double m_gdMotorRat[MAX_AXIS_NUM];        // 减速比
	std::deque<PointPVT> *m_pdePoint;     // 队列指针
};

#endif

