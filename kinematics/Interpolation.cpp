/*****************************************************************************
 *        插补及加减速控制函数                                               *
 *        SCUT, 2010                                                         *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2010-12-08                                       *
 *****************************************************************************/
#include "Interpolation.h"

#include "SpeedProcess.h"
#include <math.h>
#include "Robot.h"

#include "Setup.h"

//#include "Character.h"

// 测试
#ifndef TEST_FILE
#define TEST_FILE
#endif
#ifndef WRITE_DATA
#define WRITE_DATA  1 // 0关节位置|1关节速度|2笛卡尔位姿
#endif

/*
#ifdef TEST_FILE
#include "File.h"
#endif
*/
/*****************************************************************************
 * 函数：Interpolation()
 * 功能：构造函数
 *****************************************************************************/
Interpolation::Interpolation()
{
	m_nGrip = GRIP_FIRST;
	m_nMode = ROBOT_MODE_CR_P;

	// 变量赋初值
	for (int i=0; i<JOINT_NUM; i++)
	{
		m_dPosLim[i] = Robot::posLim[i];
		m_dNegLim[i] = Robot::negLim[i];	
	}

	// 位置极限赋初值
// 	m_dPosLim[0] = 270;
// 	m_dPosLim[1] = 200;
// 	m_dPosLim[2] = 110;
// 	m_dPosLim[3] = 110;
// 	m_dPosLim[4] = 270;
// 
// 	m_dNegLim[0] = -270;
// 	m_dNegLim[1] = -20;
// 	m_dNegLim[2] = -110;
// 	m_dNegLim[3] = -20;
// 	m_dNegLim[4] = -270;

	m_bStarPoint = false;
 }

/*****************************************************************************
 * 函数：Interpolation()
 * 功能：析构函数
 *****************************************************************************/
Interpolation::~Interpolation()
{}

void Interpolation::Set_RobotMode(IN int mode)
{
	m_nMode = mode;
}


/*****************************************************************************
 * 函数：Set_Rat()
 * 功能：设置各关节减速比
 *
 * 输入：double* rat - 减速比数组
 *****************************************************************************/
void Interpolation::Set_Rat(double* rat)
{
	m_iPath.Set_Rat(rat);
}

/******************************************************************************
 * 函数：Set_Length()
 * 功能：设置杆长
 *
 * 输入：double l1 - 杆长L1
 *       double l2 - 杆长L2
 *       double l3 - 杆长L3
 *       double l4 - 杆长L4
 *       double l5 - 杆长L5
 ******************************************************************************/
void Interpolation::Set_Length(double len[])
{
	m_iKine_IR_5.Set_Length(len);
	m_iKine_IR_6.Set_Length(len);
	m_iKine_CR_G1.Set_Length(len);
	m_iKine_CR_G2.Set_Length(len);
}

void Interpolation::Set_Tool(double tool[])
{
	m_iKine_IR_5.Set_Tool(tool);
	m_iKine_IR_6.Set_Tool(tool);
}

void Interpolation::Set_Grip(int grip)
{
	m_nGrip = grip;
}

/*****************************************************************************
 * 函数：Set_Lim()
 * 功能：设置极限
 *
 * 输入：double* pos - 正极限数组, deg
 *       double *neg - 负极限数组, deg
 *****************************************************************************/
void Interpolation::Set_Lim(double* pos, double* neg)
{
	for (int i=0; i<JOINT_NUM; i++)
	{
		m_dPosLim[i] = pos[i];
		m_dNegLim[i] = neg[i];
	}
}

/*****************************************************************************
 * 函数：Modify_RadRot()
 * 功能：使两位姿之间的姿态转动量最小
 *       wpr位姿有两种表示形式, 择二取一
 *
 * 输入：double* start - 始点位姿
 *
 * 输出：double* end   - 终点位姿
 *****************************************************************************/
void Interpolation::Modify_RadRot(double* start, double* end)
{
	int i;
	double ld_temp;

	double ld_temp1;
	double ld_ges[3]; // 姿态数组

	// end位姿的另外一组解
	for (i=3; i<6; i++)
	{
		ld_ges[i - 3] = (end[i] >= 0) ? (end[i] - 180) : (end[i] + 180); 
	}

	// 使end点姿态两组解的每个姿态转角相对于start点的变化量最小
	for (i=3; i<6; i++)
	{
		ld_temp = (-1) * start[i] + end[i];	
		// 相差超过180度,则令加(减)360度
		if ((ld_temp >= -180.0) && (ld_temp <= 180.0))
		{
			//end[i] = end[i];
		}
		else if (ld_temp > 180.0)
		{
			end[i] -= 360.0;
		}
		else
		{
			end[i] += 360.0;
		}

		ld_temp1 = (-1) * start[i] + ld_ges[i - 3];
		// 相差超过180度,则令加(减)360度
		if ((ld_temp1 >= -180.0) && (ld_temp1 <= 180.0))
		{
			//
		}
		else if (ld_temp1 > 180.0)
		{
			ld_ges[i - 3] -= 360.0;
		}
		else
		{
			ld_ges[i - 3] += 360.0;	
		}
	}

	// 相对于start点姿态的距离平方
	ld_temp = (start[3] - end[3]) * (start[3] - end[3]) +
			  (start[4] - end[4]) * (start[4] - end[4]) +
			  (start[5] - end[5]) * (start[5] - end[5]);
	ld_temp1 = (start[3] - ld_ges[0]) * (start[3] - ld_ges[0]) +
			   (start[4] - ld_ges[1]) * (start[4] - ld_ges[1]) +
			   (start[5] - ld_ges[2]) * (start[5] - ld_ges[2]);
}
/*****************************************************************************
 * 函数：If_PosLimit()
 * 功能：判断位置极限, 当极限Pos_Lim和Neg_Lim数值为零时表示没有极限设置
 *
 * 输入：double* pos - 当前点位置, 5关节, deg
 * 输出：int* flag   - 极限标志数组, -1负极限|0无极限|1正极限
 *
 * 返回：int - 0 无极限 | 1 有极限
 *****************************************************************************/
int Interpolation::If_PosLimit(double* pos, int* flag)
{
	int i;

	for (i=0; i<JOINT_NUM; i++)
	{
		flag[i] = 0; // 赋初值

		// 正极限
		if ((0 != m_dPosLim[i]) &&
			(pos[i] > m_dPosLim[i]) )
		{
			flag[i] = - (170 + i);				//??????????????????????????
		}

		// 负极限
		if ((0 != m_dNegLim[i]) &&
			(pos[i] < m_dNegLim[i]) )
		{
			flag[i] = - (176 + i);
		}
	}

	for (i=0; i<JOINT_NUM; i++)
	{
		if (0 != flag[i])
		{
			return 1;
		}
	}

	return Ok;
}

/*****************************************************************************
 * 函数：Plan_Joint()
 * 功能：PTP规划
 *
 * 输入：InterpStruct* in - 指令, 需给定Mode|JStart|JEnd|Vel|T_Acc
 *
 * 返回：int - 0正常, 其他错误
 *****************************************************************************/
int Interpolation::Plan_Joint(IN InterpStruct* in)
{
	int li_plimflag[MAX_AXIS_NUM]; // 极限标志位

	// 判断位置极限
	if (0 != If_PosLimit(in->JEnd, li_plimflag))
		return ERR_PINLIM; // 位置极限

	return Ok;
}

/*****************************************************************************
 * 函数：Plan_JointS()
 * 功能：单段PTP规划
 *
 * 输入：InterpStruct* in - 指令, 需给定Mode|JStart|JEnd|Vel|T_Acc
 * 输出：JointInterpStruct* out - 规划后的轨迹数据 JStart|JEnd|Vel|T
 *
 * 返回：int - 0正常, 其他错误
 *****************************************************************************/
int Interpolation::Plan_JointS(InterpStruct* in, JointInterpStruct* out)       //该函数主要功能是根据输入的速度in->Vel和加速时间in->T_Acc（该值始终不改变）来规划单段（两点间）的
																			   //速度out->Vel和运行时间out->T.   由于加减速时间相等，故根据in->Vel * in->T_Acc与fabs(L[i])的比较
																			    //来判断是否有匀速运动段
{
	int i = 0;
	double ld_temp;
	double L[MAX_AXIS_NUM];
	int flag_poslim[MAX_AXIS_NUM]; // 极限标志位
	
	out->Line = in->Line;   // 行号
	out->T_Acc = in->T_Acc; // 加速时间
	if (in->T_Acc <= 0)
	{
		return ERR_MOTIONPARA; // 运动参数有误
	}
	out->T = 0;
	for (i=0; i<JOINT_NUM; i++)
	{
		out->JStart[i] = in->JStart[i]; // 始点
		out->JEnd[i] = in->JEnd[i]; // 末点
	}
	
	// 判断极限
	if ((If_PosLimit(in->JStart, flag_poslim)) ||
		(If_PosLimit(in->JEnd, flag_poslim)) )
	{
		return ERR_PINLIM; // 极限
	}
	//-----------------------------  整体规划 -------------------------------//
	// 各段距离、期望速度、运动时间
	for (i=0; i<JOINT_NUM; i++)
	{
		L[i] = in->JEnd[i] - in->JStart[i];

		// 距离为0
		if (fabs(L[i]) < RT_LITTLE)
		{
			out->Vel[i] = 0;
			ld_temp = 0;
		}
		else
		{
			// 无匀速段
			if (in->Vel * in->T_Acc > fabs(L[i]))    //加速和减速段的加速度的大小是相等的，即加减速时间相等
			{
				out->Vel[i] = L[i] / in->T_Acc;
				ld_temp = 2 * in->T_Acc;
			}
			// 有匀速段
			else
			{
				out->Vel[i] = in->Vel * fabs(L[i]) / L[i];
				ld_temp = in->T_Acc + L[i] / out->Vel[i];
			}
		}

		// 取各关节时间最大值
		if (ld_temp > out->T)
		{
			out->T = ld_temp;
		}
	}

	// 时间取整
	out->T = INTERP_T * ceil(out->T / INTERP_T);    //INTERP_T差不周期，0.016s

	//-----------------------------  重新规划 -------------------------------//
	// 重新规划各轴
	if (0 == out->T)
	{
		for (i=0; i<JOINT_NUM; i++)
		{
			out->Vel[i] = 0;
		}
	}
	else
	{
		// 各关节期望速度
		for (i=0; i<JOINT_NUM; i++)
		{
			out->Vel[i] = L[i] / (out->T - out->T_Acc);
		}
	}

	return Ok;
}

/*****************************************************************************
 * 函数：Plan_JointM()
 * 功能：PTP连续规划
 *
 * 输入：std::deque<JointInterpStruct> *in - PTP轨迹序列
 *
 * 返回：int - 0正常, 其他错误
 *****************************************************************************/
int Interpolation::Plan_JointM(std::deque<JointInterpStruct> *in)
// int Plan_JointM(IN std::deque<JointInterpStruct> *in,IN std::vector<PointPVT>* outputdata)
{
/*
////////////////////////////////////
#ifdef TEST_FILE                  //
	Test_File file;               //
	file.Init_File(JOINT_NUM);    //
	int file_time = 0;            //
#endif                            //
////////////////////////////////////
*/
	//-----------------------------  定义变量 -------------------------------//
	int i, j;
	int count = (int)in->size(); // 轨迹段数目
	bool flag_break = false;// 退出循环标志

	double ld_jerk;          // 中间变量
	double ld_time;          // 当前时间（局部时间，以各个阶段的起始点作为零点的时间表示）
	double ld_pos[MAX_AXIS_NUM]; // 当前位置
	double ld_vel[MAX_AXIS_NUM]; // 当前速度（起始速度？）

	JointInterpStruct jointData; // PTP插补数据
	PointPVT ls_sendPoint;       // 输出点数据
	outputdata.clear();
	int li_plimflag[MAX_AXIS_NUM];   // 极限标志位

	// 初始化类库
	m_iPath.Clear(); // 初始化path类


	//-----------------------------  轨迹规划 -------------------------------//
	for (j=1; j<=count; j++)
	{
		jointData = in->front(); // 读取
		in->pop_front();         // 删除

		// 初始化
		if (1 == j)
		{
			for (i=0; i<JOINT_NUM; i++)
			{
				ld_pos[i] = jointData.JStart[i];
				ld_vel[i] = 0;
			}
			ld_time = 0;
		}

		int cnt_temp = 0;
		int acct = 100;
		int cnt_acct = 0;
		
		// 插补稀疏
		double pct_tgtvel = 15;
		double pct_stavel = 15;
		double pct_curvel = 15;

		//------------------------  变速段规划 ------------------------------//
		// 时间jointData.T_Acc
		// 初始位置ld_pos[i] 初始速度ld_vel[i]
		// 终点位置ld_pos[i] + 0.5 * (ld_vel[i] + jointData.Vel[i]) * jointData.T_Acc
		// 终点速度jointData.Vel[i]
		while(1)
		{
			ld_time += INTERP_T;

			if (ld_time < jointData.T_Acc)
			{			
				for (i=0; i<JOINT_NUM; i++)
				{
					// 加加速段
					if (ld_time < 0.5 * jointData.T_Acc)
					{
						ld_jerk = 4 * (jointData.Vel[i] - ld_vel[i])/ //该阶段的加加速度
							(jointData.T_Acc * jointData.T_Acc);
						ls_sendPoint.P[i] = ld_pos[i] 
							+ ld_vel[i] * ld_time 
							+ ld_jerk * ld_time * ld_time * ld_time / 6;
						ls_sendPoint.V[i] = ld_vel[i] 
										  + 0.5 * ld_jerk * ld_time * ld_time;
					}
					// 减加速段
					else
					{
						double ld_a0 = 2 * (jointData.Vel[i] - ld_vel[i]) //加加速段的加速度
									 / jointData.T_Acc;
						double ld_v0 = 0.5 * (ld_vel[i] + jointData.Vel[i]);//加加速段结束时的速度
						double ld_p0 = ld_pos[i]                            //加加速段结束时的位置
							+ 0.5 * ld_vel[i] * jointData.T_Acc 
							+ (jointData.Vel[i] - ld_vel[i]) * jointData.T_Acc / 12;
						double ld_t = ld_time - 0.5 * jointData.T_Acc;

						ld_jerk = - 4 * (jointData.Vel[i] - ld_vel[i]) //减加速段的加加速度
								/ (jointData.T_Acc * jointData.T_Acc);
						ls_sendPoint.P[i] = ld_p0 
										  + ld_v0 * ld_t 
										  + 0.5 * ld_a0 * ld_t * ld_t 
										  + ld_jerk * ld_t * ld_t * ld_t / 6;
						ls_sendPoint.V[i] = ld_v0 
										  + ld_a0 * ld_t 
										  + 0.5 * ld_jerk * ld_t * ld_t;
					}
				}
				ls_sendPoint.T = INTERP_T;
			}
			else // 最后一段时间片
			{
				for (i=0; i<JOINT_NUM; i++)
				{
					ls_sendPoint.P[i] = ld_pos[i] 
						+ 0.5 * (ld_vel[i] + jointData.Vel[i]) * jointData.T_Acc;
					ls_sendPoint.V[i] = jointData.Vel[i];
				}
				ls_sendPoint.T = ld_time - jointData.T_Acc + INTERP_T;
				
				flag_break = true;// 退出循环
			}

			//-------------------------  数据输出 ---------------------------//
			// 判断位置极限
			if (0 != If_PosLimit(ls_sendPoint.P, li_plimflag))
					return ERR_PLIM; // 位置极限
			ls_sendPoint.Line = jointData.Line; // 行号
			m_iPath.AddPoint(ls_sendPoint);// 插入队尾
			outputdata.push_back(ls_sendPoint);

/*
#ifdef TEST_FILE
#if (WRITE_DATA == 0)
			file_time += (int)(1000 * ls_sendPoint.T);
			file.Write_File(ls_sendPoint.P, file_time);
#endif
#if (WRITE_DATA == 1)
			file_time += (int)(1000 * ls_sendPoint.T);
			file.Write_File(ls_sendPoint.V, file_time);
#endif
#endif
*/
			// 退出循环
			if (flag_break)
			{
				flag_break = false;
				// 初始化
				for (i=0; i<JOINT_NUM; i++)
				{
					ld_pos[i] = ls_sendPoint.P[i];
					ld_vel[i] = ls_sendPoint.V[i];
				}
				ld_time = 0; // 初始化时间
				
				break; // 退出循环
			}
		} // end of 变速段

		//------------------------  匀速段规划 ------------------------------//
		// 时间jointData.T - 2 * jointData.T_Acc
		// 初始位置ld_pos[i] 初始速度ld_vel[i]
		// 终点位置ld_pos[i] + ld_vel[i] * jointData.T
		// 终点速度ld_vel[i]
		while(1)
		{
			// 无匀速段
			if (fabs(jointData.T - 2 * jointData.T_Acc) < RT_LITTLE)
			{
				break;
			}


			cnt_temp++;                                                                       //----------
			if (cnt_temp > 200)                                                               //----------
			{
				if(cnt_acct < acct)                                                           //----------
				{
					cnt_acct++;
					pct_curvel = pct_stavel + (pct_tgtvel - pct_stavel) * cnt_acct / acct;
				}
			}

			ld_time += INTERP_T * pct_curvel;                                                  //----------?

			if (ld_time < (jointData.T - 2 * jointData.T_Acc))
			{				
				for (i = 0; i < JOINT_NUM; i++)
				{
					ls_sendPoint.P[i] = ld_pos[i] + ld_vel[i] * ld_time;
					ls_sendPoint.V[i] = ld_vel[i];
				}
				ls_sendPoint.T = INTERP_T;
			}
			else // 最后一段时间片
			{
				for (i=0; i<JOINT_NUM; i++)
				{
					ls_sendPoint.P[i] = ld_pos[i] 
						+ ld_vel[i] * (jointData.T - 2 * jointData.T_Acc);
					ls_sendPoint.V[i] = jointData.Vel[i];
				}
				ls_sendPoint.T = ld_time 
					- (jointData.T - 2 * jointData.T_Acc) + INTERP_T;

				flag_break = true; // 退出循环
			}


			//-------------------------  数据输出 ---------------------------//
			// 判断位置极限
			if (0 != If_PosLimit(ls_sendPoint.P, li_plimflag))
					return ERR_PLIM; // 位置极限
			ls_sendPoint.Line = jointData.Line; // 行号
			m_iPath.AddPoint(ls_sendPoint);
			outputdata.push_back(ls_sendPoint);

/*
#ifdef TEST_FILE
			file_time += (int)(1000 * ls_sendPoint.T);
#if (WRITE_DATA == 0)
			file.Write_File(ls_sendPoint.P, file_time);
#endif
#if(WRITE_DATA == 1)
			file.Write_File(ls_sendPoint.V, file_time);
#endif
#endif
*/
			// 退出循环
			if (flag_break)
			{
				flag_break = false;
				// 初始化
				for (i=0; i<JOINT_NUM; i++)
				{
					ld_pos[i] = ls_sendPoint.P[i];
					ld_vel[i] = ls_sendPoint.V[i];
				}
				ld_time = 0; // 初始化时间
				
				break; // 退出循环
			}
		} // end of 匀速段

		//------------------------  减速段规划 ------------------------------//
		// 最后的减速段
		if (count == j)
		{
			// 时间-jointData.T_Acc
			// 初始位置ld_pos[i] 初始速度ld_vel[i]
			// 终点位置ld_pos[i] + 0.5 * ld_vel[i] * jointData.T_Acc
			// 终点速度0
			while(1)
			{
				ld_time += INTERP_T * pct_curvel;
				
				if (ld_time < jointData.T_Acc)
				{
					// 加减速段
					for (i=0; i<JOINT_NUM; i++)
					{
						if (ld_time < 0.5 * jointData.T_Acc)
						{
							ld_jerk = 4 * (0 - ld_vel[i]) 
								/ (jointData.T_Acc * jointData.T_Acc);
							ls_sendPoint.P[i] = ld_pos[i] 
								+ ld_vel[i] * ld_time 
								+ ld_jerk * ld_time * ld_time * ld_time / 6;
							ls_sendPoint.V[i] = ld_vel[i] 
								+ 0.5 * ld_jerk * ld_time * ld_time;
						}
						// 减减速段
						else
						{
							double ld_a0 = 2 * (0 - ld_vel[i]) 
								/ jointData.T_Acc;
							double ld_v0 = 0.5 * (ld_vel[i] + 0);
							double ld_p0 = ld_pos[i] 
								+ 0.5 * ld_vel[i] * jointData.T_Acc
								+ (0 - ld_vel[i]) * jointData.T_Acc / 12;
							double ld_t = ld_time - 0.5 * jointData.T_Acc;
							
							ld_jerk = - 4 * (0 - ld_vel[i]) 
								/ (jointData.T_Acc * jointData.T_Acc);
							ls_sendPoint.P[i] = ld_p0 
								+ ld_v0 * ld_t 
								+ 0.5 * ld_a0 * ld_t * ld_t 
								+ ld_jerk * ld_t * ld_t * ld_t / 6;
							ls_sendPoint.V[i] = ld_v0 
								+ ld_a0 * ld_t 
								+ 0.5 * ld_jerk * ld_t * ld_t;
						}
					}
					ls_sendPoint.T = INTERP_T;
				}
				else // 最后一段时间片
				{
					for (i=0; i<JOINT_NUM; i++)
					{
						ls_sendPoint.P[i] = ld_pos[i] 
							+ 0.5 * ld_vel[i] * jointData.T_Acc;
						ls_sendPoint.V[i] = 0;
					}
					ls_sendPoint.T = ld_time - jointData.T_Acc + INTERP_T;
					
					flag_break = true; // 退出循环
				}

				//-----------------------  数据输出 -------------------------//
				// 判断位置极限
				if (0 != If_PosLimit(ls_sendPoint.P, li_plimflag))
					return ERR_PLIM; // 位置极限
				ls_sendPoint.Line = jointData.Line; // 行号
				m_iPath.AddPoint(ls_sendPoint);
				outputdata.push_back(ls_sendPoint);

/*
#ifdef TEST_FILE
			file_time += (int)(1000 * ls_sendPoint.T);
#if (WRITE_DATA == 0)
			file.Write_File(ls_sendPoint.P, file_time);
#endif
#if(WRITE_DATA == 1)
			file.Write_File(ls_sendPoint.V, file_time);
#endif
#endif
*/
				// 退出循环
				if (flag_break)
				{
					flag_break = false;
					// 初始化
					for (i=0; i<JOINT_NUM; i++)
					{
						ld_pos[i] = ls_sendPoint.P[i];
						ld_vel[i] = ls_sendPoint.V[i];
					}
					ld_time = 0; // 初始化时间
				
					break; // 退出循环
				}
			}
		} // end of 最后减速段
	}
/*
#ifdef TEST_FILE
	file.Close_File();
#endif
*/
	return Ok;
}

/*****************************************************************************
 * 函数：Plan_Line()
 * 功能：规划Line插补运动
 *
 * 输入：InterpStruct* in - 指令, 需给定Mode|JStart|JEnd|Acc|Jerk|Vel|G_V|T_Acc
 *
 * 返回：int - 0正常, 其他错误
 *****************************************************************************/
int Interpolation::Plan_Line(InterpStruct* in)
// int Plan_Line(IN InterpStruct* in,std::vector<PointPVT>* outputdata)
{
/*
////////////////////////////////////
#ifdef TEST_FILE                  //
	Test_File file;               //
	int nAxisNum;                 //
	if (WRITE_DATA == 2)          //
		nAxisNum = 6;             //
	else                          //
		nAxisNum = JOINT_NUM;     //
	file.Init_File(nAxisNum);     //
	int file_time = 0;            //
#endif                            //
////////////////////////////////////
*/	
	//-----------------------------  定义变量 -------------------------------//
	// 输入 - 规划 - 逆解 - 关节插补 - 输出
	double ld_time; // 运动时间
	int li_num; // 插补点数
	int i,j,k;
	int li_plimflag[MAX_AXIS_NUM]; // 位置极限标志

	double CStart[6]; // 当前点位姿, 计算所的, 无需输入
	double CEnd[6];   // 终点位姿

	SpdPTrajInputData ls_inPTraj; // 输入位置轨迹参数
	SpdPTrajInputData ls_inGTraj; // 输入姿态轨迹参数
	SpdPTrajData ls_outPTraj; // 输出位置轨迹参数
	SpdPTrajData ls_outGTraj; // 输出姿态轨迹参数

	TrajPointPara ls_pospara, ls_gespara; 
	double ld_pos[6]; // 位姿
	double ld_temp, ld_len; 
	double ld_jointpos[MAX_AXIS_NUM]; // 关节角 - 逆解结果
	double ld_jcurr[MAX_AXIS_NUM];

	PointPVT ls_sendPoint;  // 输出点数据
	outputdata.clear();

	// 初始化类库
	m_iPath.Clear(); // 初始化path类
	SpeedProcess lc_speed; // 初始化速度处理类
	JointInterp lc_jInterp(in->JStart, 4); // 初始化关节插补类
	
	//-----------------------------  输入处理 -------------------------------//
	for (i=0; i<JOINT_NUM; i++)
	{
		 ld_jcurr[i] = in->JStart[i];
	}
	if (in->T_Acc <= 0)
	{
		return ERR_MOTIONPARA; // 输入运动参数有误
	}
	// 当前点位姿 - 正解
	switch(m_nMode)
	{
	case ROBOT_MODE_IR_5:
		m_iKine_IR_5.FKine(in->JStart, CStart);
		m_iKine_IR_5.FKine(in->JEnd, CEnd);
		break;
	case ROBOT_MODE_IR_6:
		m_iKine_IR_6.FKine(in->JStart, CStart);
		m_iKine_IR_6.FKine(in->JEnd, CEnd);
		break;
	case ROBOT_MODE_CR_P:
	case ROBOT_MODE_CR_W:
		if (GRIP_FIRST == m_nGrip)
		{
			m_iKine_CR_G1.FKine(in->JStart, CStart);
			m_iKine_CR_G1.FKine(in->JEnd, CEnd);
		}
		else
		{
			m_iKine_CR_G2.FKine(in->JStart, CStart);
			m_iKine_CR_G2.FKine(in->JEnd, CEnd);
		}
		break;
	case ROBOT_MODE_SM:
	case ROBOT_MODE_DM:
	case ROBOT_MODE_BR:
	case ROBOT_MODE_WR:
	default:
		return ERR_NOGIVENKINE;
		break;
	}

	// 修正点姿态 - 姿态变化最小
	Modify_RadRot(CStart, CEnd);

	// 极限判断 | 奇异判断
	int flag_poslim[MAX_AXIS_NUM];
	// 判断极限
	if ((If_PosLimit(in->JStart, flag_poslim)) ||
		(If_PosLimit(in->JEnd,flag_poslim) ) )
	{
		return ERR_PINLIM; // 极限
	}
	
	// 直线距离 - 位置和姿态
	ls_inPTraj.Len = 
		sqrt((CEnd[0] - CStart[0]) * (CEnd[0] - CStart[0]) + 
		     (CEnd[1] - CStart[1]) * (CEnd[1] - CStart[1]) + 
		     (CEnd[2] - CStart[2]) * (CEnd[2] - CStart[2]) );
	ls_inGTraj.Len = 
		sqrt((CEnd[3] - CStart[3]) * (CEnd[3] - CStart[3]) + 
		     (CEnd[4] - CStart[4]) * (CEnd[4] - CStart[4]) + 
		     (CEnd[5] - CStart[5]) * (CEnd[5] - CStart[5]) );


	//-----------------------------  轨迹规划 -------------------------------//
	//------------------- 位置规划 -------------------//
	ls_inPTraj.Acc = in->Acc;
	ls_inPTraj.Jerk = in->Jerk;
	ls_inPTraj.T_Acc = in->T_Acc;
	ls_inPTraj.Ve = ls_inPTraj.Vs = 0;
	ls_inPTraj.V = in->Vel;

	lc_speed.SpeedPlanningS1(&ls_inPTraj, &ls_outPTraj);
		
	ld_time = ls_outPTraj.T;

	//------------------- 姿态规划 -------------------//
	ls_inGTraj.Acc = in->Acc;
	ls_inGTraj.Jerk = in->Jerk;
	ls_inGTraj.T_Acc = in->T_Acc;
	ls_inGTraj.Ve = ls_inGTraj.Vs = 0;
	ls_inGTraj.V = in->G_V;
		
	lc_speed.SpeedPlanningS1ByTime(&ls_inGTraj, ld_time, &ls_outGTraj);

	// 插补点数
	li_num = (int)ceil(ld_time / INTERP_T);


	// 循环计算插补点数据
	for (i=0; i<=li_num; i++)
	{
		if (li_num <= 0)
		{
			ld_temp = 0;
		}
		else
		{
			ld_temp = ld_time * i / li_num;
		}

		//----------- 获取位置 ------------//
		lc_speed.Get_SpeedProcessData(&ls_outPTraj, ld_temp, &ls_pospara);
		
		if (fabs(ls_outPTraj.Len) < RT_LITTLE)
		{
			ld_len = 0;
		}
		else if (ls_pospara.Len >= ls_outPTraj.Len)
		{
			ld_len = 1;
		}
		else
		{
			ld_len = ls_pospara.Len / ls_outPTraj.Len;
		}
		
		ld_pos[0] = CStart[0] + ld_len * (CEnd[0] - CStart[0]);
		ld_pos[1] = CStart[1] + ld_len * (CEnd[1] - CStart[1]);
		ld_pos[2] = CStart[2] + ld_len * (CEnd[2] - CStart[2]);
		
		//----------- 姿态规划 ------------//
		lc_speed.Get_SpeedProcessData(&ls_outGTraj, ld_temp, &ls_gespara);

		if (fabs(ls_outGTraj.Len) < RT_LITTLE)
		{
			ld_len = 0;
		}
		else if (ls_gespara.Len >= ls_outGTraj.Len)
		{
			ld_len = 1;
		}
		else
		{
			ld_len = ls_gespara.Len / ls_outGTraj.Len;
		}

		ld_pos[3] = CStart[3] + ld_len * (CEnd[3] - CStart[3]);
		ld_pos[4] = CStart[4] + ld_len * (CEnd[4] - CStart[4]);
		ld_pos[5] = CStart[5] + ld_len * (CEnd[5] - CStart[5]);


		//---------------------------  逆解处理 -----------------------------//
		// ld_pos 笛卡尔位姿/速度  - 关节数据 ld_jointpos
		// ls_gespara.Vel | ls_pospara.Vel
		int kine_flag;
		switch(m_nMode) //ERR_NOGIVENKINE
		{
		case ROBOT_MODE_IR_5:
			kine_flag = m_iKine_IR_5.IKine(ld_pos, ld_jcurr, ld_jointpos);
			break;
		case ROBOT_MODE_IR_6:
			kine_flag = m_iKine_IR_6.IKine(ld_pos, ld_jcurr, ld_jointpos);
			break;
		case ROBOT_MODE_CR_P:
		case ROBOT_MODE_CR_W:			
			if (GRIP_FIRST == m_nGrip)
				kine_flag = m_iKine_CR_G1.IKine(ld_pos, ld_jcurr, ld_jointpos);
			else
				kine_flag = m_iKine_CR_G2.IKine(ld_pos, ld_jcurr, ld_jointpos);

			break;
		case ROBOT_MODE_SM:
		case ROBOT_MODE_DM:
		case ROBOT_MODE_BR:
		case ROBOT_MODE_WR:
		default:
			return ERR_NOGIVENKINE;
			break;
		}

		if (Ok != kine_flag)
		{
			return ERR_NOINV; // 逆解错误
		}

		ld_jcurr[0] = ld_jointpos[0];
		ld_jcurr[1] = ld_jointpos[1];
		ld_jcurr[2] = ld_jointpos[2];
		ld_jcurr[3] = ld_jointpos[3];
		ld_jcurr[4] = ld_jointpos[4];
/*
#ifdef TEST_FILE
#if (WRITE_DATA == 2)
		file_time += (int)(1000 * ls_sendPoint.T);
		file.Write_File(ld_pos, file_time);
#endif
#endif
*/
		//---------------------------  关节插补 -----------------------------//
		// 如果是最后一点,关节规划停止 - 插入3个相同的点
		int li_flag = (i == li_num) ? 5 : 1;

		for (k=0; k<li_flag; k++)
		{
			// 插入点
			lc_jInterp.InsertNewPoint(ld_jointpos);

			for (j=1; j<=lc_jInterp.Get_Num(); j++)
			{
				lc_jInterp.Get_JointInterpData(j, ls_sendPoint.P, ls_sendPoint.V);

				// 判断位置极限
				if (0 != If_PosLimit(ls_sendPoint.P, li_plimflag))
				{
					return ERR_PLIM; // 位置极限
				}

				//-----------------------  数据输出 -------------------------//
				ls_sendPoint.T = POS_T;
				ls_sendPoint.Line = in->Line; // 行号
				m_iPath.AddPoint(ls_sendPoint);
				outputdata.push_back(ls_sendPoint);
/*
#ifdef TEST_FILE
#if (WRITE_DATA == 0)
				file_time += (int)(1000 * ls_sendPoint.T);
				file.Write_File(ls_sendPoint.P, file_time);
#endif
#if(WRITE_DATA == 1)
				file_time += (int)(1000 * ls_sendPoint.T);
				file.Write_File(ls_sendPoint.V, file_time);
#endif
#endif
*/
			}
		}// end of J插补

	}// end of C插补

/*
#ifdef TEST_FILE
	file.Close_File();
#endif
*/
	return Ok;
}

/*****************************************************************************
 * 函数：Plan_Circle()
 * 功能：规划Circle插补运动
 *
 * 输入：InterpStruct* in - 指令, 需给定Mode|JStart|JMid|JEnd|Acc|Jerk|Vel|G_V|T_Acc
 *
 * 返回：int - 0正常, 其他错误
 *****************************************************************************/
int Interpolation::Plan_Circle(InterpStruct* in)
{
/*
////////////////////////////////////
#ifdef TEST_FILE                  //
	Test_File file;               //
	int nAxisNum;                 //
	if (WRITE_DATA == 2)          //
		nAxisNum = 6;             //
	else                          //
		nAxisNum = JOINT_NUM;     //
	file.Init_File(nAxisNum);     //
	int file_time = 0;            //
#endif                            //
////////////////////////////////////
*/
	//-----------------------------  定义变量 -------------------------------//
	// 输入 - 规划 - 逆解 - 输出
	int i,j,k;
	double ld_time; // 运动时间
	int li_num; // 插补点数
	int li_plimflag[MAX_AXIS_NUM]; // 位置极限标志

	double CStart[6]; // 当前点位姿, 计算所的, 无需输入
	double CMid[6];   // 中间点位姿, (x,y,z,w,p,r), 位置mm, 姿态deg
	double CEnd[6];   // 终点位姿

	SpdPTrajInputData ls_inPTraj; // 输入位置轨迹参数
	SpdPTrajInputData ls_inGTraj; // 输入姿态轨迹参数

	SpdPTrajData ls_outPTraj; // 输出位置轨迹参数
	SpdPTrajData ls_outGTraj; // 输出姿态轨迹参数

	TrajPointPara ls_pospara, ls_gespara; 
	double ld_pos[6]; // 位姿
	double ld_temp, ld_len; 
	double ld_jcurr[MAX_AXIS_NUM]; // 当前关节角
	double ld_jointpos[MAX_AXIS_NUM]; // 关节角 - 逆解结果
	int kine_flag; // 求逆解状态

	PointPVT ls_sendPoint;  // 输出点数据

	// 初始化类库
	m_iPath.Clear(); // 初始化path类
	SpeedProcess lc_speed; // 初始化速度处理类
	JointInterp lc_jInterp(in->JStart, 4); // 初始化关节插补类

	// 圆弧中间变量定义
	VectorStruct Ve1; // 输入的三点确定的平面的法向量的三个坐标值及常数
	VectorStruct Ve2; // 过第一点到第二点的中点且垂直第一点到第二点的向量的平面的法向量的三个坐标值及常数
	VectorStruct Ve3; // 过第二点到第三点的中点且垂直第二点到第三点的向量的平面的法向量的三个坐标值及常数
	VectorStruct Centre; // 圆心坐标及半径
	double ld_temp1, ld_temp2, ld_temp3, ld_temp4, ld_temp5, ld_temp6;
	
	MtxKine UVW;  // UVW新坐标系矩阵
	double Radian; // 圆弧的弧度

	//-----------------------------  输入处理 -------------------------------//
	for (i=0; i<MAX_AXIS_NUM; i++)
	{
		ld_jcurr[i] = in->JStart[i];		//当前关节角等于初始点
	}	

	if (in->T_Acc <= 0)				//加速时间不大于0则报错
	{
		return ERR_MOTIONPARA; // 输入运动参数有误
	}

	// 当前点位姿 - 正解
	switch(m_nMode)
	{
	case ROBOT_MODE_IR_5:
		m_iKine_IR_5.FKine(in->JStart, CStart);			//根据输入的插补参数（即关节空间参数），计算正解（即笛卡尔空间参数）
		m_iKine_IR_5.FKine(in->JMid, CMid);
		m_iKine_IR_5.FKine(in->JEnd, CEnd);
		break;
	case ROBOT_MODE_IR_6:
		m_iKine_IR_6.FKine(in->JStart, CStart);
		m_iKine_IR_6.FKine(in->JMid, CMid);
		m_iKine_IR_6.FKine(in->JEnd, CEnd);
		break;
	case ROBOT_MODE_CR_P:
	case ROBOT_MODE_CR_W:
		if (GRIP_FIRST == m_nGrip)
		{
			m_iKine_CR_G1.FKine(in->JStart, CStart);
			m_iKine_CR_G1.FKine(in->JMid, CMid);
			m_iKine_CR_G1.FKine(in->JEnd, CEnd);
		}
		else
		{
			m_iKine_CR_G2.FKine(in->JStart, CStart);
			m_iKine_CR_G2.FKine(in->JMid, CMid);
			m_iKine_CR_G2.FKine(in->JEnd, CEnd);
		}
		break;
	case ROBOT_MODE_SM:
	case ROBOT_MODE_DM:
	case ROBOT_MODE_BR:
	case ROBOT_MODE_WR:
	default:
		return ERR_NOGIVENKINE;
		break;
	}

	// 修正点姿态 - 姿态变化最小
	Modify_RadRot(CStart, CMid);
	Modify_RadRot(CMid, CEnd);

	// 极限判断 | 奇异判断
	int flag_poslim[MAX_AXIS_NUM];
	// 判断极限
	if ((If_PosLimit(in->JStart, flag_poslim)) ||
		(If_PosLimit(in->JEnd, flag_poslim)) ||
		(If_PosLimit(in->JMid, flag_poslim)) )
	{
		return ERR_PINLIM; // 输入点极限
	}

	//-----------------------------  圆弧处理 -------------------------------//
	// 判断三点是否共线 //
	Ve1.X = (CStart[1] - CMid[1]) * (CMid[2] - CEnd[2])
		   -(CStart[2] - CMid[2]) * (CMid[1] - CEnd[1]);                                  
	Ve1.Y = (CStart[2] - CMid[2]) * (CMid[0] - CEnd[0])                                  
		   -(CStart[0] - CMid[0]) * (CMid[2] - CEnd[2]);                                  
	Ve1.Z = (CStart[0] - CMid[0]) * (CMid[1] - CEnd[1])                                 
		   -(CStart[1] - CMid[1]) * (CMid[0] - CEnd[0]);     

	if ((fabs(Ve1.X) < RT_LITTLE) && 
		(fabs(Ve1.Y) < RT_LITTLE) &&
		(fabs(Ve1.Z) < RT_LITTLE) )
	{
		return ERR_PINLIN; // 三点共线
	}
	
	// 求三个平面的系数 //
	Ve1.Con = Ve1.X * CEnd[0] + 
		      Ve1.Y * CEnd[1] + 
			  Ve1.Z * CEnd[2];

	Ve2.X = CMid[0] - CStart[0];
	Ve2.Y = CMid[1] - CStart[1];
	Ve2.Z = CMid[2] - CStart[2];
	Ve2.Con = ( CMid[0] * CMid[0] + 
	            CMid[1] * CMid[1] +
  		        CMid[2] * CMid[2] -
  			    CStart[0] * CStart[0]   -
			    CStart[1] * CStart[1]   -
			    CStart[2] * CStart[2]    ) / 2; 
	
	Ve3.X = CEnd[0] - CMid[0];
	Ve3.Y = CEnd[1] - CMid[1];
	Ve3.Z = CEnd[2] - CMid[2];
	Ve3.Con = ( CEnd[0] * CEnd[0] + 
	            CEnd[1] * CEnd[1] +
	 	        CEnd[2] * CEnd[2] - 
	 	        CMid[0] * CMid[0] -
		        CMid[1] * CMid[1] - 
		        CMid[2] * CMid[2] ) / 2;

	// 求圆心 - 讨论六种情况 //
	if (fabs(Ve1.X) < RT_LITTLE)
	{
		if (fabs(Ve1.Y) < RT_LITTLE)
		{
			Centre.Z = Ve1.Con / Ve1.Z;

			ld_temp5 = Ve2.Con - Ve2.Z * Ve1.Con / Ve1.Z;
			ld_temp6 = Ve3.Con - Ve3.Z * Ve1.Con / Ve1.Z;

			if (fabs(Ve2.X) < RT_LITTLE)
			{
				Centre.Y = ld_temp5 / Ve2.Y;
				Centre.X = (ld_temp6 - Ve3.Y * Centre.Y) / Ve3.X;
			}
			else
			{
				Centre.Y = (ld_temp6 - Ve3.X * ld_temp5 / Ve2.X)
					/ (Ve3.Y - Ve3.X * Ve2.Y / Ve2.X);
				Centre.X = ld_temp5 / Ve2.X - 
				                Centre.Y * Ve2.Y / Ve2.X;
			}
		}
		else
		{
			ld_temp3 = Ve2.Z - Ve2.Y * Ve1.Z / Ve1.Y;                         
			ld_temp4 = Ve3.Z - Ve3.Y * Ve1.Z / Ve1.Y;                          
			ld_temp5 = Ve2.Con - Ve2.Y * Ve1.Con / Ve1.Y;                         
			ld_temp6 = Ve3.Con - Ve3.Y * Ve1.Con / Ve1.Y;  

			if (fabs(Ve2.X) < RT_LITTLE)
			{
				Centre.Z = ld_temp5 / ld_temp3;
				Centre.X = (ld_temp6 - ld_temp4 * Centre.Z) / Ve3.X;
			}
			else
			{
				Centre.Z = (ld_temp6 - Ve3.X * ld_temp5 / Ve2.X)
					/ (ld_temp4 - Ve3.X * ld_temp3 / Ve2.X);
				Centre.X = ld_temp5 / Ve2.X - 
				                ld_temp3 * Centre.Z / Ve2.X;
			}
			Centre.Y = Ve1.Con / Ve1.Y
				- Ve1.Z / Ve1.Y * Centre.Z;                                                 
		}
	}
	else 
	{
		ld_temp1 = Ve2.Y - Ve2.X * Ve1.Y / Ve1.X;
		ld_temp2 = Ve3.Y - Ve3.X * Ve1.Y / Ve1.X;
		ld_temp3 = Ve2.Z - Ve2.X * Ve1.Z / Ve1.X;
		ld_temp4 = Ve3.Z - Ve3.X * Ve1.Z / Ve1.X;
		ld_temp5 = Ve2.Con - Ve2.X * Ve1.Con / Ve1.X;
		ld_temp6 = Ve3.Con - Ve3.X * Ve1.Con / Ve1.X;

		if (fabs(ld_temp1) < RT_LITTLE)
		{
			Centre.Z = ld_temp5 / ld_temp3;
			Centre.Y = (ld_temp6 - ld_temp4 * Centre.Z) / ld_temp2;
		}
		else 
		{
			Centre.Z = (ld_temp6 - ld_temp2 * ld_temp5 / ld_temp1) 
				/ (ld_temp4 - ld_temp2 * ld_temp3 / ld_temp1 );
			Centre.Y = ld_temp5 / ld_temp1
				- ld_temp3 * Centre.Z / ld_temp1;
		}

		Centre.X = Ve1.Con / Ve1.X 
			- Ve1.Y * Centre.Y / Ve1.X 
			- Ve1.Z * Centre.Z / Ve1.X ;
	}

	// 求半径 //
	Centre.Con = sqrt((Centre.X - CStart[0]) * 
	                  (Centre.X - CStart[0])  +
	  	              (Centre.Y - CStart[1]) * 
				      (Centre.Y - CStart[1])  +
				      (Centre.Z - CStart[2]) * 
				      (Centre.Z - CStart[2])   );
	
	// 变换成UVW新坐标系 //即末端执行器的Z轴平行于法向量Ve1
	ld_temp1 = sqrt( Ve1.X * Ve1.X + 
			 	     Ve1.Y * Ve1.Y +
					 Ve1.Z * Ve1.Z );
	UVW.R11 = (CStart[0] - Centre.X) / Centre.Con;
	UVW.R21 = (CStart[1] - Centre.Y) / Centre.Con;
	UVW.R31 = (CStart[2] - Centre.Z) / Centre.Con;
	UVW.R13 = Ve1.X / ld_temp1 ;
	UVW.R23 = Ve1.Y / ld_temp1 ;
	UVW.R33 = Ve1.Z / ld_temp1 ;
	UVW.R12 = UVW.R23 * UVW.R31 - 
	               UVW.R21 * UVW.R33;
	UVW.R22 = UVW.R33 * UVW.R11 - 
	               UVW.R13 * UVW.R31;
	UVW.R32 = UVW.R13 * UVW.R21 - 
	               UVW.R23 * UVW.R11;

	ld_temp1 = ( UVW.R11 * Centre.X + 
	             UVW.R21 * Centre.Y + 
	             UVW.R31 * Centre.Z  ) * (-1);
	ld_temp2 = ( UVW.R12 * Centre.X + 
	             UVW.R22 * Centre.Y + 
	             UVW.R32 * Centre.Z  ) * (-1);
	ld_temp3 = ( UVW.R13 * Centre.X + 
	             UVW.R23 * Centre.Y + 
	             UVW.R33 * Centre.Z  ) * (-1);
	UVW.X = UVW.R11 * CEnd[0] + 
			UVW.R21 * CEnd[1] + 
			UVW.R31 * CEnd[2] + ld_temp1;
	UVW.Y = UVW.R12 * CEnd[0] +
		    UVW.R22 * CEnd[1] + 
			UVW.R32 * CEnd[2] + ld_temp2;
	UVW.Z = 0;

	// 平面圆弧弧长
	Radian = atan2(UVW.Y, UVW.X );

	if (fabs(Radian) < 0.05236)
	{
		return ERR_TOOCLS; // 若圆弧弧度小于3度,则提示点过近错误
	}

	// 距离 - 位置和姿态
	if (UVW.Y < 0)
	{
		ls_inPTraj.Len = (2 * PI + Radian) * Centre.Con;
	}
	else 
	{
		ls_inPTraj.Len = Radian * Centre.Con;
	}

	ls_inGTraj.Len = 
		sqrt((CEnd[3] - CStart[3]) * (CEnd[3] - CStart[3]) + 
		     (CEnd[4] - CStart[4]) * (CEnd[4] - CStart[4]) + 
		     (CEnd[5] - CStart[5]) * (CEnd[5] - CStart[5]) );

	//////////////////////////////////////// 整圆规划
	if (in->IfLinkage == 1)
	{
		// 整圆弧长
		ls_inPTraj.Len = 2 * PI * Centre.Con;
		ls_inGTraj.Len = 0;

		double jcurrent[MAX_AXIS_NUM];  // 中间变量, 当前关节角
		for (i=0; i<JOINT_NUM; i++)
		{
			jcurrent[i] = in->JStart[i];
		}

		// 获取五角星顶点坐标
		// 第1-6点
		int nPoint[6] = {0, 2, 4, 1, 3, 5}; // {0, 1, 2, 3, 4, 5}
		for(i=0; i<6; i++)
		{
			// 圆弧转换
			ld_temp1 = Centre.Con * cos(nPoint[i] * 0.4 * PI);
			ld_temp2 = Centre.Con * sin(nPoint[i] * 0.4 * PI);
			// 换回参考坐标系
			ld_pos[0] = UVW.R11 * ld_temp1 + UVW.R12 * ld_temp2 + Centre.X;
			ld_pos[1] = UVW.R21 * ld_temp1 + UVW.R22 * ld_temp2 + Centre.Y;
			ld_pos[2] = UVW.R31 * ld_temp1 + UVW.R32 * ld_temp2 + Centre.Z ;
			ld_pos[3] = CStart[3];
			ld_pos[4] = CStart[4];
			ld_pos[5] = CStart[5];

			// 逆解处理
			kine_flag = I_Kine(m_nMode, ld_pos, jcurrent, ld_jointpos);
			if (Ok != kine_flag)
			{
				return ERR_NOINV; // 逆解错误
			}

			// 点赋值
			for(j=0; j<JOINT_NUM; j++)
			{
				m_iStarPoint.Point[i][j] = ld_jointpos[j];
				jcurrent[j] = ld_jointpos[j]; // 当前关节角
			}
			m_iStarPoint.Mode[i] = INTERP_LINEAR;
		}

		m_iStarPoint.Count = 6;
		
		m_bStarPoint = true;

	}
	/////////////////////////////////////////////////


	//-----------------------------  轨迹规划 -------------------------------//
	//------------------- 位置规划 -------------------//
	ls_inPTraj.Acc = in->Acc;
	ls_inPTraj.Jerk = in->Jerk;
	ls_inPTraj.T_Acc = in->T_Acc;
	ls_inPTraj.Ve = ls_inPTraj.Vs = 0;
	ls_inPTraj.V = in->Vel;

	lc_speed.SpeedPlanningS1(&ls_inPTraj, &ls_outPTraj);
		
	ld_time = ls_outPTraj.T;

	//------------------- 姿态规划 -------------------//
	ls_inGTraj.Acc = in->Acc;
	ls_inGTraj.Jerk = in->Jerk;
	ls_inGTraj.T_Acc = in->T_Acc;
	ls_inGTraj.Ve = ls_inGTraj.Vs = 0;
	ls_inGTraj.V = in->G_V;
		
	lc_speed.SpeedPlanningS1ByTime(&ls_inGTraj, ld_time, &ls_outGTraj);

	// 插补点数
	li_num = (int)ceil(ld_time / INTERP_T);


	// 循环计算插补点数据
	for (i=0; i<=li_num; i++)
	{
		ld_temp = ld_time * i / li_num;

		//----------- 获取位置 ------------//
		lc_speed.Get_SpeedProcessData(&ls_outPTraj, ld_temp, &ls_pospara);

		if (fabs(ls_outPTraj.Len) < RT_LITTLE)
		{
			ld_len = 0;
		}
		else if (ls_pospara.Len >= ls_outPTraj.Len)
		{
			ld_len = ls_outPTraj.Len;
		}
		else
		{
			ld_len = ls_pospara.Len;
		}

		// 圆弧转换
		ld_temp1 = Centre.Con * cos(ld_len / Centre.Con);
		ld_temp2 = Centre.Con * sin(ld_len / Centre.Con);
		// 换回参考坐标系
		ld_pos[0] = UVW.R11 * ld_temp1 + UVW.R12 * ld_temp2 + Centre.X;
		ld_pos[1] = UVW.R21 * ld_temp1 + UVW.R22 * ld_temp2 + Centre.Y;
		ld_pos[2] = UVW.R31 * ld_temp1 + UVW.R32 * ld_temp2 + Centre.Z ;
		
	
		//----------- 姿态规划 ------------//
		lc_speed.Get_SpeedProcessData(&ls_outGTraj, ld_temp, &ls_gespara);

		if (fabs(ls_outGTraj.Len) < RT_LITTLE)
		{
			ld_len = 0;
		}
		else if (ls_gespara.Len >= ls_outGTraj.Len)
		{
			ld_len = 1;
		}
		else
		{
			ld_len = ls_gespara.Len / ls_outGTraj.Len;
		}

		ld_pos[3] = CStart[3] + ld_len * (CEnd[3] - CStart[3]);
		ld_pos[4] = CStart[4] + ld_len * (CEnd[4] - CStart[4]);
		ld_pos[5] = CStart[5] + ld_len * (CEnd[5] - CStart[5]);


		//---------------------------  逆解处理 -----------------------------//
		// ld_pos 笛卡尔位姿/速度  - 关节数据 ld_jointpos
		// ls_gespara.Vel | ls_pospara.Vel
		
		switch(m_nMode) //ERR_NOGIVENKINE
		{
		case ROBOT_MODE_IR_5:
		//	kine_flag = m_iKine_IR_5.IKine(ld_pos, ld_jcurr, ld_jointpos);
			kine_flag = m_iKine_IR_5.IKine(ld_pos, ld_jcurr, ld_jointpos);					// SSS: for special plan
			break;
		case ROBOT_MODE_IR_6:
			kine_flag = m_iKine_IR_6.IKine(ld_pos, ld_jcurr, ld_jointpos);
			break;
		case ROBOT_MODE_CR_P:
		case ROBOT_MODE_CR_W:			
			if (GRIP_FIRST == m_nGrip)
				kine_flag = m_iKine_CR_G1.IKine(ld_pos, ld_jcurr, ld_jointpos);
			else
				kine_flag = m_iKine_CR_G2.IKine(ld_pos, ld_jcurr, ld_jointpos);
			
			break;
		case ROBOT_MODE_SM:
		case ROBOT_MODE_DM:
		case ROBOT_MODE_BR:
		case ROBOT_MODE_WR:
		default:
			return ERR_NOGIVENKINE;
			break;
		}
		
		if (Ok != kine_flag)
		{
			return ERR_NOINV; // 逆解错误
		}

		for(j=0; j<JOINT_NUM; j++)
			ld_jcurr[j] = ld_jointpos[j];


#ifdef TEST_FILE		
#if (WRITE_DATA == 2)
		file_time += (int)(1000 * ls_sendPoint.T);
		file.Write_File(ld_pos, file_time);
#endif
#endif

		//---------------------------  关节插补 -----------------------------//
		// 如果是最后一点,关节规划停止 - 插入3个相同的点
		int li_flag = (i == li_num) ? 4 : 1;

		for (k=0; k<li_flag; k++)
		{
			// 插入点
			lc_jInterp.InsertNewPoint(ld_jointpos);

			for (j=1; j<=lc_jInterp.Get_Num(); j++)
			{
				lc_jInterp.Get_JointInterpData(j, ls_sendPoint.P, ls_sendPoint.V);

				// 判断位置极限
				if (0 != If_PosLimit(ls_sendPoint.P, li_plimflag))
				{
					return ERR_PLIM; // 位置极限
				}

				//-----------------------  数据输出 -------------------------//
				ls_sendPoint.T = POS_T;
				ls_sendPoint.Line = in->Line; // 行号
				m_iPath.AddPoint(ls_sendPoint);
/*
#ifdef TEST_FILE
#if (WRITE_DATA == 0)
				file_time += (int)(1000 * ls_sendPoint.T);
				//file.Write_File(ls_sendPoint.P, file_time);
				m_iKine_IR_5.FKine(ls_sendPoint.P, ld_pos);
				file.Write_File(ld_pos, file_time);
#endif
#if(WRITE_DATA == 1)
				file_time += (int)(1000 * ls_sendPoint.T);
				file.Write_File(ls_sendPoint.V, file_time);
#endif
#endif
*/
			}
		}// end of J插补	

	}// end of for
/*
#ifdef TEST_FILE
	file.Close_File();
#endif
*/

	return Ok;
}

int Interpolation::I_Kine(int mode, double gdCPos[], double gdJCurr[], double gdJPos[])
{
	// 逆解处理
	switch(mode) //ERR_NOGIVENKINE
	{
	case ROBOT_MODE_IR_5:
		return m_iKine_IR_5.IKine(gdCPos, gdJCurr, gdJPos);
		break;
	case ROBOT_MODE_IR_6:
		return m_iKine_IR_6.IKine(gdCPos, gdJCurr, gdJPos);
		break;
	case ROBOT_MODE_CR_P:
	case ROBOT_MODE_CR_W:			
		if (GRIP_FIRST == m_nGrip)
			return m_iKine_CR_G1.IKine(gdCPos, gdJCurr, gdJPos);
		else
			return m_iKine_CR_G2.IKine(gdCPos, gdJCurr, gdJPos);	
		break;
	case ROBOT_MODE_SM:
	case ROBOT_MODE_DM:
	case ROBOT_MODE_BR:
	case ROBOT_MODE_WR:
	default:
		return ERR_NOGIVENKINE;
		break;
	}
}

/*****************************************************************************
 * 函数：JointInterp()
 * 功能：构造函数
 *
 * 输入：double* pos - 当前点位置
 *       int degree  - B样条最高次幂
 *****************************************************************************/
JointInterp::JointInterp(double *pos, int degree)
{
	int i;

	if ((3 == degree) || (4 == degree))
	{
		m_nDegree = degree;
	}
	else
	{
		m_nDegree = 4;
	}

	for (i=0; i<JOINT_NUM; i++)
	{
		m_dPos0[i] = pos[i];
		m_dPos1[i] = pos[i];
		m_dPos2[i] = pos[i];
		m_dPos3[i] = pos[i];
		m_dPos4[i] = pos[i];
	}

	m_nNum = (int)(INTERP_T / POS_T);
}

/*****************************************************************************
 * 函数：~JointInterp()
 * 功能：析构函数
 *****************************************************************************/
JointInterp::~JointInterp()
{
}

/*****************************************************************************
 * 函数：InsertNewPoint()
 * 功能：插入新点
 *
 * 输入：double* pos - 插入点位置数组, 5关节
 * 
 * 返回：void
 *****************************************************************************/
void JointInterp::InsertNewPoint(double *pos)
{
	int i;
		
	// 插入新数据并排序 0-1-2-3(-4),将新数据插入3(或4)的位置
	for (i=0; i<JOINT_NUM; i++)
	{
		m_dPos0[i] = m_dPos1[i];
		m_dPos1[i] = m_dPos2[i];
		m_dPos2[i] = m_dPos3[i];
	
		if (3 == m_nDegree)
		{
			m_dPos3[i] = pos[i];
		}
		else if (4 == m_nDegree)
		{
			m_dPos3[i] = m_dPos4[i];
			m_dPos4[i] = pos[i];
		}
	}
}

/*****************************************************************************
 * 函数：Get_Num()
 * 功能：获取每次插补点数
 * 
 * 返回：int - 每次插补点数
 *****************************************************************************/
int JointInterp::Get_Num()
{
	return m_nNum;
}
/*****************************************************************************
 * 函数：Get_JointInterpData()
 * 功能：获取关节插补数据
 *
 * 输入：int count - 第count个点
 * 输出：double* outp - 第count个点的位置数组, 5关节
 *       double* outv - 第count个点的速度数组, 5关节
 * 
 * 返回：int - 错误状态, 0正常
 *****************************************************************************/
int JointInterp::Get_JointInterpData(int count, double* outp, double* outv)
{
	int i;
	double t = (double)count / m_nNum;

	// 获取Count所对应的插补数据
	for (i=0; i<JOINT_NUM; i++)
	{
		if (3 == m_nDegree)
		{
			outp[i] = ( (((-1 * t + 3) * t - 3) * t + 1) * m_dPos0[i] +
						((( 3 * t - 6) * t + 0) * t + 4) * m_dPos1[i] +
						(((-3 * t + 3) * t + 3) * t + 1) * m_dPos2[i] +
						((( 1 * t + 0) * t + 0) * t + 0) * m_dPos3[i] 
					  ) / 6;

			outv[i] = ( ((-1 * t + 2) * t - 1) * m_dPos0[i] +
					    (( 3 * t - 4) * t + 0) * m_dPos1[i] +
					    ((-3 * t + 2) * t + 1) * m_dPos2[i] +
				 	    (( 1 * t + 0) * t + 0) * m_dPos3[i]
				      ) / 2;

			outv[i] *= 1.0 / INTERP_T;

			//outa[i] = (-1 * t + 1) * Pos0[i] + 
			// 	        ( 3 * t - 2) * Pos1[i] +
			//		    (-3 * t + 1) * Pos2[i] +
			//		    ( 1 * t + 0) * Pos3[i];
		}
		else if (4 == m_nDegree)
		{
			outp[i] = ( (((( 1 * t -  4) * t + 6) * t -  4) * t +  1) * m_dPos0[i] +
					    ((((-4 * t + 12) * t - 6) * t - 12) * t + 11) * m_dPos1[i] +
					    (((( 6 * t - 12) * t - 6) * t + 12) * t + 11) * m_dPos2[i] +
					    ((((-4 * t +  4) * t + 6) * t +  4) * t +  1) * m_dPos3[i] +
					    (((( 1 * t -  0) * t + 0) * t -  0) * t +  0) * m_dPos4[i]
				      ) / 24;

			outv[i] = ( ((( 1 * t - 3) * t + 3) * t - 1) * m_dPos0[i] +
					    (((-4 * t + 9) * t - 3) * t - 3) * m_dPos1[i] +
					    ((( 6 * t - 9) * t - 3) * t + 3) * m_dPos2[i] +
					    (((-4 * t + 3) * t + 3) * t + 1) * m_dPos3[i] +
					    ((( 1 * t - 0) * t + 0) * t - 0) * m_dPos4[i]
				      ) / 6;

			outv[i] *= 1.0 / INTERP_T;
			//outa[i] = ( (( 1 * t - 2) * t + 1) * Pos0[i] +
			//		    ((-4 * t + 6) * t - 1) * Pos1[i] +
			//		    (( 6 * t - 6) * t - 1) * Pos2[i] +
			//		    ((-4 * t + 2) * t + 1) * Pos3[i] +
			//		    (( 1 * t - 0) * t + 0) * Pos4[i]
			//		   ) / 2;
		}
	}

	return Ok;
}