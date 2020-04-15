/*****************************************************************************
 *        加减速规划函数                                                     *
 *        Copyright (c) GSK Inc., 2010                                       *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2010-04-15                                       *
 *****************************************************************************/
#include <math.h>

#include "SpeedProcess.h"

SpeedProcess::SpeedProcess()
{}

SpeedProcess::~SpeedProcess()
{}

/*****************************************************************************
 * 函数：SpeedPlanningS1()
 * 功能：速度规划 - 简化S加减速
 *
 * 输入：SpdPTrajInputData *in - 输入轨迹信息
 * 输出：SpdPTrajData *out - 速度规划轨迹
 *
 * 返回：int - 输入参数有误-2 规划有误-1 正常0
 *****************************************************************************/
int SpeedProcess::SpeedPlanningS1(SpdPTrajInputData* pIn, SpdPTrajData* pOut)
{
	double ld_temp;

	if( (pIn->Len < 0) || (pIn->Vs < 0) || (pIn->Ve < 0) || (pIn->V < 0) )
	{
		return ERR_POINT; // 输入参数有误
	}

	// 初始化参数
	pOut->Len = pIn->Len;
	pOut->Vs = pIn->Vs;
	pOut->V = pIn->V;
	pOut->Ve = pIn->Ve;
	pOut->Jacc = pIn->Jerk;
	pOut->Jdec = pIn->Jerk;

	pOut->T2 = pOut->T6 = 0;

	ld_temp = 0.5 * (pOut->Vs + pOut->Ve) * pIn->T_Acc;

	// 没有运动
	if (pOut->Len < RT_LITTLE)
	{
		pOut->T1 = pOut->T3 = pOut->T4 = pOut->T5 = pOut->T7 = 0;
	}
	// 只有匀速段
	else if ( (fabs(pOut->Vs - pOut->V) < RT_LITTLE) &&
			 (fabs(pOut->V - pOut->Vs) < RT_LITTLE) )
	{
		pOut->T1 = pOut->T3 = 0;
		pOut->T4 = pOut->Len / pOut->V;
		pOut->T5 = pOut->T7 = 0;
	}
	// 不能到达末速度
	else if (ld_temp >= pOut->Len)
	{
		pOut->T1 = pOut->T3 = pIn->T_Acc / 2;
		pOut->T4 = pOut->T5 = pOut->T7 = 0;

		// 重新规划末速度
		pOut->V = pOut->Ve = 2 * pOut->Len / pIn->T_Acc - pOut->Vs;

		if (pOut->Ve < 0)
		{
			return -127;
		}
		// 只有匀速段
		else if (pOut->Vs == pOut->Ve)
		{
			pOut->T1 = pOut->T3 = 0;
			pOut->T4 = pOut->Len / pOut->V;
		}
		// 只有加速段或减速段
		else
		{
			pOut->Jacc = 4 * (pOut->Ve - pOut->Vs) / (pIn->T_Acc * pIn->T_Acc);
		}
	}
	// 期望速度等于始速度
	else if (fabs(pOut->Vs - pOut->V) < RT_LITTLE)
	{
		pOut->T1 = pOut->T3 = 0;
		pOut->T5 = pOut->T7 = pIn->T_Acc / 2;

		pOut->T4 = (pOut->Len - 0.5 * (pOut->V + pOut->Ve) * pIn->T_Acc) / pOut->V;

		pOut->Jdec = 4 * (pOut->Ve - pOut->V) / (pIn->T_Acc * pIn->T_Acc);
	}
	// 期望速度等于末速度
	else if (fabs(pOut->V - pOut->Ve) < RT_LITTLE)
	{
		pOut->T1 = pOut->T3 = pIn->T_Acc / 2;
		pOut->T5 = pOut->T7 = 0;

		pOut->T4 = (pOut->Len - 0.5 * (pOut->V + pOut->Ve) * pIn->T_Acc) / pOut->V;
		pOut->Jacc = 4 * (pOut->V - pOut->Vs) / (pIn->T_Acc * pIn->T_Acc);
	}
	// 期望速度不等于始末速度
	else
	{
		pOut->T1 = pOut->T3 = pIn->T_Acc / 2;
		pOut->T5 = pOut->T7 = pIn->T_Acc / 2;

		ld_temp = 0.5 * (pOut->Vs + 2 * pOut->V + pOut->Ve) * pIn->T_Acc;

		// 不含匀速段
		if (ld_temp >= pOut->Len)
		{
			pOut->T4 = 0;
			pOut->V = pOut->Len / pIn->T_Acc - 0.5 * (pOut->Vs + pOut->Ve);
		}
		else
		{
			pOut->T4 = (pOut->Len - ld_temp) / pOut->V;
		}

		pOut->Jacc = 4 * (pOut->V - pOut->Vs) / (pIn->T_Acc * pIn->T_Acc);
		pOut->Jdec = 4 * (pOut->Ve - pOut->V) / (pIn->T_Acc * pIn->T_Acc);

	}

	// 总时间
	pOut->T = pOut->T1 + pOut->T2 + pOut->T3 + pOut->T4 + pOut->T5 + pOut->T6 + pOut->T7;

	// 各段速度
	pOut->V1 = pOut->Vs + 0.5 * pOut->Jacc * pOut->T1 * pOut->T1;
	pOut->V2 = pOut->V1 + pOut->Jacc * pOut->T1 * pOut->T2;
	pOut->V3 = pOut->V2 + 0.5 * pOut->Jacc * pOut->T1 * pOut->T1;

	pOut->V4 = pOut->V3;
	pOut->V5 = pOut->V4 + 0.5 * pOut->Jdec * pOut->T5 * pOut->T5;
	pOut->V6 = pOut->V5 + pOut->Jdec * pOut->T5 * pOut->T6;

	// 各段路径
	pOut->L1 = (pOut->Vs + pOut->Jacc * pOut->T1 * pOut->T1 / 6) * pOut->T1;
	pOut->L2 = pOut->L1 + (pOut->V1 + 0.5 * pOut->Jacc * pOut->T1 * pOut->T2) * pOut->T2;
	pOut->L3 = pOut->L2 + (pOut->V2 + pOut->Jacc * pOut->T1 * pOut->T1 / 3) * pOut->T1;

	pOut->L4 = pOut->L3 + pOut->V3 * pOut->T4;
	pOut->L5 = pOut->L4 + (pOut->V4 + pOut->Jdec * pOut->T5 * pOut->T5 / 6) * pOut->T5;
	pOut->L6 = pOut->L5 + (pOut->V5 + 0.5 * pOut->Jdec * pOut->T5 * pOut->T6) * pOut->T6;

	pOut->L7 = pOut->L6 + (pOut->V6 + pOut->Jdec * pOut->T5 * pOut->T5 / 3) * pOut->T5;

	return Ok;
}
//

/*****************************************************************************
 * 函数：SpeedPlanningS1ByTime()
 * 功能：速度规划 - 总运动时间已知
 *
 * 输入：SpdPTrajInputData *in - 输入轨迹信息
 *		 double Time - 总运动时间
 * 输出：SpdPTrajData *out - 速度规划轨迹
 *
 * 返回：int - 输入参数有误-2 规划有误-1 正常0
 *****************************************************************************/
int SpeedProcess::SpeedPlanningS1ByTime(SpdPTrajInputData* pIn, double dTime, SpdPTrajData* pOut)
{
	if( (pIn->Len < 0) || (pIn->Vs < 0) || (pIn->Ve < 0) || (pIn->V < 0) || 
		(dTime < 0) )
	{
		return ERR_POINT; // 输入参数有误
	}

	// 初始化参数
	pOut->Len = pIn->Len;
	pOut->Vs = pIn->Vs;
	//out->V = in->V;
	pOut->Ve = pIn->Ve;
	pOut->Jacc = pIn->Jerk;
	pOut->Jdec = pIn->Jerk;

	pOut->T2 = pOut->T6 = 0;

	// 没有运动
	if (dTime < RT_LITTLE)
	{
		pOut->T1 = pOut->T3 = pOut->T4 = pOut->T5 = pOut->T7 = 0;
	}
	// 只有加减速段
	else if (dTime < 2 * pIn->T_Acc)
	{
		pOut->T4 = 0;
		pOut->T1 = pOut->T3 = dTime / 4;
		pOut->T5 = pOut->T7 = dTime / 4;

		pOut->V = 2 * pOut->Len / dTime - 0.5 * (pOut->Vs + pOut->Ve);
		if(pOut->V < 0)
		{
			return -1;
		}

		pOut->Jacc = 16 * (pOut->V - pOut->Vs) / (dTime * dTime);
		pOut->Jdec = 16 * (pOut->Ve - pOut->V) / (dTime * dTime);
	}
	// 不能到达末速度
	else if (dTime >= 2 * pIn->T_Acc)
	{
		pOut->T1 = pOut->T3 = pIn->T_Acc / 2;
		pOut->T5 = pOut->T7 = pIn->T_Acc / 2;

		pOut->T4 = dTime - 2 * pIn->T_Acc;

		// 重新规划末速度
		pOut->V = (pOut->Len - 0.5 * (pOut->Vs + pOut->Ve) * pIn->T_Acc) / 
			(dTime - pIn->T_Acc);

		pOut->Jacc = 4 * (pOut->V - pOut->Vs) / (pIn->T_Acc * pIn->T_Acc);
		pOut->Jdec = 4 * (pOut->Ve - pOut->V) / (pIn->T_Acc * pIn->T_Acc);
	}

	// 总时间
	pOut->T = pOut->T1 + pOut->T2 + pOut->T3 + pOut->T4 + pOut->T5 + pOut->T6 + pOut->T7;

	// 各段速度
	pOut->V1 = pOut->Vs + 0.5 * pOut->Jacc * pOut->T1 * pOut->T1;
	pOut->V2 = pOut->V1 + pOut->Jacc * pOut->T1 * pOut->T2;
	pOut->V3 = pOut->V2 + 0.5 * pOut->Jacc * pOut->T1 * pOut->T1;

	pOut->V4 = pOut->V3;
	pOut->V5 = pOut->V4 + 0.5 * pOut->Jdec * pOut->T5 * pOut->T5;
	pOut->V6 = pOut->V5 + pOut->Jdec * pOut->T5 * pOut->T6;

	// 各段路径
	pOut->L1 = (pOut->Vs + pOut->Jacc * pOut->T1 * pOut->T1 / 6) * pOut->T1;
	pOut->L2 = pOut->L1 + (pOut->V1 + 0.5 * pOut->Jacc * pOut->T1 * pOut->T2) * pOut->T2;
	pOut->L3 = pOut->L2 + (pOut->V2 + pOut->Jacc * pOut->T1 * pOut->T1 / 3) * pOut->T1;

	pOut->L4 = pOut->L3 + pOut->V3 * pOut->T4;
	pOut->L5 = pOut->L4 + (pOut->V4 + pOut->Jdec * pOut->T5 * pOut->T5 / 6) * pOut->T5;
	pOut->L6 = pOut->L5 + (pOut->V5 + 0.5 * pOut->Jdec * pOut->T5 * pOut->T6) * pOut->T6;

	pOut->L7 = pOut->L6 + (pOut->V6 + pOut->Jdec * pOut->T5 * pOut->T5 / 3) * pOut->T5;

	return Ok;
}

/*****************************************************************************
 * 函数：Solve_Equation3()
 * 功能：解一元三次方程 - 牛顿迭代法 - 形如：a x^3 + b x^2 + c x + d = 0
 *
 * 输入：double a, b, c, d - 方程系数
 *       double val        - 初始值
 *
 * 输出：double - 小于初始值的正根
 *****************************************************************************/
double SpeedProcess::Solve_Equation3(double a, double b, double c, double d, double val)
{
	double f0, f0d; // 函数及其倒数变量
	double x, x0;   // 迭代变量
	int count = 0;  // 求解次数

	// 赋初值
	x = val;

    do
    { 
		x0 = x;
        f0 = ((a * x + b) * x + c) * x + d; 
        f0d = (3 * a * x + 2 * b) * x + c; 
        
		x = x0 - f0 / f0d; 
    }
	while(fabs(f0) > 1e-6);
	
	// 结果不满足条件(小于初始值的正根),重新求解
	// 当求解次数超过5次结果仍不能满足要求,则返回错误
	if((x < 0) || (x > val))
	{
		val = val / 2;
		x = val;

		do
		{
			count++;
			if(count > 5)
			{
				return -1;
			}
			x0 = x;
			f0 = ((a * x + b) * x + c) * x + d; 
			f0d = (3 * a * x + 2 * b) * x + c; 

			x = x0 - f0 / f0d;
		}
		while(fabs(f0) > 1e-6);
	}

	return x;
}

/*****************************************************************************
 * 函数：Solve_Equation2()
 * 功能：解一元二次方程,形如ax^2 + bx + c = 0
 *
 * 输入：double a, b, c, d - 方程系数
 *
 * 输出：double* res1 - 方程的根1
 *       double* res2 - 方程的根2
 *
 * 返回：1 - 实根, 0 - 无实根
 *****************************************************************************/
int SpeedProcess::Solve_Equation2(double a, double b, double c,
						   double* res1, double* res2)
{
	// 无实数解,返回错误
	if(b * b < 4 * a * c)
	{
		*res1 = *res2 = 0;

		return ERR;
	}

	// 其中一个实数解
	*res1 = 0.5 * (- b + sqrt(b * b - 4 * a * c)) / a;

	// 另外一个实数解
	*res2 = 0.5 * (- b - sqrt(b * b - 4 * a * c)) / a;

	return Ok;
}

void SpeedProcess::Get_AccDecLen(SpdPTrajData *out, double* sacc, double *sdec)
{				
	// 加速段距离
	*sacc = out->Vs * (2 * out->T1 + out->T2) + 0.5 * out->Jacc * out->T1 *
		(2 * out->T1 * out->T1 + 3 * out->T1 * out->T2 + out->T2 * out->T2 );

	// 减速段距离
	*sdec = out->Ve * (2 * out->T5 + out->T6) - 0.5 * out->Jdec * out->T5 *
		(2 * out->T5 * out->T5 + 3 * out->T5 * out->T6 + out->T6 * out->T6 );

}
/*****************************************************************************
 * 函数：SpeedPlanning()
 * 功能：速度规划 - S加减速
 *
 * 输入：SpdPTrajInputData* pIn - 输入轨迹信息
 * 输出：SpdPTrajData* pOut - 速度规划轨迹
 *
 * 返回：int - 输入参数有误-2 规划有误-1 正常0
 *****************************************************************************/
int SpeedProcess::SpeedPlanningS(SpdPTrajInputData* pIn, SpdPTrajData* pOut)
{
	int li_resultflag;   // 有无解标志

	double ld_tt0,ld_tt1;          // 时间中间变量
	double ld_vv0,ld_vv1,ld_vv2;   // 速度中间变量
	double ld_ss1,ld_sacc,ld_sdec; // 位移中间变量

	if( (pIn->Len < 0) || (pIn->Vs < 0) || (pIn->Ve < 0) || (pIn->V < 0) )
	{
		return ERR_POINT; // 输入参数有误
	}

	// 初始化参数
	pOut->Len = pIn->Len;
	pOut->Vs = pIn->Vs;
	pOut->V = pIn->V;
	pOut->Ve = pIn->Ve;

	//--------------------------- 末速度不能到达 ---------------------------//
	// 假设由初速度Vs直接变速到Ve
	if(pOut->Vs <= pOut->Ve)
	{
		pOut->Jacc = pOut->Jdec = pIn->Jerk;
	}
	else
	{
		pOut->Jacc = pOut->Jdec = - pIn->Jerk;
	}

	// 变速段时间
	if(fabs(pOut->Ve - pOut->Vs) > pIn->Acc * pIn->Acc / pIn->Jerk)
	{
		pOut->T1 = pIn->Acc / pIn->Jerk;
		pOut->T2 = fabs(pOut->Ve - pOut->Vs) / pIn->Acc - pOut->T1;
		pOut->T3 = pOut->T1;
	}
	else
	{
		pOut->T1 = sqrt(fabs(pOut->Ve - pOut->Vs) / pIn->Jerk);
		pOut->T2 = 0;
		pOut->T3 = pOut->T1;
	}	
	
	// 变速段距离
	ld_sacc = pOut->Vs * (2 * pOut->T1 + pOut->T2) + 0.5 * pOut->Jacc * pOut->T1 *
		(2 * pOut->T1 * pOut->T1 + 3 * pOut->T1 * pOut->T2 + pOut->T2 * pOut->T2 );

	// 末速度不能到达,需要重新规划Ve
	if(ld_sacc >= pOut->Len)
	{
		pOut->T4 = pOut->T5 = pOut->T6 = pOut->T7 = 0;

		// 假设有匀减速段
		// 求out->T2
		li_resultflag = Solve_Equation2( 0.5 * pOut->Jacc * pOut->T1,
						  pOut->Vs + 1.5 * pOut->Jacc * pOut->T1 * pOut->T1,
						  - pOut->Len + 2 * pOut->Vs * pOut->T1 + pOut->Jacc * pOut->T1 * pOut->T1 * pOut->T1,
						  &ld_tt0, &ld_tt1);

		ld_vv0 = pOut->Vs + pOut->Jacc * pOut->T1 * (ld_tt0 + pOut->T3);
		ld_vv1 = pOut->Vs + pOut->Jacc * pOut->T1 * (ld_tt1 + pOut->T3);

		// 第一个解有效
		if( (Ok == li_resultflag) && (ld_tt0 >= 0) && (ld_vv0 >=0) )
		{
			pOut->T2 = ld_tt0;
			pOut->Ve = ld_vv0;
		}
		// 第二个解有效
		else if( (Ok == li_resultflag) && (ld_tt1 >= 0) && (ld_vv1 >=0) )
		{
			pOut->T2 = ld_tt1;
			pOut->Ve = ld_vv1;
		}
		// 假设没有匀减速段
		else
		{
			pOut->T1 = Solve_Equation3(pOut->Jacc, 0, 2 * pOut->Vs, - pOut->Len, pOut->T1);
			if(-1 == pOut->T1)
			{
				return ERR;
			}

			pOut->T2 = 0;
			pOut->T3 = pOut->T1;

			pOut->Ve = pOut->Vs + pOut->Jacc * pOut->T1 * (pOut->T2 + pOut->T3);
		}

		// 重值期望速度
		pOut->V = pOut->Ve;
	}
	//------------------------ (V > Vs) && (V > Ve) ------------------------//
	else if( (pOut->V > pOut->Vs) && (pOut->V > pOut->Ve) )
	{
		pOut->Jacc = pIn->Jerk;
		pOut->Jdec = - pIn->Jerk;

		// 存在匀加速段？
		if(pOut->V - pOut->Vs > pIn->Acc * pIn->Acc / pIn->Jerk)
		{
			pOut->T1 = pIn->Acc / pIn->Jerk;
			pOut->T2 = (pOut->V - pOut->Vs) / pIn->Acc - pOut->T1;
			pOut->T3 = pOut->T1;
		}
		else
		{
			pOut->T1 = sqrt((pOut->V - pOut->Vs) / pIn->Jerk);
			pOut->T2 = 0;
			pOut->T3 = pOut->T1;
		}
		// 存在匀减速段？
		if(pOut->V - pOut->Ve > pIn->Acc * pIn->Acc / pIn->Jerk)
		{
			pOut->T5 = pIn->Acc / pIn->Jerk;
			pOut->T6 = (pOut->V - pOut->Ve) / pIn->Acc - pOut->T5;
			pOut->T7 = pOut->T5;
		}
		else
		{
			pOut->T5 = sqrt((pOut->V - pOut->Ve) / pIn->Jerk);
			pOut->T6 = 0;
			pOut->T7 = pOut->T5;
		}

		// 加减速段距离
		Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

		ld_ss1 = ld_sacc + ld_sdec;

		// 含有匀速段？
		if(ld_ss1 <= pOut->Len)
		{
			pOut->T4 = (pOut->Len - ld_ss1) / pOut->V;
		}
		else // 不含匀速段
		{
			pOut->T4 = 0;
		
			// out->Ve较大
			if(pOut->Ve >= pOut->Vs)
			{
				// 假设实际最大速度 - 加速段包含加加速段、匀加速段和减加速段
				// 还有加减速段、减减速段，共5段
				ld_vv1 = pOut->Ve + pIn->Acc * pIn->Acc / pIn->Jerk; // max(out->Vs,out->Ve)

				// 求加速段和减速段的距离
				pOut->T1 = pIn->Acc / pIn->Jerk;
				pOut->T2 = (ld_vv1 - pOut->Vs) / pIn->Acc - pOut->T1;
				pOut->T3 = pOut->T1;
				pOut->T5 = sqrt((ld_vv1 - pOut->Ve) / pIn->Jerk);
				pOut->T6 = 0;
				pOut->T7 = pOut->T5;
			
				Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

				ld_ss1 = ld_sacc + ld_sdec;

				if(fabs(ld_ss1 - pOut->Len) < RT_LITTLE) // 有匀加速段
				{
					// vv1即为所求实际期望速度
					pOut->V = ld_vv1;
				}
				else if(ld_ss1 < pOut->Len) // 有匀加速段和匀减速段
				{
					// 实际期望速度
					ld_vv1 = pOut->V = 0.5 * (- pIn->Acc * pIn->Acc + sqrt(pIn->Acc * pIn->Acc * pIn->Acc * pIn->Acc - 
						2 * pIn->Jerk * (pIn->Acc * pIn->Acc * (pOut->Vs + pOut->Ve) - pIn->Jerk * (pOut->Vs * pOut->Vs + pOut->Ve * pOut->Ve) - 2 * pIn->Acc * pIn->Jerk * pOut->Len)))/ pIn->Jerk;

					// 重新规划时间
					pOut->T1 = pIn->Acc / pIn->Jerk;
					pOut->T2 = (ld_vv1 - pOut->Vs) / pIn->Acc - pOut->T1;
					pOut->T3 = pOut->T1;
					pOut->T5 = pIn->Acc / pIn->Jerk;
					pOut->T6 = (ld_vv1 - pOut->Ve) / pIn->Acc - pOut->T5;
					pOut->T7 = pOut->T5;
				}
				else // 是否有匀加速段?
				{
					// 假设 - 无匀加速段，整个过程只有4段
					ld_vv2 = pOut->Vs + pIn->Acc * pIn->Acc / pIn->Jerk; // min(out->Vs,out->Ve)

					if(ld_vv2 < pOut->Ve)
					{
						ld_vv2 = pOut->Ve; // 有匀加速段,期望速度介于vv1和vv2之间

						while(fabs(ld_ss1 - pOut->Len) > SERR)
						{
							// 初值
							ld_vv0 = (ld_vv1 + ld_vv2) / 2;							

							// 求加速段和减速段的距离
							pOut->T1 = pIn->Acc / pIn->Jerk;
							pOut->T2 = (ld_vv0 - pOut->Vs) / pIn->Acc - pOut->T1;
							pOut->T3 = pOut->T1;
							pOut->T5 = sqrt((ld_vv0 - pOut->Ve) / pIn->Jerk);
							pOut->T6 = 0;
							pOut->T7 = pOut->T5;			
				
							Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

							ld_ss1 = ld_sacc + ld_sdec;
	
							if(ld_ss1 > pOut->Len) // 减小期望速度
							{
								ld_vv1 = ld_vv0;
							}
							else  // 增加期望速度
							{
								ld_vv2 = ld_vv0;
							}
						}// end of while	
					}
					else
					{
						// 求加速段和减速段的距离
						pOut->T1 = sqrt((ld_vv2 - pOut->Vs) / pIn->Jerk);
						pOut->T2 = 0;
						pOut->T3 = pOut->T1;
						pOut->T5 = sqrt((ld_vv2 - pOut->Ve) / pIn->Jerk);
						pOut->T6 = 0;
						pOut->T7 = pOut->T5;			

						Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

						ld_ss1 = ld_sacc + ld_sdec;

						if(fabs(ld_ss1 - pOut->Len) < RT_LITTLE) 
						{
							// vv2即为所求实际期望速度
							ld_vv0 = ld_vv2;
						}
						else if(ld_ss1 < pOut->Len) // 有匀加速段 重新求解期望速度,介于vv1与vv2之间
						{
							while(fabs(ld_ss1 - pOut->Len) > SERR)
							{
								// 初值
								ld_vv0 = (ld_vv1 + ld_vv2) / 2;							

								// 求加速段和减速段的距离
								pOut->T1 = pIn->Acc / pIn->Jerk;
								pOut->T2 = (ld_vv0 - pOut->Vs) / pIn->Acc - pOut->T1;
								pOut->T3 = pOut->T1;
								pOut->T5 = sqrt((ld_vv0 - pOut->Ve) / pIn->Jerk);
								pOut->T6 = 0;
								pOut->T7 = pOut->T5;			

								Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

								ld_ss1 = ld_sacc + ld_sdec;
		
								if(ld_ss1 > pOut->Len) // 减小期望速度
								{
									ld_vv1 = ld_vv0;
								}
								else  // 增加期望速度
								{
									ld_vv2 = ld_vv0;
								}
							}// end of while						
						}
						else // 无匀加速段 期望速度介于out->Vs与vv2之间
						{
							// 初值
							// 期望速度的初值不能小于Vs和Ve
							// 期望速度小于Vs或Ve的情况在预处理部分已经处理
							if( (ld_vv2 + pOut->Vs) / 2 < pOut->Ve)
							{
								ld_vv1 = ld_vv2;     // 大值
								ld_vv2 = pOut->Ve; // 小值
							}
							else
							{
								ld_vv1 = ld_vv2;     // 大值
								ld_vv2 = pOut->Vs; // 小值
							}

							while(fabs(ld_ss1 - pOut->Len) > SERR)
							{
								ld_vv0 = (ld_vv1 + ld_vv2) / 2;

								// 求加速段和减速段的距离
								pOut->T1 = sqrt((ld_vv0 - pOut->Vs) / pIn->Jerk);
								pOut->T2 = 0;
								pOut->T3 = pOut->T1;
								pOut->T5 = sqrt((ld_vv0 - pOut->Ve) / pIn->Jerk);
								pOut->T6 = 0;
								pOut->T7 = pOut->T5;			
				
								Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

								ld_ss1 = ld_sacc + ld_sdec;

								if(ld_ss1 > pOut->Len) // 减小期望速度
								{
									ld_vv1 = ld_vv0;
								}
								else  // 增加期望速度
								{
									ld_vv2 = ld_vv0;
								}
							}// end of while
						}
					}
					pOut->V = ld_vv0;
				}
			} // end of out->Ve较大		
			else // out->Vs较大
			{
				// 假设实际最大速度 - 减速段包含加减速段、匀减速段和减减速段
				// 还有加加速段、减加速段，共5段
				ld_vv1 = pOut->Vs + pIn->Acc * pIn->Acc / pIn->Jerk; // max(out->Vs,out->Ve)

				// 求加速段和减速段的距离
				pOut->T1 = sqrt((ld_vv1 - pOut->Vs) / pIn->Jerk);
				pOut->T2 = 0;
				pOut->T3 = pOut->T1;
				pOut->T5 = pIn->Acc / pIn->Jerk;
				pOut->T6 = (ld_vv1 - pOut->Ve) / pIn->Acc - pOut->T5;
				pOut->T7 = pOut->T5;
			
				Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

				ld_ss1 = ld_sacc + ld_sdec;

				if(fabs(ld_ss1 - pOut->Len) < RT_LITTLE) // 有匀减速段
				{
					// vv1即为所求实际期望速度
					pOut->V = ld_vv1;
				}
				else if(ld_ss1 < pOut->Len) // 有匀加速段和匀减速段
				{
					// 实际期望速度
					ld_vv1 = pOut->V = 0.5 * (- pIn->Acc * pIn->Acc + sqrt(pIn->Acc * pIn->Acc * pIn->Acc * pIn->Acc - 
						2 * pIn->Jerk * (pIn->Acc * pIn->Acc * (pOut->Vs + pOut->Ve) - pIn->Jerk * (pOut->Vs * pOut->Vs + pOut->Ve * pOut->Ve) - 2 * pIn->Acc * pIn->Jerk * pOut->Len)))/ pIn->Jerk;

					// 重新规划时间
					pOut->T1 = pIn->Acc / pIn->Jerk;
					pOut->T2 = (ld_vv1 - pOut->Vs) / pIn->Acc - pOut->T1;
					pOut->T3 = pOut->T1;
					pOut->T5 = pIn->Acc / pIn->Jerk;
					pOut->T6 = (ld_vv1 - pOut->Ve) / pIn->Acc - pOut->T5;
					pOut->T7 = pOut->T5;
				}
				else // 是否有匀减速段?
				{
					// 假设 - 无匀减速段，整个过程只有4段
					ld_vv2 = pOut->Ve + pIn->Acc * pIn->Acc / pIn->Jerk; // min(out->Vs,out->Ve)

					if(ld_vv2 < pOut->Vs)
					{
						ld_vv2 = pOut->Vs; // 有匀减速段,期望速度介于vv1和vv2之间

						while(fabs(ld_ss1 - pOut->Len) > SERR)
						{
							// 初值
 							ld_vv0 = (ld_vv1 + ld_vv2) / 2;					

							// 求加速段和减速段的距离
							pOut->T1 = sqrt((ld_vv0 - pOut->Vs) / pIn->Jerk);
							pOut->T2 = 0;
							pOut->T3 = pOut->T1;
							pOut->T5 = pIn->Acc / pIn->Jerk;
							pOut->T6 = (ld_vv0 - pOut->Ve) / pIn->Acc - pOut->T5;
							pOut->T7 = pOut->T5;

							Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

							ld_ss1 = ld_sacc + ld_sdec;
	
							if(ld_ss1 > pOut->Len) // 减小期望速度
							{
								ld_vv1 = ld_vv0;
							}
							else  // 增加期望速度
							{
								ld_vv2 = ld_vv0;
							}
						}// end of while
					}
					else
					{
						// 求加速段和减速段的距离
						pOut->T1 = sqrt((ld_vv2 - pOut->Vs) / pIn->Jerk);
						pOut->T2 = 0;
						pOut->T3 = pOut->T1;
						pOut->T5 = sqrt((ld_vv2 - pOut->Ve) / pIn->Jerk);
						pOut->T6 = 0;
						pOut->T7 = pOut->T5;	

						Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

						ld_ss1 = ld_sacc + ld_sdec;
	
						if(fabs(ld_ss1 - pOut->Len) < RT_LITTLE)
						{
							// vv2即为所求实际期望速度
							ld_vv0 = ld_vv2;
						}
						else if(ld_ss1 < pOut->Len) // 有匀减速段 重新求解期望速度,介于vv1与vv2之间
						{
							while(fabs(ld_ss1 - pOut->Len) > SERR)
							{
								// 初值
 								ld_vv0 = (ld_vv1 + ld_vv2) / 2;					

								// 求加速段和减速段的距离
								pOut->T1 = sqrt((ld_vv0 - pOut->Vs) / pIn->Jerk);
								pOut->T2 = 0;
								pOut->T3 = pOut->T1;
								pOut->T5 = pIn->Acc / pIn->Jerk;
								pOut->T6 = (ld_vv0 - pOut->Ve) / pIn->Acc - pOut->T5;
								pOut->T7 = pOut->T5;

								Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

								ld_ss1 = ld_sacc + ld_sdec;
	
								if(ld_ss1 > pOut->Len) // 减小期望速度
								{
									ld_vv1 = ld_vv0;
								}
								else  // 增加期望速度
								{
									ld_vv2 = ld_vv0;
								}
							}// end of while
						}
						else // 无匀减速段 期望速度介于out->Ve与vv2之间
						{
							// 初值
							// 期望速度的初值不能小于Vs和Ve
							if( (ld_vv2 + pOut->Ve) / 2 < pOut->Vs)
							{
								ld_vv1 = ld_vv2;     // 大值
								ld_vv2 = pOut->Vs; // 小值
							}
							else
							{
								ld_vv1 = ld_vv2;     // 大值
								ld_vv2 = pOut->Ve; // 小值
							}

							while(fabs(ld_ss1 - pOut->Len) > SERR)
							{
								ld_vv0 = (ld_vv1 + ld_vv2) / 2;

								// 求加速段和减速段的距离
								pOut->T1 = sqrt((ld_vv0 - pOut->Vs) / pIn->Jerk);
								pOut->T2 = 0;
								pOut->T3 = pOut->T1;
								pOut->T5 = sqrt((ld_vv0 - pOut->Ve) / pIn->Jerk);
								pOut->T6 = 0;
								pOut->T7 = pOut->T5;			
				
								Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
	
								ld_ss1 = ld_sacc + ld_sdec;
	
								if(ld_ss1 > pOut->Len) // 减小期望速度
								{
									ld_vv1 = ld_vv0;
								}
								else  // 增加期望速度
								{
									ld_vv2 = ld_vv0;
								}
							}// end of while
						}
					}
					pOut->V = ld_vv0;
				}
			}// end of out->Vs较大
		}// end of 不含匀速段
	}// end of (V > Vs) && (V > Ve)
	//------------------------ (V < Vs) && (V < Ve) ------------------------//
	else if( (pOut->V < pOut->Vs) && (pOut->V < pOut->Ve) )
	{
		// 先减速,后加速
		pOut->Jacc = - pIn->Jerk;
		pOut->Jdec = pIn->Jerk;

		// 减速段,存在匀减速段?
		if(pOut->Vs - pOut->V > pIn->Acc * pIn->Acc / pIn->Jerk)
		{
			pOut->T1 = pIn->Acc / pIn->Jerk;
			pOut->T2 = (pOut->Vs - pOut->V) / pIn->Acc - pOut->T1;
			pOut->T3 = pOut->T1;
		}
		// 无匀减速段
		else
		{
			pOut->T1 = sqrt((pOut->Vs - pOut->V) / pIn->Jerk);
			pOut->T2 = 0;
			pOut->T3 = pOut->T1;
		}

		// 加速段,存在匀加速段?
		if(pOut->Ve - pOut->V > pIn->Acc * pIn->Acc / pIn->Jerk)
		{
			pOut->T5 = pIn->Acc / pIn->Jerk;
			pOut->T6 = (pOut->Ve - pOut->V) / pIn->Acc - pOut->T5;
			pOut->T7 = pOut->T5;
		}
		// 无匀加速段
		else
		{
			pOut->T5 = sqrt((pOut->Ve - pOut->V) / pIn->Jerk);
			pOut->T6 = 0;
			pOut->T7 = pOut->T5;
		}

		Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

		ld_ss1 = ld_sacc + ld_sdec;

		// 含有匀速段?
		if(ld_ss1 < pOut->Len)
		{
			pOut->T4 = (pOut->Len - ld_ss1) / pOut->V;
		}
		else // 不含匀速段,应增大期望速度V
		{
			pOut->T4 = 0;

			// Vs较大
			if(pOut->Ve <= pOut->Vs)
			{
				// 假设实际速度
				// 变速段包括 加减速、匀减速、减减速、加加速、减加速 共5段
				// 若此假设速度小于V,则值V
				ld_vv1 = pOut->Ve - pIn->Acc * pIn->Acc / pIn->Jerk;

				if(ld_vv1 > pOut->V)
				{
					// 求变速段距离
					pOut->T1 = pIn->Acc / pIn->Jerk;
					pOut->T2 = (pOut->Vs - ld_vv1) / pIn->Acc - pOut->T1;
					pOut->T3 = pOut->T1;
					pOut->T5 = sqrt((pOut->Ve - ld_vv1) / pIn->Jerk);
					pOut->T6 = 0;
					pOut->T7 = pOut->T5;

					Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

					ld_ss1 = ld_sacc + ld_sdec;
				}
				else
				{
					ld_vv1 = pOut->V;
				}

				if(fabs(ld_ss1 - pOut->Len) < RT_LITTLE) // 有匀速段
				{
					pOut->V = ld_vv1;
				}
				else if(ld_ss1 < pOut->Len) // 有匀加速段和匀减速段
				{
					// 实际期望速度
					ld_vv1 = pOut->V = 0.5 * (pIn->Acc * pIn->Acc + sqrt(pIn->Acc * pIn->Acc * pIn->Acc * pIn->Acc + 
						2 * pIn->Jerk * (pIn->Acc * pIn->Acc * (pOut->Vs + pOut->Ve) + pIn->Jerk * (pOut->Vs * pOut->Vs + pOut->Ve * pOut->Ve) - 2 * pIn->Acc * pIn->Jerk * pOut->Len)))/ pIn->Jerk;
					
					// 重新规划时间
					pOut->T1 = pIn->Acc / pIn->Jerk;
					pOut->T2 = (pOut->Vs - ld_vv1) / pIn->Acc - pOut->T1;
					pOut->T3 = pOut->T1;
					pOut->T5 = pIn->Acc / pIn->Jerk;
					pOut->T6 = (pOut->Ve - ld_vv1) / pIn->Acc - pOut->T5;
					pOut->T7 = pOut->T5;
				}
				else
				{
					// 假设 - 只有加减速段、减减速段、加加速段、减加速段
					ld_vv2 = pOut->Vs - pIn->Acc * pIn->Acc / pIn->Jerk;

					if(ld_vv2 > pOut->Ve)
					{
						ld_vv2 = pOut->Ve; // 调小,有匀减速段,期望速度介于vv2和vv1之间

						while(fabs(ld_ss1 - pOut->Len) > SERR)
						{
							// 初值
							ld_vv0 = (ld_vv1 + ld_vv2) / 2;

							// 求变速段距离
							pOut->T1 = pIn->Acc / pIn->Jerk;
							pOut->T2 = (pOut->Vs - ld_vv0) / pIn->Acc - pOut->T1;
							pOut->T3 = pOut->T1;
							pOut->T5 = sqrt((pOut->Ve - ld_vv0) / pIn->Jerk);
							pOut->T6 = 0;
							pOut->T7 = pOut->T5;

							Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
						
							ld_ss1 = ld_sacc + ld_sdec;
	
							if(ld_ss1 > pOut->Len) // 增加期望速度
							{
								ld_vv1 = ld_vv0;
							}
							else // 减小期望速度
							{
								ld_vv2 = ld_vv0;
							}
						}// end of while
					}
					else
					{
						if(ld_vv2 < pOut->V)
						{
							ld_vv2 = pOut->V; // 调大,无匀减速段
						}

						// 求变速段距离
						pOut->T1 = sqrt((pOut->Vs - ld_vv2) / pIn->Jerk);
						pOut->T2 = 0;
						pOut->T3 = pOut->T1;
						pOut->T5 = sqrt((pOut->Ve - ld_vv2) / pIn->Jerk);
						pOut->T6 = 0;
						pOut->T7 = pOut->T5;

						Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

						ld_ss1 = ld_sacc + ld_sdec;

						if(fabs(ld_ss1 - pOut->Len) < RT_LITTLE)
						{
							// vv2即为所求实际期望速度
							ld_vv0 = ld_vv2;
						}
						else if(ld_ss1 < pOut->Len) // 有匀减速段,期望速度介于vv1和vv2之间
						{
							while(fabs(ld_ss1 - pOut->Len) > SERR)
							{
								// 初值
								ld_vv0 = (ld_vv1 + ld_vv2) / 2;

								// 求变速段距离
								pOut->T1 = pIn->Acc / pIn->Jerk;
								pOut->T2 = (pOut->Vs - ld_vv0) / pIn->Acc - pOut->T1;
								pOut->T3 = pOut->T1;
								pOut->T5 = sqrt((pOut->Ve - ld_vv0) / pIn->Jerk);
								pOut->T6 = 0;
								pOut->T7 = pOut->T5;

								Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
						
								ld_ss1 = ld_sacc + ld_sdec;
	
								if(ld_ss1 > pOut->Len) // 增加期望速度
								{
									ld_vv1 = ld_vv0;
								}
								else // 减小期望速度
								{
									ld_vv2 = ld_vv0;
								}
							}// end of while
						}
						else // 无匀减速段,期望速度介于Ve与vv2之间
						{
							ld_vv1 = ld_vv2;
							ld_vv2 = pOut->Ve;
	
							while(fabs(ld_ss1 - pOut->Len) > SERR)
							{
								ld_vv0 = (ld_vv1 + ld_vv2) / 2;
	
								// 求变速段距离
								pOut->T1 = sqrt((pOut->Vs - ld_vv0) / pIn->Jerk);
								pOut->T2 = 0;
								pOut->T3 = pOut->T1;
								pOut->T5 = sqrt((pOut->Ve - ld_vv0) / pIn->Jerk);
								pOut->T6 = 0;
								pOut->T7 = pOut->T5;

								Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
						
								ld_ss1 = ld_sacc + ld_sdec;
	
								if(ld_ss1 > pOut->Len) // 增加期望速度
								{
									ld_vv1 = ld_vv0;
								}
								else // 减小期望速度
								{
									ld_vv2 = ld_vv0;
								}
							}// end of while
						}
					}
					pOut->V = ld_vv0;
				}
			}// end of Vs较大
			else if(pOut->Ve > pOut->Vs)// Ve较大
			{
				// 假设包含 加减速、减减速、加加速、匀加速、减加速 5段
				// 若此假设速度小于V,则值V
				ld_vv1 = pOut->Vs - pIn->Acc * pIn->Acc / pIn->Jerk;

				if(ld_vv1 > pOut->V)
				{
					// 求变速段距离
					pOut->T1 = sqrt((pOut->Vs - ld_vv1) / pIn->Jerk);
					pOut->T2 = 0;
					pOut->T3 = pOut->T1;
					pOut->T5 = pIn->Acc / pIn->Jerk;
					pOut->T6 = (pOut->Ve - ld_vv1) / pIn->Acc - pOut->T5;
					pOut->T7 = pOut->T5;

					Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
				
					ld_ss1 = ld_sacc + ld_sdec;
				}
				else
				{
					ld_vv1 = pOut->V; // 期望速度不小于vv1
				}

				if(fabs(ld_ss1 - pOut->Len) < RT_LITTLE)
				{
					//vv1即为所求实际期望速度
					pOut->V = ld_vv1;
				}
				else if(ld_ss1 < pOut->Len)  // 有匀减速段和匀加速段
				{
					// 实际期望速度
					ld_vv1 = pOut->V = 0.5 * (pIn->Acc * pIn->Acc + sqrt(pIn->Acc * pIn->Acc * pIn->Acc * pIn->Acc + 
						2 * pIn->Jerk * (pIn->Acc * pIn->Acc * (pOut->Vs + pOut->Ve) + pIn->Jerk * (pOut->Vs * pOut->Vs + pOut->Ve * pOut->Ve) - 2 * pIn->Acc * pIn->Jerk * pOut->Len)))/ pIn->Jerk;
					
					// 重新规划时间
					pOut->T1 = pIn->Acc / pIn->Jerk;
					pOut->T2 = (pOut->Vs - ld_vv1) / pIn->Acc - pOut->T1;
					pOut->T3 = pOut->T1;
					pOut->T5 = pIn->Acc / pIn->Jerk;
					pOut->T6 = (pOut->Ve - ld_vv1) / pIn->Acc - pOut->T5;
					pOut->T7 = pOut->T5;
				}
				else // 无匀减速段
				{
					// 假设只有 加减速、减减速、加加速、减加速 4段
					ld_vv2 = pOut->Ve - pIn->Acc * pIn->Acc / pIn->Jerk;

					if(ld_vv2 > pOut->Vs)
					{
						ld_vv2 = pOut->Vs; // 调小,有匀加速段,速度介于vv2与vv1之间

						while(fabs(ld_ss1 - pOut->Len) > SERR)
						{
							// 初值
							ld_vv0 = (ld_vv1 + ld_vv2) / 2;

							// 求变速段距离
							pOut->T1 = sqrt((pOut->Vs - ld_vv0) / pIn->Jerk);
							pOut->T2 = 0;
							pOut->T3 = pOut->T1;
							pOut->T5 = pIn->Acc / pIn->Jerk;
							pOut->T6 = (pOut->Ve - ld_vv0) / pIn->Acc - pOut->T5;
							pOut->T7 = pOut->T5;

							Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
						
							ld_ss1 = ld_sacc + ld_sdec;
	
							if(ld_ss1 > pOut->Len) // 增加期望速度
							{
								ld_vv1 = ld_vv0;
							}
							else // 减小期望速度
							{
								ld_vv2 = ld_vv0;
							}							
						}// end of while
					}
					else
					{
						if(ld_vv2 < pOut->V)
						{
							ld_vv2 = pOut->V;  // 调大,无匀加速段
						}

						// 求变速段距离
						pOut->T1 = sqrt((pOut->Vs - ld_vv2) / pIn->Jerk);
						pOut->T2 = 0;
						pOut->T3 = pOut->T1;
						pOut->T5 = sqrt((pOut->Ve - ld_vv2) / pIn->Jerk);
						pOut->T6 = 0;
						pOut->T7 = pOut->T5;

						Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);

						ld_ss1 = ld_sacc + ld_sdec;

						if(fabs(ld_ss1 - pOut->Len) < RT_LITTLE)
						{
							// vv2即为所求实际期望速段
							ld_vv0 = ld_vv2;
						}
						else if(ld_ss1 < pOut->Len) // 有匀加速段,期望速度介于vv1与vv2之间
						{
							while(fabs(ld_ss1 - pOut->Len) > SERR)
							{
								// 初值
								ld_vv0 = (ld_vv1 + ld_vv2) / 2;

								// 求变速段距离
								pOut->T1 = sqrt((pOut->Vs - ld_vv0) / pIn->Jerk);
								pOut->T2 = 0;
								pOut->T3 = pOut->T1;
								pOut->T5 = pIn->Acc / pIn->Jerk;
								pOut->T6 = (pOut->Ve - ld_vv0) / pIn->Acc - pOut->T5;
								pOut->T7 = pOut->T5;

								Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
						
								ld_ss1 = ld_sacc + ld_sdec;
	
								if(ld_ss1 > pOut->Len) // 增加期望速度
								{
									ld_vv1 = ld_vv0;
								}
								else // 减小期望速度
								{
									ld_vv2 = ld_vv0;
								}							
							}// end of while
						}
						else // 无匀加速段,期望速段介于Vs与vv2之间
						{
							// 初值
							ld_vv1 = ld_vv2;
							ld_vv2 = pOut->Vs;
	
							while(fabs(ld_ss1 - pOut->Len) > SERR)
							{
								ld_vv0 = (ld_vv1 + ld_vv2) / 2;

								// 求变速段距离
								pOut->T1 = sqrt((pOut->Vs - ld_vv0) / pIn->Jerk);
								pOut->T2 = 0;
								pOut->T3 = pOut->T1;
								pOut->T5 = sqrt((pOut->Ve - ld_vv0) / pIn->Jerk);
								pOut->T6 = 0;
								pOut->T7 = pOut->T5;

								Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
						
								ld_ss1 = ld_sacc + ld_sdec;
	
								if(ld_ss1 > pOut->Len) // 增加期望速度
								{
									ld_vv1 = ld_vv0;
								}
								else
								{
									ld_vv2 = ld_vv0;
								}
							}// end of while
						}
					}
					pOut->V = ld_vv0;
				}//end of 无匀减速段
			}// end of Ve较大	
		}// end of 不含匀速段
	}// end of (V < Vs) && (V < Ve)
	//-------------------------- V介于Vs和Ve之间 ---------------------------//
	else if( ((pOut->V >= pOut->Vs) && (pOut->V <= pOut->Ve)) ||
			 ((pOut->V <= pOut->Vs) && (pOut->V >= pOut->Ve)) )
	{
		// Vs较大
		if(pOut->Vs >= pOut->Ve)
		{
			pOut->Jacc = - pIn->Jerk;
			pOut->Jdec = - pIn->Jerk;

			// 求变速段距离
			if(pOut->Vs - pOut->V > pIn->Acc * pIn->Acc / pIn->Jerk)
			{
				pOut->T1 = pIn->Acc / pIn->Jerk;
				pOut->T2 = (pOut->Vs - pOut->V) / pIn->Acc - pOut->T1;
				pOut->T3 = pOut->T1;
			}
			else
			{
				pOut->T1 = sqrt((pOut->Vs - pOut->V) / pIn->Jerk);
				pOut->T2 = 0;
				pOut->T3 = pOut->T1;
			}

			if(pOut->V - pOut->Ve > pIn->Acc * pIn->Acc / pIn->Jerk)
			{
				pOut->T5 = pIn->Acc / pIn->Jerk;
				pOut->T6 = (pOut->V - pOut->Ve) / pIn->Acc - pOut->T5;
				pOut->T7 = pOut->T5;
			}
			else
			{
				pOut->T5 = sqrt((pOut->V - pOut->Ve) / pIn->Jerk);
				pOut->T6 = 0;
				pOut->T7 = pOut->T5;
			}

			Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
						
			ld_ss1 = ld_sacc + ld_sdec;

			// 含有匀速段
			if(ld_ss1 <= pOut->Len)
			{
				pOut->T4 = (pOut->Len - ld_ss1) / pOut->V;
			}
			else // 不含匀速段,期望速度介于Ve与V之间
			{
				pOut->T4 = 0;

				// 初值
				ld_vv1 = pOut->V;  // 大值
				ld_vv2 = pOut->Ve; // 小值

				while(fabs(ld_ss1 - pOut->Len) > SERR)
				{
					ld_vv0 = (ld_vv1 + ld_vv2) / 2;

					// 求变速段距离
					if(pOut->Vs - ld_vv0 > pIn->Acc * pIn->Acc / pIn->Jerk)
					{
						pOut->T1 = pIn->Acc / pIn->Jerk;
						pOut->T2 = (pOut->Vs - ld_vv0) / pIn->Acc - pOut->T1;
						pOut->T3 = pOut->T1;
					}
					else
					{
						pOut->T1 = sqrt((pOut->Vs - ld_vv0) / pIn->Jerk);
						pOut->T2 = 0;
						pOut->T3 = pOut->T1;
					}
	
					if(ld_vv0 - pOut->Ve > pIn->Acc * pIn->Acc / pIn->Jerk)
					{
						pOut->T5 = pIn->Acc / pIn->Jerk;
						pOut->T6 = (ld_vv0 - pOut->Ve) / pIn->Acc - pOut->T5;
						pOut->T7 = pOut->T5;
					}
					else
					{
						pOut->T5 = sqrt((ld_vv0 - pOut->Ve) / pIn->Jerk);
						pOut->T6 = 0;
						pOut->T7 = pOut->T5;
					}

					Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
						
					ld_ss1 = ld_sacc + ld_sdec;

					if(ld_ss1 > pOut->Len) // 减小期望速度
					{
						ld_vv1 = ld_vv0;
					}
					else // 增加期望速度
					{
						ld_vv2 = ld_vv0; 
					}
				}// end of while

				pOut->V = ld_vv0;
			}
		}// end of Vs较大
		else if(pOut->Vs < pOut->Ve)
		{
			pOut->Jacc = pIn->Jerk;
			pOut->Jdec = pIn->Jerk;

			// 求变速段距离
			if(pOut->V - pOut->Vs > pIn->Acc * pIn->Acc / pIn->Jerk)
			{
				pOut->T1 = pIn->Acc / pIn->Jerk;
				pOut->T2 = (pOut->V - pOut->Vs) / pIn->Acc - pOut->T1;
				pOut->T3 = pOut->T1;
			}
			else
			{
				pOut->T1 = sqrt((pOut->V - pOut->Vs) / pIn->Jerk);
				pOut->T2 = 0;
				pOut->T3 = pOut->T1;
			}

			if(pOut->Ve - pOut->V > pIn->Acc * pIn->Acc / pIn->Jerk)
			{
				pOut->T5 = pIn->Acc / pIn->Jerk;
				pOut->T6 = (pOut->Ve - pOut->V) / pIn->Acc - pOut->T5;
				pOut->T7 = pOut->T5;
			}
			else
			{
				pOut->T5 = sqrt((pOut->Ve - pOut->V) / pIn->Jerk);
				pOut->T6 = 0;
				pOut->T7 = pOut->T5;
			}

			Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
						
			ld_ss1 = ld_sacc + ld_sdec;

			// 含有匀速段
			if(ld_ss1 <= pOut->Len)
			{
				pOut->T4 = (pOut->Len - ld_ss1) / pOut->V;
			}
			else // 不含匀速段,期望速度介于Vs与V之间
			{
				pOut->T4 = 0;

				// 初值
				ld_vv1 = pOut->V;  // 大值
				ld_vv2 = pOut->Vs; // 小值

				while(fabs(ld_ss1 - pOut->Len) > SERR)
				{
					ld_vv0 = (ld_vv1 + ld_vv2) / 2;

					// 求变速段距离
					if(ld_vv0 - pOut->Vs > pIn->Acc * pIn->Acc / pIn->Jerk)
					{
						pOut->T1 = pIn->Acc / pIn->Jerk;
						pOut->T2 = (ld_vv0 - pOut->Vs) / pIn->Acc - pOut->T1;
						pOut->T3 = pOut->T1;
					}
					else
					{
						pOut->T1 = sqrt((ld_vv0 - pOut->Vs) / pIn->Jerk);
						pOut->T2 = 0;
						pOut->T3 = pOut->T1;
					}
	
					if(pOut->Ve - ld_vv0 > pIn->Acc * pIn->Acc / pIn->Jerk)
					{
						pOut->T5 = pIn->Acc / pIn->Jerk;
						pOut->T6 = (pOut->Ve - ld_vv0) / pIn->Acc - pOut->T5;
						pOut->T7 = pOut->T5;
					}
					else
					{
						pOut->T5 = sqrt((pOut->Ve - ld_vv0) / pIn->Jerk);
						pOut->T6 = 0;
						pOut->T7 = pOut->T5;
					}

					Get_AccDecLen(pOut, &ld_sacc, &ld_sdec);
						
					ld_ss1 = ld_sacc + ld_sdec;

					if(ld_ss1 > pOut->Len) // 减小期望速度
					{
						ld_vv1 = ld_vv0;
					}
					else // 增加期望速度
					{
						ld_vv2 = ld_vv0; 
					}
				}// end of while

				pOut->V = ld_vv0;
			}
		}// end of Ve较大
	}

	// 总时间
	pOut->T = pOut->T1 + pOut->T2 + pOut->T3 + pOut->T4 + pOut->T5 + pOut->T6 + pOut->T7;

	// 各段速度
	pOut->V1 = pOut->Vs + 0.5 * pOut->Jacc * pOut->T1 * pOut->T1;
	pOut->V2 = pOut->V1 + pOut->Jacc * pOut->T1 * pOut->T2;
	pOut->V3 = pOut->V2 + 0.5 * pOut->Jacc * pOut->T1 * pOut->T1;

	pOut->V4 = pOut->V3;
	pOut->V5 = pOut->V4 + 0.5 * pOut->Jdec * pOut->T5 * pOut->T5;
	pOut->V6 = pOut->V5 + pOut->Jdec * pOut->T5 * pOut->T6;

	// 各段路径
	pOut->L1 = (pOut->Vs + pOut->Jacc * pOut->T1 * pOut->T1 / 6) * pOut->T1;
	pOut->L2 = pOut->L1 + (pOut->V1 + 0.5 * pOut->Jacc * pOut->T1 * pOut->T2) * pOut->T2;
	pOut->L3 = pOut->L2 + (pOut->V2 + pOut->Jacc * pOut->T1 * pOut->T1 / 3) * pOut->T1;

	pOut->L4 = pOut->L3 + pOut->V3 * pOut->T4;
	pOut->L5 = pOut->L4 + (pOut->V4 + pOut->Jdec * pOut->T5 * pOut->T5 / 6) * pOut->T5;
	pOut->L6 = pOut->L5 + (pOut->V5 + 0.5 * pOut->Jdec * pOut->T5 * pOut->T6) * pOut->T6;

	pOut->L7 = pOut->L6 + (pOut->V6 + pOut->Jdec * pOut->T5 * pOut->T5 / 3) * pOut->T5;

	return Ok;
}

/*****************************************************************************
 * 函数：Get_SpeedProcessData()
 * 功能：获取速度规划后的轨迹点数据
 *
 * 输入：SpdPTrajData * in - 速度规划后的轨迹
 *       double t          - 时间
 *
 * 输出：TrajPointPara* out - 轨迹点数据
 *****************************************************************************/
void SpeedProcess::Get_SpeedProcessData(SpdPTrajData* pIn, double dTime, TrajPointPara* pOut)
{
	// 由时间来计算即时加速度、速度、位移
	// 加减速段由Jacc和Jdec加加速度的正负来判断
	double ld_tx;  // 时间片变量
	double ld_temp = pIn->T1 + pIn->T2 + pIn->T3 + pIn->T4 + pIn->T5 + pIn->T6 + pIn->T7;

	// [0,t1)
	if( (dTime >= 0) && (dTime < pIn->T1) )
	{
		ld_tx = dTime;

		pOut->Acc = pIn->Jacc * ld_tx;
		pOut->Vel = pIn->Vs + 0.5 * pIn->Jacc * ld_tx * ld_tx;
		pOut->Len = (pIn->Vs + pIn->Jacc * ld_tx * ld_tx / 6) * ld_tx;
	}
	// [t1,t2)
	else if( dTime < (pIn->T1 + pIn->T2))
	{
		ld_tx = dTime - pIn->T1;

		pOut->Acc = pIn->Jacc * pIn->T1;
		pOut->Vel = pIn->V1 + pIn->Jacc * pIn->T1 * ld_tx;
		pOut->Len = pIn->L1 + (pIn->V1 + 0.5 * pIn->Jacc * pIn->T1 * ld_tx) * ld_tx;
	}
	// [t2,t3)
	else if(dTime < (pIn->T1 + pIn->T2 + pIn->T3))
	{
		ld_tx = dTime - (pIn->T1 + pIn->T2);

		pOut->Acc = pIn->Jacc * (pIn->T1 - ld_tx);
		pOut->Vel = pIn->V2 + (pIn->T1 - 0.5 * ld_tx) * pIn->Jacc * ld_tx; 
		pOut->Len = pIn->L2 + (pIn->V2 + (0.5 * pIn->T1 - ld_tx / 6) * pIn->Jacc * ld_tx) * ld_tx;
	}
	// [t3,t4)
	else if(dTime < (pIn->T1 + pIn->T2 + pIn->T3 + pIn->T4))
	{
		ld_tx = dTime - (pIn->T1 + pIn->T2 + pIn->T3);

		pOut->Acc = 0;
		pOut->Vel = pIn->V3;
		pOut->Len = pIn->L3 + pIn->V3 * ld_tx;

	}
	// [t4,t5)
	else if(dTime < (pIn->T1 + pIn->T2 + pIn->T3 + pIn->T4 + pIn->T5))
	{
		ld_tx = dTime - (pIn->T1 + pIn->T2 + pIn->T3 + pIn->T4);

		pOut->Acc = pIn->Jdec * ld_tx;
		pOut->Vel = pIn->V4 + 0.5 * pIn->Jdec * ld_tx * ld_tx;
		pOut->Len = pIn->L4 + (pIn->V4 + pIn->Jdec * ld_tx * ld_tx / 6) * ld_tx;
	}
	// [t5,t6)
	else if(dTime < (pIn->T1 + pIn->T2 + pIn->T3 + pIn->T4 + pIn->T5 + pIn->T6))
	{
		ld_tx = dTime - (pIn->T1 + pIn->T2 + pIn->T3 + pIn->T4 + pIn->T5);

		pOut->Acc = pIn->Jdec * pIn->T5;
		pOut->Vel = pIn->V5 + pIn->Jdec * pIn->T5 * ld_tx;
		pOut->Len = pIn->L5 + (pIn->V5 + 0.5 * pIn->Jdec * pIn->T5 * ld_tx) * ld_tx;
	}
	// [t6,t7]
	else if(dTime <= (pIn->T1 + pIn->T2 + pIn->T3 + pIn->T4 + pIn->T5 + pIn->T6 + pIn->T7))
	{
		ld_tx = dTime - (pIn->T1 + pIn->T2 + pIn->T3 + pIn->T4 + pIn->T5 + pIn->T6);

		pOut->Acc = pIn->Jdec * (pIn->T5 - ld_tx);
		pOut->Vel = pIn->V6 + (pIn->T5 - 0.5 * ld_tx) * pIn->Jdec * ld_tx;
		pOut->Len = pIn->L6 + (pIn->V6 + (0.5 * pIn->T5 - ld_tx / 6) * pIn->Jdec * ld_tx) * ld_tx;
	}
	else if (fabs(dTime - ld_temp) < RT_LITTLE)
	{
		ld_tx =  pIn->T7;

		pOut->Acc = pIn->Jdec * (pIn->T5 - ld_tx);
		pOut->Vel = pIn->V6 + (pIn->T5 - 0.5 * ld_tx) * pIn->Jdec * ld_tx;
		pOut->Len = pIn->L6 + (pIn->V6 + (0.5 * pIn->T5 - ld_tx / 6) * pIn->Jdec * ld_tx) * ld_tx;
	}
	else
	{
		pOut->Acc = 0;
		pOut->Vel = 0;
		pOut->Len = 0;
	}
}

