/*****************************************************************************
 *        Robot�˶�ѧ���                                                    *
 *        SCUT, 2010                                                         *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2010-12-08                                       *
 *****************************************************************************/
#include <math.h>
#include "Kine.h"
#include "Setup.h"
#include <iostream>
#include <fstream>

using namespace std;

#define KINE_RATIO_V    30
#define KINE_DAMP_EI    80
#define KINE_RATIO_W   0.008
#define KINE_DAMP_EB   0.09

int GetMatrixnum = 0;

/*****************************************************************************
 * ������Kine()
 * ���ܣ����캯��
 *****************************************************************************/
Kine::Kine()
{
}

/*****************************************************************************
 * ������~Kine()
 * ���ܣ���������
 *****************************************************************************/
Kine::~Kine()
{
}

/*****************************************************************************
 * ������RadInRange()
 * ���ܣ��ɵ�ǰת���Ż�����ת��
 *           - �뵱ǰֵ�Ƚ�,���Ƕȱ任Ϊ(-360,0)��(0,+360)��Χ��
 *
 * ���룺double* id_rad - ����ת��(����)
 *       double* id_deg - ��ǰת��(�Ƕ�)
 * �����double* id_rad - ����ת��(����)
 *****************************************************************************/
void Kine::RadInRange(double* pdRad, double* pdDeg)
{
	if ((*pdRad) > PI2)
	{
		for(;;)
		{
			*pdRad -= PI2;	
			if ((*pdRad) <= PI2)
				if (fabs(*pdRad - (*pdDeg) * PI_RAD) >
					fabs(*pdRad - PI2 - (*pdDeg) * PI_RAD))
					*pdRad -= PI2;
				break;
		}
	}
	else if ((*pdRad) < - PI2)
	{
		for (;;)
		{
			*pdRad += PI2;	
			if((*pdRad) >= - PI2)
				if( fabs(*pdRad - (*pdDeg) * PI_RAD) >
					fabs(*pdRad + PI2 - (*pdDeg) * PI_RAD))
					*pdRad += PI2;
				break;
		}
	}
	else if ((*pdRad) >= 0)
	{
		if (fabs(*pdRad - (*pdDeg) * PI_RAD) >
			fabs(*pdRad - PI2 - (*pdDeg) * PI_RAD))
			*pdRad -= PI2;
	}
	else 
	{
		if (fabs(*pdRad - (*pdDeg) * PI_RAD) >
			fabs(*pdRad + PI2 - (*pdDeg) * PI_RAD))
			*pdRad += PI2;
	}
}

/******************************************************************************
 * ������Set_Length()
 * ���ܣ����ø˳�
 *
 * ���룺double l1 - �˳�L1
 *       double l2 - �˳�L2
 *       double l3 - �˳�L3
 *       double l4 - �˳�L4
 *       double l5 - �˳�L5
 ******************************************************************************/
void Kine_IR_FiveDoF::Set_Length(double gdLen[])
{
	m_dL1 = gdLen[0] + gdLen[1];
	m_dL2 = gdLen[2];
	m_dL3 = gdLen[3];
	m_dL4 = gdLen[4];
	m_dL5 = gdLen[5];

	kine.Set_Length(gdLen);
}
void Kine_IR_FiveDoF::Set_Tool(double gdTool[])
{
	int i;
	for (i=0; i<6; i++)
	{
		m_dTool[i] = 0;//gdTool[i];
	}
}
/******************************************************************************
 * ������FKine()
 * ���ܣ�����
 *
 * ���룺double* gdJPos - �ؽ�ת��, 5�ؽ�
 * �����double* gdCPos - ����λ��, (x,y,z,w,p,r),X-Y-Z�̶���,
 *
 * ���أ�int - 0�ɹ�,
 ******************************************************************************/
int Kine_IR_FiveDoF::FKine(double gdJPos[], double gdCPos[])
{
#ifdef ROBOT_IR_5_T
	return kine.FKine(gdJPos, gdCPos);
#endif

	// �ؽڽǶ� - �ؽڻ��� - ������� - Tcp���� - λ��
	// id_jPos  -   jRad   - lm_Wrist - lm_Tcp  - id_cPos
	double ld_jRad[5];// = gd_rad[0];
	
	double c23, s23;
	
	double ld_temp[8];     // �м����
	MtxKine lm_Wrist;      // �м����,�������
	MtxKine lm_Tool;       // �м����,���߾���
	MtxKine lm_Tcp;        // �м����,TCP����

    //--------------------- �ؽڻ��� -------------------------//
	ld_jRad[0] = gdJPos[0] * PI_RAD;
	ld_jRad[1] = gdJPos[1] * PI_RAD;
	ld_jRad[2] = gdJPos[2] * PI_RAD;
	ld_jRad[3] = gdJPos[3] * PI_RAD;
	ld_jRad[4] = gdJPos[4] * PI_RAD;

    //--------------------- ������� -------------------------//
    c23 = cos(ld_jRad[1]+ld_jRad[2]);
    s23 = sin(ld_jRad[1]+ld_jRad[2]);	

    // ����r11 //
	lm_Wrist.R11 = cos(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * cos(ld_jRad[4]) 
		+ sin(ld_jRad[0]) * sin(ld_jRad[3]) * cos(ld_jRad[4])
		- cos(ld_jRad[0]) * s23 * sin(ld_jRad[4]);
  
     // ����r12 //
	lm_Wrist.R12 = - cos(ld_jRad[0]) * c23 * sin(ld_jRad[3])
				   + sin(ld_jRad[0]) * cos(ld_jRad[3]);

     // ����r13 //
	lm_Wrist.R13 = cos(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * sin(ld_jRad[4]) 
		+ sin(ld_jRad[0]) * sin(ld_jRad[3]) * sin(ld_jRad[4])
		+ cos(ld_jRad[0]) * s23 * cos(ld_jRad[4]);
        
    // ����r21 //
	lm_Wrist.R21 = sin(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * cos(ld_jRad[4]) 
		- cos(ld_jRad[0]) * sin(ld_jRad[3]) * cos(ld_jRad[4])
		- sin(ld_jRad[0]) * s23 * sin(ld_jRad[4]);

    // ����r22 //    
	lm_Wrist.R22 = - sin(ld_jRad[0]) * c23 * sin(ld_jRad[3])
				   - cos(ld_jRad[0]) * cos(ld_jRad[3]);
        
    // ����r23 //    
	lm_Wrist.R23 = sin(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * sin(ld_jRad[4]) 
		- cos(ld_jRad[0]) * sin(ld_jRad[3]) * sin(ld_jRad[4])
		+ sin(ld_jRad[0]) * s23 * cos(ld_jRad[4]);
        
    // ����r31 //
	lm_Wrist.R31 = s23 * cos(ld_jRad[3]) * cos(ld_jRad[4]) 
		+ c23 * sin(ld_jRad[4]);

    // ����r32 //    
	lm_Wrist.R32 = - s23 * sin(ld_jRad[3]);
    
    // ����r33 //
	lm_Wrist.R33 = + s23 * cos(ld_jRad[3]) * sin(ld_jRad[4]) 
        - c23 * cos(ld_jRad[4]);

    // ����x //
	lm_Wrist.X = lm_Wrist.R13 * m_dL5
		+ cos(ld_jRad[0]) * s23 * (m_dL3 + m_dL4)
		+ cos(ld_jRad[0]) * cos(ld_jRad[1]) * m_dL2;

    // ����y //
	lm_Wrist.Y = lm_Wrist.R23 * m_dL5
		+ sin(ld_jRad[0]) * s23 * (m_dL3 + m_dL4)
		+ sin(ld_jRad[0]) * cos(ld_jRad[1]) * m_dL2;
    
    // ����z //
	lm_Wrist.Z = lm_Wrist.R33 * m_dL5
		- c23 * (m_dL3 + m_dL4)
		+ sin(ld_jRad[1]) * m_dL2;

    //--------------------- Tcp���� -------------------------//
    // ���㹤�ߵı任���� //
	ld_temp[1] = sin(m_dTool[3] * PI_RAD);//0
	ld_temp[2] = cos(m_dTool[3] * PI_RAD);//1
	ld_temp[3] = sin(m_dTool[4] * PI_RAD);//0
	ld_temp[4] = cos(m_dTool[4] * PI_RAD);//1
	ld_temp[5] = sin(m_dTool[5] * PI_RAD);//0
	ld_temp[6] = cos(m_dTool[5] * PI_RAD);//1
	
    lm_Tool.R11 = ld_temp[2] * ld_temp[4];                                       //1
    lm_Tool.R12 = ld_temp[2] * ld_temp[3] * ld_temp[5] - ld_temp[1] * ld_temp[6];//0
    lm_Tool.R13 = ld_temp[2] * ld_temp[3] * ld_temp[6] + ld_temp[1] * ld_temp[5];//0
    lm_Tool.R21 = ld_temp[1] * ld_temp[4];                                       //0
    lm_Tool.R22 = ld_temp[1] * ld_temp[3] * ld_temp[5] + ld_temp[2] * ld_temp[6];//1
    lm_Tool.R23 = ld_temp[1] * ld_temp[3] * ld_temp[6] - ld_temp[2] * ld_temp[5];//0
    lm_Tool.R31 = -ld_temp[3];                                                   //0
    lm_Tool.R32 = ld_temp[4] * ld_temp[5];                                       //0
    lm_Tool.R33 = ld_temp[4] * ld_temp[6];                                       //1

    // ����Tcp��̬����Ĳ��� //
    lm_Tcp.R11 = lm_Wrist.R11 * lm_Tool.R11 + lm_Wrist.R12 * lm_Tool.R21 + lm_Wrist.R13 * lm_Tool.R31;
    lm_Tcp.R21 = lm_Wrist.R21 * lm_Tool.R11 + lm_Wrist.R22 * lm_Tool.R21 + lm_Wrist.R23 * lm_Tool.R31;
    lm_Tcp.R31 = lm_Wrist.R31 * lm_Tool.R11 + lm_Wrist.R32 * lm_Tool.R21 + lm_Wrist.R33 * lm_Tool.R31;
    lm_Tcp.R32 = lm_Wrist.R31 * lm_Tool.R12 + lm_Wrist.R32 * lm_Tool.R22 + lm_Wrist.R33 * lm_Tool.R32;
    lm_Tcp.R33 = lm_Wrist.R31 * lm_Tool.R13 + lm_Wrist.R32 * lm_Tool.R23 + lm_Wrist.R33 * lm_Tool.R33;
    lm_Tcp.R12 = lm_Wrist.R11 * lm_Tool.R12 + lm_Wrist.R12 * lm_Tool.R22 + lm_Wrist.R13 * lm_Tool.R32;
    lm_Tcp.R22 = lm_Wrist.R21 * lm_Tool.R12 + lm_Wrist.R22 * lm_Tool.R22 + lm_Wrist.R23 * lm_Tool.R32;

    // ����Tcpλ�� //
	lm_Tcp.X = lm_Wrist.X + lm_Wrist.R11 * (m_dTool[0])
                          + lm_Wrist.R12 * (m_dTool[1])
                          + lm_Wrist.R13 * (m_dTool[2]);//lm_Wrist.X
	lm_Tcp.Y = lm_Wrist.Y + lm_Wrist.R21 * (m_dTool[0])
                          + lm_Wrist.R22 * (m_dTool[1])
                          + lm_Wrist.R23 * (m_dTool[2]);//lm_Wrist.Y
	lm_Tcp.Z = lm_Wrist.Z + lm_Wrist.R31 * (m_dTool[0])
                          + lm_Wrist.R32 * (m_dTool[1])
                          + lm_Wrist.R33 * (m_dTool[2]);//lm_Wrist.Z


    //--------------------- Tcpλ�� -------------------------//
    // ���㹤������ϵRPY�ǣ�X-Y-Z�̶�������ϵ�� - rad //
	ld_temp[5] = atan2(-lm_Tcp.R31, sqrt(lm_Tcp.R11 * lm_Tcp.R11 + lm_Tcp.R21 * lm_Tcp.R21));

    if (fabs(ld_temp[5] - PI / 2) < RT_LITTLE)
    {
        ld_temp[4] = 0;
        ld_temp[6] = atan2(lm_Tcp.R12, lm_Tcp.R22);
    }
    else if (fabs(ld_temp[5] + PI / 2) < RT_LITTLE)
    {
        ld_temp[4] = 0;
        ld_temp[6] = -atan2(lm_Tcp.R12, lm_Tcp.R22);
    }
    else
    {
        ld_temp[0] = 1 / cos(ld_temp[5]);

		ld_temp[1] = lm_Tcp.R21 * ld_temp[0];
		ld_temp[2] = lm_Tcp.R11 * ld_temp[0];
		if (fabs(ld_temp[1]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		if (fabs(ld_temp[2]) < RT_LITTLE)
		{
			ld_temp[2] = 0;
		}
        ld_temp[4] = atan2(ld_temp[1], ld_temp[2]);

		ld_temp[1] = lm_Tcp.R32 * ld_temp[0];
		ld_temp[2] = lm_Tcp.R33 * ld_temp[0];
		if (fabs(ld_temp[1]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		if (fabs(ld_temp[2]) < RT_LITTLE)
		{
			ld_temp[2] = 0;
		}
        ld_temp[6] = atan2(ld_temp[1], ld_temp[2]);
    }

	gdCPos[0] = lm_Tcp.X;      // mm   
	gdCPos[1] = lm_Tcp.Y;      // mm
	gdCPos[2] = lm_Tcp.Z + m_dL1; // mm + L1
	gdCPos[3] = ld_temp[4] * PI_DEG;   // deg
	gdCPos[4] = ld_temp[5] * PI_DEG;   // deg
	gdCPos[5] = ld_temp[6] * PI_DEG;   // deg

	return Ok;
}


/******************************************************************************
 * ������FKineMatrix()
 * ���ܣ�����,���4��4����
 *
 * ���룺double* TransfMatrix - �ؽ�ת��, 5�ؽ�
 *
 * ���أ�int - 0�ɹ�,
 ******************************************************************************/
void Kine_IR_FiveDoF::FKineMatrix(double gdJPos[])
{
	
	m_dL_GetMatrix[0] = 0;
	m_dL_GetMatrix[1]=Robot::robotLen[0]+Robot::robotLen[1];
	m_dL_GetMatrix[2]=Robot::robotLen[2];
	m_dL_GetMatrix[3]=Robot::robotLen[3];
	m_dL_GetMatrix[4]=Robot::robotLen[4];
	m_dL_GetMatrix[5]=Robot::robotLen[5];

	for (int i=0; i<6; i++)
	{
		m_dTool[i] = 0;//gdTool[i];
	}
	// �ؽڽǶ� - �ؽڻ��� - ������� - Tcp���� - λ��
	// id_jPos  -   jRad   - lm_Wrist - lm_Tcp  - id_cPos
	double ld_jRad[5]={0};// = gd_rad[0];
	double gdCPos[6]={0};
	double c23=0, s23=0;

	double ld_temp[8]={0};     // �м����
	MtxKine lm_Wrist;      // �м����,�������
	MtxKine lm_Tool;       // �м����,���߾���
	MtxKine lm_Tcp;        // �м����,TCP����

	//--------------------- �ؽڻ��� -------------------------//
	ld_jRad[0] = gdJPos[0] * PI_RAD;
	ld_jRad[1] = gdJPos[1] * PI_RAD;
	ld_jRad[2] = gdJPos[2] * PI_RAD;
	ld_jRad[3] = gdJPos[3] * PI_RAD;
	ld_jRad[4] = gdJPos[4] * PI_RAD;

	//--------------------- ������� -------------------------//
	c23 = cos(ld_jRad[1]+ld_jRad[2]);
	s23 = sin(ld_jRad[1]+ld_jRad[2]);	

	// ����r11 //
	lm_Wrist.R11 = cos(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * cos(ld_jRad[4]) 
		+ sin(ld_jRad[0]) * sin(ld_jRad[3]) * cos(ld_jRad[4])
		- cos(ld_jRad[0]) * s23 * sin(ld_jRad[4]);

	// ����r12 //
	lm_Wrist.R12 = - cos(ld_jRad[0]) * c23 * sin(ld_jRad[3])
		+ sin(ld_jRad[0]) * cos(ld_jRad[3]);

	// ����r13 //
	lm_Wrist.R13 = cos(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * sin(ld_jRad[4]) 
		+ sin(ld_jRad[0]) * sin(ld_jRad[3]) * sin(ld_jRad[4])
		+ cos(ld_jRad[0]) * s23 * cos(ld_jRad[4]);

	// ����r21 //
	lm_Wrist.R21 = sin(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * cos(ld_jRad[4]) 
		- cos(ld_jRad[0]) * sin(ld_jRad[3]) * cos(ld_jRad[4])
		- sin(ld_jRad[0]) * s23 * sin(ld_jRad[4]);

	// ����r22 //    
	lm_Wrist.R22 = - sin(ld_jRad[0]) * c23 * sin(ld_jRad[3])
		- cos(ld_jRad[0]) * cos(ld_jRad[3]);

	// ����r23 //    
	lm_Wrist.R23 = sin(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * sin(ld_jRad[4]) 
		- cos(ld_jRad[0]) * sin(ld_jRad[3]) * sin(ld_jRad[4])
		+ sin(ld_jRad[0]) * s23 * cos(ld_jRad[4]);

	// ����r31 //
	lm_Wrist.R31 = + s23 * cos(ld_jRad[3]) * cos(ld_jRad[4]) 
		+ c23 * sin(ld_jRad[4]);

	// ����r32 //    
	lm_Wrist.R32 = - s23 * sin(ld_jRad[3]);

	// ����r33 //
	lm_Wrist.R33 = + s23 * cos(ld_jRad[3]) * sin(ld_jRad[4]) 
		- c23 * cos(ld_jRad[4]);

	// ����x //
	lm_Wrist.X = lm_Wrist.R13 * m_dL_GetMatrix[5]
		+ cos(ld_jRad[0]) * s23 * (m_dL_GetMatrix[3] + m_dL_GetMatrix[4])
		+ cos(ld_jRad[0]) * cos(ld_jRad[1]) * m_dL_GetMatrix[2];

	// ����y //
	lm_Wrist.Y = lm_Wrist.R23 * m_dL_GetMatrix[5]
		+ sin(ld_jRad[0]) * s23 * (m_dL_GetMatrix[3] + m_dL_GetMatrix[4])
		+ sin(ld_jRad[0]) * cos(ld_jRad[1]) * m_dL_GetMatrix[2];

	// ����z //
	lm_Wrist.Z = lm_Wrist.R33 * m_dL_GetMatrix[5]
		- c23 * (m_dL_GetMatrix[3] + m_dL_GetMatrix[4])
		+ sin(ld_jRad[1]) * m_dL_GetMatrix[2];

	//--------------------- Tcp���� -------------------------//
	// ���㹤�ߵı任���� //
	ld_temp[1] = sin(m_dTool[3] * PI_RAD);//0
	ld_temp[2] = cos(m_dTool[3] * PI_RAD);//1
	ld_temp[3] = sin(m_dTool[4] * PI_RAD);//0
	ld_temp[4] = cos(m_dTool[4] * PI_RAD);//1
	ld_temp[5] = sin(m_dTool[5] * PI_RAD);//0
	ld_temp[6] = cos(m_dTool[5] * PI_RAD);//1

	lm_Tool.R11 = ld_temp[2] * ld_temp[4];                                       //1
	lm_Tool.R12 = ld_temp[2] * ld_temp[3] * ld_temp[5] - ld_temp[1] * ld_temp[6];//0
	lm_Tool.R13 = ld_temp[2] * ld_temp[3] * ld_temp[6] + ld_temp[1] * ld_temp[5];//0
	lm_Tool.R21 = ld_temp[1] * ld_temp[4];                                       //0
	lm_Tool.R22 = ld_temp[1] * ld_temp[3] * ld_temp[5] + ld_temp[2] * ld_temp[6];//1
	lm_Tool.R23 = ld_temp[1] * ld_temp[3] * ld_temp[6] - ld_temp[2] * ld_temp[5];//0
	lm_Tool.R31 = -ld_temp[3];                                                   //0
	lm_Tool.R32 = ld_temp[4] * ld_temp[5];                                       //0
	lm_Tool.R33 = ld_temp[4] * ld_temp[6];                                       //1

	// ����Tcp��̬����Ĳ��� //
	lm_Tcp.R11 = lm_Wrist.R11 * lm_Tool.R11 + lm_Wrist.R12 * lm_Tool.R21 + lm_Wrist.R13 * lm_Tool.R31;//lm_Wrist.R11
	lm_Tcp.R21 = lm_Wrist.R21 * lm_Tool.R11 + lm_Wrist.R22 * lm_Tool.R21 + lm_Wrist.R23 * lm_Tool.R31;//lm_Wrist.R21
	lm_Tcp.R31 = lm_Wrist.R31 * lm_Tool.R11 + lm_Wrist.R32 * lm_Tool.R21 + lm_Wrist.R33 * lm_Tool.R31;//lm_Wrist.R31
	lm_Tcp.R32 = lm_Wrist.R31 * lm_Tool.R12 + lm_Wrist.R32 * lm_Tool.R22 + lm_Wrist.R33 * lm_Tool.R32;//lm_Wrist.R32
	lm_Tcp.R33 = lm_Wrist.R31 * lm_Tool.R13 + lm_Wrist.R32 * lm_Tool.R23 + lm_Wrist.R33 * lm_Tool.R33;//lm_Wrist.R33
	lm_Tcp.R12 = lm_Wrist.R11 * lm_Tool.R12 + lm_Wrist.R12 * lm_Tool.R22 + lm_Wrist.R13 * lm_Tool.R32;//lm_Wrist.R12
	lm_Tcp.R22 = lm_Wrist.R21 * lm_Tool.R12 + lm_Wrist.R22 * lm_Tool.R22 + lm_Wrist.R23 * lm_Tool.R32;//lm_Wrist.R22

	// ����Tcpλ�� //
	lm_Tcp.X = lm_Wrist.X + lm_Wrist.R11 * (m_dTool[0])
		+ lm_Wrist.R12 * (m_dTool[1])
		+ lm_Wrist.R13 * (m_dTool[2]);//lm_Wrist.X
	lm_Tcp.Y = lm_Wrist.Y + lm_Wrist.R21 * (m_dTool[0])
		+ lm_Wrist.R22 * (m_dTool[1])
		+ lm_Wrist.R23 * (m_dTool[2]);//lm_Wrist.Y
	lm_Tcp.Z = lm_Wrist.Z + lm_Wrist.R31 * (m_dTool[0])
		+ lm_Wrist.R32 * (m_dTool[1])
		+ lm_Wrist.R33 * (m_dTool[2]);//lm_Wrist.Z

	////����
	//lm_Tcp.R11 =  -0.1210526305284929;//lm_Wrist.R11
	//lm_Tcp.R21 = -0.99245557933853790;//lm_Wrist.R21
	//lm_Tcp.R31 = -0.019446945311264285;//lm_Wrist.R31
	//lm_Tcp.R32 = 0.047971428772492331;//lm_Wrist.R32
	//lm_Tcp.R33 = 0.99865938053952430;//lm_Wrist.R33
	//lm_Tcp.R12 = 0.99138707076099175;//lm_Wrist.R12
	//lm_Tcp.R22 = -0.12186229092490403;

	//lm_Tcp.X = 61.03;//lm_Wrist.X
	//lm_Tcp.Y = 84.29;//lm_Wrist.Y
	//lm_Tcp.Z = 318.58;//lm_Wrist.Z

	//--------------------- Tcpλ�� -------------------------//
	// ���㹤������ϵRPY�� - rad //
	ld_temp[5] = atan2(-lm_Tcp.R31, sqrt(lm_Tcp.R11 * lm_Tcp.R11 + lm_Tcp.R21 * lm_Tcp.R21));

	if (fabs(ld_temp[5] - PI / 2) < RT_LITTLE)
	{
		ld_temp[4] = 0;
		ld_temp[6] = atan2(lm_Tcp.R12, lm_Tcp.R22);
	}
	else if (fabs(ld_temp[5] + PI / 2) < RT_LITTLE)
	{
		ld_temp[4] = 0;
		ld_temp[6] = -atan2(lm_Tcp.R12, lm_Tcp.R22);
	}
	else
	{
		ld_temp[0] = 1 / cos(ld_temp[5]);

		ld_temp[1] = lm_Tcp.R21 * ld_temp[0];
		ld_temp[2] = lm_Tcp.R11 * ld_temp[0];
		if (fabs(ld_temp[1]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		if (fabs(ld_temp[2]) < RT_LITTLE)
		{
			ld_temp[2] = 0;
		}
		ld_temp[4] = atan2(ld_temp[1], ld_temp[2]);

		ld_temp[1] = lm_Tcp.R32 * ld_temp[0];
		ld_temp[2] = lm_Tcp.R33 * ld_temp[0];
		if (fabs(ld_temp[1]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		if (fabs(ld_temp[2]) < RT_LITTLE)
		{
			ld_temp[2] = 0;
		}
		ld_temp[6] = atan2(ld_temp[1], ld_temp[2]);
	}

	gdCPos[0] = lm_Tcp.X;      // mm   
	gdCPos[1] = lm_Tcp.Y;      // mm
	gdCPos[2] = lm_Tcp.Z + m_dL_GetMatrix[1]; // mm + L1
	gdCPos[3] = ld_temp[4] * PI_DEG;   // deg
	gdCPos[4] = ld_temp[5] * PI_DEG;   // deg
	gdCPos[5] = ld_temp[6] * PI_DEG;   // deg

	for (int i=0; i<6; i++)
	{
		if (fabs(gdCPos[i])<RT_LITTLE)
		{
			gdCPos[i] = 0;
		}
	}



	ofstream f;
	f.open("F:\\MoRoController\\transformation matrix.txt",ios::app);

		f<<GetMatrixnum<<"[ "<<gdJPos[0]<<"  "<<gdJPos[1]<<"  "<<gdJPos[2]<<"  "<<gdJPos[3]<<"  "<<gdJPos[4]<<" ]"<<endl;
		f<<lm_Wrist.R11<<"  "<<lm_Wrist.R12<<"  "<<lm_Wrist.R13<<"  "<<gdCPos[0]<<"  "
			<<lm_Wrist.R21<<"  "<<lm_Wrist.R22<<"  "<<lm_Wrist.R23<<"  "<<gdCPos[1]<<"  "
			<<lm_Wrist.R31<<"  "<<lm_Wrist.R32<<"  "<<lm_Wrist.R33<<"  "<<gdCPos[2]<<"  "
			<<0<<"  "<<0<<"  "<<0<<"  "<<1<<endl;
		//f<<"  "<<endl;
	f.close();

	f.open("F:\\MoRoController\\xyzrpy.txt",ios::app);

	//f<<GetMatrixnum<<"[ "<<gdJPos[0]<<"  "<<gdJPos[1]<<"  "<<gdJPos[2]<<"  "<<gdJPos[3]<<"  "<<gdJPos[4]<<" ]"<<endl;
	f<<gdCPos[0]<<"  "<<gdCPos[1]<<"  "<<gdCPos[2]<<"  "<<gdCPos[3]<<"  "<<gdCPos[4]<<"  "<<gdCPos[5]<<endl;
	//f<<"  "<<endl;
	f.close();

	GetMatrixnum++;
}

/******************************************************************************
 * ������IKine()
 * ���ܣ����
 *
 * ���룺double* gdCPos  - λ������, (x,y,z,w,p,r)
 *       double* gdJCurr - ��ǰ�ؽ�ת��, 5�ؽ�
 * �����double* id_jPos  - ���ؽ�ת��, 5�ؽ�
 *
 * ���أ�int - 0�ɹ�, ��������
 ******************************************************************************/
int Kine_IR_FiveDoF::IKine(double gdCPos[], double gdJCurr[], double gdJPos[])
{
	
#ifdef ROBOT_IR_5_T
	return kine.IKine(gdCPos, gdJCurr, gdJPos);
#endif


	// λ�� - Tcp���� - ������� - �ؽڻ��� - �ؽڽǶ�
	// id_cPos - lm_Tcp - lm_Wrist    - ld_jRad     - id_jPos
	int i;
	int result;
	int li_flag[4] = {0};      // ���������,0Ϊ�н�

	double s1,c1,s3,c3, s4, c4, s5, c5, s23, c23;

	double ld_temp[8];       // �м����
	MtxKine lm_Wrist;        // �м����,�������
	MtxKine lm_Tool;         // �м����,���߾���
	MtxKine lm_Tcp;          // �м����,TCP����
	double gd_rad[4][5];     // �м����,�������

    //--------------------- TCP���� -------------------------//
// 	gdCPos[3]=169.534;
// 	gdCPos[4]=-0.245618;
// 	gdCPos[5]=179.999;

	ld_temp[1] = sin(gdCPos[3] * PI_RAD);
	ld_temp[2] = cos(gdCPos[3] * PI_RAD);
	ld_temp[3] = sin(gdCPos[4] * PI_RAD);
	ld_temp[4] = cos(gdCPos[4] * PI_RAD);
	ld_temp[5] = sin(gdCPos[5] * PI_RAD);
	ld_temp[6] = cos(gdCPos[5] * PI_RAD);

	lm_Tcp.R11 = ld_temp[2] * ld_temp[4];
    lm_Tcp.R12 = ld_temp[2] * ld_temp[3] * ld_temp[5] - ld_temp[1] * ld_temp[6];
    lm_Tcp.R13 = ld_temp[2] * ld_temp[3] * ld_temp[6] + ld_temp[1] * ld_temp[5];
    lm_Tcp.R21 = ld_temp[1] * ld_temp[4];
    lm_Tcp.R22 = ld_temp[1] * ld_temp[3] * ld_temp[5] + ld_temp[2] * ld_temp[6];
    lm_Tcp.R23 = ld_temp[1] * ld_temp[3] * ld_temp[6] - ld_temp[2] * ld_temp[5];
    lm_Tcp.R31 = -ld_temp[3];
    lm_Tcp.R32 = ld_temp[4] * ld_temp[5];
    lm_Tcp.R33 = ld_temp[4] * ld_temp[6];

    //--------------------- ���߾��� -------------------------//
    // ���㹤�ߵı任���� //
	ld_temp[1] = sin(m_dTool[3] * PI_RAD);
	ld_temp[2] = cos(m_dTool[3] * PI_RAD);
	ld_temp[3] = sin(m_dTool[4] * PI_RAD);
	ld_temp[4] = cos(m_dTool[4] * PI_RAD);
	ld_temp[5] = sin(m_dTool[5] * PI_RAD);
	ld_temp[6] = cos(m_dTool[5] * PI_RAD);
	
    lm_Tool.R11 = ld_temp[2] * ld_temp[4];
    lm_Tool.R12 = ld_temp[2] * ld_temp[3] * ld_temp[5] - ld_temp[1] * ld_temp[6];
    lm_Tool.R13 = ld_temp[2] * ld_temp[3] * ld_temp[6] + ld_temp[1] * ld_temp[5];
    lm_Tool.R21 = ld_temp[1] * ld_temp[4];
    lm_Tool.R22 = ld_temp[1] * ld_temp[3] * ld_temp[5] + ld_temp[2] * ld_temp[6];
    lm_Tool.R23 = ld_temp[1] * ld_temp[3] * ld_temp[6] - ld_temp[2] * ld_temp[5];
    lm_Tool.R31 = -ld_temp[3];
    lm_Tool.R32 = ld_temp[4] * ld_temp[5];
    lm_Tool.R33 = ld_temp[4] * ld_temp[6];

    // ��̬���� //
    ld_temp[1] = lm_Tool.R12;
    lm_Tool.R12 = lm_Tool.R21;
    lm_Tool.R21 = ld_temp[1];

    ld_temp[1] = lm_Tool.R13;
    lm_Tool.R13 = lm_Tool.R31;
    lm_Tool.R31 = ld_temp[1];

    ld_temp[1] = lm_Tool.R23;
    lm_Tool.R23 = lm_Tool.R32;
    lm_Tool.R32 = ld_temp[1];
    
	// λ������ //
	lm_Tool.X = -( lm_Tool.R11 * m_dTool[0] + 
				   lm_Tool.R12 * m_dTool[1] +
                   lm_Tool.R13 * m_dTool[2] );
	lm_Tool.Y = -( lm_Tool.R21 * m_dTool[0] + 
				   lm_Tool.R22 * m_dTool[1] +
                   lm_Tool.R23 * m_dTool[2] );
	lm_Tool.Z = -( lm_Tool.R31 * m_dTool[0] + 
				   lm_Tool.R32 * m_dTool[1] +
                   lm_Tool.R33 * m_dTool[2] );

    //--------------------- ������� -------------------------//
    // ��̬���� //
    lm_Wrist.R11 = lm_Tcp.R11 * lm_Tool.R11 + lm_Tcp.R12 * lm_Tool.R21 + lm_Tcp.R13 * lm_Tool.R31;
    lm_Wrist.R12 = lm_Tcp.R11 * lm_Tool.R12 + lm_Tcp.R12 * lm_Tool.R22 + lm_Tcp.R13 * lm_Tool.R32;
    lm_Wrist.R13 = lm_Tcp.R11 * lm_Tool.R13 + lm_Tcp.R12 * lm_Tool.R23 + lm_Tcp.R13 * lm_Tool.R33;
    lm_Wrist.R21 = lm_Tcp.R21 * lm_Tool.R11 + lm_Tcp.R22 * lm_Tool.R21 + lm_Tcp.R23 * lm_Tool.R31;
    lm_Wrist.R22 = lm_Tcp.R21 * lm_Tool.R12 + lm_Tcp.R22 * lm_Tool.R22 + lm_Tcp.R23 * lm_Tool.R32;
    lm_Wrist.R23 = lm_Tcp.R21 * lm_Tool.R13 + lm_Tcp.R22 * lm_Tool.R23 +   lm_Tcp.R23 * lm_Tool.R33;
    lm_Wrist.R31 = lm_Tcp.R31 * lm_Tool.R11 + lm_Tcp.R32 * lm_Tool.R21 + lm_Tcp.R33 * lm_Tool.R31;
    lm_Wrist.R32 = lm_Tcp.R31 * lm_Tool.R12 + lm_Tcp.R32 * lm_Tool.R22 + lm_Tcp.R33 * lm_Tool.R32;
    lm_Wrist.R33 = lm_Tcp.R31 * lm_Tool.R13 + lm_Tcp.R32 * lm_Tool.R23 + lm_Tcp.R33 * lm_Tool.R33;

    // λ�� //
	lm_Wrist.X = lm_Tcp.R11 * lm_Tool.X + lm_Tcp.R12 * lm_Tool.Y + lm_Tcp.R13 * lm_Tool.Z
		+ gdCPos[0] - lm_Wrist.R13 * m_dL5;
	lm_Wrist.Y = lm_Tcp.R21 * lm_Tool.X + lm_Tcp.R22 * lm_Tool.Y + lm_Tcp.R23 * lm_Tool.Z
		+ gdCPos[1] - lm_Wrist.R23 * m_dL5;
	lm_Wrist.Z = lm_Tcp.R31 * lm_Tool.X + lm_Tcp.R32 * lm_Tool.Y + lm_Tcp.R33 * lm_Tool.Z
		+ (gdCPos[2] - m_dL1) - lm_Wrist.R33 * m_dL5;

    //--------------------- �ؽڻ��� -------------------------//
    //------ ת��1 -------//
	ld_temp[0] = atan2(lm_Wrist.Y, lm_Wrist.X);
	ld_temp[1] = atan2(- lm_Wrist.Y, - lm_Wrist.X);
	RadInRange(&ld_temp[0], &gdJCurr[0]);
	RadInRange(&ld_temp[1], &gdJCurr[0]);

	gd_rad[0][0] = gd_rad[1][0] = ld_temp[0];
	gd_rad[2][0] = gd_rad[3][0] = ld_temp[1];

    //------ ת��3 ------//
    for (i=0; i<2; i++)
    {
        s1 = sin(gd_rad[2*i][0]);
        c1 = cos(gd_rad[2*i][0]);
	
		ld_temp[0] = c1 * lm_Wrist.X + s1 * lm_Wrist.Y;
		ld_temp[1] = ld_temp[0] * ld_temp[0] + lm_Wrist.Z * lm_Wrist.Z
			- m_dL2 * m_dL2 - (m_dL3 + m_dL4) * (m_dL3 + m_dL4);

		ld_temp[0] = ld_temp[1] / (2 * m_dL2 * (m_dL3 + m_dL4));
		ld_temp[1] = ld_temp[0] * ld_temp[0];

        if (ld_temp[1] <= 1)
        {
            ld_temp[2] = sqrt(1 - ld_temp[1]);

			ld_temp[3] = atan2(ld_temp[0], ld_temp[2]);
			ld_temp[4] = atan2(ld_temp[0],-ld_temp[2]);
			RadInRange(&ld_temp[3], &gdJCurr[2]);
			RadInRange(&ld_temp[4], &gdJCurr[2]);

            gd_rad[2*i][2]   = ld_temp[3];
            gd_rad[2*i+1][2] = ld_temp[4];
        }
		else if (ld_temp[1] - 1 < RT_LITTLE)
		{
            ld_temp[2] = 0;

			ld_temp[3] = atan2(ld_temp[0], ld_temp[2]);
			ld_temp[4] = atan2(ld_temp[0],-ld_temp[2]);
			RadInRange(&ld_temp[3], &gdJCurr[2]);
			RadInRange(&ld_temp[4], &gdJCurr[2]);

            gd_rad[2*i][2]   = ld_temp[3];
            gd_rad[2*i+1][2] = ld_temp[4];
		}
        else
        {
			li_flag[2*i]   = 1;     // ��־ - ������޽�
			li_flag[2*i+1] = 1;
        }
    }
    if (li_flag[0] && li_flag[2])
    {
        return ERR_NOINV; //  ���ؽ��޿������ֵ //
    }

    //------ ת��2,4,5 ------//
    for (i=0; i<4; i++)
    {
		if(li_flag[i] == 0)
		{
			// ת��1��ת��3��������ֵ //
			s1 = sin(gd_rad[i][0]);
			c1 = cos(gd_rad[i][0]);
			s3 = sin(gd_rad[i][2]);
			c3 = cos(gd_rad[i][2]);

			// ����ת��2��ת��3�ĺ� //
			ld_temp[0] = c1 * lm_Wrist.X + s1 * lm_Wrist.Y;
			ld_temp[1] = m_dL3 + m_dL4 + s3 * m_dL2;

			ld_temp[2] = 1 / (ld_temp[0] * ld_temp[0] + lm_Wrist.Z * lm_Wrist.Z);

			s23 = (ld_temp[0] * ld_temp[1] + lm_Wrist.Z * c3 * m_dL2) * ld_temp[2];
			c23 = (ld_temp[0] * c3 * m_dL2 - lm_Wrist.Z * ld_temp[1]) * ld_temp[2];
    
			// ����ת��2 //
			ld_temp[5] = atan2(s23, c23) - gd_rad[i][2];
			RadInRange(&ld_temp[5], &gdJCurr[1]);
			gd_rad[i][1] = ld_temp[5];

			// ����ת��4 //			
			//s4 = - lm_Wrist.R12 * c1 * c23 - lm_Wrist.R22 * s1 * c23 - lm_Wrist.R32 * s23;
			//c4 = + lm_Wrist.R12 * s1 - lm_Wrist.R22 * c1;
			s4 = + lm_Wrist.R13 * s1 - lm_Wrist.R23 * c1;
			c4 = + lm_Wrist.R13 * c1 * c23 + lm_Wrist.R23 * s1 * c23 + lm_Wrist.R33 * s23; 
			ld_temp[5] = atan2(s4, c4);
			RadInRange(&ld_temp[5], &gdJCurr[3]);
			gd_rad[i][3] = ld_temp[5];

			// ����ת��5 //
			s4 = sin(gd_rad[i][3]);
			c4 = cos(gd_rad[i][3]);

//			s5 = - lm_Wrist.R11 * c1 * s23 
//				 - lm_Wrist.R21 * s1 * s23
//				 + lm_Wrist.R31 * c23;
//			c5 =   lm_Wrist.R13 * c1 * s23
//				 + lm_Wrist.R23 * s1 * s23
//				 - lm_Wrist.R33 * c23;
			s5 =   lm_Wrist.R13 * (c1 * c23 * c4 + s1 * s4)
				 + lm_Wrist.R23 * (s1 * c23 * c4 - c1 * s4)
				 + lm_Wrist.R33 * (+ s23 * c4);
			c5 =   lm_Wrist.R13 * c1 * s23
				 + lm_Wrist.R23 * s1 * s23
				 - lm_Wrist.R33 * c23;

			ld_temp[5] = atan2(s5, c5);
			RadInRange(&ld_temp[5], &gdJCurr[4]);
			gd_rad[i][4] = ld_temp[5];
		}
	}
	
	//------ ��ѽ�� ------//
	for (i=0; i<4; i++)
	{
		if (0 == li_flag[i])  // ��ȡ��Ծ���ֵ
		{
			ld_temp[i] = fabs(gd_rad[i][0] - gdJCurr[0] * PI_RAD) + 
			      	     fabs(gd_rad[i][1] - gdJCurr[1] * PI_RAD) + 
					     fabs(gd_rad[i][2] - gdJCurr[2] * PI_RAD) + 
					     fabs(gd_rad[i][3] - gdJCurr[3] * PI_RAD) +
					     fabs(gd_rad[i][4] - gdJCurr[4] * PI_RAD);
		}
	}
	for (i=0; i<4; i++)
	{
		if (0 == li_flag[i])
		{
			s1 = ld_temp[i]; // �õ�һ����Чֵ �� ��ʼ�� �м����s1
			result = i;
			break;           // �Ƴ���ʼ��
		}
	}
	for (i=0; i<4; i++)
	{
		if ((0 == li_flag[i]) && (ld_temp[i] <= s1))  // ��Чֵ | ��Ծ���ֵ��С
		{
			//ld_jRad = gd_rad[i];
			s1 = ld_temp[i];
			result = i;
		}
	}
	//------ �ؽڽǶ� ------//
	gdJPos[0] = gd_rad[result][0] * PI_DEG;
	gdJPos[1] = gd_rad[result][1] * PI_DEG;
	gdJPos[2] = gd_rad[result][2] * PI_DEG;
	gdJPos[3] = gd_rad[result][3] * PI_DEG;
	gdJPos[4] = gd_rad[result][4] * PI_DEG;

	return Ok;
}

// š��ĸ��ҵ��������⣨ҡ�������ṹ��ֻ��theta2\theta3�������ؽڽǲ��䣩
int Kine_IR_FiveDoF::IKine_s(double gdCPos[], double gdJCurr[], double gdJPos[])
{
	// λ�� - Tcp���� - ������� - �ؽڻ��� - �ؽڽǶ�
	// id_cPos - lm_Tcp - lm_Wrist    - ld_jRad     - id_jPos
	int i;
	int result;
	int li_flag[4] = {0};      // ���������,0Ϊ�н�

	double s1,c1,s3,c3, s4, c4, s5, c5, s23, c23;

	double ld_temp[8];       // �м����
	MtxKine lm_Wrist;        // �м����,�������
	MtxKine lm_Tool;         // �м����,���߾���
	MtxKine lm_Tcp;          // �м����,TCP����
	double gd_rad[4][5];     // �м����,�������

    //--------------------- TCP���� -------------------------//
	ld_temp[1] = sin(gdCPos[3] * PI_RAD);
	ld_temp[2] = cos(gdCPos[3] * PI_RAD);
	ld_temp[3] = sin(gdCPos[4] * PI_RAD);
	ld_temp[4] = cos(gdCPos[4] * PI_RAD);
	ld_temp[5] = sin(gdCPos[5] * PI_RAD);
	ld_temp[6] = cos(gdCPos[5] * PI_RAD);

	lm_Tcp.R11 = ld_temp[2] * ld_temp[4];
    lm_Tcp.R12 = ld_temp[2] * ld_temp[3] * ld_temp[5] - ld_temp[1] * ld_temp[6];
    lm_Tcp.R13 = ld_temp[2] * ld_temp[3] * ld_temp[6] + ld_temp[1] * ld_temp[5];
    lm_Tcp.R21 = ld_temp[1] * ld_temp[4];
    lm_Tcp.R22 = ld_temp[1] * ld_temp[3] * ld_temp[5] + ld_temp[2] * ld_temp[6];
    lm_Tcp.R23 = ld_temp[1] * ld_temp[3] * ld_temp[6] - ld_temp[2] * ld_temp[5];
    lm_Tcp.R31 = -ld_temp[3];
    lm_Tcp.R32 = ld_temp[4] * ld_temp[5];
    lm_Tcp.R33 = ld_temp[4] * ld_temp[6];

    //--------------------- ���߾��� -------------------------//
    // ���㹤�ߵı任���� //
	ld_temp[1] = sin(m_dTool[3] * PI_RAD);
	ld_temp[2] = cos(m_dTool[3] * PI_RAD);
	ld_temp[3] = sin(m_dTool[4] * PI_RAD);
	ld_temp[4] = cos(m_dTool[4] * PI_RAD);
	ld_temp[5] = sin(m_dTool[5] * PI_RAD);
	ld_temp[6] = cos(m_dTool[5] * PI_RAD);
	
    lm_Tool.R11 = ld_temp[2] * ld_temp[4];
    lm_Tool.R12 = ld_temp[2] * ld_temp[3] * ld_temp[5] - ld_temp[1] * ld_temp[6];
    lm_Tool.R13 = ld_temp[2] * ld_temp[3] * ld_temp[6] + ld_temp[1] * ld_temp[5];
    lm_Tool.R21 = ld_temp[1] * ld_temp[4];
    lm_Tool.R22 = ld_temp[1] * ld_temp[3] * ld_temp[5] + ld_temp[2] * ld_temp[6];
    lm_Tool.R23 = ld_temp[1] * ld_temp[3] * ld_temp[6] - ld_temp[2] * ld_temp[5];
    lm_Tool.R31 = -ld_temp[3];
    lm_Tool.R32 = ld_temp[4] * ld_temp[5];
    lm_Tool.R33 = ld_temp[4] * ld_temp[6];

    // ��̬���� //
    ld_temp[1] = lm_Tool.R12;
    lm_Tool.R12 = lm_Tool.R21;
    lm_Tool.R21 = ld_temp[1];

    ld_temp[1] = lm_Tool.R13;
    lm_Tool.R13 = lm_Tool.R31;
    lm_Tool.R31 = ld_temp[1];

    ld_temp[1] = lm_Tool.R23;
    lm_Tool.R23 = lm_Tool.R32;
    lm_Tool.R32 = ld_temp[1];
    
	// λ������ //
	lm_Tool.X = -( lm_Tool.R11 * m_dTool[0] + 
				   lm_Tool.R12 * m_dTool[1] +
                   lm_Tool.R13 * m_dTool[2] );
	lm_Tool.Y = -( lm_Tool.R21 * m_dTool[0] + 
				   lm_Tool.R22 * m_dTool[1] +
                   lm_Tool.R23 * m_dTool[2] );
	lm_Tool.Z = -( lm_Tool.R31 * m_dTool[0] + 
				   lm_Tool.R32 * m_dTool[1] +
                   lm_Tool.R33 * m_dTool[2] );

    //--------------------- ������� -------------------------//
    // ��̬���� //
    lm_Wrist.R11 = lm_Tcp.R11 * lm_Tool.R11 + lm_Tcp.R12 * lm_Tool.R21 + lm_Tcp.R13 * lm_Tool.R31;
    lm_Wrist.R12 = lm_Tcp.R11 * lm_Tool.R12 + lm_Tcp.R12 * lm_Tool.R22 + lm_Tcp.R13 * lm_Tool.R32;
    lm_Wrist.R13 = lm_Tcp.R11 * lm_Tool.R13 + lm_Tcp.R12 * lm_Tool.R23 + lm_Tcp.R13 * lm_Tool.R33;
    lm_Wrist.R21 = lm_Tcp.R21 * lm_Tool.R11 + lm_Tcp.R22 * lm_Tool.R21 + lm_Tcp.R23 * lm_Tool.R31;
    lm_Wrist.R22 = lm_Tcp.R21 * lm_Tool.R12 + lm_Tcp.R22 * lm_Tool.R22 + lm_Tcp.R23 * lm_Tool.R32;
    lm_Wrist.R23 = lm_Tcp.R21 * lm_Tool.R13 + lm_Tcp.R22 * lm_Tool.R23 + lm_Tcp.R23 * lm_Tool.R33;
    lm_Wrist.R31 = lm_Tcp.R31 * lm_Tool.R11 + lm_Tcp.R32 * lm_Tool.R21 + lm_Tcp.R33 * lm_Tool.R31;
    lm_Wrist.R32 = lm_Tcp.R31 * lm_Tool.R12 + lm_Tcp.R32 * lm_Tool.R22 + lm_Tcp.R33 * lm_Tool.R32;
    lm_Wrist.R33 = lm_Tcp.R31 * lm_Tool.R13 + lm_Tcp.R32 * lm_Tool.R23 + lm_Tcp.R33 * lm_Tool.R33;

    // λ�� //
	lm_Wrist.X = lm_Tcp.R11 * lm_Tool.X + lm_Tcp.R12 * lm_Tool.Y + lm_Tcp.R13 * lm_Tool.Z
		+ gdCPos[0] - lm_Wrist.R13 * m_dL5;
	lm_Wrist.Y = lm_Tcp.R21 * lm_Tool.X + lm_Tcp.R22 * lm_Tool.Y + lm_Tcp.R23 * lm_Tool.Z
		+ gdCPos[1] - lm_Wrist.R23 * m_dL5;
	lm_Wrist.Z = lm_Tcp.R31 * lm_Tool.X + lm_Tcp.R32 * lm_Tool.Y + lm_Tcp.R33 * lm_Tool.Z
		+ (gdCPos[2] - m_dL1) - lm_Wrist.R33 * m_dL5;

    //--------------------- �ؽڻ��� -------------------------//
    //------ ת��1 -------//
	ld_temp[0] = atan2(lm_Wrist.Y, lm_Wrist.X);
	ld_temp[1] = atan2(- lm_Wrist.Y, - lm_Wrist.X);
	RadInRange(&ld_temp[0], &gdJCurr[0]);
	RadInRange(&ld_temp[1], &gdJCurr[0]);

	gd_rad[0][0] = gd_rad[1][0] = ld_temp[0] = gdJCurr[0];									//SSS: add "= gdJCurr[0]" keep theta1
	gd_rad[2][0] = gd_rad[3][0] = ld_temp[1] = gdJCurr[0];									//SSS: add "= gdJCurr[0]" keep theta1

    //------ ת��3 ------//
    for (i=0; i<2; i++)
    {
        s1 = sin(gd_rad[2*i][0]);
        c1 = cos(gd_rad[2*i][0]);
	
		ld_temp[0] = c1 * lm_Wrist.X + s1 * lm_Wrist.Y;
		ld_temp[1] = ld_temp[0] * ld_temp[0] + lm_Wrist.Z * lm_Wrist.Z
			- m_dL2 * m_dL2 - (m_dL3 + m_dL4) * (m_dL3 + m_dL4);

		ld_temp[0] = ld_temp[1] / (2 * m_dL2 * (m_dL3 + m_dL4));
		ld_temp[1] = ld_temp[0] * ld_temp[0];

        if (ld_temp[1] <= 1)
        {
            ld_temp[2] = sqrt(1 - ld_temp[1]);

			ld_temp[3] = atan2(ld_temp[0], ld_temp[2]);
			ld_temp[4] = atan2(ld_temp[0],-ld_temp[2]);
			RadInRange(&ld_temp[3], &gdJCurr[2]);
			RadInRange(&ld_temp[4], &gdJCurr[2]);

            gd_rad[2*i][2]   = ld_temp[3];
            gd_rad[2*i+1][2] = ld_temp[4];
        }
		else if (ld_temp[1] - 1 < RT_LITTLE)
		{
            ld_temp[2] = 0;

			ld_temp[3] = atan2(ld_temp[0], ld_temp[2]);
			ld_temp[4] = atan2(ld_temp[0],-ld_temp[2]);
			RadInRange(&ld_temp[3], &gdJCurr[2]);
			RadInRange(&ld_temp[4], &gdJCurr[2]);

            gd_rad[2*i][2]   = ld_temp[3];
            gd_rad[2*i+1][2] = ld_temp[4];
		}
        else
        {
			li_flag[2*i]   = 1;     // ��־ - ������޽�
			li_flag[2*i+1] = 1;
        }
    }
    if (li_flag[0] && li_flag[2])
    {
        return ERR_NOINV; //  ���ؽ��޿������ֵ //
    }

    //------ ת��2,4,5 ------//
    for (i=0; i<4; i++)
    {
		if(li_flag[i] == 0)
		{
			// ת��1��ת��3��������ֵ //
			s1 = sin(gd_rad[i][0]);
			c1 = cos(gd_rad[i][0]);
			s3 = sin(gd_rad[i][2]);
			c3 = cos(gd_rad[i][2]);

			// ����ת��2��ת��3�ĺ� //
			ld_temp[0] = c1 * lm_Wrist.X + s1 * lm_Wrist.Y;
			ld_temp[1] = m_dL3 + m_dL4 + s3 * m_dL2;

			ld_temp[2] = 1 / (ld_temp[0] * ld_temp[0] + lm_Wrist.Z * lm_Wrist.Z);

			s23 = (ld_temp[0] * ld_temp[1] + lm_Wrist.Z * c3 * m_dL2) * ld_temp[2];
			c23 = (ld_temp[0] * c3 * m_dL2 - lm_Wrist.Z * ld_temp[1]) * ld_temp[2];
    
			// ����ת��2 //
			ld_temp[5] = atan2(s23, c23) - gd_rad[i][2];
			RadInRange(&ld_temp[5], &gdJCurr[1]);
			gd_rad[i][1] = ld_temp[5];

			// ����ת��4 //			
			//s4 = - lm_Wrist.R12 * c1 * c23 - lm_Wrist.R22 * s1 * c23 - lm_Wrist.R32 * s23;
			//c4 = + lm_Wrist.R12 * s1 - lm_Wrist.R22 * c1;
			s4 = + lm_Wrist.R13 * s1 - lm_Wrist.R23 * c1;
			c4 = + lm_Wrist.R13 * c1 * c23 + lm_Wrist.R23 * s1 * c23 + lm_Wrist.R33 * s23; 
			ld_temp[5] = atan2(s4, c4);
			RadInRange(&ld_temp[5], &gdJCurr[3]);
			gd_rad[i][3] = ld_temp[5] = gdJCurr[3];																//SSS: add "= gdJCurr[3]" keep theta4

			// ����ת��5 //
			s4 = sin(gd_rad[i][3]);
			c4 = cos(gd_rad[i][3]);

//			s5 = - lm_Wrist.R11 * c1 * s23 
//				 - lm_Wrist.R21 * s1 * s23
//				 + lm_Wrist.R31 * c23;
//			c5 =   lm_Wrist.R13 * c1 * s23
//				 + lm_Wrist.R23 * s1 * s23
//				 - lm_Wrist.R33 * c23;
			s5 =   lm_Wrist.R13 * (c1 * c23 * c4 + s1 * s4)
				+ lm_Wrist.R23 * (s1 * c23 * c4 - c1 * s4)
				+ lm_Wrist.R33 * (+ s23 * c4);
			c5 =   lm_Wrist.R13 * c1 * s23
				+ lm_Wrist.R23 * s1 * s23
				 - lm_Wrist.R33 * c23;

			ld_temp[5] = atan2(s5, c5);
			RadInRange(&ld_temp[5], &gdJCurr[4]);
			gd_rad[i][4] = ld_temp[5] = gdJCurr[4];																			//SSS: add "= gdJCurr[4]" keep theta5
		}
	}
	
	//------ ��ѽ�� ------//
	for (i=0; i<4; i++)
	{
		if (0 == li_flag[i])  // ��ȡ��Ծ���ֵ
		{
			ld_temp[i] = fabs(gd_rad[i][0] - gdJCurr[0] * PI_RAD) + 
			      	     fabs(gd_rad[i][1] - gdJCurr[1] * PI_RAD) + 
					     fabs(gd_rad[i][2] - gdJCurr[2] * PI_RAD) + 
					     fabs(gd_rad[i][3] - gdJCurr[3] * PI_RAD) +
					     fabs(gd_rad[i][4] - gdJCurr[4] * PI_RAD);
		}
	}
	for (i=0; i<4; i++)
	{
		if (0 == li_flag[i])
		{
			s1 = ld_temp[i]; // �õ�һ����Чֵ �� ��ʼ�� �м����s1
			result = i;
			break;           // �Ƴ���ʼ��
		}
	}
	for (i=0; i<4; i++)
	{
		if ((0 == li_flag[i]) && (ld_temp[i] <= s1))  // ��Чֵ | ��Ծ���ֵ��С
		{
			//ld_jRad = gd_rad[i];
			s1 = ld_temp[i];
			result = i;
		}
	}
	//------ �ؽڽǶ� ------//
	gdJPos[0] = gd_rad[result][0] * PI_DEG;
	gdJPos[1] = gd_rad[result][1] * PI_DEG;
	gdJPos[2] = gd_rad[result][2] * PI_DEG;
	gdJPos[3] = gd_rad[result][3] * PI_DEG;
	gdJPos[4] = gd_rad[result][4] * PI_DEG;

	return Ok;
}

/******************************************************************************
 * ������Vel_FKine()
 * ���ܣ��ٶ����, ��������ϵ�ٶ�
 *
 * ���룺double* gdJPos - ��ǰ�ؽ�ת��, 5�ؽ�, deg
 *       double* gdJVel - ��ǰ�ؽ��ٶ�, 5�ؽ�, deg/s
 * �����double* gdCVel - ĩ���ٶ�, [vx,vy,vz,wx,wy,wz], mm/s, deg/s
 *
 * ���أ�int - 0�ɹ�, ��������
 ******************************************************************************/
int Kine_IR_FiveDoF::Vel_FKine(double gdJPos[], double gdJVel[], double gdCVel[])
{
#ifdef ROBOT_IR_5_T
	return kine.Vel_FKine(gdJPos, gdJVel, gdCVel);
#endif


	int i;
	
	double p[5], v[5];
	for (i=0; i<5; i++)
	{
		p[i] = gdJPos[i] * PI_RAD;
		v[i] = gdJVel[i] * PI_RAD;
	}
	
	double c2 = cos(p[1]);
	double s3 = sin(p[2]);
	double c3 = cos(p[2]);
	double s23 = sin(p[1] + p[2]);
	double c23 = cos(p[1] + p[2]);
	double s4 = sin(p[3]);
	double c4 = cos(p[3]);
	double s5 = sin(p[4]);
	double c5 = cos(p[4]);

	
	gdCVel[0] =
			  - (m_dL2 * c2 * s4 * c5 + (m_dL3 + m_dL4) * s23 * s4 * c5 + s23 * s4 * m_dL5) * v[0]
			  + ((m_dL2 * s3 + m_dL3 + m_dL4) * c4 * c5 + m_dL2 * c3 * s5 + c4 * m_dL5) * v[1]
			  + (m_dL3 * c5 + m_dL4 * c5 + m_dL5) * c4 * v[2]
			  + m_dL5 * v[4];
	gdCVel[1] =
			  - (m_dL2 * c2 * c4 + (m_dL3 + m_dL4) * s23 * c4 + (s23 * c4 * c5 + c23 * s5) * m_dL5) * v[0]
			  - (m_dL2 * s3 * s4 + (m_dL3 + m_dL4) * s4 + s4 * c5 * m_dL5) * v[1]
			  - (m_dL3 + m_dL4 + c5 * m_dL5) * s4 * v[2]
			  + s5 * m_dL5 * v[3];
	gdCVel[2] =
		      - (m_dL2 * c2 + (m_dL3 + m_dL4) * s23) * c4 * s5 * v[0]
			  + ((m_dL2 * s3 + m_dL3 + m_dL4) * c4 * s5 - m_dL2 * c3 * c5) * v[1]
			  + (m_dL3 + m_dL4) * c4 * s5 * v[2];	
	gdCVel[3] =
		      + (s23 * c4 * c5 + c23 * s5) * v[0]
			  + s4 * c5 * v[1]
			  + s4 * c5 * v[2]
			  - s5 * v[3];
	gdCVel[4] =
		      - s23 * s4 * v[0]
			  + c4 * v[1]
			  + c4 * v[2]
			  + v[4];
	gdCVel[5] = 
		      + (s23 * c4 * s5 - c23 * c5) * v[0]
			  + s4 * s5 * v[1]
			  + s4 * s5 * v[2]
			  + c5 * v[3];

	// ת��Ϊ�Ƕ�
	gdCVel[3] *= PI_DEG;
	gdCVel[4] *= PI_DEG;
	gdCVel[5] *= PI_DEG;
	
	return Ok;
}

/******************************************************************************
 * ������Vel_IKine()
 * ���ܣ��ٶ����
 *
 * ���룺double gdJPos[] - ��ǰ�ؽ�ת��, 5�ؽ�, deg
 *       double gdCVel[] - ��ǰĩ���ٶ�, [vx,vy,vz,wx,wy,wz], mm/s, deg/s
 * �����double gdJVel[] - �ؽ��ٶ�, 5�ؽ�, deg/s
 *
 * ���أ�int - 0�ɹ�, ��������
 ******************************************************************************/
int Kine_IR_FiveDoF::Vel_IKine(double gdJPos[5], double gdCVel[6], double gdJVel[5])
{
#ifdef ROBOT_IR_5_T
	return kine.Vel_IKine(gdJPos, gdCVel, gdJVel);
#endif

	int i;
	double rad[5];
	Matrix lm_Jacobian(6, 5);
	Matrix lm_cvel(6, 1);
	Matrix lm_jvel(5, 1);
	Matrix lm_inv(5,5);
	
	double s2, c2, s3, c3, s23, c23, s234, c234, s4, c4, s5, c5;
	double l1, l2, l34, l5;
	
	lm_cvel.Mtx[0] = gdCVel[0];
	lm_cvel.Mtx[1] = gdCVel[1];
	lm_cvel.Mtx[2] = gdCVel[2];
	lm_cvel.Mtx[3] = gdCVel[3] * PI_RAD;
	lm_cvel.Mtx[4] = gdCVel[4] * PI_RAD;
	lm_cvel.Mtx[5] = gdCVel[5] * PI_RAD;
	
	
    //------------------------- �ؽڻ��� -----------------------------//
	for(i=0; i<5; i++)
	{		
		rad[i] = (gdJPos[i]) * PI_RAD;
	}
	
	s2 = sin(rad[1]);
	c2 = cos(rad[1]);
	s3 = sin(rad[2]);
	c3 = cos(rad[2]);
	s23 = sin(rad[1] + rad[2]);
	c23 = cos(rad[1] + rad[2]);
	s234 = sin(rad[1] + rad[2] + rad[3]);
	c234 = cos(rad[1] + rad[2] + rad[3]);
	s4 = sin(rad[3]);
	c4 = cos(rad[3]);
	s5 = sin(rad[4]);
	c5 = cos(rad[4]);
	
	l1 = m_dL1;
	l2 = m_dL2;
	l34 = m_dL3 + m_dL4;
	l5 = m_dL5;
	
	//--------------------------- ������ -------------------------------//
	// [0,4]-[5,4]
	lm_Jacobian.Mtx[0*5 + 4] = l5;
	lm_Jacobian.Mtx[1*5 + 4] = 0;
	lm_Jacobian.Mtx[2*5 + 4] = 0;
	lm_Jacobian.Mtx[3*5 + 4] = 0;
	lm_Jacobian.Mtx[4*5 + 4] = 1;
	lm_Jacobian.Mtx[5*5 + 4] = 0;
	
	//--------------------------- ������ -------------------------------//
	// [0,3]-[5,3]
	lm_Jacobian.Mtx[0*5 + 3] =  0;
	lm_Jacobian.Mtx[1*5 + 3] =  l5*s5;
	lm_Jacobian.Mtx[2*5 + 3] =  0;
	lm_Jacobian.Mtx[3*5 + 3] = -s5;
	lm_Jacobian.Mtx[4*5 + 3] =  0;
	lm_Jacobian.Mtx[5*5 + 3] =  c5;
	
	//--------------------------- ������ -------------------------------//
	// [0,2]
	lm_Jacobian.Mtx[0*5 + 2] =  l34*c4*c5 + l5*c4;
	lm_Jacobian.Mtx[1*5 + 2] = -l34*s4 - l5*s4*c5;
	lm_Jacobian.Mtx[2*5 + 2] =  l34*c4*s5;
	lm_Jacobian.Mtx[3*5 + 2] =  s4*s5;
	lm_Jacobian.Mtx[4*5 + 2] =  c4;
	lm_Jacobian.Mtx[5*5 + 2] =  s4*s5;
	
	//--------------------------- �ڶ��� -------------------------------//
	// [0,1]
	lm_Jacobian.Mtx[0*5 + 1] =  l2*s3*c4*c5 + l34*c4*c5 + l2*c3*s5 + l5*c4;
	lm_Jacobian.Mtx[1*5 + 1] = -l2*s3*s4 - l34*s4 - l5*s4*c5;
	lm_Jacobian.Mtx[2*5 + 1] =  l2*s3*c4*s5 + l34*c4*s5 - l2*c3*c5;
	lm_Jacobian.Mtx[3*5 + 1] =  s4*c5;
	lm_Jacobian.Mtx[4*5 + 1] =  c4;	
	lm_Jacobian.Mtx[5*5 + 1] =  s4*s5;
	
	//---------------------------- ��һ�� --------------------------------//
	// [0,0]
	lm_Jacobian.Mtx[0*5 + 0] = -l2*c2*s4*c5 - l34*s23*s4*c5 - l5*s23*s4;
	lm_Jacobian.Mtx[1*5 + 0] = -l2*c2*c4 - l34*s23*c4 - l5*(s23*c4*c5+c23*s5);
	lm_Jacobian.Mtx[2*5 + 0] = -l2*c2*c4*s5 - l34*s23*c4*s5;
	lm_Jacobian.Mtx[3*5 + 0] =  s23*c4*c5 + c23*s5;
	lm_Jacobian.Mtx[4*5 + 0] = -s23*s4;
	lm_Jacobian.Mtx[5*5 + 0] =  s23*c4*s5 - c23*c5;

	//Inv((&Trv(lm_Jacobian) * lm_Jacobian), &lm_inv);
	lm_inv = Inv((Trv(lm_Jacobian) * lm_Jacobian), lm_inv);	
	
	lm_jvel = lm_inv * (Trv(lm_Jacobian) * lm_cvel);
	
	for(i=0; i<5; i++)
	{
		gdJVel[i] = lm_jvel.Mtx[i] * PI_DEG;
	}

/*
	double c2 = cos((gdJPos[1]) * PI_RAD);  // ת��Ϊ����
	double s23 = sin((gdJPos[1] + gdJPos[2]) * PI_RAD);
	double c23 = cos((gdJPos[1] + gdJPos[2]) * PI_RAD);
	double s3 = sin(gdJPos[2] * PI_RAD);
	double c3 = cos(gdJPos[2] * PI_RAD);
	double s4 = sin((gdJPos[3]) * PI_RAD);
	double c4 = cos((gdJPos[3]) * PI_RAD);
	double s5 = sin((gdJPos[4]) * PI_RAD);
	double c5 = cos((gdJPos[4]) * PI_RAD);
	
	double vx = gdCVel[0];
	double vy = gdCVel[1];
	double vz = gdCVel[2];
	double wx = gdCVel[3] * PI_RAD;
	double wy = gdCVel[4] * PI_RAD;
	double wz = gdCVel[5] * PI_RAD;

	double a11 = - (m_dL2 * c2 + (m_dL3+ m_dL4) * s23)* s4 * c5;
	double a12 = m_dL2 * s3 * c4 * c5 + (m_dL3 + m_dL4) * c4 * c5 + m_dL2 * c3 * s5;
	double a13 = (m_dL3 + m_dL4) * c4 * c5;
	double d1 = vx - m_dL5 * wy;
	//double a21 = - (m_dL2 * c2 + (m_dL3 + m_dL4) * s23 + m_dL5 * s23 * c5) * c4;
	//double a22 = - (m_dL2 * s3 + (m_dL3 + m_dL4) + m_dL5 * c5) * s4;
	//double a23 = - ((m_dL3 + m_dL4) + m_dL5 * c5) * s4;
	//double d2 = vy - s5 * m_dL5 * (wz * c5 - wx * s5);
	double a21 = - (m_dL2 * c2 + (m_dL3 + m_dL4) * s23) * c4;
	double a22 = - (m_dL2 * s3 + (m_dL3 + m_dL4)) * s4;
	double a23 = - ((m_dL3 + m_dL4)) * s4;
	double d2 = vy + m_dL5 * wx;

	double a31 = - (m_dL2 * c2 + (m_dL3 + m_dL4) * s23) * c4 * s5;
	double a32 = m_dL2 * s3 * c4 * s5 + (m_dL3 + m_dL4) * c4 * s5 - m_dL2 * c3 * c5;
	double a33 = (m_dL3 + m_dL4) * c4 * s5;
	double d3 = vz;

	Matrix mtr1(3, 3), inv_mtr1(3,3);
	Matrix mtr2(3, 1);
	Matrix mtr(3, 1);

	mtr1.Set(0, 0, a11);
	mtr1.Set(0, 1, a12);
	mtr1.Set(0, 2, a13);
	mtr1.Set(1, 0, a21);
	mtr1.Set(1, 1, a22);
	mtr1.Set(1, 2, a23);
	mtr1.Set(2, 0, a31);
	mtr1.Set(2, 1, a32);
	mtr1.Set(2, 2, a33);

	mtr2.Set(0, 0, d1);
	mtr2.Set(1, 0, d2);
	mtr2.Set(2, 0, d3);

	if (0 != Inv(&mtr1, &inv_mtr1))
	{
		return ERR_NOINV;
	}

	mtr = inv_mtr1 * mtr2;

	gdJVel[0] = mtr.Get(0,0);
	gdJVel[1] = mtr.Get(1,0);
	gdJVel[2] = mtr.Get(2,0);

	gdJVel[3] = wz * c5 - wx * s5 + c23 * gdJVel[0];
	gdJVel[4] = wy + s23 * s4 * gdJVel[0] - c4 * (gdJVel[1] + gdJVel[2]);
	
	// ת��Ϊ�Ƕ�
	gdJVel[0] *= PI_DEG;
	gdJVel[1] *= PI_DEG;
	gdJVel[2] *= PI_DEG;
	gdJVel[3] *= PI_DEG;
	gdJVel[4] *= PI_DEG;
*/

/*
	double c2 = cos(gdJPos[1] * PI_RAD);  // ת��Ϊ����
	double s23 = sin((gdJPos[1] + gdJPos[2]) * PI_RAD);
	double c23 = cos((gdJPos[1] + gdJPos[2]) * PI_RAD);
	double s3 = sin(gdJPos[2] * PI_RAD);
	double c3 = cos(gdJPos[2] * PI_RAD);
	double s4 = sin(gdJPos[3] * PI_RAD);
	double c4 = cos(gdJPos[3] * PI_RAD);
	double s5 = sin(gdJPos[4] * PI_RAD);
	double c5 = cos(gdJPos[4] * PI_RAD);
	//double s6 = sin(gdJPos[5] * PI_RAD);
	//double c6 = cos(gdJPos[5] * PI_RAD);
	double s6 = 0;
	double c6 = 1;
	
	double vel[6];       // �ѿ����ٶ�
	double jacobian[36]; // �ſ˱Ⱦ�������

	Matrix lm_vel;   // TCP�ٶȾ���
	Matrix lm_jac;   // �ſ˱Ⱦ���
	Matrix lm_jv(6, 1);    // �ؽ��ٶȾ���

	// TCP�ٶ�
	vel[0] = gdCVel[0];
	vel[1] = gdCVel[1];
	vel[2] = gdCVel[2];
	vel[3] = gdCVel[3] * PI_RAD;
	vel[4] = gdCVel[4] * PI_RAD;
	vel[5] = gdCVel[5] * PI_RAD;

	// TCP�ٶȾ����ʼ��
	lm_vel.Init(6, 1, vel);

	// vx
	jacobian[0] = 
		- (c2 * s4 * c5 * c6 + c2 * c4 * s6) * m_dL2
		- (s23 * s4 * c5 * c6 + s23 * c4 * s6) * (m_dL3 + m_dL4)
		- (s23 * c4 * c5 * s6 + c23 * s5 * s6 + s23 * s4 * c6) * (m_dL5 + 0);
	jacobian[1] = 
		+ (s3 * c4 * c5 * c6 + c3 * s5 * c6 - s3 * s4 * s6) * m_dL2
		+ (c4 * c5 * c6 - s4 * s6) * (m_dL3 + m_dL4)
		+ (c4 * c6 - s4 * c5 * s6) * (m_dL5 + 0);
	jacobian[2] = 
		+ (c4 * c5 * c6 - s4 * s6) * (m_dL3 + m_dL4)
		+ (c4 * c6 - s4 * c5 * s6) * (m_dL5 + 0);
	jacobian[3] = s5 * s6 * (m_dL5 + 0);
	jacobian[4] = c6 * (m_dL5 + 0);
	jacobian[5] = 0;

	// vy
	jacobian[6] =
		+ (c2 * s4 * c5 * s6 - c2 * c4 * c6) * m_dL2
		+ (s23 * s4 * c5 * s6 - s23 * c4 * c6) * (m_dL3 + m_dL4)
		- (s23 * c4 * c5 * c6 + c23 * s5 * c6 - s23 * s4 * s6) * (m_dL5 + 0);
	jacobian[7] = 
		- (s3 * c4 * c5 * s6 + c3 * s5 * s6 + s3 * s4 * c6) * m_dL2
		- (c4 * c5 * s6 + s4 * c6) * (m_dL3 + m_dL4)
		- (c4 * s6 + s4 * c5 * c6) * (m_dL5 + 0);
	jacobian[8] =
		- (c4 * c5 * s6 + s4 * c6) * (m_dL3 + m_dL4)
		- (c4 * s6 + s4 * c5 * c6) * (m_dL5 + 0);
	jacobian[9] = s5 * c6 * (m_dL5 + 0);
	jacobian[10] = - s6 * (m_dL5 + 0);
	jacobian[11] = 0;

	// vz
	jacobian[12] = - c2 * c4 * s5 * m_dL2 - s23 * c4 * s5 * (m_dL3 + m_dL4);
	jacobian[13] = (s3 * c4 * s5 - c3 * c5) * m_dL2 + c4 * s5 * (m_dL3 + m_dL4);
	jacobian[14] = + c4 * s5 * (m_dL3 + m_dL4);
	jacobian[15] = 0;
	jacobian[16] = 0;
	jacobian[17] = 0;

	// wx	
	jacobian[18] = + s23 * c4 * c5 * c6 + c23 * s5 * c6 - s23 * s4 * s6;
	jacobian[19] = s4 * c5 * c6 + c4 * s6;
	jacobian[20] = jacobian[19];
	jacobian[21] = - s5 * c6;
	jacobian[22] = s6;
	jacobian[23] = 0;

	// wy
	jacobian[24] = - (s23 * c4 * c5 * s6 + c23 * s5 * s6 + s23 * s4 * c6);
	jacobian[25] = - s4 * c5 * s6 + c4 * c6;
	jacobian[26] = jacobian[25];
	jacobian[27] = s5 * s6;
	jacobian[28] = c6;
	jacobian[29] = 0;

	// wz
	jacobian[30] = + s23 * c4 * s5 - c23 * c5;
	jacobian[31] = s4 * s5;
	jacobian[32] = s4 * s5;
	jacobian[33] = c5;
	jacobian[34] = 0;
	jacobian[35] = 1;

	// �ſ˱Ⱦ����ʼ��
	lm_jac.Init(6, 6, jacobian);

	Matrix lm_inv_jac(6, 6); // �ſ˱������

	if (0 != Inv(&lm_jac, &lm_inv_jac))
	{
		return ERR_NOINV;
	}

	lm_jv = lm_inv_jac * lm_vel;

	// ת��Ϊ�Ƕ�
	gdJVel[0] = lm_jv.Get(0, 0) * PI_DEG;
	gdJVel[1] = lm_jv.Get(1, 0) * PI_DEG;
	gdJVel[2] = lm_jv.Get(2, 0) * PI_DEG;
	gdJVel[3] = lm_jv.Get(3, 0) * PI_DEG;
	gdJVel[4] = lm_jv.Get(4, 0) * PI_DEG;
	//gdJVel[5] = lm_jv.Get(5, 0) * PI_DEG;
*/

	return Ok;
}

/******************************************************************************
 * ������Set_Length()
 * ���ܣ����ø˳�
 *
 * ���룺double gdLen[] - �˳�L1,L2,L3,L4,L5,L6
 ******************************************************************************/
void Kine_IR_SixDoF::Set_Length(double gdLen[])
{
	m_dL1 = gdLen[0] + gdLen[1];
	m_dL2 = gdLen[2];
	m_dL3 = gdLen[3];
	m_dL4 = gdLen[4];
	m_dL5 = gdLen[5];
	m_dL6 = gdLen[6];
}
void Kine_IR_SixDoF::Set_Tool(double gdTool[])
{
	int i;
	for (i=0; i<6; i++)
	{
		m_dTool[i] = 0;//gdTool[i];
	}
}

/******************************************************************************
 * ������FKine()
 * ���ܣ�����
 *
 * ���룺double* gdJPos - �ؽ�ת��, 5�ؽ�
 * �����double* gdCPos - ����λ��, (x,y,z,w,p,r)
 *
 * ���أ�int - 0�ɹ�,
 ******************************************************************************/
int Kine_IR_SixDoF::FKine(double gdJPos[], double gdCPos[])
{
	// �ؽڽǶ� - �ؽڻ��� - ������� - Tcp���� - λ��
	// id_jPos  -   jRad   - lm_Wrist - lm_Tcp  - id_cPos
	double ld_jRad[6];// = gd_rad[0];
	
	double c23, s23;
	
	double ld_temp[8];     // �м����
	MtxKine lm_Wrist;      // �м����,�������
	MtxKine lm_Tool;       // �м����,���߾���
	MtxKine lm_Tcp;        // �м����,TCP����

    //--------------------- �ؽڻ��� -------------------------//
	ld_jRad[0] = gdJPos[0] * PI_RAD;
	ld_jRad[1] = gdJPos[1] * PI_RAD;
	ld_jRad[2] = gdJPos[2] * PI_RAD;
	ld_jRad[3] = gdJPos[3] * PI_RAD;
	ld_jRad[4] = gdJPos[4] * PI_RAD;
	ld_jRad[5] = gdJPos[5] * PI_RAD;

    //--------------------- ������� -------------------------//
    c23 = cos(ld_jRad[1]+ld_jRad[2]);
    s23 = sin(ld_jRad[1]+ld_jRad[2]);	

	ld_temp[1] = cos(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * cos(ld_jRad[4]) 
		+ sin(ld_jRad[0]) * sin(ld_jRad[3]) * cos(ld_jRad[4])
		- cos(ld_jRad[0]) * s23 * sin(ld_jRad[4]);
	ld_temp[2] = - cos(ld_jRad[0]) * c23 * sin(ld_jRad[3])
				 + sin(ld_jRad[0]) * cos(ld_jRad[3]);

    // ����r11 //
	lm_Wrist.R11 = ld_temp[1] * cos(ld_jRad[5])
				 + ld_temp[2] * sin(ld_jRad[5]);
  
     // ����r12 //
	lm_Wrist.R12 = - ld_temp[1] * sin(ld_jRad[5])
				   + ld_temp[2] * cos(ld_jRad[5]);

     // ����r13 //
	lm_Wrist.R13 = cos(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * sin(ld_jRad[4]) 
		+ sin(ld_jRad[0]) * sin(ld_jRad[3]) * sin(ld_jRad[4])
		+ cos(ld_jRad[0]) * s23 * cos(ld_jRad[4]);
	
	ld_temp[1] = sin(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * cos(ld_jRad[4]) 
		- cos(ld_jRad[0]) * sin(ld_jRad[3]) * cos(ld_jRad[4])
		- sin(ld_jRad[0]) * s23 * sin(ld_jRad[4]);
	ld_temp[2] = - sin(ld_jRad[0]) * c23 * sin(ld_jRad[3])
		- cos(ld_jRad[0]) * cos(ld_jRad[3]);

    // ����r21 //
	lm_Wrist.R21 = ld_temp[1] * cos(ld_jRad[5])
				 + ld_temp[2] * sin(ld_jRad[5]);

    // ����r22 //    
	lm_Wrist.R22 = - ld_temp[1] * sin(ld_jRad[5])
				   + ld_temp[2] * cos(ld_jRad[5]);
        
    // ����r23 //    
	lm_Wrist.R23 = sin(ld_jRad[0]) * c23 * cos(ld_jRad[3]) * sin(ld_jRad[4]) 
		- cos(ld_jRad[0]) * sin(ld_jRad[3]) * sin(ld_jRad[4])
		+ sin(ld_jRad[0]) * s23 * cos(ld_jRad[4]);
        
	ld_temp[1] = + s23 * cos(ld_jRad[3]) * cos(ld_jRad[4]) 
		+ c23 * sin(ld_jRad[4]);
	ld_temp[2] = - s23 * sin(ld_jRad[3]);

    // ����r31 //
	lm_Wrist.R31 = ld_temp[1] * cos(ld_jRad[5])
				 + ld_temp[2] * sin(ld_jRad[5]);

    // ����r32 //    
	lm_Wrist.R32 = - ld_temp[1] * sin(ld_jRad[5])
				   + ld_temp[2] * cos(ld_jRad[5]);
    
    // ����r33 //
	lm_Wrist.R33 = + s23 * cos(ld_jRad[3]) * sin(ld_jRad[4]) 
        - c23 * cos(ld_jRad[4]);

    // ����x //
	lm_Wrist.X = lm_Wrist.R13 * (m_dL5 +m_dL6)
		+ cos(ld_jRad[0]) * s23 * (m_dL3 + m_dL4)
		+ cos(ld_jRad[0]) * cos(ld_jRad[1]) * m_dL2;

    // ����y //
	lm_Wrist.Y = lm_Wrist.R23 * (m_dL5 +m_dL6)
		+ sin(ld_jRad[0]) * s23 * (m_dL3 + m_dL4)
		+ sin(ld_jRad[0]) * cos(ld_jRad[1]) * m_dL2;
    
    // ����z //
	lm_Wrist.Z = lm_Wrist.R33 * (m_dL5 +m_dL6)
		- c23 * (m_dL3 + m_dL4)
		+ sin(ld_jRad[1]) * m_dL2;

    //--------------------- Tcp���� -------------------------//
    // ���㹤�ߵı任���� //
	ld_temp[1] = sin(m_dTool[3] * PI_RAD);
	ld_temp[2] = cos(m_dTool[3] * PI_RAD);
	ld_temp[3] = sin(m_dTool[4] * PI_RAD);
	ld_temp[4] = cos(m_dTool[4] * PI_RAD);
	ld_temp[5] = sin(m_dTool[5] * PI_RAD);
	ld_temp[6] = cos(m_dTool[5] * PI_RAD);
	
    lm_Tool.R11 = ld_temp[2] * ld_temp[4];
    lm_Tool.R12 = ld_temp[2] * ld_temp[3] * ld_temp[5] - ld_temp[1] * ld_temp[6];
    lm_Tool.R13 = ld_temp[2] * ld_temp[3] * ld_temp[6] + ld_temp[1] * ld_temp[5];
    lm_Tool.R21 = ld_temp[1] * ld_temp[4];
    lm_Tool.R22 = ld_temp[1] * ld_temp[3] * ld_temp[5] + ld_temp[2] * ld_temp[6];
    lm_Tool.R23 = ld_temp[1] * ld_temp[3] * ld_temp[6] - ld_temp[2] * ld_temp[5];
    lm_Tool.R31 = -ld_temp[3];
    lm_Tool.R32 = ld_temp[4] * ld_temp[5];
    lm_Tool.R33 = ld_temp[4] * ld_temp[6];

    // ����Tcp��̬����Ĳ��� //
    lm_Tcp.R11 = lm_Wrist.R11 * lm_Tool.R11 + lm_Wrist.R12 * lm_Tool.R21 + lm_Wrist.R13 * lm_Tool.R31;
    lm_Tcp.R21 = lm_Wrist.R21 * lm_Tool.R11 + lm_Wrist.R22 * lm_Tool.R21 + lm_Wrist.R23 * lm_Tool.R31;
    lm_Tcp.R31 = lm_Wrist.R31 * lm_Tool.R11 + lm_Wrist.R32 * lm_Tool.R21 + lm_Wrist.R33 * lm_Tool.R31;
    lm_Tcp.R32 = lm_Wrist.R31 * lm_Tool.R12 + lm_Wrist.R32 * lm_Tool.R22 + lm_Wrist.R33 * lm_Tool.R32;
    lm_Tcp.R33 = lm_Wrist.R31 * lm_Tool.R13 + lm_Wrist.R32 * lm_Tool.R23 + lm_Wrist.R33 * lm_Tool.R33;
    lm_Tcp.R12 = lm_Wrist.R11 * lm_Tool.R12 + lm_Wrist.R12 * lm_Tool.R22 + lm_Wrist.R13 * lm_Tool.R32;
    lm_Tcp.R22 = lm_Wrist.R21 * lm_Tool.R12 + lm_Wrist.R22 * lm_Tool.R22 + lm_Wrist.R23 * lm_Tool.R32;

    // ����Tcpλ�� //
	lm_Tcp.X = lm_Wrist.X + lm_Wrist.R11 * (m_dTool[0])
                          + lm_Wrist.R12 * (m_dTool[1])
                          + lm_Wrist.R13 * (m_dTool[2]);
	lm_Tcp.Y = lm_Wrist.Y + lm_Wrist.R21 * (m_dTool[0])
                          + lm_Wrist.R22 * (m_dTool[1])
                          + lm_Wrist.R23 * (m_dTool[2]);
	lm_Tcp.Z = lm_Wrist.Z + lm_Wrist.R31 * (m_dTool[0])
                          + lm_Wrist.R32 * (m_dTool[1])
                          + lm_Wrist.R33 * (m_dTool[2]);

    //--------------------- Tcpλ�� -------------------------//
    // ���㹤������ϵRPY�� - rad //
	ld_temp[5] = atan2(-lm_Tcp.R31, sqrt(lm_Tcp.R11 * lm_Tcp.R11 + lm_Tcp.R21 * lm_Tcp.R21));

    if (fabs(ld_temp[5] - PI / 2) < RT_LITTLE)
    {
        ld_temp[4] = 0;
        ld_temp[6] = atan2(lm_Tcp.R12, lm_Tcp.R22);
    }
    else if (fabs(ld_temp[5] + PI / 2) < RT_LITTLE)
    {
        ld_temp[4] = 0;
        ld_temp[6] = -atan2(lm_Tcp.R12, lm_Tcp.R22);
    }
    else
    {
        ld_temp[0] = 1 / cos(ld_temp[5]);

		ld_temp[1] = lm_Tcp.R21 * ld_temp[0];
		ld_temp[2] = lm_Tcp.R11 * ld_temp[0];
		if (fabs(ld_temp[1]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		if (fabs(ld_temp[2]) < RT_LITTLE)
		{
			ld_temp[2] = 0;
		}
        ld_temp[4] = atan2(ld_temp[1], ld_temp[2]);

		ld_temp[1] = lm_Tcp.R32 * ld_temp[0];
		ld_temp[2] = lm_Tcp.R33 * ld_temp[0];
		if (fabs(ld_temp[1]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		if (fabs(ld_temp[2]) < RT_LITTLE)
		{
			ld_temp[2] = 0;
		}
        ld_temp[6] = atan2(ld_temp[1], ld_temp[2]);
    }

	gdCPos[0] = lm_Tcp.X;      // mm   
	gdCPos[1] = lm_Tcp.Y;      // mm
	gdCPos[2] = lm_Tcp.Z + m_dL1; // mm + L1
	gdCPos[3] = ld_temp[4] * PI_DEG;   // deg
	gdCPos[4] = ld_temp[5] * PI_DEG;   // deg
	gdCPos[5] = ld_temp[6] * PI_DEG;   // deg

	return Ok;
}

/******************************************************************************
 * ������IKine()
 * ���ܣ����
 *
 * ���룺double* gdCPos  - λ������, (x,y,z,w,p,r)
 *       double* gdJCurr - ��ǰ�ؽ�ת��, 5�ؽ�
 * �����double* id_jPos  - ���ؽ�ת��, 5�ؽ�
 *
 * ���أ�int - 0�ɹ�, ��������
 ******************************************************************************/
int Kine_IR_SixDoF::IKine(double gdCPos[], double gdJCurr[], double gdJPos[])
{
	// λ�� - Tcp���� - ������� - �ؽڻ��� - �ؽڽǶ�
	// id_cPos - lm_Tcp - lm_Wrist    - ld_jRad     - id_jPos
	int i;
	int result;
	int li_flag[8] = {0};      // ���������,0Ϊ�н�

	double s1,c1,s3,c3, s4, c4, s5, c5, s23, c23, s6, c6;

	double ld_temp[8];       // �м����
	MtxKine lm_Wrist;        // �м����,�������
	MtxKine lm_Tool;         // �м����,���߾���
	MtxKine lm_Tcp;          // �м����,TCP����
	double gd_rad[8][6];     // �м�������������

    //--------------------- TCP���� -------------------------//
	ld_temp[1] = sin(gdCPos[3] * PI_RAD);
	ld_temp[2] = cos(gdCPos[3] * PI_RAD);
	ld_temp[3] = sin(gdCPos[4] * PI_RAD);
	ld_temp[4] = cos(gdCPos[4] * PI_RAD);
	ld_temp[5] = sin(gdCPos[5] * PI_RAD);
	ld_temp[6] = cos(gdCPos[5] * PI_RAD);

	lm_Tcp.R11 = ld_temp[2] * ld_temp[4];
    lm_Tcp.R12 = ld_temp[2] * ld_temp[3] * ld_temp[5] - ld_temp[1] * ld_temp[6];
    lm_Tcp.R13 = ld_temp[2] * ld_temp[3] * ld_temp[6] + ld_temp[1] * ld_temp[5];
    lm_Tcp.R21 = ld_temp[1] * ld_temp[4];
    lm_Tcp.R22 = ld_temp[1] * ld_temp[3] * ld_temp[5] + ld_temp[2] * ld_temp[6];
    lm_Tcp.R23 = ld_temp[1] * ld_temp[3] * ld_temp[6] - ld_temp[2] * ld_temp[5];
    lm_Tcp.R31 = -ld_temp[3];
    lm_Tcp.R32 = ld_temp[4] * ld_temp[5];
    lm_Tcp.R33 = ld_temp[4] * ld_temp[6];

    //--------------------- ���߾��� -------------------------//
    // ���㹤�ߵı任���� //
	ld_temp[1] = sin(m_dTool[3] * PI_RAD);
	ld_temp[2] = cos(m_dTool[3] * PI_RAD);
	ld_temp[3] = sin(m_dTool[4] * PI_RAD);
	ld_temp[4] = cos(m_dTool[4] * PI_RAD);
	ld_temp[5] = sin(m_dTool[5] * PI_RAD);
	ld_temp[6] = cos(m_dTool[5] * PI_RAD);
	
    lm_Tool.R11 = ld_temp[2] * ld_temp[4];
    lm_Tool.R12 = ld_temp[2] * ld_temp[3] * ld_temp[5] - ld_temp[1] * ld_temp[6];
    lm_Tool.R13 = ld_temp[2] * ld_temp[3] * ld_temp[6] + ld_temp[1] * ld_temp[5];
    lm_Tool.R21 = ld_temp[1] * ld_temp[4];
    lm_Tool.R22 = ld_temp[1] * ld_temp[3] * ld_temp[5] + ld_temp[2] * ld_temp[6];
    lm_Tool.R23 = ld_temp[1] * ld_temp[3] * ld_temp[6] - ld_temp[2] * ld_temp[5];
    lm_Tool.R31 = -ld_temp[3];
    lm_Tool.R32 = ld_temp[4] * ld_temp[5];
    lm_Tool.R33 = ld_temp[4] * ld_temp[6];

    // ��̬���� //
    ld_temp[1] = lm_Tool.R12;
    lm_Tool.R12 = lm_Tool.R21;
    lm_Tool.R21 = ld_temp[1];

    ld_temp[1] = lm_Tool.R13;
    lm_Tool.R13 = lm_Tool.R31;
    lm_Tool.R31 = ld_temp[1];

    ld_temp[1] = lm_Tool.R23;
    lm_Tool.R23 = lm_Tool.R32;
    lm_Tool.R32 = ld_temp[1];
    
	// λ������ //
	lm_Tool.X = -( lm_Tool.R11 * m_dTool[0] + 
				   lm_Tool.R12 * m_dTool[1] +
                   lm_Tool.R13 * m_dTool[2] );
	lm_Tool.Y = -( lm_Tool.R21 * m_dTool[0] + 
				   lm_Tool.R22 * m_dTool[1] +
                   lm_Tool.R23 * m_dTool[2] );
	lm_Tool.Z = -( lm_Tool.R31 * m_dTool[0] + 
				   lm_Tool.R32 * m_dTool[1] +
                   lm_Tool.R33 * m_dTool[2] );

    //--------------------- ������� -------------------------//
    // ��̬���� //
    lm_Wrist.R11 = lm_Tcp.R11 * lm_Tool.R11 + lm_Tcp.R12 * lm_Tool.R21 + lm_Tcp.R13 * lm_Tool.R31;
    lm_Wrist.R12 = lm_Tcp.R11 * lm_Tool.R12 + lm_Tcp.R12 * lm_Tool.R22 + lm_Tcp.R13 * lm_Tool.R32;
    lm_Wrist.R13 = lm_Tcp.R11 * lm_Tool.R13 + lm_Tcp.R12 * lm_Tool.R23 + lm_Tcp.R13 * lm_Tool.R33;
    lm_Wrist.R21 = lm_Tcp.R21 * lm_Tool.R11 + lm_Tcp.R22 * lm_Tool.R21 + lm_Tcp.R23 * lm_Tool.R31;
    lm_Wrist.R22 = lm_Tcp.R21 * lm_Tool.R12 + lm_Tcp.R22 * lm_Tool.R22 + lm_Tcp.R23 * lm_Tool.R32;
    lm_Wrist.R23 = lm_Tcp.R21 * lm_Tool.R13 + lm_Tcp.R22 * lm_Tool.R23 + lm_Tcp.R23 * lm_Tool.R33;
    lm_Wrist.R31 = lm_Tcp.R31 * lm_Tool.R11 + lm_Tcp.R32 * lm_Tool.R21 + lm_Tcp.R33 * lm_Tool.R31;
    lm_Wrist.R32 = lm_Tcp.R31 * lm_Tool.R12 + lm_Tcp.R32 * lm_Tool.R22 + lm_Tcp.R33 * lm_Tool.R32;
    lm_Wrist.R33 = lm_Tcp.R31 * lm_Tool.R13 + lm_Tcp.R32 * lm_Tool.R23 + lm_Tcp.R33 * lm_Tool.R33;

    // λ�� //
	lm_Wrist.X = lm_Tcp.R11 * lm_Tool.X + lm_Tcp.R12 * lm_Tool.Y + lm_Tcp.R13 * lm_Tool.Z
		+ gdCPos[0] - lm_Wrist.R13 * (m_dL5 + m_dL6);
	lm_Wrist.Y = lm_Tcp.R21 * lm_Tool.X + lm_Tcp.R22 * lm_Tool.Y + lm_Tcp.R23 * lm_Tool.Z
		+ gdCPos[1] - lm_Wrist.R23 * (m_dL5 + m_dL6);
	lm_Wrist.Z = lm_Tcp.R31 * lm_Tool.X + lm_Tcp.R32 * lm_Tool.Y + lm_Tcp.R33 * lm_Tool.Z
		+ (gdCPos[2] - m_dL1) - lm_Wrist.R33 * (m_dL5 + m_dL6);

    //--------------------- �ؽڻ��� -------------------------//
    //------ ת��1 -------//
	ld_temp[0] = atan2(lm_Wrist.Y, lm_Wrist.X);
	ld_temp[1] = atan2(- lm_Wrist.Y, - lm_Wrist.X);
	RadInRange(&ld_temp[0], &gdJCurr[0]);
	RadInRange(&ld_temp[1], &gdJCurr[0]);

	gd_rad[0][0] = gd_rad[1][0] = ld_temp[0];
	gd_rad[2][0] = gd_rad[3][0] = ld_temp[1];

    //------ ת��3 ------//
    for (i=0; i<2; i++)
    {
        s1 = sin(gd_rad[2*i][0]);
        c1 = cos(gd_rad[2*i][0]);
	
		ld_temp[0] = c1 * lm_Wrist.X + s1 * lm_Wrist.Y;
		ld_temp[1] = ld_temp[0] * ld_temp[0] + lm_Wrist.Z * lm_Wrist.Z
			- m_dL2 * m_dL2 - (m_dL3 + m_dL4) * (m_dL3 + m_dL4);

		ld_temp[0] = ld_temp[1] / (2 * m_dL2 * (m_dL3 + m_dL4));
		ld_temp[1] = ld_temp[0] * ld_temp[0];

        if (ld_temp[1] <= 1)
        {
            ld_temp[2] = sqrt(1 - ld_temp[1]);

			ld_temp[3] = atan2(ld_temp[0], ld_temp[2]);
			ld_temp[4] = atan2(ld_temp[0],-ld_temp[2]);
			RadInRange(&ld_temp[3], &gdJCurr[2]);
			RadInRange(&ld_temp[4], &gdJCurr[2]);

            gd_rad[2*i][2]   = ld_temp[3];
            gd_rad[2*i+1][2] = ld_temp[4];
        }
		else if (ld_temp[1] - 1 < RT_LITTLE)
		{
            ld_temp[2] = 0;

			ld_temp[3] = atan2(ld_temp[0], ld_temp[2]);
			ld_temp[4] = atan2(ld_temp[0],-ld_temp[2]);
			RadInRange(&ld_temp[3], &gdJCurr[2]);
			RadInRange(&ld_temp[4], &gdJCurr[2]);

            gd_rad[2*i][2]   = ld_temp[3];
            gd_rad[2*i+1][2] = ld_temp[4];
		}
        else
        {
			li_flag[2*i]   = 1;     // ��־ - ������޽�
			li_flag[2*i+1] = 1;
        }
    }
    if (li_flag[0] && li_flag[2])
    {
        return ERR_NOINV; //  ���ؽ��޿������ֵ //
    }

    //------ ת��2,4,5,6 ------//
    for (i=0; i<4; i++)
    {
		if(li_flag[i] == 0)
		{
			// ת��1��ת��3��������ֵ //
			s1 = sin(gd_rad[i][0]);
			c1 = cos(gd_rad[i][0]);
			s3 = sin(gd_rad[i][2]);
			c3 = cos(gd_rad[i][2]);

			// ����ת��2��ת��3�ĺ� //
			ld_temp[0] = c1 * lm_Wrist.X + s1 * lm_Wrist.Y;
			ld_temp[1] = m_dL3 + m_dL4 + s3 * m_dL2;

			ld_temp[2] = 1 / (ld_temp[0] * ld_temp[0] + lm_Wrist.Z * lm_Wrist.Z);

			s23 = (ld_temp[0] * ld_temp[1] + lm_Wrist.Z * c3 * m_dL2) * ld_temp[2];
			c23 = (ld_temp[0] * c3 * m_dL2 - lm_Wrist.Z * ld_temp[1]) * ld_temp[2];
    
			// ����ת��2 //
			ld_temp[5] = atan2(s23, c23) - gd_rad[i][2];
			RadInRange(&ld_temp[5], &gdJCurr[1]);
			gd_rad[i][1] = ld_temp[5];

			// ����ת��4 //			
			s4 = + lm_Wrist.R13 * s1 - lm_Wrist.R23 * c1;
			c4 = + lm_Wrist.R13 * c1 * c23 + lm_Wrist.R23 * s1 * c23 + lm_Wrist.R33 * s23;
			ld_temp[5] = atan2(s4, c4);
			RadInRange(&ld_temp[5], &gdJCurr[3]);
			gd_rad[i][3] = ld_temp[5];

			// ����ת��5 //
			s4 = sin(gd_rad[i][3]);
			c4 = cos(gd_rad[i][3]);

			s5 =   lm_Wrist.R13 * (c1 * c23 * c4 + s1 * s4)
				 + lm_Wrist.R23 * (s1 * c23 * c4 - c1 * s4)
				 + lm_Wrist.R33 * (+ s23 * c4);
			c5 =   lm_Wrist.R13 * c1 * s23
				 + lm_Wrist.R23 * s1 * s23
				 - lm_Wrist.R33 * c23;
			ld_temp[5] = atan2(s5, c5);
			RadInRange(&ld_temp[5], &gdJCurr[4]);
			gd_rad[i][4] = ld_temp[5];

			// ����ת��6 //
			s6 =  lm_Wrist.R11 * (- c1 * c23 * s4 + s1 * c4)
				+ lm_Wrist.R21 * (- s1 * c23 * s4 - c1 * c4)
				+ lm_Wrist.R31 * (- s23 * s4);
			c6 =  lm_Wrist.R12 * (- c1 * c23 * s4 + s1 * c4)
				+ lm_Wrist.R22 * (- s1 * c23 * s4 - c1 * c4)
				+ lm_Wrist.R32 * (- s23 * s4);

			ld_temp[6] = atan2(s6, c6);
			RadInRange(&ld_temp[6], &gdJCurr[5]);
			gd_rad[i][5] = ld_temp[6];

			// ��4��6�ؽڡ���ת�� //
			gd_rad[i+4][0] = gd_rad[i][0];
			gd_rad[i+4][1] = gd_rad[i][1];
			gd_rad[i+4][2] = gd_rad[i][2];
			ld_temp[5] = gd_rad[i][3] + PI;
			RadInRange(&ld_temp[5], &gdJCurr[3]);
			gd_rad[i+4][3] = ld_temp[5];
			ld_temp[5] = - gd_rad[i][4];
			RadInRange(&ld_temp[5], &gdJCurr[4]);
			gd_rad[i+4][4] = ld_temp[5];
			ld_temp[5] = gd_rad[i][5] + PI;
			RadInRange(&ld_temp[5], &gdJCurr[5]);
			gd_rad[i+4][5] = ld_temp[5];

		}
	}
	
	//------ ��ѽ�� ------//
	for (i=0; i<8; i++)
	{
		if (0 == li_flag[i])  // ��ȡ��Ծ���ֵ
		{
			ld_temp[i] = fabs(gd_rad[i][0] - gdJCurr[0] * PI_RAD) + 
			      	     fabs(gd_rad[i][1] - gdJCurr[1] * PI_RAD) + 
					     fabs(gd_rad[i][2] - gdJCurr[2] * PI_RAD) + 
					     fabs(gd_rad[i][3] - gdJCurr[3] * PI_RAD) +
					     fabs(gd_rad[i][4] - gdJCurr[4] * PI_RAD);
		}
	}
	for (i=0; i<8; i++)
	{
		if (0 == li_flag[i])
		{
			s1 = ld_temp[i]; // �õ�һ����Чֵ �� ��ʼ�� �м����s1
			result = i;
			break;           // �Ƴ���ʼ��
		}
	}
	for (i=0; i<8; i++)
	{
		if ((0 == li_flag[i]) && (ld_temp[i] <= s1))  // ��Чֵ | ��Ծ���ֵ��С
		{
			//ld_jRad = gd_rad[i];
			s1 = ld_temp[i];
			result = i;
		}
	}
	//------ �ؽڽǶ� ------//
	gdJPos[0] = gd_rad[result][0] * PI_DEG;
	gdJPos[1] = gd_rad[result][1] * PI_DEG;
	gdJPos[2] = gd_rad[result][2] * PI_DEG;
	gdJPos[3] = gd_rad[result][3] * PI_DEG;
	gdJPos[4] = gd_rad[result][4] * PI_DEG;
	gdJPos[5] = gd_rad[result][5] * PI_DEG;

	return Ok;
}
/******************************************************************************
 * ������Vel_FKine()
 * ���ܣ��ٶ����, ��������ϵ�ٶ�
 *
 * ���룺double* gdJPos - ��ǰ�ؽ�ת��, 5�ؽ�, deg
 *       double* gdJVel - ��ǰ�ؽ��ٶ�, 5�ؽ�, deg/s
 * �����double* gdCVel - ĩ���ٶ�, [vx,vy,vz,wx,wy,wz], mm/s, deg/s
 *
 * ���أ�int - 0�ɹ�, ��������
 ******************************************************************************/
int Kine_IR_SixDoF::Vel_FKine(IN double gdJPos[6], IN double gdJVel[6], OUT double gdCVel[6])
{
	int i;
	double p[6], v[6];

	for (i=0; i<6; i++)
	{
		p[i] = gdJPos[i] * PI_RAD;
		v[i] = gdJVel[i] * PI_RAD;
	}

	double c2 = cos(p[1]);
	double s3 = sin(p[2]);
	double c3 = cos(p[2]);
	double s23 = sin(p[1] + p[2]);
	double c23 = cos(p[1] + p[2]);
	double s4 = sin(p[3]);
	double c4 = cos(p[3]);
	double s5 = sin(p[4]);
	double c5 = cos(p[4]);
	double s6 = sin(p[5]);
	double c6 = cos(p[5]);

	double temp[5];
	temp[0] =
		- (c2 * s4 * c5 * c6 + c2 * c4 * s6) * m_dL2
		- (s23 * s4 * c5 * c6 + s23 * c4 * s6) * (m_dL3 + m_dL4)
		- (s23 * c4 * c5 * s6 + c23 * s5 * s6 + s23 * s4 * c6) * (m_dL5 + m_dL6);
	temp[1] = 
		+ (s3 * c4 * c5 * c6 + c3 * s5 * c6 - s3 * s4 * s6) * m_dL2
		+ (c4 * c5 * c6 - s4 * s6) * (m_dL3 + m_dL4)
		+ (c4 * c6 - s4 * c5 * s6) * (m_dL5 + m_dL6);
	temp[2] =
		+ (c4 * c5 * c6 - s4 * s6) * (m_dL3 + m_dL4)
		+ (c4 * c6 - s4 * c5 * s6) * (m_dL5 + m_dL6);
	temp[3] = s5 * s6 * (m_dL5 + m_dL6);
	temp[4] = c6 * (m_dL5 + m_dL6);

	gdCVel[0] = temp[0] * v[0] + temp[1] * v[1] + temp[2] * v[2] + temp[3] * v[3] + temp[4] * v[4];

	temp[0] =
		+ (c2 * s4 * c5 * s6 - c2 * c4 * c6) * m_dL2
		+ (s23 * s4 * c5 * s6 - s23 * c4 * c6) * (m_dL3 + m_dL4)
		- (s23 * c4 * c5 * c6 + c23 * s5 * c6 - s23 * s4 * s6) * (m_dL5 + m_dL6);
	temp[1] = 
		- (s3 * c4 * c5 * s6 + c3 * s5 * s6 + s3 * s4 * c6) * m_dL2
		- (c4 * c5 * s6 + s4 * c6) * (m_dL3 + m_dL4)
		- (c4 * s6 + s4 * c5 * c6) * (m_dL5 + m_dL6);
	temp[2] =
		- (c4 * c5 * s6 + s4 * c6) * (m_dL3 + m_dL4)
		- (c4 * s6 + s4 * c5 * c6) * (m_dL5 + m_dL6);
	temp[3] = s5 * c6 * (m_dL5 + m_dL6);
	temp[4] = - s6 * (m_dL5 + m_dL6);

	gdCVel[1] = temp[0] * v[0] + temp[1] * v[1] + temp[2] * v[2] + temp[3] * v[3] + temp[4] * v[4];

	temp[0] = - c2 * c4 * s5 * m_dL2 - s23 * c4 * s5 * (m_dL3 + m_dL4);
	temp[1] = (s3 * c4 * s5 - c3 * c5) * m_dL2 + c4 * s5 * (m_dL3 + m_dL4);
	temp[2] = + c4 * s5 * (m_dL3 + m_dL4);

	gdCVel[2] = temp[0] * v[0] + temp[1] * v[1] + temp[2] * v[2];

	temp[0] = + (s23 * c4 * c5 * c6 + c23 * s5 * c6 - s23 * s4 * s6);
	temp[1] = s4 * c5 * c6 + c4 * s6;
	temp[2] = temp[1];

	gdCVel[3] = temp[0] * v[0] + temp[1] * v[1] + temp[2] * v[2]
		- s5 * c6 * v[3] + s6 * v[4];

	temp[0] = - (s23 * c4 * c5 * s6 + c23 * s5 * s6 + s23 * s4 * c6);
	temp[1] = - s4 * c5 * s6 + c4 * c6;
	temp[2] = temp[1];

	gdCVel[4] = temp[0] * v[0] + temp[1] * v[1] + temp[2] * v[2]
		+ s5 * s6 * v[3] + c6 * v[4];

	gdCVel[5] = + (s23 * c4 * s5 - c23 * c5) * v[0]
		+ s4 * s5 * v[1]
		+ s4 * s5 * v[2]
		+ c5 * v[3]
		+ v[5];
	
	// ת��Ϊ�Ƕ�
	gdCVel[3] *= PI_DEG;
	gdCVel[4] *= PI_DEG;
	gdCVel[5] *= PI_DEG;

	return Ok;
}
/******************************************************************************
 * ������Vel_IKine()
 * ���ܣ��ٶ����
 *
 * ���룺double gdJPos[] - ��ǰ�ؽ�ת��, 5�ؽ�, deg
 *       double gdCVel[] - ��ǰĩ���ٶ�, [vx,vy,vz,wx,wy,wz], mm/s, deg/s
 * �����double gdJVel[] - �ؽ��ٶ�, 5�ؽ�, deg/s
 *
 * ���أ�int - 0�ɹ�, ��������
 ******************************************************************************/
int Kine_IR_SixDoF::Vel_IKine(IN double gdJPos[6], IN double gdCVel[6], OUT double gdJVel[6])
{
	double c2 = cos(gdJPos[1] * PI_RAD);  // ת��Ϊ����
	double s23 = sin((gdJPos[1] + gdJPos[2]) * PI_RAD);
	double c23 = cos((gdJPos[1] + gdJPos[2]) * PI_RAD);
	double s3 = sin(gdJPos[2] * PI_RAD);
	double c3 = cos(gdJPos[2] * PI_RAD);
	double s4 = sin(gdJPos[3] * PI_RAD);
	double c4 = cos(gdJPos[3] * PI_RAD);
	double s5 = sin(gdJPos[4] * PI_RAD);
	double c5 = cos(gdJPos[4] * PI_RAD);
	double s6 = sin(gdJPos[5] * PI_RAD);
	double c6 = cos(gdJPos[5] * PI_RAD);
	
	double vel[6];       // �ѿ����ٶ�
	double jacobian[36]; // �ſ˱Ⱦ�������

	Matrix lm_vel;   // TCP�ٶȾ���
	Matrix lm_jac;   // �ſ˱Ⱦ���
	Matrix lm_jv(6, 1);    // �ؽ��ٶȾ���

	// TCP�ٶ�
	vel[0] = gdCVel[0];
	vel[1] = gdCVel[1];
	vel[2] = gdCVel[2];
	vel[3] = gdCVel[3] * PI_RAD;
	vel[4] = gdCVel[4] * PI_RAD;
	vel[5] = gdCVel[5] * PI_RAD;

	// TCP�ٶȾ����ʼ��
	lm_vel.Init(6, 1, vel);

	// vx
	jacobian[0] = 
		- (c2 * s4 * c5 * c6 + c2 * c4 * s6) * m_dL2
		- (s23 * s4 * c5 * c6 + s23 * c4 * s6) * (m_dL3 + m_dL4)
		- (s23 * c4 * c5 * s6 + c23 * s5 * s6 + s23 * s4 * c6) * (m_dL5 + m_dL6);
	jacobian[1] = 
		+ (s3 * c4 * c5 * c6 + c3 * s5 * c6 - s3 * s4 * s6) * m_dL2
		+ (c4 * c5 * c6 - s4 * s6) * (m_dL3 + m_dL4)
		+ (c4 * c6 - s4 * c5 * s6) * (m_dL5 + m_dL6);
	jacobian[2] = 
		+ (c4 * c5 * c6 - s4 * s6) * (m_dL3 + m_dL4)
		+ (c4 * c6 - s4 * c5 * s6) * (m_dL5 + m_dL6);
	jacobian[3] = s5 * s6 * (m_dL5 + m_dL6);
	jacobian[4] = c6 * (m_dL5 + m_dL6);
	jacobian[5] = 0;

	// vy
	jacobian[6] =
		+ (c2 * s4 * c5 * s6 - c2 * c4 * c6) * m_dL2
		+ (s23 * s4 * c5 * s6 - s23 * c4 * c6) * (m_dL3 + m_dL4)
		- (s23 * c4 * c5 * c6 + c23 * s5 * c6 - s23 * s4 * s6) * (m_dL5 + m_dL6);
	jacobian[7] = 
		- (s3 * c4 * c5 * s6 + c3 * s5 * s6 + s3 * s4 * c6) * m_dL2
		- (c4 * c5 * s6 + s4 * c6) * (m_dL3 + m_dL4)
		- (c4 * s6 + s4 * c5 * c6) * (m_dL5 + m_dL6);
	jacobian[8] =
		- (c4 * c5 * s6 + s4 * c6) * (m_dL3 + m_dL4)
		- (c4 * s6 + s4 * c5 * c6) * (m_dL5 + m_dL6);
	jacobian[9] = s5 * c6 * (m_dL5 + m_dL6);
	jacobian[10] = - s6 * (m_dL5 + m_dL6);
	jacobian[11] = 0;

	// vz
	jacobian[12] = - c2 * c4 * s5 * m_dL2 - s23 * c4 * s5 * (m_dL3 + m_dL4);
	jacobian[13] = (s3 * c4 * s5 - c3 * c5) * m_dL2 + c4 * s5 * (m_dL3 + m_dL4);
	jacobian[14] = + c4 * s5 * (m_dL3 + m_dL4);
	jacobian[15] = 0;
	jacobian[16] = 0;
	jacobian[17] = 0;

	// wx	
	jacobian[18] = + s23 * c4 * c5 * c6 + c23 * s5 * c6 - s23 * s4 * s6;
	jacobian[19] = s4 * c5 * c6 + c4 * s6;
	jacobian[20] = jacobian[19];
	jacobian[21] = - s5 * c6;
	jacobian[22] = s6;
	jacobian[23] = 0;

	// wy
	jacobian[24] = - (s23 * c4 * c5 * s6 + c23 * s5 * s6 + s23 * s4 * c6);
	jacobian[25] = - s4 * c5 * s6 + c4 * c6;
	jacobian[26] = jacobian[25];
	jacobian[27] = s5 * s6;
	jacobian[28] = c6;
	jacobian[29] = 0;

	// wz
	jacobian[30] = + s23 * c4 * s5 - c23 * c5;
	jacobian[31] = s4 * s5;
	jacobian[32] = s4 * s5;
	jacobian[33] = c5;
	jacobian[34] = 0;
	jacobian[35] = 1;

	// �ſ˱Ⱦ����ʼ��
	lm_jac.Init(6, 6, jacobian);

	Matrix lm_inv_jac(6, 6); // �ſ˱������

	if (0 != Inv(&lm_jac, &lm_inv_jac))
	{
		return ERR_NOINV;
	}

	lm_jv = lm_inv_jac * lm_vel;

	// ת��Ϊ�Ƕ�
	gdJVel[0] = lm_jv.Get(0, 0) * PI_DEG;
	gdJVel[1] = lm_jv.Get(1, 0) * PI_DEG;
	gdJVel[2] = lm_jv.Get(2, 0) * PI_DEG;
	gdJVel[3] = lm_jv.Get(3, 0) * PI_DEG;
	gdJVel[4] = lm_jv.Get(4, 0) * PI_DEG;
	gdJVel[5] = lm_jv.Get(5, 0) * PI_DEG;
	
	return Ok;
}

/******************************************************************************
 * ������Set_Length()
 * ���ܣ����ø˳�
 *
 * ���룺gdLen - �˳�L1��L2��L3��L4��L5
 ******************************************************************************/
void Kine_CR_FiveDoF_G1::Set_Length(double gdLen[5])
{
	m_dL1 = gdLen[0] + gdLen[1];
	m_dL2 = gdLen[2];
	m_dL3 = gdLen[3];
	m_dL4 = gdLen[4];
	m_dL5 = gdLen[5];
}

/******************************************************************************
 * ������FKine()
 * ���ܣ�����
 *
 * ���룺double gdJPos[] - �ؽ�ת��, 5�ؽ�
 * �����double gdCPos[] - ����λ��, (x,y,z,w,p,r)
 *
 * ���أ�int - 0�ɹ�,
 ******************************************************************************/
int Kine_CR_FiveDoF_G1::FKine(double gdJPos[], double gdCPos[])
{
	// �ؽڽǶ� - �ؽڻ��� - Tcp���� - λ��
	// id_jPos  -   jRad   - lm_Tcp  - id_cPos
	int i;
	double jrad[5]; // �ؽڻ���
		
	double ld_temp[8];     // �м����
	MtxKine lm_Tcp;        // �м����,TCP����

    //--------------------- �ؽڻ��� -------------------------//
	for (i=0; i<5; i++)
	{
		jrad[i] = gdJPos[i] * PI_RAD;
	}

    //--------------------- Tcp���� -------------------------//
	double c23 = cos(jrad[1] + jrad[2]);
	double s23 = sin(jrad[1] + jrad[2]);
	double c234 = cos(jrad[1] + jrad[2] + jrad[3]);
	double s234 = sin(jrad[1] + jrad[2] + jrad[3]);

    // ����Tcp��̬����Ĳ��� //
    lm_Tcp.R11 =   cos(jrad[0]) * c234 * cos(jrad[4]) + sin(jrad[0]) * sin(jrad[4]);
	lm_Tcp.R12 = - cos(jrad[0]) * c234 * sin(jrad[4]) + sin(jrad[0]) * cos(jrad[4]);
	lm_Tcp.R13 =   cos(jrad[0]) * s234;
	lm_Tcp.R21 =   sin(jrad[0]) * c234 * cos(jrad[4]) - cos(jrad[0]) * sin(jrad[4]);
	lm_Tcp.R22 = - sin(jrad[0]) * c234 * sin(jrad[4]) - cos(jrad[0]) * cos(jrad[4]);
	lm_Tcp.R23 =   sin(jrad[0]) * s234;
	lm_Tcp.R31 =   s234 * cos(jrad[4]);
	lm_Tcp.R32 = - s234 * sin(jrad[4]);
	lm_Tcp.R33 = - c234;

    // ����Tcpλ�� //
	lm_Tcp.X =  lm_Tcp.R13 * (m_dL4 + m_dL5)
			  + cos(jrad[0]) * c23 * m_dL3
			  + cos(jrad[0]) * cos(jrad[1]) * m_dL2;
	lm_Tcp.Y =  lm_Tcp.R23 * (m_dL4 + m_dL5)
			  + sin(jrad[0]) * c23 * m_dL3
			  + sin(jrad[0]) * cos(jrad[1]) * m_dL2;
	lm_Tcp.Z =  lm_Tcp.R33 * (m_dL4 + m_dL5)
			  + s23 * m_dL3
			  + sin(jrad[1]) * m_dL2
			  + m_dL1;
    //--------------------- Tcpλ�� -------------------------//
    // ���㹤������ϵRPY�� - rad //
	ld_temp[5] = atan2(-lm_Tcp.R31, sqrt(lm_Tcp.R11 * lm_Tcp.R11 + lm_Tcp.R21 * lm_Tcp.R21));

    if (fabs(ld_temp[5] - PI / 2) < RT_LITTLE)
    {
        ld_temp[4] = 0;
        ld_temp[6] = atan2(lm_Tcp.R12, lm_Tcp.R22);
    }
    else if (fabs(ld_temp[5] + PI / 2) < RT_LITTLE)
    {
        ld_temp[4] = 0;
        ld_temp[6] = -atan2(lm_Tcp.R12, lm_Tcp.R22);
    }
    else
    {
        ld_temp[0] = 1 / cos(ld_temp[5]);

		ld_temp[1] = lm_Tcp.R21 * ld_temp[0];
		ld_temp[2] = lm_Tcp.R11 * ld_temp[0];
		if (fabs(ld_temp[1]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		if (fabs(ld_temp[2]) < RT_LITTLE)
		{
			ld_temp[2] = 0;
		}
        ld_temp[4] = atan2(ld_temp[1], ld_temp[2]);

		ld_temp[1] = lm_Tcp.R32 * ld_temp[0];
		ld_temp[2] = lm_Tcp.R33 * ld_temp[0];
		if (fabs(ld_temp[1]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		if (fabs(ld_temp[2]) < RT_LITTLE)
		{
			ld_temp[2] = 0;
		}
        ld_temp[6] = atan2(ld_temp[1], ld_temp[2]);
    }

	gdCPos[0] = lm_Tcp.X;      // mm   
	gdCPos[1] = lm_Tcp.Y;      // mm
	gdCPos[2] = lm_Tcp.Z; // mm
	gdCPos[3] = ld_temp[4] * PI_DEG;   // deg
	gdCPos[4] = ld_temp[5] * PI_DEG;   // deg
	gdCPos[5] = ld_temp[6] * PI_DEG;   // deg

	return Ok;
}

int Kine_CR_FiveDoF_G1::FKine_Inc(IN double gdJPos[5], IN double inc[3], OUT double gdCPos[3])
{
	// �ؽڽǶ� - �ؽڻ��� - Tcp���� - λ��
	// id_jPos  -   jRad   - lm_Tcp  - id_cPos
	int i;
	double jrad[5]; // �ؽڻ���
	
//	double ld_temp[8];     // �м����
	MtxKine lm_Tcp;        // �м����,TCP����
	
    //--------------------- �ؽڻ��� -------------------------//
	for (i=0; i<5; i++)
	{
		jrad[i] = gdJPos[i] * PI_RAD;
	}
	
    //--------------------- Tcp���� -------------------------//
	double c23 = cos(jrad[1] + jrad[2]);
	double s23 = sin(jrad[1] + jrad[2]);
	double c234 = cos(jrad[1] + jrad[2] + jrad[3]);
	double s234 = sin(jrad[1] + jrad[2] + jrad[3]);
	
    // ����Tcp��̬����Ĳ��� //
    lm_Tcp.R11 =   cos(jrad[0]) * c234 * cos(jrad[4]) + sin(jrad[0]) * sin(jrad[4]);
	lm_Tcp.R12 = - cos(jrad[0]) * c234 * sin(jrad[4]) + sin(jrad[0]) * cos(jrad[4]);
	lm_Tcp.R13 =   cos(jrad[0]) * s234;
	lm_Tcp.R21 =   sin(jrad[0]) * c234 * cos(jrad[4]) - cos(jrad[0]) * sin(jrad[4]);
	lm_Tcp.R22 = - sin(jrad[0]) * c234 * sin(jrad[4]) - cos(jrad[0]) * cos(jrad[4]);
	lm_Tcp.R23 =   sin(jrad[0]) * s234;
	lm_Tcp.R31 =   s234 * cos(jrad[4]);
	lm_Tcp.R32 = - s234 * sin(jrad[4]);
	lm_Tcp.R33 = - c234;
	
    // ����Tcpλ�� //
	lm_Tcp.X =  lm_Tcp.R13 * (m_dL4 + m_dL5)
		+ cos(jrad[0]) * c23 * m_dL3
		+ cos(jrad[0]) * cos(jrad[1]) * m_dL2;
	lm_Tcp.Y =  lm_Tcp.R23 * (m_dL4 + m_dL5)
		+ sin(jrad[0]) * c23 * m_dL3
		+ sin(jrad[0]) * cos(jrad[1]) * m_dL2;
	lm_Tcp.Z =  lm_Tcp.R33 * (m_dL4 + m_dL5)
		+ s23 * m_dL3
		+ sin(jrad[1]) * m_dL2
		+ m_dL1;	

	gdCPos[0] = lm_Tcp.X + lm_Tcp.R11*inc[0] + lm_Tcp.R12*inc[1] + lm_Tcp.R13*inc[2];      // mm   
	gdCPos[1] = lm_Tcp.Y + lm_Tcp.R21*inc[0] + lm_Tcp.R22*inc[1] + lm_Tcp.R23*inc[2];      // mm
	gdCPos[2] = lm_Tcp.Z + lm_Tcp.R31*inc[0] + lm_Tcp.R32*inc[1] + lm_Tcp.R33*inc[2]; // mm

	
	return Ok;
}

/******************************************************************************
 * ������IKine()
 * ���ܣ����
 *
 * ���룺double* gdCPos  - λ������, (x,y,z,w,p,r)
 *       double* gdJCurr - ��ǰ�ؽ�ת��, 5�ؽ�
 * �����double* gdJPos  - ���ؽ�ת��, 5�ؽ�
 *
 * ���أ�int - 0�ɹ�, ��������
 ******************************************************************************/
 int Kine_CR_FiveDoF_G1::IKine(double* gdCPos, double* gdJCurr, double* gdJPos)
 {
	//   λ��  - Tcp���� - �ؽڻ��� - �ؽڽǶ�
	// id_cPos - lm_Tcp  - ld_jRad  - id_jPos
	int i;
	int result;
	int li_flag[4] = {0};      // ���������,0Ϊ�н�

	double s1,c1,s2,c2, s3, c3, s5, c5, s234, c234;

	double ld_temp[8];       // �м����
	MtxKine lm_Tcp;          // �м����,TCP����
	double gd_rad[4][5];     // �м�������������

    //--------------------- TCP���� -------------------------//
	ld_temp[1] = sin(gdCPos[3] * PI_RAD);
	ld_temp[2] = cos(gdCPos[3] * PI_RAD);
	ld_temp[3] = sin(gdCPos[4] * PI_RAD);
	ld_temp[4] = cos(gdCPos[4] * PI_RAD);
	ld_temp[5] = sin(gdCPos[5] * PI_RAD);
	ld_temp[6] = cos(gdCPos[5] * PI_RAD);

	lm_Tcp.R11 = ld_temp[2] * ld_temp[4];
    lm_Tcp.R12 = ld_temp[2] * ld_temp[3] * ld_temp[5] - ld_temp[1] * ld_temp[6];
    lm_Tcp.R13 = ld_temp[2] * ld_temp[3] * ld_temp[6] + ld_temp[1] * ld_temp[5];
    lm_Tcp.R21 = ld_temp[1] * ld_temp[4];
    lm_Tcp.R22 = ld_temp[1] * ld_temp[3] * ld_temp[5] + ld_temp[2] * ld_temp[6];
    lm_Tcp.R23 = ld_temp[1] * ld_temp[3] * ld_temp[6] - ld_temp[2] * ld_temp[5];
    lm_Tcp.R31 = -ld_temp[3];
    lm_Tcp.R32 = ld_temp[4] * ld_temp[5];
    lm_Tcp.R33 = ld_temp[4] * ld_temp[6];

	lm_Tcp.X = - (m_dL4 + m_dL5) * lm_Tcp.R13 + gdCPos[0];
	lm_Tcp.Y = - (m_dL4 + m_dL5) * lm_Tcp.R23 + gdCPos[1];
	lm_Tcp.Z = - (m_dL4 + m_dL5) * lm_Tcp.R33 + gdCPos[2];

    //--------------------- �ؽڻ��� -------------------------//
    //------ ת��1 -------//
	ld_temp[0] = atan2(lm_Tcp.Y, lm_Tcp.X);
	ld_temp[1] = atan2(- lm_Tcp.Y, - lm_Tcp.X);
	RadInRange(&ld_temp[0], &gdJCurr[0]);
	RadInRange(&ld_temp[1], &gdJCurr[0]);

	gd_rad[0][0] = gd_rad[1][0] = ld_temp[0];
	gd_rad[2][0] = gd_rad[3][0] = ld_temp[1];

    //------ ת��5,3 ------//
    for (i=0; i<2; i++)
    {
        s1 = sin(gd_rad[2*i][0]);
        c1 = cos(gd_rad[2*i][0]);
	
		s5 = s1 * lm_Tcp.R11 - c1 * lm_Tcp.R21;
		c5 = s1 * lm_Tcp.R12 - c1 * lm_Tcp.R22;

		// ����ת��5 // 
		ld_temp[0] = atan2(s5, c5);
		RadInRange(&ld_temp[0], &gdJCurr[4]);

        gd_rad[2*i][4]   = ld_temp[0];
        gd_rad[2*i+1][4] = ld_temp[0];

		// ����ת��3 //
		ld_temp[2] = (
			  (lm_Tcp.X * c1 + lm_Tcp.Y * s1) * (lm_Tcp.X * c1 + lm_Tcp.Y * s1)
			+ (lm_Tcp.Z - m_dL1) * (lm_Tcp.Z - m_dL1) 
			- (m_dL2 * m_dL2 + m_dL3 * m_dL3)	) / (2 * m_dL2 * m_dL3);
		ld_temp[0] = 1 - ld_temp[2] * ld_temp[2];

		if (fabs(ld_temp[0]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		else if (ld_temp[0] < 0)
		{
			li_flag[2*i]   = 1;     // ��־ - ������޽�
			li_flag[2*i+1] = 1;
		}
		else
		{
			ld_temp[1] = sqrt(ld_temp[0]);
		}		

		ld_temp[3] = atan2(ld_temp[1], ld_temp[2]);
		ld_temp[4] = atan2(-ld_temp[1], ld_temp[2]);
		RadInRange(&ld_temp[3], &gdJCurr[2]);
		RadInRange(&ld_temp[4], &gdJCurr[2]);

		gd_rad[2*i][2]   = ld_temp[3];
        gd_rad[2*i+1][2] = ld_temp[4];
    }

	if (li_flag[0] && li_flag[2])
	{
		return ERR_NOINV; //  ���ؽ��޿������ֵ //
    }
	
    //------ ת��2,4 ------//
    for (i=0; i<4; i++)
    {
		if (0 == li_flag[i])
		{
			s1 = sin(gd_rad[i][0]);
	        c1 = cos(gd_rad[i][0]);

			s3 = sin(gd_rad[i][2]);
			c3 = cos(gd_rad[i][2]);
		
			// ����ת��2 //
			ld_temp[1] = lm_Tcp.Z - m_dL1;
			ld_temp[2] = lm_Tcp.X * c1 + lm_Tcp.Y * s1;
			ld_temp[3] = ld_temp[1] * ld_temp[1] + ld_temp[2] * ld_temp[2];

			s2 = (ld_temp[1] * (m_dL3 * c3 + m_dL2) - ld_temp[2] * m_dL3 * s3) / ld_temp[3];
			c2 = (ld_temp[2] * (m_dL3 * c3 + m_dL2) + ld_temp[1] * m_dL3 * s3) / ld_temp[3];

			ld_temp[4] = atan2(s2, c2);
			RadInRange(&ld_temp[4], &gdJCurr[1]);

			gd_rad[i][1] = ld_temp[4];


			// ����ת��4 //
			s234 = lm_Tcp.R13 * c1 + lm_Tcp.R23 * s1;
			c234 = - lm_Tcp.R33;
			
			ld_temp[0] = atan2(s234, c234); // 234֮��

			
			ld_temp[3] = ld_temp[0] - gd_rad[i][1] - gd_rad[i][2];
			RadInRange(&ld_temp[3], &gdJCurr[3]);
			
			gd_rad[i][3] = ld_temp[3];
		}
	}
	
	//------ ��ѽ�� ------//
	for (i=0; i<4; i++)
	{
		if (0 == li_flag[i])  // ��ȡ��Ծ���ֵ
		{
			ld_temp[i] = fabs(gd_rad[i][0] - gdJCurr[0] * PI_RAD) + 
			      	     fabs(gd_rad[i][1] - gdJCurr[1] * PI_RAD) + 
					     fabs(gd_rad[i][2] - gdJCurr[2] * PI_RAD) + 
					     fabs(gd_rad[i][3] - gdJCurr[3] * PI_RAD) +
					     fabs(gd_rad[i][4] - gdJCurr[4] * PI_RAD);
		}
	}
	for (i=0; i<4; i++)
	{
		if (0 == li_flag[i])
		{
			s1 = ld_temp[i]; // �õ�һ����Чֵ �� ��ʼ�� �м����s1
			result = i;
			break;           // �Ƴ���ʼ��
		}
	}
	for (i=0; i<4; i++)
	{
		if ((0 == li_flag[i]) && (ld_temp[i] <= s1))  // ��Чֵ | ��Ծ���ֵ��С
		{
			//ld_jRad = gd_rad[i];
			s1 = ld_temp[i];
			result = i;
		}
	}
	//------ �ؽڽǶ� ------//
	gdJPos[0] = gd_rad[result][0] * PI_DEG;
	gdJPos[1] = gd_rad[result][1] * PI_DEG;
	gdJPos[2] = gd_rad[result][2] * PI_DEG;
	gdJPos[3] = gd_rad[result][3] * PI_DEG;
	gdJPos[4] = gd_rad[result][4] * PI_DEG;


	return Ok;
 }

/******************************************************************************
 * ������Vel_FKine()
 * ���ܣ��ٶ�����, ��������ϵ�ٶ�
 *
 * ���룺double gdJPos[]  - ��ǰ�ؽ�ת��, 5�ؽ�, deg
 *       double gdJVel[] - ��ǰ�ؽ��ٶ�, 5�ؽ�, deg/s
 * �����double gdCVel[] - ĩ���ٶ�, [vx,vy,vz,wx,wy,wz], mm/s, deg/s
 *
 * ���أ�int - 0�ɹ�, ��������
 ******************************************************************************/
int Kine_CR_FiveDoF_G1::Vel_FKine(double gdJPos[], double gdJVel[], double gdCVel[])
{
/*
	int i;

	double c2 = cos((gdJPos[1]) * PI_RAD);  // ת��Ϊ����
	double s23 = sin((gdJPos[1] + gdJPos[2]) * PI_RAD);
	double c23 = cos((gdJPos[1] + gdJPos[2]) * PI_RAD);
	double s4 = sin((gdJPos[3]) * PI_RAD);
	double c4 = cos((gdJPos[3]) * PI_RAD);
	double s234 = sin((gdJPos[1] + gdJPos[2] + gdJPos[3]) * PI_RAD);
	double c234 = cos((gdJPos[1] + gdJPos[2] + gdJPos[3]) * PI_RAD);
	double s5 = sin((gdJPos[4]) * PI_RAD);
	double c5 = cos((gdJPos[4]) * PI_RAD);

	double v[MAX_AXIS_NUM];
	for (i=0; i<JOINT_NUM; i++)
	{
		v[i] = gdJVel[i] * PI_RAD;
	}

	double temp1 = m_dL2 * c2 + m_dL3 * c23 + s234 * m_dL4 + s234 * m_dL5;
	double temp2 = m_dL2 * s23 + m_dL3 * s4 + m_dL4 + m_dL5;
	double temp3 = m_dL3 * s4 + m_dL4 + m_dL5;

	gdCVel[0] = - temp1 * s5 * v[0] 
			  + temp2 * c5 * v[1]
			  + temp3 * c5 * v[2]
			  + (m_dL4 + m_dL5) * c5 * v[3];
	gdCVel[1] = - temp1 * c5 * v[0] 
			  - temp2 * s5 * v[1]
			  - temp3 * s5 * v[2]
			  - (m_dL4 + m_dL5) * s5 * v[3];
	gdCVel[2] = - (m_dL2 * c23 + m_dL3 * c4) * v[1]
			  - m_dL3 * c4 * v[2];
	gdCVel[3] = + s234 * c5 * v[0]
			  + s5 * (v[1] + v[2] + v[3]);
	gdCVel[4] = - s234 * s5 * v[0]
			  + c5 * (v[1] + v[2] + v[3]);
	gdCVel[5] = - c234 * v[0] + v[4];

	// ת��Ϊ�Ƕ�
	gdCVel[3] *= PI_DEG;
	gdCVel[4] *= PI_DEG;
	gdCVel[5] *= PI_DEG;
*/

	int i;

	double rad[5];
	double s1, c1, s2, c2, s3, c3, s23, c23, c34, s234, c234, s4, c4, s5, c5;
	double l1, l2, l3, l45;
	
	Matrix lm_jvel(5, 1);
	Matrix lm_cvel(6, 1);
	Matrix lm_Jacobian(6, 5);
	
    //------------------------- �ؽڻ��� -----------------------------//
	for(i=0; i<5; i++)
	{		
		rad[i] = (gdJPos[i]) * PI_RAD;
		lm_jvel.Mtx[i] = gdJVel[i] * PI_RAD;
	}
	
	s1 = sin(rad[0]);
	c1 = cos(rad[0]);
	s2 = sin(rad[1]);
	c2 = cos(rad[1]);
	s3 = sin(rad[2]);
	c3 = cos(rad[2]);
	s23 = sin(rad[1] + rad[2]);
	c23 = cos(rad[1] + rad[2]);
	c34 = cos(rad[2] + rad[3]);
	s234 = sin(rad[1] + rad[2] + rad[3]);
	c234 = cos(rad[1] + rad[2] + rad[3]);
	s4 = sin(rad[3]);
	c4 = cos(rad[3]);
	s5 = sin(rad[4]);
	c5 = cos(rad[4]);

	l1 = m_dL1;
	l2 = m_dL2;
	l3 = m_dL3;
	l45 = m_dL4 + m_dL5;


	// ��������ϵ�ſɱȾ���
	//--------------------------- ������ -------------------------------//
	// [0,4]-[5,4]
	lm_Jacobian.Mtx[0*5 + 4] = 0;
	lm_Jacobian.Mtx[1*5 + 4] = 0;
	lm_Jacobian.Mtx[2*5 + 4] = 0;
	lm_Jacobian.Mtx[3*5 + 4] = 0;
	lm_Jacobian.Mtx[4*5 + 4] = 0;
	lm_Jacobian.Mtx[5*5 + 4] = 1;
	
	//--------------------------- ������ -------------------------------//
	// [0,3]-[5,3]
	lm_Jacobian.Mtx[0*5 + 3] =  l45*c5;
	lm_Jacobian.Mtx[1*5 + 3] =  -l45*s5;
	lm_Jacobian.Mtx[2*5 + 3] =  0;
	lm_Jacobian.Mtx[3*5 + 3] =  s5;
	lm_Jacobian.Mtx[4*5 + 3] =  c5;
	lm_Jacobian.Mtx[5*5 + 3] =  0;
	
	//--------------------------- ������ -------------------------------//
	// [0,2]
	lm_Jacobian.Mtx[0*5 + 2] =  c5*(s4*l3+l45);
	lm_Jacobian.Mtx[1*5 + 2] = -s5*(s4*l3+l45);
	lm_Jacobian.Mtx[2*5 + 2] = -c4*l3;
	lm_Jacobian.Mtx[3*5 + 2] =  s5;
	lm_Jacobian.Mtx[4*5 + 2] =  c5;
	lm_Jacobian.Mtx[5*5 + 2] =  0;
	
	//--------------------------- �ڶ��� -------------------------------//
	// [0,1]
	lm_Jacobian.Mtx[0*5 + 1] =  c5*(s23*l2 + s4*l3 + l45);
	lm_Jacobian.Mtx[1*5 + 1] = -s5*(s23*l2 + s4*l3 + l45);
	lm_Jacobian.Mtx[2*5 + 1] = -c23*l2 - c4*l3;
	lm_Jacobian.Mtx[3*5 + 1] =  s5;
	lm_Jacobian.Mtx[4*5 + 1] =  c5;	
	lm_Jacobian.Mtx[5*5 + 1] =  0;
	
	//---------------------------- ��һ�� --------------------------------//
	// [0,0]
	lm_Jacobian.Mtx[0*5 + 0] = -s5*(c2*l2 + c23*l3 + s234*l45);
	lm_Jacobian.Mtx[1*5 + 0] = -c5*(c2*l2 + c23*l3 + s234*l45);
	lm_Jacobian.Mtx[2*5 + 0] =  0;
	lm_Jacobian.Mtx[3*5 + 0] =  s234*c5;
	lm_Jacobian.Mtx[4*5 + 0] = -s234*s5;
	lm_Jacobian.Mtx[5*5 + 0] = -c234;


	lm_cvel = lm_Jacobian*lm_jvel;

	gdCVel[0] = lm_cvel.Mtx[0];
	gdCVel[1] = lm_cvel.Mtx[1];
	gdCVel[2] = lm_cvel.Mtx[2];
	gdCVel[3] = lm_cvel.Mtx[3] * PI_DEG;
	gdCVel[4] = lm_cvel.Mtx[4] * PI_DEG;
	gdCVel[5] = lm_cvel.Mtx[5] * PI_DEG;



	return Ok;
}

/******************************************************************************
 * ������Vel_IKine()
 * ���ܣ��ٶ����
 *
 * ���룺double gdJPos[] - ��ǰ�ؽ�ת��, 5�ؽ�, deg
 *       double gdCVel[] - ��ǰĩ���ٶ�, [vx,vy,vz,wx,wy,wz], m/s, deg/s
 * �����double gdJVel[] - �ؽ��ٶ�, 5�ؽ�, deg/s
 *
 * ���أ�int - 0�ɹ�, ��������
 ******************************************************************************/
int Kine_CR_FiveDoF_G1::Vel_IKine(double gdJPos[], double gdCVel[], double gdJVel[])
{
	int i;
	double rad[5];
	Matrix lm_Jacobian(6, 5);
	Matrix lm_cvel(6, 1);
	Matrix lm_jvel(5, 1);
	Matrix lm_inv(5,5);
	double ld_wvel[6];
	
	double s1, c1, s2, c2, s3, c3, s23, c23, s234, c234, s4, c4, s5, c5;
	double l1, l2, l3, l45;
	double c34;

	double ki, kb;
	double ei, eb;

	MtxKine lm_Tcp;        // �м����,TCP����	


    //------------------------- �ؽڻ��� -----------------------------//
	for(i=0; i<5; i++)
	{		
		rad[i] = (gdJPos[i]) * PI_RAD;
	}
	
	s1 = sin(rad[0]);
	c1 = cos(rad[0]);
	s2 = sin(rad[1]);
	c2 = cos(rad[1]);
	s3 = sin(rad[2]);
	c3 = cos(rad[2]);
	s23 = sin(rad[1] + rad[2]);
	c23 = cos(rad[1] + rad[2]);
	c34 = cos(rad[2] + rad[3]);
	s234 = sin(rad[1] + rad[2] + rad[3]);
	c234 = cos(rad[1] + rad[2] + rad[3]);
	s4 = sin(rad[3]);
	c4 = cos(rad[3]);
	s5 = sin(rad[4]);
	c5 = cos(rad[4]);
	
	l1 = m_dL1;
	l2 = m_dL2;
	l3 = m_dL3;
	l45 = m_dL4 + m_dL5;

	
	// ���ٶ�
	ld_wvel[0] = gdCVel[0];
	ld_wvel[1] = gdCVel[1];
	ld_wvel[2] = gdCVel[2];
	ld_wvel[3] = gdCVel[3]*PI_RAD;
	ld_wvel[4] = gdCVel[4]*PI_RAD;
	ld_wvel[5] = gdCVel[5]*PI_RAD;
	ld_wvel[0] = ld_wvel[0] - ld_wvel[4]*l45; 
	ld_wvel[1] = ld_wvel[1] + ld_wvel[3]*l45;
	ld_wvel[2] = ld_wvel[2];


	// ����Tcp��̬����Ĳ��� //
    lm_Tcp.R11 =   c1 * c234 * c5 + s1 * s5;
	lm_Tcp.R12 = - c1 * c234 * s5 + s1 * c5;
	lm_Tcp.R13 =   c1 * s234;
	lm_Tcp.R21 =   s1 * c234 * c5 - c1 * s5;
	lm_Tcp.R22 = - s1 * c234 * s5 - c1 * c5;
	lm_Tcp.R23 =   s1 * s234;
	lm_Tcp.R31 =   s234 * c5;
	lm_Tcp.R32 = - s234 * s5;
	lm_Tcp.R33 = - c234;

	// ������ϵ�����ٶ�
	lm_cvel.Mtx[0] = lm_Tcp.R11*ld_wvel[0] + lm_Tcp.R12*ld_wvel[1] + lm_Tcp.R13*ld_wvel[2];
	lm_cvel.Mtx[1] = lm_Tcp.R21*ld_wvel[0] + lm_Tcp.R22*ld_wvel[1] + lm_Tcp.R23*ld_wvel[2];
	lm_cvel.Mtx[2] = lm_Tcp.R31*ld_wvel[0] + lm_Tcp.R32*ld_wvel[1] + lm_Tcp.R33*ld_wvel[2];
	lm_cvel.Mtx[3] = lm_Tcp.R11*ld_wvel[3] + lm_Tcp.R12*ld_wvel[4] + lm_Tcp.R13*ld_wvel[5];
	lm_cvel.Mtx[4] = lm_Tcp.R21*ld_wvel[3] + lm_Tcp.R22*ld_wvel[4] + lm_Tcp.R23*ld_wvel[5];
	lm_cvel.Mtx[5] = lm_Tcp.R31*ld_wvel[3] + lm_Tcp.R32*ld_wvel[4] + lm_Tcp.R33*ld_wvel[5];

	// ������ϵ�ſɱȾ���
	//--------------------------- ������ -------------------------------//
	// [0,4]-[5,4]
	lm_Jacobian.Mtx[0*5 + 4] = 0;
	lm_Jacobian.Mtx[1*5 + 4] = 0;
	lm_Jacobian.Mtx[2*5 + 4] = 0;
	lm_Jacobian.Mtx[3*5 + 4] = c1*s234;
	lm_Jacobian.Mtx[4*5 + 4] = s1*s234;
	lm_Jacobian.Mtx[5*5 + 4] = -c234;
	
	//--------------------------- ������ -------------------------------//
	// [0,3]-[5,3]
	lm_Jacobian.Mtx[0*5 + 3] =  0;
	lm_Jacobian.Mtx[1*5 + 3] =  0;
	lm_Jacobian.Mtx[2*5 + 3] =  0;
	lm_Jacobian.Mtx[3*5 + 3] =  s1;
	lm_Jacobian.Mtx[4*5 + 3] =  -c1;
	lm_Jacobian.Mtx[5*5 + 3] =  0;
	
	//--------------------------- ������ -------------------------------//
	// [0,2]
	lm_Jacobian.Mtx[0*5 + 2] = -c1*(s23*l3);
	lm_Jacobian.Mtx[1*5 + 2] = -s1*(s23*l3);
	lm_Jacobian.Mtx[2*5 + 2] =  c23*l3;
	lm_Jacobian.Mtx[3*5 + 2] =  s1;
	lm_Jacobian.Mtx[4*5 + 2] =  -c1;
	lm_Jacobian.Mtx[5*5 + 2] =  0;
	
	//--------------------------- �ڶ��� -------------------------------//
	// [0,1]
	lm_Jacobian.Mtx[0*5 + 1] = -c1*(s2*l2 + s23*l3);
	lm_Jacobian.Mtx[1*5 + 1] = -s1*(s2*l2 + s23*l3);
	lm_Jacobian.Mtx[2*5 + 1] =  c2*l2 + c23*l3;
	lm_Jacobian.Mtx[3*5 + 1] =  s1;
	lm_Jacobian.Mtx[4*5 + 1] =  -c1;	
	lm_Jacobian.Mtx[5*5 + 1] =  0;
	
	//---------------------------- ��һ�� --------------------------------//
	// [0,0]
	lm_Jacobian.Mtx[0*5 + 0] = -s1*(c2*l2 + c23*l3);
	lm_Jacobian.Mtx[1*5 + 0] =  c1*(c2*l2 + c23*l3);
	lm_Jacobian.Mtx[2*5 + 0] =  0;
	lm_Jacobian.Mtx[3*5 + 0] =  0;
	lm_Jacobian.Mtx[4*5 + 0] =  0;
	lm_Jacobian.Mtx[5*5 + 0] =  1;
	
	// ϵ��
	ki = SQUARE(c2*l2+c23*l3) + SQUARE(s234);
	kb = SQUARE(s3);
	ei = (fabs(ki) <= KINE_DAMP_EI) ? KINE_RATIO_V*(1-fabs(ki)/KINE_DAMP_EI) : 0;
	eb = (fabs(kb) <= KINE_DAMP_EB) ? KINE_RATIO_W*(1-fabs(kb)/KINE_DAMP_EB) : 0;
    ei = ki / (SQUARE(ki)+ei);
	eb = kb / (SQUARE(kb)+eb);
	
	// Inv(JT*J)
	// 1��
	lm_inv.Mtx[0*5+0] = 1 * ei;
	lm_inv.Mtx[0*5+1] = lm_inv.Mtx[1*5+0] = 0;
	lm_inv.Mtx[0*5+2] = lm_inv.Mtx[2*5+0] = 0;
	lm_inv.Mtx[0*5+3] = lm_inv.Mtx[3*5+0] = 0;
	lm_inv.Mtx[0*5+4] = lm_inv.Mtx[4*5+0] = c234 * ei;

	// 2��
	lm_inv.Mtx[1*5+1] = eb / SQUARE(l2);
	lm_inv.Mtx[1*5+2] = lm_inv.Mtx[2*5+1] = -eb*(c3*l2+l3)/(SQUARE(l2)*l3);
	lm_inv.Mtx[1*5+3] = lm_inv.Mtx[3*5+1] = eb*c3/(l2*l3);
	lm_inv.Mtx[1*5+4] = lm_inv.Mtx[4*5+1] = 0;

	// 3��
	lm_inv.Mtx[2*5+2] = eb * (SQUARE(l2)+SQUARE(l3)+2*l2*l3*c3)/(SQUARE(l2*l3));
	lm_inv.Mtx[2*5+3] = lm_inv.Mtx[3*5+2] = -eb*(l2+l3*c3)/(l2*SQUARE(l3));
	lm_inv.Mtx[2*5+4] = lm_inv.Mtx[4*5+2] = 0;

	// 4��
	lm_inv.Mtx[3*5+3] = eb * (1+SQUARE(l3*s3))/(SQUARE(l3));
	lm_inv.Mtx[3*5+4] = lm_inv.Mtx[4*5+3] = 0;

	// 5��
	lm_inv.Mtx[4*5+4] = ei * (1 + SQUARE(c2*l2+c23*l3));	



	// �ؽ��ٶ�
	lm_jvel = lm_inv * (Trv(lm_Jacobian) * lm_cvel);
	
	for(i=0; i<5; i++)
	{
		gdJVel[i] = lm_jvel.Mtx[i] * PI_DEG;
	}

	
	if ( ((gdJPos[2] < SINGULARITY_ZONE) && (gdJPos[2]>0) && (gdJVel[2]<0)) ||
		 ((gdJPos[2] >-SINGULARITY_ZONE) && (gdJPos[2]<0) && (gdJVel[2]>0)) )
	{
		return ERR_NSINGU;
	}

	return Ok;
}

void Kine_CR_FiveDoF_G2::Set_Length(double gdLen[])
{
	double len_G2[5];
	len_G2[0] = gdLen[4] + gdLen[5];
	len_G2[1] = gdLen[3];
	len_G2[2] = gdLen[2];
	len_G2[3] = gdLen[1];
	len_G2[4] = gdLen[0];

	kine.Set_Length(gdLen);
}


int Kine_CR_FiveDoF_G2::FKine(double gdJPos[], double gdCPos[])
{
	double jpos_G2[5];

	// jpos_G2[0] = - gdJPos[4];
	// jpos_G2[1] = gdJPos[3];
	// jpos_G2[2] = gdJPos[2];
	// jpos_G2[3] = gdJPos[1];
	// jpos_G2[4] = -gdJPos[0];
	jpos_G2[0] = gdJPos[4];
	jpos_G2[1] = gdJPos[3];
	jpos_G2[2] = gdJPos[2];
	jpos_G2[3] = gdJPos[1];
	jpos_G2[4] = gdJPos[0];

	return kine.FKine(jpos_G2, gdCPos);
}

int Kine_CR_FiveDoF_G2::FKine_Inc(IN double gdJPos[5], IN double inc[3], OUT double gdCPos[3])
{
	double jpos_G2[5];
	
	jpos_G2[0] = - gdJPos[4];
	jpos_G2[1] = gdJPos[3];
	jpos_G2[2] = gdJPos[2];
	jpos_G2[3] = gdJPos[1];
	jpos_G2[4] = -gdJPos[0];
	
	return kine.FKine_Inc(jpos_G2, inc ,gdCPos);
}

int Kine_CR_FiveDoF_G2::IKine(double gdCPos[], double gdJCurr[], double gdJPos[])
{
	double jpos_G2[5];
	double jcurrpos_G2[5];
	int flag;
	
	// jcurrpos_G2[0] = - gdJCurr[4];
	// jcurrpos_G2[1] =   gdJCurr[3];
	// jcurrpos_G2[2] =   gdJCurr[2];
	// jcurrpos_G2[3] =   gdJCurr[1];
	// jcurrpos_G2[4] = - gdJCurr[0];
	jcurrpos_G2[0] =   gdJCurr[4];
	jcurrpos_G2[1] =   gdJCurr[3];
	jcurrpos_G2[2] =   gdJCurr[2];
	jcurrpos_G2[3] =   gdJCurr[1];
	jcurrpos_G2[4] =   gdJCurr[0];

	flag = kine.IKine(gdCPos, jcurrpos_G2, jpos_G2);

	if (0 == flag)
	{
		gdJPos[0] = - jpos_G2[4];
		gdJPos[1] =   jpos_G2[3];
		gdJPos[2] =   jpos_G2[2];
		gdJPos[3] =   jpos_G2[1];
		gdJPos[4] = - jpos_G2[0];
	}
	else 
	{
		return flag;
	}

	return Ok;
}

int Kine_CR_FiveDoF_G2::Vel_FKine(double gdJPos[], double gdJVel[], double gdCVel[])
{
	double jpos_G2[5];
	double jvel_G2[5];
	
	jpos_G2[0] = - gdJPos[4];
	jpos_G2[1] =   gdJPos[3];
	jpos_G2[2] =   gdJPos[2];
	jpos_G2[3] =   gdJPos[1];
	jpos_G2[4] = - gdJPos[0];

	jvel_G2[0] = gdJVel[4];
	jvel_G2[1] = gdJVel[3];
	jvel_G2[2] = gdJVel[2];
	jvel_G2[3] = gdJVel[1];
	jvel_G2[4] = gdJVel[0];

	return kine.Vel_FKine(jpos_G2, jvel_G2, gdCVel);
}

int Kine_CR_FiveDoF_G2::Vel_IKine(double gdJPos[5], double gdCVel[6], double gdJVel[5])
{
	double jpos_G2[5];
	double jvel_G2[5];
	int flag;
	
	jpos_G2[0] = - gdJPos[4];
	jpos_G2[1] =   gdJPos[3];
	jpos_G2[2] =   gdJPos[2];
	jpos_G2[3] =   gdJPos[1];
	jpos_G2[4] = - gdJPos[0];

	double cvel[6];
	cvel[0] = gdCVel[0];
	cvel[1] = gdCVel[1];
	cvel[2] = gdCVel[2];
	cvel[3] = gdCVel[3];
	cvel[4] = gdCVel[4];
	cvel[5] = gdCVel[5];
	
	flag = kine.Vel_IKine(jpos_G2, cvel, jvel_G2);
	
	if (0 == flag)
	{
		gdJVel[0] = jvel_G2[4];
		gdJVel[1] = jvel_G2[3];
		gdJVel[2] = jvel_G2[2];
		gdJVel[3] = jvel_G2[1];
		gdJVel[4] = jvel_G2[0];
	}
	else 
	{
		return flag;
	}

	return Ok;
}


void Trans_PosToMtx(double* pos, MtxKine* output, int inv)
{
	double ld_temp[6];
	
	// ����λ�˱任���� //
	ld_temp[0] = sin(pos[3] * PI_RAD);
	ld_temp[1] = cos(pos[3] * PI_RAD);
	ld_temp[2] = sin(pos[4] * PI_RAD);
	ld_temp[3] = cos(pos[4] * PI_RAD);
	ld_temp[4] = sin(pos[5] * PI_RAD);
	ld_temp[5] = cos(pos[5] * PI_RAD);
	
    output->R11 = ld_temp[1] * ld_temp[3];
    output->R12 = ld_temp[1] * ld_temp[2] * ld_temp[4] - ld_temp[0] * ld_temp[5];
    output->R13 = ld_temp[1] * ld_temp[2] * ld_temp[5] + ld_temp[0] * ld_temp[4];
    output->R21 = ld_temp[0] * ld_temp[3];
    output->R22 = ld_temp[0] * ld_temp[2] * ld_temp[4] + ld_temp[1] * ld_temp[5];
    output->R23 = ld_temp[0] * ld_temp[2] * ld_temp[5] - ld_temp[1] * ld_temp[4];
    output->R31 = -ld_temp[2];
    output->R32 = ld_temp[3] * ld_temp[4];
    output->R33 = ld_temp[3] * ld_temp[5];
	
	// �������
	if(1 == inv)
	{
		// ��̬���� - ת�þ��� //
		ld_temp[0] = output->R12;
		output->R12 = output->R21;
		output->R21 = ld_temp[0];
		
		ld_temp[0] = output->R13;
		output->R13 = output->R31;
		output->R31 = ld_temp[0];
		
		ld_temp[0] = output->R23;
		output->R23 = output->R32;
		output->R32 = ld_temp[0];
		
		// λ������ //
		output->X = -( output->R11 * pos[0] + 
			output->R12 * pos[1] +
			output->R13 * pos[2] );
		output->Y = -( output->R21 * pos[0] + 
			output->R22 * pos[1] +
			output->R23 * pos[2] );
		output->Z = -( output->R31 * pos[0] + 
			output->R32 * pos[1] +
			output->R33 * pos[2] );
	}
	else
	{
		output->X = pos[0];
		output->Y = pos[1];
		output->Z = pos[2]; 
	}
}
void Trans_MtxToPos(MtxKine* input, double* outpos)
{
	double ld_temp[6];
	
    //--------------------- ���λ�� -------------------------//
    // ����RPY�� - rad //
	ld_temp[4] = atan2(- input->R31, 
		sqrt(input->R11 * input->R11 + input->R21 * input->R21));
	
    if (fabs(ld_temp[4] - PI / 2) < RT_LITTLE)
    {
        ld_temp[3] = 0;
        ld_temp[5] = atan2(input->R12, input->R22);
    }
    else if (fabs(ld_temp[4] + PI / 2) < RT_LITTLE)
    {
        ld_temp[3] = 0;
        ld_temp[5] = -atan2(input->R12, input->R22);
    }
    else
    {
        ld_temp[0] = 1 / cos(ld_temp[4]);
		
		ld_temp[1] = input->R21 * ld_temp[0];
		ld_temp[2] = input->R11 * ld_temp[0];
		if(fabs(ld_temp[1]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		if(fabs(ld_temp[2]) < RT_LITTLE)
		{
			ld_temp[2] = 0;
		}
        ld_temp[3] = atan2(ld_temp[1], ld_temp[2]);
		
		ld_temp[1] = input->R32 * ld_temp[0];
		ld_temp[2] = input->R33 * ld_temp[0];
		if(fabs(ld_temp[1]) < RT_LITTLE)
		{
			ld_temp[1] = 0;
		}
		if(fabs(ld_temp[2]) < RT_LITTLE)
		{
			ld_temp[2] = 0;
		}
        ld_temp[5] = atan2(ld_temp[1], ld_temp[2]);
    }
	
	outpos[0] = input->X;      // mm   
	outpos[1] = input->Y;      // mm
	outpos[2] = input->Z;      // mm
	outpos[3] = ld_temp[3] * PI_DEG;   // deg
	outpos[4] = ld_temp[4] * PI_DEG;   // deg
	outpos[5] = ld_temp[5] * PI_DEG;   // deg
}
void Mtx_Multiply(MtxKine* input, MtxKine* middle, MtxKine* output, int inv)
{
	double ld_temp[3];
	
    // ��̬���� //
    output->R11 = input->R11 * middle->R11 + 
		input->R12 * middle->R21 + 
		input->R13 * middle->R31;
    output->R12 = input->R11 * middle->R12 + 
		input->R12 * middle->R22 + 
		input->R13 * middle->R32;
    output->R13 = input->R11 * middle->R13 + 
		input->R12 * middle->R23 + 
		input->R13 * middle->R33;
    output->R21 = input->R21 * middle->R11 + 
		input->R22 * middle->R21 + 
		input->R23 * middle->R31;
    output->R22 = input->R21 * middle->R12 + 
		input->R22 * middle->R22 + 
		input->R23 * middle->R32;
    output->R23 = input->R21 * middle->R13 + 
		input->R22 * middle->R23 + 
		input->R23 * middle->R33;
    output->R31 = input->R31 * middle->R11 + 
		input->R32 * middle->R21 + 
		input->R33 * middle->R31;
    output->R32 = input->R31 * middle->R12 + 
		input->R32 * middle->R22 + 
		input->R33 * middle->R32;
    output->R33 = input->R31 * middle->R13 + 
		input->R32 * middle->R23 + 
		input->R33 * middle->R33;
	
    // λ�� //
	output->X = input->R11 * middle->X +
		input->R12 * middle->Y + 
		input->R13 * middle->Z + input->X;
	output->Y = input->R21 * middle->X + 
		input->R22 * middle->Y + 
		input->R23 * middle->Z + input->Y;
	output->Z = input->R31 * middle->X + 
		input->R32 * middle->Y + 
		input->R33 * middle->Z + input->Z;
	
	// �������
	if(1 == inv)
	{
		// ��̬���� - ת�þ��� //
		ld_temp[0] = output->R12;
		output->R12 = output->R21;
		output->R21 = ld_temp[0];
		
		ld_temp[0] = output->R13;
		output->R13 = output->R31;
		output->R31 = ld_temp[0];
		
		ld_temp[0] = output->R23;
		output->R23 = output->R32;
		output->R32 = ld_temp[0];
		
		// λ������ //
		ld_temp[0] = output->X;
		ld_temp[1] = output->Y;
		ld_temp[2] = output->Z;
		output->X = -( output->R11 * ld_temp[0] + 
			output->R12 * ld_temp[1] +
			output->R13 * ld_temp[2] );
		output->Y = -( output->R21 * ld_temp[0] + 
			output->R22 * ld_temp[1] +
			output->R23 * ld_temp[2] );
		output->Z = -( output->R31 * ld_temp[0] + 
			output->R32 * ld_temp[1] +
			output->R33 * ld_temp[2] );
	}
}

void Robot_IncreTransTool(IN double* currpos, IN double* increpos, OUT double* outpos)
{
	MtxKine lm_input;
	MtxKine lm_middle;
	MtxKine lm_output;
	
	Trans_PosToMtx(currpos, &lm_input, 0);
	Trans_PosToMtx(increpos, &lm_middle, 0);
	Mtx_Multiply(&lm_input, &lm_middle, &lm_output, 0);
	Trans_MtxToPos(&lm_output, outpos);
}