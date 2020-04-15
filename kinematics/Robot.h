#ifndef __ROBOT_H__
#define __ROBOT_H__

//#include "StdAfx.h"

///////////////////////////////////////////////////////////////////////// 宏定义
// [[ 机器人构型
#define ROBOT_MODE_SM    0  // 机器人构型模式- 单模块
#define ROBOT_MODE_DM	 7	// 机器人构型模式- 双模块(云台)
#define ROBOT_MODE_CR_P  1  // 机器人构型模式- 爬杆
#define ROBOT_MODE_CR_W  2  // 机器人构型模式- 爬壁
#define ROBOT_MODE_IR_5  3  // 机器人构型模式- 5Dof机械手
#define ROBOT_MODE_BR    4  // 机器人构型模式- 双足
#define ROBOT_MODE_WR    5  // 机器人构型模式- 轮式
#define ROBOT_MODE_IR_6  6  // 机器人构型模式- 6Dof机械手
// ]]

// [[ 末端执行器类型
//#define ROBOT_ENDEFFECTOR_COMMON    0  // 末端执行器类型：一般手爪
//#define ROBOT_ENDEFFECTOR_MASTER	1  // 末端执行器类型：主机器人手爪
// ]]
//#define ROBOT_IR_5_T   

#define CAN_NAME_CML5  "IXXAT0"
#define CAN_NAME_CML6  "IXXATV30"

#define SCREW_DEGREE    180
#define SCREW_PITCH     6.0/360   // 6mm/360deg

#define SINGULARITY_ZONE  6.0     // 奇异区域大小, deg

// [[ 轴数
#ifndef MAX_AXIS_NUM
#define MAX_AXIS_NUM     6  // 包括轴最大数目
#endif
#define JOINT_NUM	5// 当前关节数目
// ]]

// [[ 周期
#define INTERP_T	   0.016    // 插补周期,s
#define POS_T          0.008    // 位控周期,s
#define TEACH_T        0.200    // 示教周期,s
#define IDLE_T         0.200    // 空闲周期,s
// ]]
  
// [[ 圆周率相关 
#ifndef PI
#define PI      3.1415926535898 // 圆周率
#endif
#define PI2     6.2831853071796 // 2倍圆周率 
#define PI_2    1.5707963267949 // 1/2圆周率  
#define PI_RAD  0.0174532925199 // 角度转换为弧度参数
#define PI_DEG 57.2957795130823 // 弧度转换为角度参数
// ]]

// [[ 系统参数
#define RT_LITTLE   0.00001     // 系统可识别的最小正量
#define SERR         0.00001    // 误差允许范围
#define Ok                 0    // 正常
#define ERR               -1    // 错误
// ]]

// [[ 速度、加速度限制
#define MAX_TEACHVEL_JOINT Robot::teachLim[0]   // 关节示教最大速度, deg/s
#define MAX_TEACHACC_JOINT Robot::teachLim[1]   // 关节示教最大加速度, deg/s*s
#define MAX_TEACHVEL_CV    Robot::teachLim[2]   // 笛卡尔示教最大线速度, mm/s
#define MAX_TEACHACC_CV    Robot::teachLim[3]   // 笛卡尔示教最大线加速度, mm/s*s
#define MAX_TEACHVEL_CW    Robot::teachLim[4]   // 笛卡尔示教最大角速度, deg/s
#define MAX_TEACHACC_CW    Robot::teachLim[5]   // 笛卡尔示教最大角加速度, deg/s*s
//#define MAX_TEACHDEC_JOINT 50   // 关节示教最大减速度, deg/s*s
//#define MAX_TEACHDEC_CV   100axisNum   // 笛卡尔示教最大线减速度, mm/s*s
//#define MAX_TEACHDEC_CW    50   // 笛卡尔示教最大角减速度, deg/s*s
#define MAX_AUTOVEL_JOINT Robot::autoLim[0]   // 关节最大速度, deg/s
#define MAX_AUTOACC_JOINT Robot::autoLim[1]   // 关节最大加速度, deg/s*s
#define MAX_AUTOVEL_CV    Robot::autoLim[2]   // 笛卡尔最大线速度, mm/s
#define MAX_AUTOACC_CV    Robot::autoLim[3]   // 笛卡尔最大线加速度, mm/s*s
#define MAX_AUTOVEL_CW    Robot::autoLim[4]   // 笛卡尔最大角速度, deg/s
#define MAX_AUTOACC_CW    Robot::autoLim[5]   // 笛卡尔最大角加速度, deg/s*s
#define ACC_T             Robot::autoLim[6]   // 加速时间, ms
//
#define MAX_JERK_JOINT   (Robot::autoLim[1] * 1) // 关节加加速度, deg/s*s*s
#define MAX_JERK_CAR_VC  (Robot::autoLim[3] * 4) // 笛卡尔线加加速度, mm/s*s*s
#define MAX_JERK_CAR_CW  (Robot::autoLim[5] * 4) // 笛卡尔角加加速度, deg/s*s*s
// ]]


// [[ 运动状态
#define FLAG_RUNNING      1     // 机器人运动状态 - 运行
#define FLAG_STOP         0     // 机器人运动状态 - 停止
#define FLAG_ERROR       -1     // 机器人运动状态 - 运行错误
// ]]

// [[使能状态
#define FLAG_ENABLE       1     // 使能
#define FLAG_DISABLE     -1     // 非使能
// ]]

// [[ 坐标系宏定义
#define COORDINATE_JOINT  1     // 关节坐标系
#define COORDINATE_WORLD  2     // 世界坐标系
// ]]

// [[ 运动模式宏定义
#define RUNMODE_TEACH     1     // 示教运动模式
#define RUNMODE_AUTO      2     // 再现运动模式
// ]]

// [[ 任务模式
#define MODE_INTERP       1     // 插补运动模式
#define MODE_WAITTIME     2     // 等待模式
#define MODE_SETIO        3     // IO输出
#define MODE_GOTO         7
#define MODE_GOPEN        16    //GU
#define MODE_GCLOSE       17	//GU

// ]]

// [[ 插补模式
#define INTERP_JOINT      1     // Joint
#define INTERP_LINEAR     2     // Linear
#define INTERP_CIRCLE     3     // Circle
#define INTERP_PARABOLA   4
#define INTERP_SPLINE     5
// ]]

// [[ 工具编号
#define GRIP_FIRST        1
#define GRIP_SECOND       2
// ]]

// [[ 控件编号
#define ID_MYDIALOG_MONITOR 001
#define ID_MYDIALOG_TOOL    002
#define ID_MYDIALOG_MASTER  003
#define ID_MYDIALOG_WHEEL   004
// ]]

#define ROBOT_FKINE_JOINT1  1
#define ROBOT_FKINE_JOINT2  2
#define ROBOT_FKINE_JOINT3  3
#define ROBOT_FKINE_JOINT4  4
#define ROBOT_FKINE_JOINT5  5
#define ROBOT_FKINE_JOINT6  6
#define ROBOT_FKINE_TCP     7


#define MATRIX_COND_LIN  1  // 雅可比线速度条件数
#define MATRIX_COND_ANG  2  // 雅可比角速度条件数
#define MATRIX_COND_ALL  3  // 雅可比速度条件数

// 位姿矩阵结构
#ifndef _POSEMATRIX_
#define _POSEMATRIX_
class PoseMatrix
{
public:
	double R11;
	double R12;
	double R13;
	double R21;
	double R22;
	double R23;
	double R31;
	double R32;
	double R33;
	
	double X;
	double Y;
	double Z;
};
#endif

#define CUBE(n)  ((n)*(n)*(n))
#define SQUARE(n) ((n)*(n))


///////////////////////////////////////////////////////////////////// 报错处理宏
// [[ 通讯错误
#define ERR_CANCOMM     -110    // CAN通讯错误
#define ERR_AMPCOMM     -111    // AMP通讯错误
#define ERR_GRIPCOMM    -112    // 手爪通讯错误
#define ERR_GRIPDISABLE -113    // 手爪未使能
#define ERR_LINKAGEINIT -114    // Linkage初始化错误
#define ERR_SERIALCOMM  -115    // 串口通讯错误
#define ERR_CAN20		-116	// Can2.0通讯错误

// ]]

// [[ 程序错误
#define ERR_MOTIONPARA  -119    // 输入运动参数有误
#define ERR_POINT       -120    // 输入点数据有误
#define ERR_PINLIM      -121    // 输入点处于极限
#define ERR_PINSINGU    -122    // 输入点处于奇异位形
#define ERR_POWORKSPA   -123    // 输入点超出工作空间
#define ERR_PINLIN      -124    // 圆弧三点共线
#define ERR_TOOCLS      -125    // 圆弧三点过近
#define ERR_EQUALPOS    -126    // 点的位置相同
#define ERR_OLBRL       -127    // 程序段过短,不够加速/减速
// ]]

// [[ 规划错误
#define ERR_NOGIVENKINE -129    // 未给定运动学算式
#define ERR_PLIM        -130    // 位置极限
#define ERR_NOINV       -131    // 无逆解
#define ERR_NOCHARACTER -132    
// ]]

// [[ 运动错误
#define ERR_EMGSTOP	    -133	// 紧急停止
#define ERR_CMODESTOP   -134    // 运动模式改变急停
#define ERR_SINGU       -135    // 奇异状态
#define ERR_NSINGU      -136    // 接近奇异位形
#define ERR_OWORKSPACE  -137    // 超出工作空间
#define ERR_DATAEMPTY   -138    // 数据空穴
// ]]

// [[ 速度极限
#define ERR_SVLIM       -140    // J1轴速度超限
#define ERR_LVLIM       -141    // J2轴速度超限
#define ERR_UVLIM       -142    // J3轴速度超限
#define ERR_RVLIM       -143    // J4轴速度超限
#define ERR_BVLIM       -144    // J5轴速度超限
#define ERR_TVLIM       -145    // J6轴速度超限
// ]]

// [[ 伺服错误
#define SV_SALM			-150	// J1轴伺服报警
#define SV_LALM			-151	// J2轴伺服报警
#define SV_UALM			-152	// J3轴伺服报警
#define SV_RALM			-153	// J4轴伺服报警
#define SV_BALM			-154	// J5轴伺服报警
#define SV_TALM			-155	// J6轴伺服报警
#define SV_MOVEALM	-158 // 当前状态不能运动
#define SV_LINKALM		-156	// 联动伺服报警
#define SV_NOZERO       -157    // 零点丢失
// ]] 

// [[ 软极限报警
#define OTS_SSLIMP    	-170	// J1+软限位
#define OTS_LSLIMP		-171	// J2+软限位
#define OTS_USLIMP		-172	// J3+软限位
#define OTS_RSLIMP		-173	// J4+软限位
#define OTS_BSLIMP		-174	// J5+软限位
#define OTS_TSLIMP		-175	// J6+软限位
#define OTS_SSLIMN		-176	// J1-软限位
#define OTS_LSLIMN		-177	// J2-软限位
#define OTS_USLIMN		-178	// J3-软限位
#define OTS_RSLIMN		-179	// J4-软限位
#define OTS_BSLIMN		-180	// J5-软限位
#define OTS_TSLIMN		-181	// J6-软限位
// ]] 软极限报警

//////////////////////////////////////////////////////////////////////////AMP错误宏
// Amplifier specific error objects
// #define  AmpErrorNodeState     "Drive state invalid for operation"
// #define  AmpErrorpvtSegPos     "PVT segment position out of range"
// #define  AmpErrorpvtSegVel     "PVT segment velocity out of range"
// #define  AmpErrorpvtBufferFull "PVT trajectory buffer full"
// #define  AmpErrorbadDeviceID   "Device does not seem to be a Copley amplifier"
// #define  AmpErrorbadHomeParam  "Bad parameter passed to home function"
// #define  AmpErrorbadMoveParam  "Bad parameter passed to move function"
// #define  AmpErrorInMotion      "The amplifier is currently executing a move"
// #define  AmpErrorGuardError    "The amplifier did not respond to a node guarding or heartbeat message in time"
// #define  AmpErrorFault         "The amplifier detected a latching fault"
// #define  AmpErrorShortCircuit  "The amplifier detected a short circuit condition"
// #define  AmpErrorAmpTemp       "The amplifier detected an over temperature error"
// #define  AmpErrorMotorTemp     "The amplifier detected a motor temperature error"
// #define  AmpErrorOverVolt      "The amplifier detected an over voltage condition"
// #define  AmpErrorUnderVolt     "The amplifier detected an under voltage condition"
// #define  AmpErrorEncoderPower  "The amplifier detected an encoder power error"
// #define  AmpErrorPhaseErr      "The amplifier detected a phasing error"
// #define  AmpErrorTrackErr      "The amplifier detected a tracking error."
// #define  AmpErrorPosLim        "Positive limit switch is active"
// #define  AmpErrorNegLim        "Negative limit switch is active"
// #define  AmpErrorPosSoftLim    "Positive software limit is active"
// #define  AmpErrorNegSoftLim    "Negative software limit is active"
// #define  AmpErrorTrackWarn     "Position tracking warning"
// #define  AmpErrorUnknown       "An unknown amplifier error occurred"
// #define  AmpErrorReset         "An amplifier reset was detected"
// #define  AmpErrorDisabled      "The amplifier is currently disabled"
// #define  AmpErrorQuickStopMode "The amplifier is currently in quick stop mode"
// #define  AmpErrorNoUserUnits   "User units are not enabled in CML_Settings.h"
// #define  AmpErrorAbort         "Trajectory aborted"
// #define  AmpErrorpvtPosUnavail "The PVT segment position is not available."
// #define  AmpErrorVelWin        "Velocity tracking window exceeded."
// 
// #define  AmpFaultMemory        "Fatal hardware error: Amplifier flash data is corrupt."
// #define  AmpFaultADC           "Fatal hardware error: An A/D offset error has occurred."
// #define  AmpFaultShortCircuit  "The amplifier latched a short circuit condition"
// #define  AmpFaultAmpTemp       "The amplifier latched an over temperature error"
// #define  AmpFaultMotorTemp     "The amplifier latched a motor temperature error"
// #define  AmpFaultOverVolt      "The amplifier latched an over voltage condition"
// #define  AmpFaultUnderVolt     "The amplifier latched an under voltage condition"
// #define  AmpFaultEncoderPower  "The amplifier latched an encoder power error"
// #define  AmpFaultPhaseErr      "The amplifier latched a phasing error"
// #define  AmpFaultTrackErr      "The amplifier latched a tracking error."
// #define  AmpFaultI2TLimit      "Current limited by i^2t algorithm."
// #define  AmpFaultUnknown 	   "Some unknown amplifier latched fault has occurred"
// 
// #define  TrjErrorNoneAvailable "No trajectory segments currently available"

//////////////////////////////////////////////////////////////////////////
// #define  AmpErrorNodeState     -182
// #define  AmpErrorpvtSegPos     -183
// #define  AmpErrorpvtSegVel     -184
// #define  AmpErrorpvtBufferFull -185
// #define  AmpErrorbadDeviceID   -186
// #define  AmpErrorbadHomeParam  -187
// #define  AmpErrorbadMoveParam  -188
// #define  AmpErrorInMotion      -189
// #define  AmpErrorGuardError    -190
// #define  AmpErrorFault         -191
// #define  AmpErrorShortCircuit  -192
// #define  AmpErrorAmpTemp       -193
// #define  AmpErrorMotorTemp     -194
// #define  AmpErrorOverVolt      -195
// #define  AmpErrorUnderVolt     -196
// #define  AmpErrorEncoderPower  -197
// #define  AmpErrorPhaseErr      -198
// #define  AmpErrorTrackErr      -199
// #define  AmpErrorPosLim        -200
// #define  AmpErrorNegLim        -201
// #define  AmpErrorPosSoftLim    -202
// #define  AmpErrorNegSoftLim    -203
// #define  AmpErrorTrackWarn     -204
// #define  AmpErrorUnknown       -205
// #define  AmpErrorReset         -206
// #define  AmpErrorDisabled      -207
// #define  AmpErrorQuickStopMode -208
// #define  AmpErrorNoUserUnits   -209
// #define  AmpErrorAbort         -210
// #define  AmpErrorpvtPosUnavail -211
// #define  AmpErrorVelWin        -212
// 
// #define  AmpFaultMemory        -213
// #define  AmpFaultADC           -214
// #define  AmpFaultShortCircuit  -215
// #define  AmpFaultAmpTemp       -216
// #define  AmpFaultMotorTemp     -217
// #define  AmpFaultOverVolt      -218
// #define  AmpFaultUnderVolt     -219
// #define  AmpFaultEncoderPower  -220
// #define  AmpFaultPhaseErr      -221
// #define  AmpFaultTrackErr      -222
// #define  AmpFaultI2TLimit      -223
// #define  AmpFaultUnknown 	   -224
// 
// #define  TrjErrorNoneAvailable -225

///////////////////////////////////////////////////////////////////// 参数描述宏
#define IN       // 输入参数 
#define OUT      // 输出参数 
#define DUMMY    // 哑元参数－不使用参数的值，仅为帮助函数重载解析等目的而设置的参数 
#define OPTIONAL // 可选参数－通常指可以为NULL的指针参数，带默认值的参数不需要这样标明 
#define RESERVED // 保留参数－这个参数当前未被支持，留待以后扩展；或者该参数为内部使用，用户无需关心 
#define CHANGED  // 参数类型或用途与较早版本相比发生了变化 
#define ADDED    // 新增的参数 
#define NOTE     // 需要注意的参数－参数意义发生变化或者与习惯用法不同 
#define WRKBUF   // 工作缓冲区－为避免频繁分配临时资源而传入的临时工作区 
#define TODO     // 尚未实现的接口、类、算法等 
#define UNDONE   // 已取消的接口、类、算法等 
#define FOR_DBG  // 标记为调试方便而临时增加的代码 


#endif
