#ifndef __Matrix_H__
#define __Matrix_H__
/*****************************************************************************
 *        矩阵定义                                                           *
 *        SCUT, 2011                                                         *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2011-03-28                                       *
 *****************************************************************************/
#include "Robot.h"

// 位姿矩阵类
class MtxKine
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

class Matrix
{
public:
	double* Mtx;    // 动态分配用来存放数组的空间
	int     Row;    // 矩阵的行数
    int     Col;    // 矩阵的列数
    int     N;      // 矩阵元素个数

public:
	Matrix();                // 定义一个矩阵，需要用Init来初始化
	Matrix(IN int row, IN int col);                // 初始化为row行col列的零矩阵
	Matrix(IN int row, IN int col, IN double mtx[]);  // 用数组创建一个矩阵
    Matrix(IN const Matrix &obj);               // copy构造矩阵
	~Matrix();

	// 初始化由Matrix()创建的矩阵
	void Init(IN int row, IN int col);               // 初始化为row行col列的零矩阵
	void Init(IN int row, IN int col, IN double mtx[]); // 用数组初始化一个矩阵
	void Init(IN const Matrix &obj);              // copy初始化矩阵

	int GetRow()const { return this->Row; }// 访问矩阵行数
	int GetCol()const { return this->Col; }// 访问矩阵列数
	int GetN()const   { return this->N;   }// 访问矩阵元素个数
    double* GetMtx()const { return this->Mtx; }// 获取该矩阵的数组

	// 用下标访问矩阵元素
	double Get(IN const int i, IN const int j)const; 
	// 用下标修改矩阵元素值
    void Set(IN const int i, IN const int j, IN const double e); 

public:
	// 重载了一些常用操作符，包括 +，-，x，=，负号，正号，
    // A = B
    Matrix &operator= (const Matrix &obj);
	// +A
	Matrix operator+ ()const;
    // -A
    Matrix  operator- ()const;


public:
    // A + B
    friend Matrix operator+ (IN const Matrix &A, IN const Matrix &B);
    // A - B
    friend Matrix operator- (IN const Matrix &A, IN const Matrix &B);
    // A * B 两矩阵相乘
    friend Matrix operator* (IN const Matrix &A, IN const Matrix &B);
    // a * B 实数与矩阵相乘
    friend Matrix operator* (IN const double &a, IN const Matrix &B);
	// A x B 两矩阵x乘
	friend Matrix operator& (IN const Matrix &A, IN const Matrix &B);

    // A 的转置
    friend Matrix Trv(IN const Matrix &A);
    // A 的行列式值，采用列主元消去法
    // 求行列式须将矩阵化为三角阵,此处为了防止修改原矩阵，采用传值调用
    friend double Det(IN Matrix m);
    // A 的逆矩阵，采用高斯-若当列主元消去法
    friend int Inv(IN Matrix* in, OUT Matrix* B);
	friend Matrix Inv(IN Matrix in, OUT Matrix B);
	
	// 齐次矩阵的逆矩阵
	friend int Inv_Trans(IN Matrix* in, OUT Matrix* B);
};

class Vector
{
public:
	double X;
	double Y;
	double Z;

	Vector();                // 定义一个向量
	Vector(IN double x, IN double y, IN double z);    // 初始化向量
	~Vector();

public:
	// 重载了一些常用操作符，包括 +，-，x，=，负号，正号，
    // A = B
    Vector &operator= (const Vector &obj);
	// +A
	Vector operator+ ()const;
    // -A
    Vector operator- ()const;

    // A + B
    friend Vector operator+ (IN const Vector &A, IN const Vector &B);
    // A - B
    friend Vector operator- (IN const Vector &A, IN const Vector &B);
    // A * B
    friend double operator* (IN const Vector &A, IN const Vector &B);
    // a * B 实数与向量相乘
    friend Vector operator* (IN const double &a, IN const Vector &B);
	// A x B 两向量x乘
	friend Vector operator& (IN const Vector &A, IN const Vector &B);

	friend Vector Norm_Vec(IN Vector &A);
	friend double Mulx(Vector A, Vector B);
	friend double Muly(Vector A, Vector B);
	friend double Mulz(Vector A, Vector B);

};


#endif
