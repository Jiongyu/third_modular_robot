/*****************************************************************************
 *        矩阵操作                                                           *
 *        SCUT, 2011                                                         *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2011-03-28                                       *
 *****************************************************************************/
#include <stdlib.h>
#include <math.h>
#include "Matrix.h"
#include "Robot.h"
#include <iostream>

Matrix::Matrix()
{}
/*****************************************************************************
 * 函数：Matrix()
 * 功能：构造一个row行col列的零矩阵
 *
 * 输入：int row   - 行数
 *       int col   - 列数
 *****************************************************************************/
Matrix::Matrix(int row, int col)
{
	Init(row, col);
}

void Matrix::Init(int row, int col)
{
	int i;	
	
	this->Row = row;
	this->Col = col;
	this->N = this->Row * this->Col;
	
	this->Mtx = new double[this->N];	
	
	for(i = 0; i < this->N; i++)
	{
		this->Mtx[i] = 0.0;
	}
}

/*****************************************************************************
 * 函数：Matrix()
 * 功能：构造一个row行col列的矩阵, 由数组赋值
 *
 * 输入：int row   - 行数
 *       int col   - 列数
 *       double mtx[] - row*col数组
 *****************************************************************************/
Matrix::Matrix(int row, int col, double mtx[])
{
	Init(row, col, mtx);

}

/*****************************************************************************
 * 函数：Matrix()
 * 功能：初始化一个row行col列的矩阵, 由数组赋值
 *
 * 输入：int row   - 行数
 *       int col   - 列数
 *       double mtx[] - row*col数组
 *****************************************************************************/
void Matrix::Init(int row, int col, double mtx[])
{
	int i;
	
	this->Row = row;
	this->Col = col;
	this->N = this->Row * this->Col;
	
	this->Mtx = new double[this->N];
	
	
	for (i=0; i<this->N; i++)
	{
		this->Mtx[i] = mtx[i];
	}	
}

/*****************************************************************************
 * 函数：Matrix()
 * 功能：拷贝构造函数，因为成员变量含有动态空间，防止传递参数等操作发生错误
 *****************************************************************************/
Matrix::Matrix(const Matrix &obj)
{
	Init(obj);

}
void Matrix::Init(const Matrix &obj)
{
	int i;
	
    this->Row = obj.GetRow();                                         
    this->Col = obj.GetCol();                                       
    this->N   = obj.GetN();                                            
    this->Mtx = new double[N];
	
    for (i=0; i<N; i++)
	{
		this->Mtx[i] = obj.GetMtx()[i];
	}
}

Matrix::~Matrix()
{
	delete[] this->Mtx;
}


/*****************************************************************************
 * 函数：Get()
 * 功能：获取矩阵元素,注意这里矩阵下标从(0,0)开始
 *****************************************************************************/
double Matrix::Get(const int i, const int j)const
{	
    return this->Mtx[i * this->Col + j];
}

/*****************************************************************************
 * 函数：Set()
 * 功能：修改矩阵元素
 *****************************************************************************/
void Matrix::Set(const int i, const int j, const double e)
{
	this->Mtx[i * this->Col + j] = e; 
}

/*****************************************************************************
 * 函数：=
 * 功能：等号
 *****************************************************************************/
Matrix &Matrix::operator= (const Matrix &obj)
{                
    if(this == &obj)    // 将一个矩阵赋给它自己时简单做返回即可
       return *this;

    delete[] this->Mtx; // 首先删除目的操作数的动态空间
    this->Row = obj.GetRow();
    this->Col = obj.GetCol();
    this->N   = obj.GetN();
    this->Mtx = new double[N]; // 重新分配空间保存obj的数组

    for (int i=0; i<N; i++)
	{
       this->Mtx[i] = obj.GetMtx()[i];  
	}

    return *this;
} 
  
/*****************************************************************************
 * 函数：+
 * 功能：正号
 *****************************************************************************/
Matrix Matrix::operator+ ()const
{
	return *this;
}   

/*****************************************************************************
 * 函数：-
 * 功能：负号操作符，返回值为该矩阵的负矩阵，原矩阵不变
 *****************************************************************************/
Matrix Matrix::operator- ()const
{                
    // 为了不改变原来的矩阵，此处从新构造一个矩阵
    Matrix _A(this->Row, this->Col);

    for (int i=0; i<_A.N; i++)
       _A.Mtx[i] = -(this->Mtx[i]);
    return _A;       
}   

/*****************************************************************************
 * 函数：+
 * 功能：矩阵加法
 *****************************************************************************/
Matrix operator+ (const Matrix &A, const Matrix &B)
{                
    Matrix AB(A.Row, A.Col);

    if (A.Row!=B.Row || A.Col!=B.Col)
	{
		// 如果矩阵A和B行列数不一致则不可相加
		exit(0);        
    }

    for (int i=0; i<AB.N; i++)
       AB.Mtx[i] = A.Mtx[i] + B.Mtx[i];
  
	return AB;       
} 

/*****************************************************************************
 * 函数：-
 * 功能：矩阵减法，用加上一个负矩阵来实现
 *****************************************************************************/
Matrix operator- (const Matrix &A, const Matrix &B)
{                
    return (A + (-B));
}

/*****************************************************************************
 * 函数：*
 * 功能：矩阵乘法
 *
 * 输入：Matrix &A - 矩阵A
 *       Matrix &B - 矩阵B
 * 返回：Matrix - 相乘结果矩阵
 *****************************************************************************/
Matrix operator* (const Matrix &A, const Matrix &B)
{                
    if (A.Col != B.Row)
	{
		// A的列数必须和B的行数一致
		//cout << "Can't multiply\n";
		exit(0);
    }                                                                          

    Matrix AB(A.Row, B.Col); // AB用于保存乘积

    for (int i=0; i<AB.Row; i++)
	{
		for (int j=0; j<AB.Col; j++)
		{
			for (int k=0; k<A.Col; k++)
				AB.Set(i, j, AB.Get(i,j) + A.Get(i,k)*B.Get(k,j));
		}
	}
	
	return AB;
}

/*****************************************************************************
 * 函数：*
 * 功能：矩阵与实数相乘        
 *
 * 输入：double &a - 实数A
 *       Matrix &B - 矩阵B
 * 返回：Matrix - 相乘结果矩阵
 *****************************************************************************/
Matrix operator* (const double &a, const Matrix &B)
{
    Matrix aB(B);
    for (int i=0; i<aB.Row; i++)
	{
       for (int j=0; j<aB.Col; j++)
          aB.Set(i,j, a*B.Get(i,j));
	}

    return aB;
}

Matrix operator& (const Matrix &A, const Matrix &B)
{
    Matrix AB(A.Row, A.Col);
	
    if (A.Row!=B.Row || A.Col!=B.Col)
	{
		// 如果矩阵A和B行列数不一致则不可相x
		exit(0);
    }

	if ( (A.Row != 3) || (A.Col != 1) )
	{
		// 非3x1矩阵不能相x
		exit(1);
	}

	AB.Set(0,0,   A.Get(1,0) * B.Get(2,0) - A.Get(2,0) * B.Get(1,0));
	AB.Set(1,0, - A.Get(0,0) * B.Get(2,0) + A.Get(2,0) * B.Get(0,0));
	AB.Set(2,0,   A.Get(0,0) * B.Get(1,0) - A.Get(1,0) * B.Get(0,0));

	return AB;
}

/*****************************************************************************
 * 函数：Trv()
 * 功能：矩阵的转置
 * 描述：矩阵的转置 将(i,j)与(j,i)互换
 *       此函数返回一个矩阵的转置矩阵，并不改变原来的矩阵         
 *
 * 输入：Matrix* A - 矩阵A
 * 返回：Matrix* B - 矩阵B
 *****************************************************************************/
Matrix Trv(const Matrix &A)
{
    Matrix B(A.Col, A.Row);

    for (int i=0; i<B.Row; i++)
	{
       for (int j=0; j<B.Col; j++)
          B.Set(i, j, A.Get(j,i));
	}
    
	return B;       
}

/*****************************************************************************
 * 函数：Det()
 * 功能：矩阵行列式值 - 采用列主元消去法
 *
 * 输入：Matrix* m  - 矩阵m
 *
 * 输出：
 * 返回：double - 行列式值
 *****************************************************************************/
double Det(Matrix m)
{
	int i, j, k, ind;
	double detValue;
	double max;
	double temp;
	
    if (m.Row != m.Col)      // 矩阵必须为n*n的才可进行行列式求值 
	{          
        exit(0);             // 如果不满足行列数相等返回
	}
	
	Matrix A(m);
	
    detValue = 1.0;  // 用于保存行列式值
	
	for (i=0; i<A.Row-1; i++)    // 需要n-1步列化零操作
	{ 
		//------------------ 选主元 ---------------------------------
		max = fabs(A.Mtx[i * A.Col + i]);// 主元初始默认为右下方矩阵首个元素
		ind = i;                         // 主元行号默认为右下方矩阵首行 
		
		for (j=i+1; j<A.Row; j++)   // 选择列主元
		{
			if (fabs(A.Mtx[j * A.Col + i]) > max)    // 遇到绝对值更大的元素
			{
				max = fabs(A.Mtx[j * A.Col + i]);   // 更新主元值
				ind = j;                            // 更新主元行号
			}
		}//loop j    
		
		//------------------- 移动主元行 -----------------------------
		if (fabs(max) < RT_LITTLE)
		{
			return 0.0;    // 右下方矩阵首行为零，显然行列式值为零
		}
		
		if (ind != i)                      // 主元行非右下方矩阵首行   
		{ 
			for (j=i; j<A.Row; j++)// 将主元行与右下方矩阵首行互换      
			{
				temp = A.Mtx[i * A.Col + j];
				A.Mtx[i * A.Col + j] = A.Mtx[ind * A.Col + j];
				A.Mtx[ind * A.Col + j] = temp;
			}
			detValue = -detValue;          // 互换行列式两行，行列式值反号
		}                            
		
		//------------------- 消元 ---------------------------------- 
		for (j=i+1; j<A.Row; j++)      // 遍历行
		{
			temp = A.Mtx[j * A.Col + i] / A.Mtx[i * A.Col + i];
			for (k=i; k<A.Row; k++)// 遍历行中每个元素，行首置0
			{
				A.Mtx[j * A.Col + k] = 
					A.Mtx[j * A.Col + k] - temp * A.Mtx[i * A.Col + k];
			}
		} 
		
		// 每步消元都会产生一个对角线上元素，将其累乘
		detValue *= A.Mtx[i * A.Col + i]; 
	}// loop i 
    
	// 注意矩阵最后一个元素在消元的过程中没有被累乘到
	detValue *= A.Mtx[(A.Row - 1) * A.Col + (A.Row - 1)];
	
	return detValue;
}//Det()        

/*****************************************************************************
 * 函数：Inv()
 * 功能：逆矩阵 - 高斯-若当消去法，按列选主元
 *
 * 输入：Matrix* in  - 矩阵in
 *
 * 输出：Matrix* B - 矩阵B
 * 返回：int - 0正常，其他错误
 *****************************************************************************/                             
int Inv(Matrix* in, Matrix* B)
{
	int i, j, k;
	int ind;
	double max;
	double temp;	

	if ((in->Row != in->Col) ||       // 只可求狭义逆矩阵，即行列数相同
		(B->Row != B->Col) ||
		(in->Row != B->Row) )
	{
	   return -1;  // 输入数据有误
    }

	Matrix A(in->Row, in->Col);
	for (i=0; i<in->N; i++)
	{
		A.Mtx[i] = in->Mtx[i];
	}
    
	// 对角元素置1,其余元素为0
    for (i=0; i<A.Row; i++)
	{
       for (j=0; j<A.Col; j++)
	   {
          if (i == j)
			  B->Mtx[i * B->Col + j] = 1.0;
		  else
			  B->Mtx[i * B->Col + j] = 0.0;
	   }
	}
     
    // 对矩阵A进行A.Row次消元运算，每次保证第i列只有对角线上非零
    // 同时以同样的操作施与矩阵B，结果A变为单位阵B为所求逆阵
    for (i=0; i<A.Row; i++)
	{
		//------------------ 选主元 --------------------------------------
		max = fabs(A.Mtx[i * A.Col + i]); // 主元初始默认为右下方矩阵首个元素
		ind = i;                  // 主元行号默认为右下方矩阵首行
		// 结果第ind行为列主元行
		for (j=i+1; j<A.Row; j++)
		{
			if (fabs(A.Mtx[j * A.Col + i]) > max)   // 遇到绝对值更大的元素
			{
				max = fabs(A.Mtx[j * A.Col +i]);   // 更新主元值
				ind = j;                 // 更新主元行号
			}
		}  
		
		if (fabs(max) < RT_LITTLE)
		{
			return -2; // 行列式为零,无逆矩阵
		}
				                                  
       //------------------- 移动主元行 --------------------------------
       if (ind != i)                      // 主元行不是右下方矩阵首行
	   {
		   for (j=i; j<A.Row; j++)    // 将主元行与右下方矩阵首行互换
		   {
			   temp = A.Mtx[i * A.Col + j];

			   A.Mtx[i * A.Col + j] = A.Mtx[ind * A.Col + j];
			   A.Mtx[ind * A.Col + j] = temp;
		   }
		   for (j=0; j<B->Row; j++)
		   {
			   temp = B->Mtx[i * B->Col + j]; // 对矩阵B施以相同操作
			   // B与A阶数相同，可在一个循环中
			   B->Mtx[i * B->Col + j] = B->Mtx[ind * B->Col + j];
			   B->Mtx[ind * B->Col + j] = temp;
		   }
	   }
	   //--------------------- 消元 -----------------------------------
	   // 第k次消元操作，以第k行作为主元行，将其上下各行的第k列元素化为零
       // 同时以同样的参数对B施以同样的操作，此时可以将B看作A矩阵的一部分
	   for (k=0; k<A.Col; k++)
	   {
		   if (k != i)
		   {
			   temp = - A.Mtx[k * A.Col + i] / A.Mtx[i * A.Col + i];
			   for (j=0; j<A.Row; j++)
			   {
				   A.Mtx[k * A.Col + j] = 
				   		A.Mtx[k * A.Col + j] + temp * A.Mtx[i * A.Col + j];
			   }
			   for (j=0; j<B->Row; j++)
			   {
				   B->Mtx[k * B->Col + j] = 
				   		B->Mtx[k * B->Col + j] + temp * B->Mtx[i * B->Col +j];
			   }
		   }//end if 
	   }//loop k
	   
	   temp = 1.0 / A.Mtx[i * A.Col + i];
	   
	   for (j=0; j<A.Row; j++)
	   {
		   A.Mtx[i * A.Col + j] = temp * A.Mtx[i * A.Col + j];
	   }
	   for (j=0; j<B->Row; j++)
	   {
		   B->Mtx[i * B->Col + j] = temp * B->Mtx[i * B->Col + j];
	   }
	}//loop i
	
	return 0;
}// Inv()


Matrix Inv(Matrix in, Matrix B)
{
	int i, j, k;
	int ind;
	double max;
	double temp;	

	if ((in.Row != in.Col) ||       // 只可求狭义逆矩阵，即行列数相同
		(B.Row != B.Col) ||
		(in.Row != B.Row) )
	{
		 std::cout<<"输入数据有误"<<std::endl;
		 //return -1;
    }

	Matrix A(in.Row, in.Col);
	for (i=0; i<in.N; i++)
	{
		A.Mtx[i] = in.Mtx[i];
	}
    
	// 对角元素置1,其余元素为0
    for (i=0; i<A.Row; i++)
	{
       for (j=0; j<A.Col; j++)
	   {
          if (i == j)
			  B.Mtx[i * B.Col + j] = 1.0;
		  else
			  B.Mtx[i * B.Col + j] = 0.0;
	   }
	}
     
    // 对矩阵A进行A.Row次消元运算，每次保证第i列只有对角线上非零
    // 同时以同样的操作施与矩阵B，结果A变为单位阵B为所求逆阵
    for (i=0; i<A.Row; i++)
	{
		//------------------ 选主元 --------------------------------------
		max = fabs(A.Mtx[i * A.Col + i]); // 主元初始默认为右下方矩阵首个元素
		ind = i;                  // 主元行号默认为右下方矩阵首行
		// 结果第ind行为列主元行
		for (j=i+1; j<A.Row; j++)
		{
			if (fabs(A.Mtx[j * A.Col + i]) > max)   // 遇到绝对值更大的元素
			{
				max = fabs(A.Mtx[j * A.Col +i]);   // 更新主元值
				ind = j;                 // 更新主元行号
			}
		}  
		
		if (fabs(max) < RT_LITTLE)
		{
			std::cout<<"行列式为零,无逆矩阵"<<std::endl;
			//return -2; // 
		}
				                                  
       //------------------- 移动主元行 --------------------------------
       if (ind != i)                      // 主元行不是右下方矩阵首行
	   {
		   for (j=i; j<A.Row; j++)    // 将主元行与右下方矩阵首行互换
		   {
			   temp = A.Mtx[i * A.Col + j];

			   A.Mtx[i * A.Col + j] = A.Mtx[ind * A.Col + j];
			   A.Mtx[ind * A.Col + j] = temp;
		   }
		   for (j=0; j<B.Row; j++)
		   {
			   temp = B.Mtx[i * B.Col + j]; // 对矩阵B施以相同操作
			   // B与A阶数相同，可在一个循环中
			   B.Mtx[i * B.Col + j] = B.Mtx[ind * B.Col + j];
			   B.Mtx[ind * B.Col + j] = temp;
		   }
	   }
	   //--------------------- 消元 -----------------------------------
	   // 第k次消元操作，以第k行作为主元行，将其上下各行的第k列元素化为零
       // 同时以同样的参数对B施以同样的操作，此时可以将B看作A矩阵的一部分
	   for (k=0; k<A.Col; k++)
	   {
		   if (k != i)
		   {
			   temp = - A.Mtx[k * A.Col + i] / A.Mtx[i * A.Col + i];
			   for (j=0; j<A.Row; j++)
			   {
				   A.Mtx[k * A.Col + j] = 
				   		A.Mtx[k * A.Col + j] + temp * A.Mtx[i * A.Col + j];
			   }
			   for (j=0; j<B.Row; j++)
			   {
				   B.Mtx[k * B.Col + j] = 
				   		B.Mtx[k * B.Col + j] + temp * B.Mtx[i * B.Col +j];
			   }
		   }//end if 
	   }//loop k
	   
	   temp = 1.0 / A.Mtx[i * A.Col + i];
	   
	   for (j=0; j<A.Row; j++)
	   {
		   A.Mtx[i * A.Col + j] = temp * A.Mtx[i * A.Col + j];
	   }
	   for (j=0; j<B.Row; j++)
	   {
		   B.Mtx[i * B.Col + j] = temp * B.Mtx[i * B.Col + j];
	   }
	}//loop i
	
	return B;
}// Inv()

int Inv_Trans(Matrix* in, Matrix* B)
{
	int i,j;
	double temp;

	if ((in->Row != in->Col) ||   // 4*4齐次变换矩阵
		(B->Row  != B->Col)  ||
		(in->Row != B->Row)  ||
		(in->Row != 4) )
	{
		return -1;  // 输入数据有误
    }

    for (i=0; i<in->Row - 1; i++)
	{
		for (j=0; j<in->Col - 1; j++)
			B->Set(i, j, in->Get(j,i));

		B->Set(3, i, 0); // 
		B->Set(i, 3, 0); //
	}
	B->Set(3, 3, 0);


	for (i=0; i<B->Row - 1; i++)
	{
		temp = 0;
		for (j=0; j<in->Col-1; j++)
		{
			B->Set(i, j, in->Get(j,i));
			temp += B->Get(i, j) * in->Get(i, 3);
		}

		B->Set(i, 3, temp);
	}


	return 0;
}



Vector::Vector()
{}
/*****************************************************************************
 * 函数：Vector()
 * 功能：构造一个向量
 *
 * 输入：double x  - x
 *       double y  - y
 *       double z  - z
 *****************************************************************************/
Vector::Vector(double x, double y, double z)
{
	X = x;
	Y = y;
	Z = z;
}

Vector::~Vector()
{
}

/*****************************************************************************
 * 函数：=
 * 功能：等号
 *****************************************************************************/
Vector &Vector::operator= (const Vector &obj)
{                
    if(this == &obj)    // 将一个向量赋给它自己时简单做返回即可
       return *this;

	this->X = obj.X;
	this->Y = obj.Y;
	this->Z = obj.Z;

    return *this;
} 
  
/*****************************************************************************
 * 函数：+
 * 功能：正号
 *****************************************************************************/
Vector Vector::operator+ ()const
{
	return *this;
}   

/*****************************************************************************
 * 函数：-
 * 功能：负号操作符，返回值为该向量的负向量，原向量不变
 *****************************************************************************/
Vector Vector::operator- ()const
{                
    // 为了不改变原来的向量，此处从新构造一个向量
    Vector _A;

    _A.X = -this->X;
	_A.Y = -this->Y;
	_A.Z = -this->Z;

    return _A;       
}  


/*****************************************************************************
 * 函数：+
 * 功能：向量加法
 *****************************************************************************/
Vector operator+ (const Vector &A, const Vector &B)
{                
    Vector AB;

	AB.X = A.X + B.X;
	AB.Y = A.Y + B.Y;
	AB.Z = A.Z + B.Z;
  
	return AB;       
} 

/*****************************************************************************
 * 函数：-
 * 功能：向量减法，用加上一个负向量来实现
 *****************************************************************************/
Vector operator- (const Vector &A, const Vector &B)
{                
    return (A + (-B));
}

/*****************************************************************************
 * 函数：*
 * 功能：向量乘法
 *
 * 输入：Vector &A - 向量A
 *       Vector &B - 向量B
 * 返回：double - 相乘结果
 *****************************************************************************/
double operator* (const Vector &A, const Vector &B)
{    
	return A.X*B.X + A.Y* B.Y + A.Z*B.Z;
}

/*****************************************************************************
 * 函数：*
 * 功能：向量与实数相乘        
 *
 * 输入：double &a - 实数A
 *       Vector &B - 向量B
 * 返回：Vector - 相乘结果向量
 *****************************************************************************/
Vector operator* (const double &a, const Vector &B)
{
    Vector aB;
    
	aB.X = a*B.X;
	aB.Y = a*B.Y;
	aB.Z = a*B.Z;

    return aB;
}

Vector operator& (const Vector &A, const Vector &B)
{
    Vector AB;
	
	AB.X = A.Y*B.Z - A.Z*B.Y;
	AB.Y = A.Z*B.X - A.X*B.Z;
	AB.Z = A.X*B.Y - A.Y*B.X;  

	return AB;
}

Vector Norm_Vec(IN Vector &A)
{
	Vector _A;
	double temp = sqrt(SQUARE(A.X) + SQUARE(A.Y) + SQUARE(A.Z));
	if (temp< RT_LITTLE)
	{
		_A.X = 0;
		_A.Y = 0;
		_A.Z = 0;
	}
	else
	{
		_A.X = A.X / temp;
		_A.Y = A.Y / temp;
		_A.Z = A.Z / temp;
	}

	return _A;
}

double Mulx(IN Vector A, IN Vector B)
{
	return A.Y*B.Z - A.Z*B.Y;
}

double Muly(IN Vector A, IN Vector B)
{
	return A.Z*B.X - A.X*B.Z;
}

double Mulz(IN Vector A, IN Vector B)
{
	return A.X-B.Y - A.Y-B.X;
}




