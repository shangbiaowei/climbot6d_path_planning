#ifndef M_Matrix_H__
#define M_Matrix_H__
/*****************************************************************************
 *        矩阵定义                                                           *
 *        GDUT, 2020                                                        *                                     *
 *****************************************************************************/
//#include "Robot.h"

#define IN  //输入参数
#define OUT //输出参数

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

class m_Matrix
{
public:
	double* Mtx;    // 动态分配用来存放数组的空间
	int     Row;    // 矩阵的行数
    int     Col;    // 矩阵的列数
    int     N;      // 矩阵元素个数

public:
	m_Matrix();                // 定义一个矩阵，需要用Init来初始化
	m_Matrix(IN int row, IN int col);                // 初始化为row行col列的零矩阵
	m_Matrix(IN int row, IN int col, IN double mtx[]);  // 用数组创建一个矩阵
    m_Matrix(IN const m_Matrix &obj);               // copy构造矩阵
	~m_Matrix();

	// 初始化由Matrix()创建的矩阵
	void Init(IN int row, IN int col);               // 初始化为row行col列的零矩阵
	void Init(IN int row, IN int col, IN double mtx[]); // 用数组初始化一个矩阵
	void Init(IN const m_Matrix &obj);              // copy初始化矩阵

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
    m_Matrix &operator= (const m_Matrix &obj);
	// +A
	m_Matrix operator+ ()const;
    // -A
    m_Matrix  operator- ()const;


public:
    // A + B
    friend m_Matrix operator+ (IN const m_Matrix &A, IN const m_Matrix &B);
    // A - B
    friend m_Matrix operator- (IN const m_Matrix &A, IN const m_Matrix &B);
    // A * B 两矩阵相乘
    friend m_Matrix operator* (IN const m_Matrix &A, IN const m_Matrix &B);
    // a * B 实数与矩阵相乘
    friend m_Matrix operator* (IN const double &a, IN const m_Matrix &B);
	// A x B 两矩阵x乘
	friend m_Matrix operator& (IN const m_Matrix &A, IN const m_Matrix &B);

    // A 的转置
    friend m_Matrix Trv(IN const m_Matrix &A);
    // A 的行列式值，采用列主元消去法
    // 求行列式须将矩阵化为三角阵,此处为了防止修改原矩阵，采用传值调用
    friend double Det(IN m_Matrix m);
    // A 的逆矩阵，采用高斯-若当列主元消去法
    friend int Inv(IN m_Matrix* in, OUT m_Matrix* B);
	
	// 齐次矩阵的逆矩阵
	friend int Inv_Trans(IN m_Matrix* in, OUT m_Matrix* B);
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
	// A在B上投影(B为单位向量)
	friend Vector vec_projection(IN const Vector &A, IN const Vector &B);

	// 求解两向量角度
	friend double v_angle(IN const Vector &A, IN const Vector &B);

	friend Vector Norm_Vec(IN Vector &A);
	friend double Mulx(Vector A, Vector B);
	friend double Muly(Vector A, Vector B);
	friend double Mulz(Vector A, Vector B);

};


#endif
