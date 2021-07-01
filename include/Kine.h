#ifndef KINE_H__
#define KINE_H__

#define IN  //输入参数
#define OUT //输出参数
/***********************************************************
* Climbot-6D运动学定义
* GDUT,2020
***********************************************************/
#include <vector>
#include "pole.h"
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>


class Kine
{
	public:
		Kine();
		~Kine();

	/***********************************************************
	* 函数：FKine()
	* 功能：正解
	*
	* 输入：double* gdJPos - 关节转角，6关节
	* 输出：double* gdCPos - 正解位姿，(x,y,z,w,p,r)
	* 返回：int — 0 成功，其他错误
	**********************************************************/
		virtual int FKine(IN double gdJPos[], OUT double gdCPos[])=0;

	/***********************************************************
	* 函数：IKine()
	* 功能：逆解
	*
	* 输入：double* gdCPos - 位姿数组，(x,y,z,w,p,r)
	*      double* gdJCurr - 当前关节转角，6关节            
	* 输出：double* gdJPos - 逆解关节转角，6关节
	* 返回：int — 0 成功，其他错误
	**********************************************************/
		virtual int IKine(IN double gdCPos[], IN double gdJCurr[], OUT double gdJPos[])=0;


		protected:
		//将角度变换为(-360,0)或(0,+360)范围内
		void RadInRange(double* pdRad, double* pdDeg);
};

/***********************************************************
* 六自由度攀爬机器人运动学-G1
***********************************************************/
class Kine_CR_SixDoF_G1: public Kine
{
	public:
		// 初始化杆长 
		void Set_Length(IN double gdLen[]);
		// 正解
		int FKine(IN double gdJPos[6], OUT double gdCPos[6]);
		// 逆解
		int IKine(IN double gdCPos[6], IN double gdJCurr[6], OUT double gdJPos[6]);
	private:
		double L01;
		double L2;
		double L34;
		double L56;
};

/***********************************************************
* 六自由度攀爬机器人运动学-G2
***********************************************************/
class Kine_CR_SixDoF_G2: public Kine
{
	public:
	// 初始化杆长 
		void Set_Length(IN double gdLen[7]);
	// 正解
		int FKine(IN double gdJPos[6], OUT double gdCPos[6]);
	// 逆解
		int IKine(IN double gdCPos[6], IN double gdJCurr[6], OUT double gdJPos[6]);
	private:
    	Kine_CR_SixDoF_G1 kine;
		double L_G2[4];
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

	int FKine_Inc(IN double gdJPos[5], IN double inc[3], OUT double gdCPos[3]);
	
private:
	Kine_CR_FiveDoF_G1 kine;
};


void Trans_PosToMtx(double* pos, MtxKine* output, int inv);  //实现物体坐标系在夹持器坐标系下的矩阵表示
void Trans_MtxToPos(MtxKine* input, double* outpos);

/***********************************************************
* 函数功能：根据机器人的构型求解其四根连杆在全局坐标系下的端点
* 输入：机器人杆长：gdLen(7),当前关节角：gdJPos(6)
* 输出：机器人四根连杆的端点:Linkage6D
***********************************************************/
void Linkage6D(IN double gdLen[7],IN double* gdJPos,std::vector<double> &cur_truss,int length,int angle,const int Grip_Id,
				std::vector<std::vector<double>> &Linkage);


#endif