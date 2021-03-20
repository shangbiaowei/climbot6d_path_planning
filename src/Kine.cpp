#include "../include/Kine.h"
#include <iostream>
#include <math.h>
#include <stdio.h>


//windows下math.h/cmath都不包含M_PI定义，Ubuntu下无问题
#ifndef PI
#define PI      3.1415926535898 // 圆周率
#endif
#define PI2     6.2831853071796 // 2倍圆周率 
#define PI_2    1.5707963267949 // 1/2圆周率  
#define PI_RAD  0.0174532925199 // 角度转换为弧度参数
#define PI_DEG  57.2957795130823 // 弧度转换为角度参数

/**********************************************
* 六自由度初始构型
*
*	 ⊙——<|>——⊙——<|>——<
*	 │
*	 ⊙——<|>——<
**********************************************/

/***********************************************************
* 函数：Kine()
* 功能：构造函数
***********************************************************/
Kine::Kine()
{
}

/***********************************************************
* 函数：~Kine()
* 功能：析构函数
***********************************************************/
Kine::~Kine()
{
}

/***********************************************************
* 函数：RadInRange()
* 功能：由当前转角优化计算转角
*           - 与当前值比较,将角度变换为(-360,0)或(0,+360)范围内
*
* 输入：double* id_rad - 计算转角(弧度)
*       double* id_deg - 当前转角(角度)
* 输出：double* id_rad - 计算转角(弧度)
***********************************************************/
void Kine::RadInRange(double* pdRad, double* pdDeg)
{
    if ((*pdRad) > PI2)
	{
		for(;;)
		{
			*pdRad -= PI2;	
			if ((*pdRad) <= PI2)
				{if (fabs(*pdRad - (*pdDeg) * PI_RAD) > fabs(*pdRad - PI2 - (*pdDeg) * PI_RAD))
					*pdRad -= PI2;
				break;}
		}
	}
    else if ((*pdRad) < - PI2)
	{
		for (;;)
		{
			*pdRad += PI2;	
			if((*pdRad) >= - PI2)
				{if( fabs(*pdRad - (*pdDeg) * PI_RAD) > fabs(*pdRad + PI2 - (*pdDeg) * PI_RAD))
					*pdRad += PI2;
				break;}
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

/***********************************************************
* 函数：SetLength()
* 功能：设置杆长
*
* 输入： double l0 — 杆长0
*		double l1 — 杆长1
*		double l2 — 杆长2
*		double l3 — 杆长3
*		double l4 — 杆长4
*		double l5 — 杆长5
*		double l6 — 杆长6
***********************************************************/
void Kine_CR_SixDoF_G1::Set_Length(double* gdLen)
{
	L01 = gdLen[0] + gdLen[1];
	L2 = gdLen[2];
	L34 = gdLen[3] + gdLen[4];
	L56 = gdLen[5] + gdLen[6];

//	Kine.Set_Length(gdLen);
}

/***********************************************************
* 函数：Fkine()
* 功能：正解
*
* 输入： double gdJPos[] - 关节转角，6关节
* 输出： double gdCPos[] - 正解位姿，(x,y,z,w,p,r)
*
* 返回： int - 0 成功
***********************************************************/
int Kine_CR_SixDoF_G1::FKine(double* gdJPos, double* gdCPos)
{
	int i;
	double jrad[6];      //关节弧度

	double  temp[8];   //中间变量
	MtxKine lm_Tcp;      //中间变量，TCP矩阵

	//关节弧度
	/*jrad[0]=gdJPos[0]*PI_RAD;
	jrad[1]=gdJPos[1]*PI_RAD;
	jrad[2]=gdJPos[2]*PI_RAD;
	jrad[3]=gdJPos[3]*PI_RAD;
	jrad[4]=gdJPos[4]*PI_RAD;
	jrad[5]=gdJPos[5]*PI_RAD; */
	for ( i = 0; i < 6; i++)
	{
		jrad[i] = gdJPos[i] * PI_RAD;
//		std::cout << jrad[i] << std::endl;
	}
	

	//TCP矩阵
	double c23 = cos(jrad[1]+jrad[2]);
	double s23 = sin(jrad[1]+jrad[2]);

	//计算TCP姿态矩阵参数
	lm_Tcp.R11 = (cos(jrad[0]) * c23 * cos(jrad[3]) * cos(jrad[4]) + sin(jrad[0]) * sin(jrad[3]) * cos(jrad[4]) - cos(jrad[0]) * s23 * sin(jrad[4])) * cos(jrad[5]) + (-cos(jrad[0]) * c23 * sin(jrad[3]) + sin(jrad[0]) * cos(jrad[3])) * sin(jrad[5]);
	lm_Tcp.R12 = -(cos(jrad[0]) * c23 * cos(jrad[3]) * cos(jrad[4])  + sin(jrad[0]) * sin(jrad[3]) * cos(jrad[4]) - cos(jrad[0]) * s23 * sin(jrad[4])) * sin(jrad[5]) + (-cos(jrad[0]) * c23 * sin(jrad[3]) + sin(jrad[0]) * cos(jrad[3])) * cos(jrad[5]);
	lm_Tcp.R13 = cos(jrad[0]) * c23 * cos(jrad[3]) * sin(jrad[4]) + sin(jrad[0]) * sin(jrad[3]) * sin(jrad[4]) + cos(jrad[0]) * s23 * cos(jrad[4]);
	lm_Tcp.R21 = (sin(jrad[0]) * c23 * cos(jrad[3]) * cos(jrad[4]) - cos(jrad[0]) * sin(jrad[3]) * cos(jrad[4]) - sin(jrad[0]) * s23 * sin(jrad[4])) * cos(jrad[5]) + (-sin(jrad[0]) * c23 * sin(jrad[3]) - cos(jrad[0]) * cos(jrad[3])) * sin(jrad[5]);
	lm_Tcp.R22 = -(sin(jrad[0]) * c23 * cos(jrad[3]) * cos(jrad[4]) - cos(jrad[0]) * sin(jrad[3]) * cos(jrad[4]) - sin(jrad[0]) * s23 * sin(jrad[4])) * sin(jrad[5]) + (-sin(jrad[0]) * c23 * sin(jrad[3]) - cos(jrad[0]) * cos(jrad[3])) * cos(jrad[5]);
	lm_Tcp.R23 = sin(jrad[0]) * c23 * cos(jrad[3]) * sin(jrad[4]) - cos(jrad[0]) * sin(jrad[3]) * sin(jrad[4]) + sin(jrad[0]) * s23 * cos(jrad[4]);
	lm_Tcp.R31 = (s23 * cos(jrad[3]) * cos(jrad[4]) + c23 * sin(jrad[4])) * cos(jrad[5]) - s23 * sin(jrad[3]) * sin(jrad[5]);
	lm_Tcp.R32 = -(s23 * cos(jrad[3]) * cos(jrad[4]) + c23 * sin(jrad[4])) * sin(jrad[5]) - s23 * sin(jrad[3]) * cos(jrad[5]);
	lm_Tcp.R33 = s23 * cos(jrad[3]) * sin(jrad[4]) - c23 * cos(jrad[4]);

	//计算TCP位置
	lm_Tcp.X = L56 * lm_Tcp.R13 + L34 * cos(jrad[0]) * s23 + L2 * cos(jrad[0]) * cos(jrad[1]);
 	lm_Tcp.Y = L56 * lm_Tcp.R23 + L34 * sin(jrad[0]) * s23 + L2 * sin(jrad[0]) * cos(jrad[1]);
	lm_Tcp.Z = L56 * lm_Tcp.R33 - L34 * c23 + L2 * sin(jrad[1]) + L01;


	//计算PRY角
	temp[5] = atan2(- lm_Tcp.R31, sqrt(lm_Tcp.R11 * lm_Tcp.R11 + lm_Tcp.R21 * lm_Tcp.R21));

	if (fabs(temp[5] - PI_2) < 0.00001)
	{
		temp[4] = 0;
		temp[6] = atan2(lm_Tcp.R12, lm_Tcp.R22);
	}
	else if (fabs(temp[5] + PI_2) < 0.00001)
	{
		temp[4] = 0;
		temp[6] = -atan2(lm_Tcp.R12, lm_Tcp.R22);
	}
	else
	{
		temp[0] = 1 / cos(temp[5]);

		temp[1] = lm_Tcp.R21 * temp[0];
		temp[2] = lm_Tcp.R11 * temp[0];
		if (fabs(temp[1]) < 0.00001)
		{
			temp[1] = 0;
		}
		if (fabs(temp[2]) < 0.00001)
		{
			temp[2] = 0;
		}
		temp[4] = atan2(temp[1], temp[2]);

		temp[1] = lm_Tcp.R32 * temp[0];
		temp[2] = lm_Tcp.R33 * temp[0];
		if (fabs(temp[1]) < 0.00001)
		{
			temp[1] = 0;
		}
		if (fabs(temp[2]) < 0.00001)
		{
			temp[2] = 0;
		}
		temp[6] = atan2(temp[1], temp[2]);
	}


/*	std::cout << "temp[4]" << temp[4] << std::endl;
	std::cout << "temp[5]" << temp[5] << std::endl;
	std::cout << "temp[6]" << temp[6] << std::endl;
*/

	gdCPos[0] = lm_Tcp.X;
	gdCPos[1] = lm_Tcp.Y;
	gdCPos[2] = lm_Tcp.Z;
	gdCPos[3] = temp[4] * PI_DEG;
	gdCPos[4] = temp[5] * PI_DEG; 
	gdCPos[5] = temp[6] * PI_DEG;

	return 0;
}

/***********************************************************
* 函数：Ikine()
* 功能：逆解
*
* 输入： double gdCPos[] - 位姿数组，(x,y,z,w,p,r)
* 		double gdJCurr[] - 当前关节转角，6关节
* 输出： double gdJPos[] - 逆解关节转角，6关节
*
* 返回： int - 0 成功
***********************************************************/
int Kine_CR_SixDoF_G1::IKine(double *gdCPos, double *gdJCurr, double *gdJPos)
{
	int i,j;
	int result = 0;
	int flag[8] = {0,0,0,0,0,0,0,0};  //八组解的情况，0为有解

	double JLimitUp[6] = {360.0 * PI_RAD, 210.0 * PI_RAD, 210.0 * PI_RAD, 360.0 * PI_RAD, 120.0 * PI_RAD, 360.0 * PI_RAD};
	double JLimitLow[6] = {-360.0 * PI_RAD, -30.0 * PI_RAD, -30.0 * PI_RAD,-360.0 * PI_RAD, -120.0 * PI_RAD, -360.0 * PI_RAD};   //关节角限位
	double s1, c1, s3, c3, s4, c4, s23, c23;
	double s4_, c4_, s5_, c5_, s6_, c6_, s23_, c23_;
	double temp[8];   //中间变量
	MtxKine lm_Tcp;  //中间变量，TCP矩阵
	double rad[8][6]; //中间变量，八组逆解
	

	//TCP矩阵
	temp[1]	= sin(gdCPos[3] * PI_RAD);
	temp[2]	= cos(gdCPos[3] * PI_RAD);
	temp[3]	= sin(gdCPos[4] * PI_RAD);
	temp[4]	= cos(gdCPos[4] * PI_RAD);
	temp[5]	= sin(gdCPos[5] * PI_RAD);
	temp[6]	= cos(gdCPos[5] * PI_RAD);

	lm_Tcp.R11 = temp[2] * temp[4];
	lm_Tcp.R12 = temp[2] * temp[3] * temp[5] - temp[1] * temp[6];
    lm_Tcp.R13 = temp[2] * temp[3] * temp[6] + temp[1] * temp[5];
    lm_Tcp.R21 = temp[1] * temp[4];
    lm_Tcp.R22 = temp[1] * temp[3] * temp[5] + temp[2] * temp[6];
    lm_Tcp.R23 = temp[1] * temp[3] * temp[6] - temp[2] * temp[5];
    lm_Tcp.R31 = -temp[3];
    lm_Tcp.R32 = temp[4] * temp[5];
    lm_Tcp.R33 = temp[4] * temp[6];

/*	std::cout << "R11" << lm_Tcp.R11 << std::endl;
	std::cout << "R12" << lm_Tcp.R12 << std::endl;
	std::cout << "R13" << lm_Tcp.R13 << std::endl;
	std::cout << "R21" << lm_Tcp.R21 << std::endl;
	std::cout << "R22" << lm_Tcp.R22 << std::endl;
	std::cout << "R23" << lm_Tcp.R23 << std::endl;
	std::cout << "R31" << lm_Tcp.R31 << std::endl;
	std::cout << "R32" << lm_Tcp.R32 << std::endl;
	std::cout << "R33" << lm_Tcp.R33 << std::endl;*/

	lm_Tcp.X = gdCPos[0] - L56 * lm_Tcp.R13;
	lm_Tcp.Y = gdCPos[1] - L56 * lm_Tcp.R23;
	lm_Tcp.Z = gdCPos[2] - L56 * lm_Tcp.R33 - L01;

	// std::cout << "x" << lm_Tcp.X << std::endl;
	// std::cout << "y" << lm_Tcp.Y << std::endl;
	// std::cout << "z" << lm_Tcp.Z << std::endl; 

	//关节弧度
	//求解theta1（两个解）
	temp[0] = atan2(lm_Tcp.Y, lm_Tcp.X);
	temp[1] = atan2(-lm_Tcp.Y, -lm_Tcp.X);


	RadInRange(&temp[0], &gdJCurr[0]);
	RadInRange(&temp[1], &gdJCurr[0]);

	rad[0][0] = rad[1][0] = temp[0];
	rad[2][0] = rad[3][0] = temp[1];

//	std::cout << "rad[0][0] = rad[1][0] = " << temp[0] << std::endl;
//	std::cout << "rad[2][0] = rad[3][0] = " << temp[1] << std::endl;
	
	//求解theta3（两个解）
	for (i = 0; i < 2; i++)
	{
		s1= sin(rad[2*i][0]);
		c1= cos(rad[2*i][0]);

		temp[7] = c1 * lm_Tcp.X + s1 * lm_Tcp.Y;
		temp[2] = (temp[7] * temp[7] + lm_Tcp.Z * lm_Tcp.Z - L2 * L2 - L34 * L34) / (2 * L2 * L34);
		temp[0] = 1 - temp[2] * temp[2];

		if (fabs(temp[0]) < 0.00001)
		{
			temp[1] = 0;
			
		}
		else if (temp[0] < 0)
		{
			flag[2*i] = 1;    //代表此组解无解
			flag[2*i+1] = 1;
			//break;
		}
		else
		{
			temp[1] = sqrt(temp[0]);
		}
		

		temp[3] = atan2(temp[2], temp[1]);
		temp[4] = atan2(temp[2], -temp[1]);


		RadInRange(&temp[3], &gdJCurr[2]);
		RadInRange(&temp[4], &gdJCurr[2]);

		
		rad[2*i][2] = temp[3];
		rad[2*i+1][2] = temp[4];
	}

	if(flag[0] && flag[2])
	{
		return 1;    //腰关节无可用逆解值
	}

	//求解theta2，4，5，6
	for ( i = 0; i < 4; ++i)
	{
		if (0 == flag[i])
		{

			s1 = sin(rad[i][0]);
			c1 = cos(rad[i][0]);
			s3 = sin(rad[i][2]);
			c3 = cos(rad[i][2]);

			//计算theta2和theta3的和
			temp[1] = c1 * lm_Tcp.X + s1 * lm_Tcp.Y;
			temp[2] = L34 + s3 * L2;
			temp[3] = c3 * L2;

			s23_ = temp[1] * temp[2] + lm_Tcp.Z * temp[3];
			c23_ = temp[1] * temp[3] - lm_Tcp.Z * temp[2];

			//求解theta2
			temp[4] = atan2(s23_, c23_);

			temp[5] = temp[4] - rad[i][2];
			RadInRange(&temp[5], &gdJCurr[1]);
			rad[i][1] = temp[5];

	//		std::cout << "rad["<<i<<"][1] = " << temp[5] << std::endl;
		
			
			//求解theta4
			s23 = sin(rad[i][1] + rad[i][2]);
			c23 = cos(rad[i][1] + rad[i][2]);
			temp[6] = c1 * c23;
			temp[7] = s1 * c23;

			s4_= s1 * lm_Tcp.R13 - c1 * lm_Tcp.R23;  //并非真正s4
			c4_= temp[6] * lm_Tcp.R13 +temp[7] * lm_Tcp.R23 + s23 * lm_Tcp.R33;
			
			temp[0] = atan2(s4_, c4_);
			RadInRange(&temp[0], &gdJCurr[3]);
			rad[i][3] = temp[0];

			//std::cout << "rad["<<i<<"][3] = " << rad[i][3] << std::endl;

			//求解theta5
			s4 = sin(rad[i][3]);
			c4 = cos(rad[i][3]);

			s5_ = lm_Tcp.R13 * (temp[6] * c4 + s1 * s4) + lm_Tcp.R23 * (temp[7] * c4 - c1 * s4) + lm_Tcp.R33 * s23 * c4;
			c5_ = lm_Tcp.R13 * c1 * s23 + lm_Tcp.R23 * s1 * s23 - lm_Tcp.R33 * c23;
			
			temp[1] = atan2(s5_, c5_);
			RadInRange(&temp[1], &gdJCurr[4]);
			rad[i][4] = temp[1];

			//求解theta6
			temp[2] = -temp[6] * s4 + s1 * c4;
			temp[3] = -temp[7] * s4 - c1 * c4;
			temp[4] = -s23 * s4;

			s6_ = lm_Tcp.R11 * temp[2] + lm_Tcp.R21 * temp[3] + lm_Tcp.R31 * temp[4];
			c6_ = lm_Tcp.R12 * temp[2] + lm_Tcp.R22 * temp[3] + lm_Tcp.R32 * temp[4];

			temp[5] = atan2(s6_, c6_);
			RadInRange(&temp[5], &gdJCurr[5]);
			rad[i][5] = temp[5];

			temp[0] = rad[i][3] + PI;
			temp[1] = -rad[i][4];
			temp[2] = rad[i][5] + PI;
			//第4、6关节“翻转”
			rad[i+4][0] = rad[i][0];
			rad[i+4][1] = rad[i][1];
			rad[i+4][2] = rad[i][2];
			RadInRange(&temp[0], &gdJCurr[3]);
			rad[i + 4][3] = temp[0];
			RadInRange(&temp[1], &gdJCurr[4]);
			rad[i + 4][4] = temp[1];
			RadInRange(&temp[2], &gdJCurr[5]);
			rad[i + 4][5] = temp[2];
		}	
	}
	

	//关节角限位
	//择优选择
	for ( i = 0; i < 8; i++)
	{
		//关节角检测
		for ( j = 0; j < 6; j++)
		{
			if (rad[i][j] > JLimitUp[j] || rad[i][j] < JLimitLow[j])
			{
				flag[i] = 1;  //解不存在
				break;	  //提前终止
			}									
		}
		if (flag[i] == 0)
		{
			temp[i] = fabs(rad[i][0] - gdJCurr[0] * PI_RAD) + 
						fabs(rad[i][1] - gdJCurr[1] * PI_RAD) + 
						fabs(rad[i][2] - gdJCurr[2] * PI_RAD) + 
						fabs(rad[i][3] - gdJCurr[3] * PI_RAD) +
						fabs(rad[i][4] - gdJCurr[4] * PI_RAD) +
						fabs(rad[i][5] - gdJCurr[5] * PI_RAD);
			if(!result)
			{
				result = i;
				s3 = temp[i];	//用第一个有效值来初始化中间变量s1
			}
			else if (temp[i] <= s3)
			{
				result = i;
				s3 = temp[i];
			}
		}	
	}

	if(result)
	{
		gdJPos[0] = rad[result][0] * PI_DEG;
		gdJPos[1] = rad[result][1] * PI_DEG;
		gdJPos[2] = rad[result][2] * PI_DEG;
		gdJPos[3] = rad[result][3] * PI_DEG;
		gdJPos[4] = rad[result][4] * PI_DEG;
		gdJPos[5] = rad[result][5] * PI_DEG;
		return 0;
	}
	else
	{
		return 1;
	}
}

void Kine_CR_SixDoF_G2::Set_Length(double* gdLen)
{

	// double L_G2[4];
	L_G2[0] = gdLen[5] + gdLen[6];
	L_G2[1] = gdLen[3] + gdLen[4];
	L_G2[2] = gdLen[2];
	L_G2[3] = gdLen[0] + gdLen[1];

/*	std::cout << "L_G2[0]" << L_G2[0] << std::endl;
	std::cout << "L_G2[1]" << L_G2[1] << std::endl;
	std::cout << "L_G2[2]" << L_G2[2] << std::endl;
	std::cout << "L_G2[3]" << L_G2[3] << std::endl;
*/
	kine.Set_Length(gdLen);
}

int Kine_CR_SixDoF_G2::FKine(double* gdJPos, double* gdCPos)
{
	if(kine.FKine(gdJPos, gdCPos) == 1)
		return 1;

	MtxKine mtx_trans_grip;
	// Trans_PosToMtx(gdCPos, &mtx_trans_grip, 1);
    mtx_trans_grip.R11 = cos(gdCPos[3] * PI_RAD) * cos(gdCPos[4] * PI_RAD);
	mtx_trans_grip.R12 = sin(gdCPos[3] * PI_RAD) * cos(gdCPos[4] * PI_RAD); 
	mtx_trans_grip.R13 = -sin(gdCPos[4] * PI_RAD); 
    mtx_trans_grip.R21 = cos(gdCPos[3] * PI_RAD) * sin(gdCPos[4] * PI_RAD) * sin(gdCPos[5] * PI_RAD) - sin(gdCPos[3] * PI_RAD) * cos(gdCPos[5] * PI_RAD);
	mtx_trans_grip.R22 = sin(gdCPos[3] * PI_RAD) * sin(gdCPos[4] * PI_RAD) * sin(gdCPos[5] * PI_RAD) + cos(gdCPos[3] * PI_RAD) * cos(gdCPos[5] * PI_RAD);
	mtx_trans_grip.R23 = cos(gdCPos[4] * PI_RAD) * sin(gdCPos[5] * PI_RAD);
    mtx_trans_grip.R31 = cos(gdCPos[3] * PI_RAD) * sin(gdCPos[4] * PI_RAD) * cos(gdCPos[5] * PI_RAD) + sin(gdCPos[3] * PI_RAD) * sin(gdCPos[5] * PI_RAD);
	mtx_trans_grip.R32 = sin(gdCPos[3] * PI_RAD) * sin(gdCPos[4] * PI_RAD) * cos(gdCPos[5] * PI_RAD) - cos(gdCPos[3] * PI_RAD) * sin(gdCPos[5] * PI_RAD);
	mtx_trans_grip.R33 = cos(gdCPos[4] * PI_RAD) * cos(gdCPos[5] * PI_RAD);
    mtx_trans_grip.X = - gdCPos[0]; 
	mtx_trans_grip.Y = - gdCPos[1]; 
	mtx_trans_grip.Z = - gdCPos[2];  //基座交替旋转矩阵

	// std::cout << std::endl;
	// std::cout << mtx_trans_grip.R11 << " " << mtx_trans_grip.R12 << " " << mtx_trans_grip.R13 << std::endl;
	// std::cout << mtx_trans_grip.R21 << " " << mtx_trans_grip.R22 << " " << mtx_trans_grip.R23 << std::endl;
	// std::cout << mtx_trans_grip.R31 << " " << mtx_trans_grip.R32 << " " << mtx_trans_grip.R33 << std::endl;
	// std::cout << std::endl;

    // double alpha_G0_G7 = 3.1416;
    // MtxKine mtx_G0_G7;
    // mtx_G0_G7.R11 = 1.0;mtx_G0_G7.R12 = 0.0;mtx_G0_G7.R13 = 0.0;
    // mtx_G0_G7.R21 = 0.0;mtx_G0_G7.R22 = cos(alpha_G0_G7);mtx_G0_G7.R23 = sin(alpha_G0_G7);
    // mtx_G0_G7.R31 = 0.0;mtx_G0_G7.R32 = -sin(alpha_G0_G7);mtx_G0_G7.R33 = cos(alpha_G0_G7);
    // mtx_G0_G7.X =gdCPos[0];   mtx_G0_G7.Y = gdCPos[1];     mtx_G0_G7.Z = gdCPos[2];  //基座交替旋转矩阵
	// std::cout << std::endl;
	// std::cout << mtx_G0_G7.R11 << " " << mtx_G0_G7.R12 << " " << mtx_G0_G7.R13 << std::endl;
	// std::cout << mtx_G0_G7.R21 << " " << mtx_G0_G7.R22 << " " << mtx_G0_G7.R23 << std::endl;
	// std::cout << mtx_G0_G7.R31 << " " << mtx_G0_G7.R32 << " " << mtx_G0_G7.R33 << std::endl;
	// std::cout << std::endl;

	float in_point[3] = {0,0,0};
	float out_point[3];
	PointCoordinateC(&mtx_trans_grip, in_point, out_point);

	// MtxKine mtx_G0_G7;
    // mtx_G0_G7.R11 = 1.0;mtx_G0_G7.R12 = 0.0;mtx_G0_G7.R13 = 0.0;
    // mtx_G0_G7.R21 = 0.0;mtx_G0_G7.R22 = -1;mtx_G0_G7.R23 = 0;
    // mtx_G0_G7.R31 = 0.0;mtx_G0_G7.R32 = 0;mtx_G0_G7.R33 = -1;
    // mtx_G0_G7.X = 0;   mtx_G0_G7.Y = 0;     mtx_G0_G7.Z = 0;  //基座交替旋转矩阵
	// float world_zero[3] = {0,0,0};
	// float in_vecx[3] = {1, 0, 0}, out_vecx[3];
	// float in_vecy[3] = {0, 1, 0}, out_vecy[3];
	// float in_vecz[3] = {0, 0, 1}, out_vecz[3];
	// PointCoordinateC(&mtx_G0_G7, world_zero, world_zero);
	// PointCoordinateC(&mtx_G0_G7, in_vecx, in_vecx);
	// PointCoordinateC(&mtx_G0_G7, in_vecy, in_vecy);
	// PointCoordinateC(&mtx_G0_G7, in_vecz, in_vecz);
	// PointCoordinateC(&mtx_trans_grip, world_zero, world_zero);
	// PointCoordinateC(&mtx_trans_grip, in_vecx, in_vecx);
	// PointCoordinateC(&mtx_trans_grip, in_vecy, in_vecy);
	// PointCoordinateC(&mtx_trans_grip, in_vecz, in_vecz);
	// for(int i = 0;i<3;++i)
    // {
    //     out_vecx[i] = in_vecx[i] - world_zero[i];
    //     out_vecy[i] = in_vecy[i] - world_zero[i];
    //     out_vecz[i] = in_vecz[i] - world_zero[i];
    // }
	// MtxKine mtx_pole2pole1; //此时杆件1上夹持点固结基座标系
    // double pos_p2top1[6] = {0,0,0,0,0,0};
	// mtx_pole2pole1.R11 = out_vecx[0];mtx_pole2pole1.R12 = out_vecx[1];mtx_pole2pole1.R13 = out_vecx[2];
    // mtx_pole2pole1.R21 = out_vecy[0];mtx_pole2pole1.R22 = out_vecy[1];mtx_pole2pole1.R23 = out_vecy[2];
    // mtx_pole2pole1.R31 = out_vecz[0];mtx_pole2pole1.R32 = out_vecz[1];mtx_pole2pole1.R33 = out_vecz[2];
    // mtx_pole2pole1.X = 0;            mtx_pole2pole1.Y = 0;            mtx_pole2pole1.Z = 0;  //xyz不作设置
    // Trans_MtxToPos(&mtx_pole2pole1,pos_p2top1); //杆件2点在基座坐标系下表示

	// std::cout << "before = ";
	// for (int i = 0; i < 6;++i)
	// {
	// 	std::cout << pos_p2top1[i] << " ";
	// }
	// std::cout << std::endl;

	gdCPos[0] = out_point[0];
	gdCPos[1] = out_point[1];
	gdCPos[2] = out_point[2];
	// gdCPos[3] = pos_p2top1[3];
	// gdCPos[4] = pos_p2top1[4]; 
	// gdCPos[5] = pos_p2top1[5];

	return 0;
}

// int Kine_CR_SixDoF_G2::FKine(double* gdJPos, double* gdCPos)
// {
// 	int i;
// 	double jrad[6];      //关节弧度

// 	double  temp[8];   //中间变量
// 	MtxKine lm_Tcp;      //中间变量，TCP矩阵

// 	//关节弧度
// 	/*jrad[0]=gdJPos[0]*PI_RAD;
// 	jrad[1]=gdJPos[1]*PI_RAD;
// 	jrad[2]=gdJPos[2]*PI_RAD;
// 	jrad[3]=gdJPos[3]*PI_RAD;
// 	jrad[4]=gdJPos[4]*PI_RAD;
// 	jrad[5]=gdJPos[5]*PI_RAD; */
// 	for ( i = 0; i < 6; i++)
// 	{
// 		jrad[i] = gdJPos[i] * PI_RAD;
// //		std::cout << jrad[i] << std::endl;
// 	}
	

// 	//TCP矩阵
// 	double c23 = cos(jrad[1]+jrad[2]);
// 	double s23 = sin(jrad[1]+jrad[2]);

// 	//计算TCP姿态矩阵参数
// 	lm_Tcp.R11 = sin(jrad[5]) * (cos(jrad[3]) * sin(jrad[0]) - sin(jrad[3]) * (cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2]) - cos(jrad[0]) * sin(jrad[1]) * sin(jrad[2]))) - cos(jrad[5]) * (sin(jrad[4]) * (cos(jrad[0]) * cos(jrad[1]) * sin(jrad[2]) + cos(jrad[0]) * cos(jrad[2]) * sin(jrad[1])) - cos(jrad[4]) * (sin(jrad[0]) * sin(jrad[3]) + cos(jrad[3]) * (cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2]) - cos(jrad[0]) * sin(jrad[1]) * sin(jrad[2]))));
// 	lm_Tcp.R12 = sin(jrad[5]) * (sin(jrad[4]) * (cos(jrad[0]) * cos(jrad[1]) * sin(jrad[2]) + cos(jrad[0]) * cos(jrad[2]) * sin(jrad[1])) - cos(jrad[4]) * (sin(jrad[0]) * sin(jrad[3]) + cos(jrad[3]) * (cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2]) - cos(jrad[0]) * sin(jrad[1]) * sin(jrad[2])))) + cos(jrad[5]) * (cos(jrad[3]) * sin(jrad[0]) - sin(jrad[3]) * (cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2]) - cos(jrad[0]) * sin(jrad[1]) * sin(jrad[2])));
// 	lm_Tcp.R13 = -cos(jrad[4]) * (cos(jrad[0]) * cos(jrad[1]) * sin(jrad[2]) + cos(jrad[0]) * cos(jrad[2]) * sin(jrad[1])) - sin(jrad[4]) * (sin(jrad[0]) * sin(jrad[3]) + cos(jrad[3]) * (cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2]) - cos(jrad[0]) * sin(jrad[1]) * sin(jrad[2])));
// 	lm_Tcp.R21 = -cos(jrad[5]) * (sin(jrad[4]) * (cos(jrad[1]) * sin(jrad[0]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[0]) * sin(jrad[1])) + cos(jrad[4]) * (cos(jrad[0]) * sin(jrad[3]) - cos(jrad[3]) * (cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0]) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[2])))) - sin(jrad[5]) * (cos(jrad[0]) * cos(jrad[3]) + sin(jrad[3]) * (cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0]) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[2])));
// 	lm_Tcp.R22 = sin(jrad[5]) * (sin(jrad[4]) * (cos(jrad[1]) * sin(jrad[0]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[0]) * sin(jrad[1])) + cos(jrad[4]) * (cos(jrad[0]) * sin(jrad[3]) - cos(jrad[3]) * (cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0]) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[2])))) - cos(jrad[5]) * (cos(jrad[0]) * cos(jrad[3]) + sin(jrad[3]) * (cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0]) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[2])));
// 	lm_Tcp.R23 = sin(jrad[4]) * (cos(jrad[0]) * sin(jrad[3]) - cos(jrad[3]) * (cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0]) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[2]))) - cos(jrad[4]) * (cos(jrad[1]) * sin(jrad[0]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[0]) * sin(jrad[1]));
// 	lm_Tcp.R32 = sin(jrad[0]) * (sin(jrad[1]) * (cos(jrad[3]) * cos(jrad[4]) - sin(jrad[3]) * sin(jrad[4])) + cos(jrad[1]) * cos(jrad[2]) * (cos(jrad[3]) * sin(jrad[4]) + cos(jrad[4]) * sin(jrad[3]))) + cos(jrad[0]) * sin(jrad[2]) * (cos(jrad[3]) * sin(jrad[4]) + cos(jrad[4]) * sin(jrad[3]));
// 	lm_Tcp.R31 = sin(jrad[3]) * sin(jrad[5]) * (cos(jrad[1]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[1])) - cos(jrad[5]) * (sin(jrad[4]) * (cos(jrad[1]) * cos(jrad[2]) - sin(jrad[1]) * sin(jrad[2])) + cos(jrad[3]) * cos(jrad[4]) * (cos(jrad[1]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[1])));
// 	lm_Tcp.R32 = sin(jrad[5]) * (sin(jrad[4]) * (cos(jrad[1]) * cos(jrad[2]) - sin(jrad[1]) * sin(jrad[2])) + cos(jrad[3]) * cos(jrad[4]) * (cos(jrad[1]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[1]))) + cos(jrad[5]) * sin(jrad[3]) * (cos(jrad[1]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[1]));
// 	lm_Tcp.R33 = cos(jrad[3]) * sin(jrad[4]) * (cos(jrad[1]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[1])) - cos(jrad[4]) * (cos(jrad[1]) * cos(jrad[2]) - sin(jrad[1]) * sin(jrad[2]));

// 	//计算TCP位置
// 	lm_Tcp.X = (L_G2[0]) * (cos(jrad[4]) * (cos(jrad[0]) * cos(jrad[1]) * sin(jrad[2]) + cos(jrad[0]) * cos(jrad[2]) * sin(jrad[1])) + sin(jrad[4]) * (sin(jrad[0]) * sin(jrad[3]) + cos(jrad[3]) * (cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2]) - cos(jrad[0]) * sin(jrad[1]) * sin(jrad[2])))) - (L_G2[2]) * (cos(jrad[0]) * cos(jrad[1]) * sin(jrad[2]) + cos(jrad[0]) * cos(jrad[2]) * sin(jrad[1])) - cos(jrad[0]) * cos(jrad[1]) * L_G2[1];
// 	lm_Tcp.Y = (L_G2[0]) * (cos(jrad[4]) * (cos(jrad[1]) * sin(jrad[0]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[0]) * sin(jrad[1])) - sin(jrad[4]) * (cos(jrad[0]) * sin(jrad[3]) - cos(jrad[3]) * (cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0]) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[2])))) - (L_G2[2]) * (cos(jrad[1]) * sin(jrad[0]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[0]) * sin(jrad[1])) - cos(jrad[1]) * L_G2[1] * sin(jrad[0]);
// 	lm_Tcp.Z = L_G2[1] * sin(jrad[1]) - L_G2[3] - (L_G2[2]) * (cos(jrad[1]) * cos(jrad[2]) - sin(jrad[1]) * sin(jrad[2])) + (L_G2[0]) * (cos(jrad[4]) * (cos(jrad[1]) * cos(jrad[2]) - sin(jrad[1]) * sin(jrad[2])) - cos(jrad[3]) * sin(jrad[4]) * (cos(jrad[1]) * sin(jrad[2]) + cos(jrad[2]) * sin(jrad[1])));

// 	//计算PRY角
// 	temp[5] = atan2(-lm_Tcp.R31, sqrt(lm_Tcp.R11 * lm_Tcp.R11 + lm_Tcp.R21 * lm_Tcp.R21));

// 	if (fabs(temp[5] - PI_2) < 0.00001)
// 	{
// 		temp[4] = 0;
// 		temp[6] = atan2(lm_Tcp.R12, lm_Tcp.R22);
// 	}
// 	else if (fabs(temp[5] + PI_2) < 0.00001)
// 	{
// 		temp[4] = 0;
// 		temp[6] = -atan2(lm_Tcp.R12, lm_Tcp.R22);
// 	}
// 	else
// 	{
// 		temp[0] = 1 / cos(temp[5]);

// 		temp[1] = lm_Tcp.R21 * temp[0];
// 		temp[2] = lm_Tcp.R11 * temp[0];
// 		if (fabs(temp[1]) < 0.00001)
// 		{
// 			temp[1] = 0;
// 		}
// 		if (fabs(temp[2]) < 0.00001)
// 		{
// 			temp[2] = 0;
// 		}
// 		temp[4] = atan2(temp[1], temp[2]);

// 		temp[1] = lm_Tcp.R32 * temp[0];
// 		temp[2] = lm_Tcp.R33 * temp[0];
// 		if (fabs(temp[1]) < 0.00001)
// 		{
// 			temp[1] = 0;
// 		}
// 		if (fabs(temp[2]) < 0.00001)
// 		{
// 			temp[2] = 0;
// 		}
// 		temp[6] = atan2(temp[1], temp[2]);
// 	}


// /*	std::cout << "temp[4]" << temp[4] << std::endl;
// 	std::cout << "temp[5]" << temp[5] << std::endl;
// 	std::cout << "temp[6]" << temp[6] << std::endl;
// */

// 	gdCPos[0] = lm_Tcp.X;
// 	gdCPos[1] = lm_Tcp.Y;
// 	gdCPos[2] = lm_Tcp.Z;
// 	gdCPos[3] = temp[4] * PI_DEG;
// 	gdCPos[4] = temp[5] * PI_DEG; 
// 	gdCPos[5] = temp[6] * PI_DEG;

// 	return 0;	
// }



int Kine_CR_SixDoF_G2::IKine(double* gdCPos, double* gdJCurr, double* gdJPos)
{
	double jpos_G2[6];
	double jcurrpos_G2[6];
	//int Trx[4][4] = {{1,0,0,0},{0,-1,0,0},{0,0,-1,0},{0,0,0,1}};
	int flag;

	jcurrpos_G2[0] = gdJCurr[0];
	jcurrpos_G2[1] = gdJCurr[1];
	jcurrpos_G2[2] = gdJCurr[2];
	jcurrpos_G2[3] = gdJCurr[3];
	jcurrpos_G2[4] = gdJCurr[4];
	jcurrpos_G2[5] = gdJCurr[5];


    // Trx = [1 0 0 0;0 -1 0 0; 0 0 -1 0; 0 0 0 1];
    // EETrMtx = Trx / robot.TarGraspMtx * Trx;
	MtxKine mtx_trans_grip;
	Trans_PosToMtx(gdCPos, &mtx_trans_grip, 0);
	mtx_trans_grip.X = -gdCPos[0]; 
	mtx_trans_grip.Y = -gdCPos[1]; 
	mtx_trans_grip.Z = -gdCPos[2];  //基座交替旋转矩阵

    MtxKine mtx_G0_G7;
    mtx_G0_G7.R11 = 1.0;mtx_G0_G7.R12 = 0.0;mtx_G0_G7.R13 = 0.0;
    mtx_G0_G7.R21 = 0.0;mtx_G0_G7.R22 = -1.0;mtx_G0_G7.R23 = 0;
    mtx_G0_G7.R31 = 0.0;mtx_G0_G7.R32 = 0;mtx_G0_G7.R33 = -1.0;
    mtx_G0_G7.X = 0;   mtx_G0_G7.Y = 0;     mtx_G0_G7.Z = 0;  //基座交替旋转矩阵

	float in_point[3] = {0,0,0};
	float out_point[3];
	// PointCoordinateC(&mtx_G0_G7, in_point, out_point);
	PointCoordinateC(&mtx_trans_grip, in_point, out_point);

	for (int i = 0; i < 6;++i)
	{
		if(fabs(out_point[i]) < 0.00001)
		{
			out_point[i] = 0;
		}
	}


	gdCPos[0] = out_point[0];
	gdCPos[1] = out_point[1];
	gdCPos[2] = out_point[2];
	// gdCPos[3] = pos_p2top1[3];
	// gdCPos[4] = pos_p2top1[4];
	// gdCPos[5] = pos_p2top1[5];
	// for (int i = 0; i < 6;++i)
	// {
	// 	std::cout << gdCPos[i] << " ";
	// }
	// std::cout << std::endl;

	flag = kine.IKine(gdCPos, jcurrpos_G2, jpos_G2);

	if(0 == flag)
	{
		// std::cout << "success" << std::endl;
		gdJPos[0] = jpos_G2[0];
		gdJPos[1] = jpos_G2[1];
		gdJPos[2] = jpos_G2[2];
		gdJPos[3] = jpos_G2[3];
		gdJPos[4] = jpos_G2[4];
		gdJPos[5] = jpos_G2[5];
	}
	else
	{
		return flag;
	}
	return 0;
	
}

void Trans_PosToMtx(double* pos, MtxKine* output, int inv)//该算法以 Z-Y-X欧拉角坐标系(绕当前坐标系) 表示
{
	double ld_temp[6];
	
	// 计算位姿变换矩阵 //
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
	
	// 求逆矩阵
	if(1 == inv)
	{
		// 姿态求逆 - 转置矩阵 //
		ld_temp[0] = output->R12;
		output->R12 = output->R21;
		output->R21 = ld_temp[0];
		
		ld_temp[0] = output->R13;
		output->R13 = output->R31;
		output->R31 = ld_temp[0];
		
		ld_temp[0] = output->R23;
		output->R23 = output->R32;
		output->R32 = ld_temp[0];
		
		// 位置求逆 //
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
	
    //--------------------- 输出位姿 -------------------------//
    // 计算RPY角 - rad //
	ld_temp[4] = atan2(- input->R31, sqrt(input->R11 * input->R11 + input->R21 * input->R21));
	
    if (fabs(ld_temp[4] - PI / 2) < 0.00001)
    {
        ld_temp[3] = 0;
        ld_temp[5] = atan2(input->R12, input->R22);
    }
    else if (fabs(ld_temp[4] + PI / 2) < 0.00001)
    {
        ld_temp[3] = 0;
        ld_temp[5] = -atan2(input->R12, input->R22);
    }
    else
    {
        ld_temp[0] = 1 / cos(ld_temp[4]);
		
		ld_temp[1] = input->R21 * ld_temp[0];
		ld_temp[2] = input->R11 * ld_temp[0];
		if(fabs(ld_temp[1]) < 0.00001)
		{
			ld_temp[1] = 0;
		}
		if(fabs(ld_temp[2]) < 0.00001)
		{
			ld_temp[2] = 0;
		}
        ld_temp[3] = atan2(ld_temp[1], ld_temp[2]);
		
		ld_temp[1] = input->R32 * ld_temp[0];
		ld_temp[2] = input->R33 * ld_temp[0];
		if(fabs(ld_temp[1]) < 0.00001)
		{
			ld_temp[1] = 0;
		}
		if(fabs(ld_temp[2]) < 0.00001)
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

//求解机器人四根连杆在全局坐标系下端点
std::vector<std::vector<double> > Linkage6D(IN double gdLen[7],IN double* gdJPos,std::vector<double> &cur_truss,int length,int angle)
{
	std::vector<std::vector<double>> Linkage;
	std::vector<double> link_point(6, 0);
	float tmp_link[6];
	double L01 = gdLen[0] + gdLen[1];
	double L2 = gdLen[2];
	double L34 = gdLen[3] + gdLen[4];
	double L56 = gdLen[5] + gdLen[6];

	double c1 = cos(gdJPos[0] * PI_RAD);
	double s1 = sin(gdJPos[0] * PI_RAD);
	double c2 = cos(gdJPos[1] * PI_RAD);
	double s2 = sin(gdJPos[1] * PI_RAD);
	double c23 = cos(gdJPos[1] * PI_RAD + gdJPos[2] * PI_RAD);
	double s23 = sin(gdJPos[1] * PI_RAD + gdJPos[2] * PI_RAD);
	double c4 = cos(gdJPos[3] * PI_RAD);
	double s4 = sin(gdJPos[3] * PI_RAD);
	double c5 = cos(gdJPos[4] * PI_RAD);
	double s5 = sin(gdJPos[4] * PI_RAD);

	float tmp_before[3];
	float tmp_after[3];

	//第一根连杆L01
	// link_point = {0, 0, 0, 0, 0, L01};
	tmp_before[0] = 0;tmp_before[1] = 0;tmp_before[2] = 0;
	transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after);
	link_point[0] = tmp_after[0];
	link_point[1] = tmp_after[1];
	link_point[2] = tmp_after[2];
	tmp_before[0] = 0;tmp_before[1] = 0;tmp_before[2] = L01;
	transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after);
	link_point[3] = tmp_after[0];
	link_point[4] = tmp_after[1];
	link_point[5] = tmp_after[2];	
	Linkage.push_back(link_point);
	//第二根连杆L2
	// link_point = {0, 0, L01, L2 * c1 * c2, L2 * c2 * s1, L01 + L2 * s2};
	tmp_before[0] = 0;tmp_before[1] = 0;tmp_before[2] = L01;
	transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after);
	link_point[0] = tmp_after[0];
	link_point[1] = tmp_after[1];
	link_point[2] = tmp_after[2];
	tmp_before[0] = L2 * c1 * c2;tmp_before[1] = L2 * c2 * s1;tmp_before[2] = L01 + L2 * s2;
	transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after);
	link_point[3] = tmp_after[0];
	link_point[4] = tmp_after[1];
	link_point[5] = tmp_after[2];	
	Linkage.push_back(link_point);
	//第三根连杆L34
	// link_point = {L2*c1*c2, L2*c2*s1, L01+L2*s2,L34*c1*s23+L2*c1*c2, L34*s1*s23+ L2*c2*s1, -L34*c23+L01+L2*s2};
	tmp_before[0] = L2 * c1 * c2;tmp_before[1] = L2 * c2 * s1;tmp_before[2] = L01 + L2 * s2;
	transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after);
	link_point[0] = tmp_after[0];
	link_point[1] = tmp_after[1];
	link_point[2] = tmp_after[2];
	tmp_before[0] = L34*c1*s23+L2*c1*c2;tmp_before[1] =L34*s1*s23+ L2*c2*s1;tmp_before[2] = -L34*c23+L01+L2*s2;
	transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after);
	link_point[3] = tmp_after[0];
	link_point[4] = tmp_after[1];
	link_point[5] = tmp_after[2];	
	Linkage.push_back(link_point);
	//第四根连杆L56
	// link_point = {L34*c1*s23+L2*c1*c2, L34*s1*s23+ L2*c2*s1, -L34*c23+L01+L2*s2,(c1*c23*c4*s5 + s1*s4*s5 + c1*s23*c5)*L56 + L34*c1*s23+L2*c1*c2,
    // (s1*c23*c4*s5 -c1*s4*s5 + s1*s23*c5)*L56+L34*s1*s23+ L2*c2*s1,(s23*c4*s5 - c23*c5)*L56 - L34*c23+L01+L2*s2};
	tmp_before[0] = L34*c1*s23+L2*c1*c2;tmp_before[1] =L34*s1*s23+ L2*c2*s1;tmp_before[2] = -L34*c23+L01+L2*s2;
	transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after);
	link_point[0] = tmp_after[0];
	link_point[1] = tmp_after[1];
	link_point[2] = tmp_after[2];
	tmp_before[0] =(c1*c23*c4*s5 + s1*s4*s5 + c1*s23*c5)*L56 + L34*c1*s23+L2*c1*c2;
	tmp_before[1] =(s1*c23*c4*s5 -c1*s4*s5 + s1*s23*c5)*L56+L34*s1*s23+ L2*c2*s1;
	tmp_before[2] =(s23*c4*s5 - c23*c5)*L56 - L34*c23+L01+L2*s2;
	transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after);
	link_point[3] = tmp_after[0];
	link_point[4] = tmp_after[1];
	link_point[5] = tmp_after[2];
	Linkage.push_back(link_point);

	// int Grip_Id = 7;
	// // 交换夹持器
	// if(Grip_Id == 7)
	// {
	// 	//原来的目标坐标系就是现在的基坐标系，世界坐标系下
		
	// 	//交换夹持端时，根据右手坐标系规则，y轴和z轴反转
	// }

	return Linkage;
}


// int Linktest()
// {
// 	double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
//     double gdjPos[6] = {7.95526,147.273,157.015,-13.1725,57.9791,15.1831};
//     double curpos1[6],curpos2[6];
//     std::vector<std::vector<double>> link;
//     link = Linkage6D(len,gdjPos,truss1,60,28);	//#include"envirnoment.h"
//     for (int i = 0; i < link.size();++i)
//     {
//         for (int j = 0; j < link[0].size();++j)
//         {
//             std::cout << link[i][j] << " ";
//         }
//         std::cout << std::endl;
//     }
// }

/*
//测试机器人正逆运动学
int main()
{
   // //初始化机器人
    // // double len[7] = {174.85, 167.2, 369.2, 167.2, 202, 167.2, 174.85};
    // double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
    // Kine_CR_SixDoF_G1 climbot6d_G1;
    // Kine_CR_SixDoF_G2 climbot6d_G2;
    // climbot6d_G1.Set_Length(len);
    // climbot6d_G2.Set_Length(len);

    // double gdcPos[6] = {667.6,0, 0,-93 ,71,90};
    // double gdjPos[6] = {0,0,0,0,0,0};
    // double gdposik[6];

    // // double cur[6] = {90,0,0,0,0,0};
    // // // climbot6d_G1.FKine(cur, gdcPos);
    // // // std::cout << "G1正解 = ";
    // // // for (int i = 0; i < 6;++i)
    // // // {
    // // //     std::cout << gdcPos[i] << " ";
    // // // }
    // // // std::cout << std::endl;

    // // // climbot6d_G2.FKine(cur, gdcPos);
    // // // std::cout << "G2正解 = ";
    // // // for (int i = 0; i < 6;++i)
    // // // {
    // // //     std::cout << gdcPos[i] << " ";
    // // // }
    // // // std::cout << std::endl;
    // climbot6d_G1.IKine(gdcPos, gdjPos, gdposik);
    // std::cout << "逆解 = ";
    // for (int i = 0; i < 6;++i)
    // {
    //     std::cout << gdposik[i] * 0.0174532925199 << " ";
    // }
	return 0;
}

*/