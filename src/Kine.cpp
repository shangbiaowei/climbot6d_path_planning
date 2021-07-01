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

	// std::cout << "G1 mtx" << std::endl;
	// std::cout << lm_Tcp.R11 << " " << lm_Tcp.R12 << " "<<lm_Tcp.R13 << std::endl;
	// std::cout << lm_Tcp.R21 << " " << lm_Tcp.R22 << " "<<lm_Tcp.R23 << std::endl;
	// std::cout << lm_Tcp.R31 << " " << lm_Tcp.R32 << " "<<lm_Tcp.R33 << std::endl;
	// std::cout << lm_Tcp.X << " " << lm_Tcp.Y << " "<<lm_Tcp.Z << std::endl;

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

	// std::cout << "R11 " << lm_Tcp.R11 << std::endl;
	// std::cout << "R12 " << lm_Tcp.R12 << std::endl;
	// std::cout << "R13 " << lm_Tcp.R13 << std::endl;
	// std::cout << "R21 " << lm_Tcp.R21 << std::endl;
	// std::cout << "R22 " << lm_Tcp.R22 << std::endl;
	// std::cout << "R23 " << lm_Tcp.R23 << std::endl;
	// std::cout << "R31 " << lm_Tcp.R31 << std::endl;
	// std::cout << "R32 " << lm_Tcp.R32 << std::endl;
	// std::cout << "R33 " << lm_Tcp.R33 << std::endl;

	lm_Tcp.X = gdCPos[0] - L56 * lm_Tcp.R13;
	lm_Tcp.Y = gdCPos[1] - L56 * lm_Tcp.R23;
	lm_Tcp.Z = gdCPos[2] - L56 * lm_Tcp.R33 - L01;

	// if(fabs(lm_Tcp.X) < 0.01)
	// 	lm_Tcp.X = 0;
	// if(fabs(lm_Tcp.Y) < 0.01)
	// 	lm_Tcp.Y = 0;
	// if(fabs(lm_Tcp.Z) < 0.01)
	// 	lm_Tcp.Z = 0;

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

	// std::cout << "rad[0][0] = rad[1][0] = " << temp[0] << std::endl;
	// std::cout << "rad[2][0] = rad[3][0] = " << temp[1] << std::endl;
	
	//求解theta3（两个解）
	for (i = 0; i < 2; i++)
	{
		s1= sin(rad[2*i][0]);
		c1= cos(rad[2*i][0]);

		if(fabs(s1) < 0.001)
		{
			s1 = 0;
		}
		if(fabs(c1) < 0.001)
		{
			c1 = 0;
		}

		temp[7] = c1 * lm_Tcp.X + s1 * lm_Tcp.Y;
		temp[2] = (temp[7] * temp[7] + lm_Tcp.Z * lm_Tcp.Z - L2 * L2 - L34 * L34) / (2 * L2 * L34);
		temp[0] = 1 - temp[2] * temp[2];

		// std::cout << "temp G1 " << temp[7] << " " << temp[2] << " " << temp[0] << std::endl;

		if (fabs(temp[0]) < 0.0001)
		{
			temp[1] = 0;
		}
		else if(temp[0] < 0)
		{
			flag[2*i] = 1;    //代表此组解无解
			flag[2*i+1] = 1;
			// std::cout << "wujie" << std::endl;
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
	// double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
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
	// if(kine.FKine(gdJPos, gdCPos) == 1)
	// 	return 1;

	// MtxKine mtx_trans_grip;
	// // Trans_PosToMtx(gdCPos, &mtx_trans_grip, 1);
    // mtx_trans_grip.R11 = cos(gdCPos[3] * PI_RAD) * cos(gdCPos[4] * PI_RAD);
	// mtx_trans_grip.R12 = sin(gdCPos[3] * PI_RAD) * cos(gdCPos[4] * PI_RAD); 
	// mtx_trans_grip.R13 = -sin(gdCPos[4] * PI_RAD); 
    // mtx_trans_grip.R21 = cos(gdCPos[3] * PI_RAD) * sin(gdCPos[4] * PI_RAD) * sin(gdCPos[5] * PI_RAD) - sin(gdCPos[3] * PI_RAD) * cos(gdCPos[5] * PI_RAD);
	// mtx_trans_grip.R22 = sin(gdCPos[3] * PI_RAD) * sin(gdCPos[4] * PI_RAD) * sin(gdCPos[5] * PI_RAD) + cos(gdCPos[3] * PI_RAD) * cos(gdCPos[5] * PI_RAD);
	// mtx_trans_grip.R23 = cos(gdCPos[4] * PI_RAD) * sin(gdCPos[5] * PI_RAD);
    // mtx_trans_grip.R31 = cos(gdCPos[3] * PI_RAD) * sin(gdCPos[4] * PI_RAD) * cos(gdCPos[5] * PI_RAD) + sin(gdCPos[3] * PI_RAD) * sin(gdCPos[5] * PI_RAD);
	// mtx_trans_grip.R32 = sin(gdCPos[3] * PI_RAD) * sin(gdCPos[4] * PI_RAD) * cos(gdCPos[5] * PI_RAD) - cos(gdCPos[3] * PI_RAD) * sin(gdCPos[5] * PI_RAD);
	// mtx_trans_grip.R33 = cos(gdCPos[4] * PI_RAD) * cos(gdCPos[5] * PI_RAD);
    // mtx_trans_grip.X = -gdCPos[0]; 
	// mtx_trans_grip.Y = -gdCPos[1]; 
	// mtx_trans_grip.Z = -gdCPos[2];  //基座交替旋转矩阵


	// float in_point[3] = {0,0,0};
	// float out_point[3];
	// PointCoordinateC(&mtx_trans_grip, in_point, out_point);


	// gdCPos[0] = out_point[0];
	// gdCPos[1] = out_point[1];
	// gdCPos[2] = out_point[2];
	// return 0;

	int i;
	double jrad[6];      //关节弧度

	double  temp[8];   //中间变量
	MtxKine lm_Tcp;      //中间变量，TCP矩阵

	//关节弧度
	jrad[0]=-gdJPos[5]*PI_RAD;
	jrad[1]=-gdJPos[4]*PI_RAD;
	jrad[2]=-gdJPos[3]*PI_RAD;
	jrad[3]=-gdJPos[2]*PI_RAD;
	jrad[4]=-gdJPos[1]*PI_RAD;
	jrad[5]=-gdJPos[0]*PI_RAD; 
	
	//TCP矩阵
	double c23 = cos(jrad[1]+jrad[2]);
	double s23 = sin(jrad[1]+jrad[2]);

	//计算TCP姿态矩阵参数
	lm_Tcp.R11 = sin(jrad[5]) * (cos(jrad[2]) * sin(jrad[0]) + cos(jrad[0]) * cos(jrad[1]) * sin(jrad[2])) - cos(jrad[5]) * (cos(jrad[4]) * (cos(jrad[3]) * (sin(jrad[0]) * sin(jrad[2]) - cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2])) + cos(jrad[0]) * sin(jrad[1]) * sin(jrad[3])) - sin(jrad[4]) * (sin(jrad[3]) * (sin(jrad[0]) * sin(jrad[2]) - cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2])) - cos(jrad[0]) * cos(jrad[3]) * sin(jrad[1])));
	lm_Tcp.R12 = sin(jrad[5]) * (cos(jrad[4]) * (cos(jrad[3]) * (sin(jrad[0]) * sin(jrad[2]) - cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2])) + cos(jrad[0]) * sin(jrad[1]) * sin(jrad[3])) - sin(jrad[4]) * (sin(jrad[3]) * (sin(jrad[0]) * sin(jrad[2]) - cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2])) - cos(jrad[0]) * cos(jrad[3]) * sin(jrad[1]))) + cos(jrad[5]) * (cos(jrad[2]) * sin(jrad[0]) + cos(jrad[0]) * cos(jrad[1]) * sin(jrad[2]));
	lm_Tcp.R13 = cos(jrad[4]) * (sin(jrad[3]) * (sin(jrad[0]) * sin(jrad[2]) - cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2])) - cos(jrad[0]) * cos(jrad[3]) * sin(jrad[1])) + sin(jrad[4]) * (cos(jrad[3]) * (sin(jrad[0]) * sin(jrad[2]) - cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2])) + cos(jrad[0]) * sin(jrad[1]) * sin(jrad[3]));
	lm_Tcp.R21 = cos(jrad[5]) * (cos(jrad[4]) * (cos(jrad[3]) * (cos(jrad[0]) * sin(jrad[2]) + cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0])) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[3])) - sin(jrad[4]) * (sin(jrad[3]) * (cos(jrad[0]) * sin(jrad[2]) + cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0])) + cos(jrad[3]) * sin(jrad[0]) * sin(jrad[1]))) - sin(jrad[5]) * (cos(jrad[0]) * cos(jrad[2]) - cos(jrad[1]) * sin(jrad[0]) * sin(jrad[2]));
	lm_Tcp.R22 = -sin(jrad[5]) * (cos(jrad[4]) * (cos(jrad[3]) * (cos(jrad[0]) * sin(jrad[2]) + cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0])) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[3])) - sin(jrad[4]) * (sin(jrad[3]) * (cos(jrad[0]) * sin(jrad[2]) + cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0])) + cos(jrad[3]) * sin(jrad[0]) * sin(jrad[1]))) - cos(jrad[5]) * (cos(jrad[0]) * cos(jrad[2]) - cos(jrad[1]) * sin(jrad[0]) * sin(jrad[2]));
	lm_Tcp.R23 = -cos(jrad[4]) * (sin(jrad[3]) * (cos(jrad[0]) * sin(jrad[2]) + cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0])) + cos(jrad[3]) * sin(jrad[0]) * sin(jrad[1])) - sin(jrad[4]) * (cos(jrad[3]) * (cos(jrad[0]) * sin(jrad[2]) + cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0])) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[3]));
	lm_Tcp.R31 = -cos(jrad[5]) * (cos(jrad[4]) * (cos(jrad[1]) * sin(jrad[3]) + cos(jrad[2]) * cos(jrad[3]) * sin(jrad[1])) + sin(jrad[4]) * (cos(jrad[1]) * cos(jrad[3]) - cos(jrad[2]) * sin(jrad[1]) * sin(jrad[3]))) - sin(jrad[1]) * sin(jrad[2]) * sin(jrad[5]);
	lm_Tcp.R32 = sin(jrad[5]) * (cos(jrad[4]) * (cos(jrad[1]) * sin(jrad[3]) + cos(jrad[2]) * cos(jrad[3]) * sin(jrad[1])) + sin(jrad[4]) * (cos(jrad[1]) * cos(jrad[3]) - cos(jrad[2]) * sin(jrad[1]) * sin(jrad[3]))) - cos(jrad[5]) * sin(jrad[1]) * sin(jrad[2]);
	lm_Tcp.R33 = sin(jrad[4]) * (cos(jrad[1]) * sin(jrad[3]) + cos(jrad[2]) * cos(jrad[3]) * sin(jrad[1])) - cos(jrad[4]) * (cos(jrad[1]) * cos(jrad[3]) - cos(jrad[2]) * sin(jrad[1]) * sin(jrad[3]));

	//计算TCP位置
	lm_Tcp.X = L_G2[2] * (cos(jrad[3]) * (sin(jrad[0]) * sin(jrad[2]) - cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2])) + cos(jrad[0]) * sin(jrad[1]) * sin(jrad[3])) - (L_G2[3]) * (cos(jrad[4]) * (sin(jrad[3]) * (sin(jrad[0]) * sin(jrad[2]) - cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2])) - cos(jrad[0]) * cos(jrad[3]) * sin(jrad[1])) + sin(jrad[4]) * (cos(jrad[3]) * (sin(jrad[0]) * sin(jrad[2]) - cos(jrad[0]) * cos(jrad[1]) * cos(jrad[2])) + cos(jrad[0]) * sin(jrad[1]) * sin(jrad[3]))) - cos(jrad[0]) * sin(jrad[1]) * (L_G2[1]);
	lm_Tcp.Y = (cos(jrad[4]) * (sin(jrad[3]) * (cos(jrad[0]) * sin(jrad[2]) + cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0])) + cos(jrad[3]) * sin(jrad[0]) * sin(jrad[1])) + sin(jrad[4]) * (cos(jrad[3]) * (cos(jrad[0]) * sin(jrad[2]) + cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0])) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[3]))) * (L_G2[3]) - L_G2[2] * (cos(jrad[3]) * (cos(jrad[0]) * sin(jrad[2]) + cos(jrad[1]) * cos(jrad[2]) * sin(jrad[0])) - sin(jrad[0]) * sin(jrad[1]) * sin(jrad[3])) - sin(jrad[0]) * sin(jrad[1]) * (L_G2[1]);
	lm_Tcp.Z = (L_G2[3]) * (cos(jrad[4]) * (cos(jrad[1]) * cos(jrad[3]) - cos(jrad[2]) * sin(jrad[1]) * sin(jrad[3])) - sin(jrad[4]) * (cos(jrad[1]) * sin(jrad[3]) + cos(jrad[2]) * cos(jrad[3]) * sin(jrad[1]))) - L_G2[0] + L_G2[2] * (cos(jrad[1]) * sin(jrad[3]) + cos(jrad[2]) * cos(jrad[3]) * sin(jrad[1])) - cos(jrad[1]) * (L_G2[1]);

	// std::cout << "G1 mtx" << std::endl;
	// std::cout << lm_Tcp.R11 << " " << lm_Tcp.R12 << " "<<lm_Tcp.R13 << std::endl;
	// std::cout << lm_Tcp.R21 << " " << lm_Tcp.R22 << " "<<lm_Tcp.R23 << std::endl;
	// std::cout << lm_Tcp.R31 << " " << lm_Tcp.R32 << " "<<lm_Tcp.R33 << std::endl;
	// std::cout << lm_Tcp.X << " " << lm_Tcp.Y << " "<<lm_Tcp.Z << std::endl;

	//计算PRY角
	temp[5] = atan2(-lm_Tcp.R31, sqrt(lm_Tcp.R11 * lm_Tcp.R11 + lm_Tcp.R21 * lm_Tcp.R21));

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


int Kine_CR_SixDoF_G2::IKine(double* gdCPos, double* gdJCurr, double* gdJPos)
{
	double jpos_G2[6];
	// double jcurrpos_G2[6];
	//int Trx[4][4] = {{1,0,0,0},{0,-1,0,0},{0,0,-1,0},{0,0,0,1}};
	int flag;

    double yaw = gdCPos[3]* PI_RAD,pitch =gdCPos[4]* PI_RAD,droll = gdCPos[5] * PI_RAD;

	//EulerAngles to RotationMatrix
    ::Eigen::Vector3d ea0(yaw, pitch, droll);
    ::Eigen::Matrix3d curmtx;
    
    curmtx = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
        * ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
        * ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

    ::Eigen::Matrix4d R;
    R << curmtx(0,0), curmtx(0,1), curmtx(0,2), gdCPos[0],
        curmtx(1,0),  curmtx(1,1), curmtx(1,2), gdCPos[1], 
        curmtx(2,0),  curmtx(2,1), curmtx(2,2), gdCPos[2],
        0, 0 ,0 ,1;
	//逆运动学
    ::Eigen::Matrix4d Trx;
    Trx << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;
    ::Eigen::Matrix4d tarmtx;
    tarmtx = Trx * (R * Trx).inverse();

	::Eigen::Matrix3d trans_euler(3, 3);
	trans_euler << tarmtx(0, 0), tarmtx(0, 1), tarmtx(0, 2), tarmtx(1, 0), tarmtx(1, 1), tarmtx(1, 2), tarmtx(2, 0), tarmtx(2, 1), tarmtx(2, 2);

	Eigen::Vector3d eulerAngle=trans_euler.eulerAngles(2,1,0);	//旋转矩阵转换成欧拉角zyx

	double gdcpos_g1[6];
	gdcpos_g1[0] = tarmtx(0, 3);
	gdcpos_g1[1] = tarmtx(1, 3);
	gdcpos_g1[2] = tarmtx(2, 3);
	gdcpos_g1[3] = eulerAngle[0] * PI_DEG;
	gdcpos_g1[4] = eulerAngle[1] * PI_DEG;
	gdcpos_g1[5] = eulerAngle[2] * PI_DEG;

	for (int i = 0; i < 6;++i)
	{
		if(fabs(gdcpos_g1[i]) < 0.00001)
		{
			gdcpos_g1[i] = 0;
		}
	}

	// std::cout << "G1"
	// 		  << " ";
	// for (int i = 0; i < 6;++i)
	// {
	// 	std::cout<<gdcpos_g1[i]<<",";
	// }	std::cout  << std::endl;

	flag = kine.IKine(gdcpos_g1, gdJCurr, jpos_G2);

	if(0 == flag)
	{
		// std::cout << "success" << std::endl;
		// gdJPos[0] = jpos_G2[0];
		// gdJPos[1] = -(jpos_G2[1])+180;
		// gdJPos[2] = -(jpos_G2[2])+180;
		// gdJPos[3] = jpos_G2[3];
		// gdJPos[4] = -jpos_G2[4];
		// gdJPos[5] = jpos_G2[5];

		gdJPos[0] = jpos_G2[0];
		gdJPos[1] = jpos_G2[1];
		gdJPos[2] = jpos_G2[2];
		gdJPos[3] = jpos_G2[3];
		gdJPos[4] = jpos_G2[4];
		gdJPos[5] = jpos_G2[5];
		return 0;
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
void Linkage6D(IN double gdLen[7],IN double* gdJPos,std::vector<double> &cur_truss,int length,int angle,const int Grip_Id,
				std::vector<std::vector<double>> &Linkage)
{
	if(Grip_Id == 0)
	{
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
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after,Grip_Id);
		link_point[0] = tmp_after[0];
		link_point[1] = tmp_after[1];
		link_point[2] = tmp_after[2];
		tmp_before[0] = 0;tmp_before[1] = 0;tmp_before[2] = L01;
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after,Grip_Id);
		link_point[3] = tmp_after[0];
		link_point[4] = tmp_after[1];
		link_point[5] = tmp_after[2];	
		Linkage.push_back(link_point);
		//第二根连杆L2
		// link_point = {0, 0, L01, L2 * c1 * c2, L2 * c2 * s1, L01 + L2 * s2};
		link_point[0] = tmp_after[0];
		link_point[1] = tmp_after[1];
		link_point[2] = tmp_after[2];
		tmp_before[0] = L2 * c1 * c2;tmp_before[1] = L2 * c2 * s1;tmp_before[2] = L01 + L2 * s2;
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after,Grip_Id);
		link_point[3] = tmp_after[0];
		link_point[4] = tmp_after[1];
		link_point[5] = tmp_after[2];	
		Linkage.push_back(link_point);
		//第三根连杆L34
		// link_point = {L2*c1*c2, L2*c2*s1, L01+L2*s2,L34*c1*s23+L2*c1*c2, L34*s1*s23+ L2*c2*s1, -L34*c23+L01+L2*s2};
		link_point[0] = tmp_after[0];
		link_point[1] = tmp_after[1];
		link_point[2] = tmp_after[2];
		tmp_before[0] = L34*c1*s23+L2*c1*c2;tmp_before[1] =L34*s1*s23+ L2*c2*s1;tmp_before[2] = -L34*c23+L01+L2*s2;
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after,Grip_Id);
		link_point[3] = tmp_after[0];
		link_point[4] = tmp_after[1];
		link_point[5] = tmp_after[2];	
		Linkage.push_back(link_point);
		//第四根连杆L56
		// link_point = {L34*c1*s23+L2*c1*c2, L34*s1*s23+ L2*c2*s1, -L34*c23+L01+L2*s2,(c1*c23*c4*s5 + s1*s4*s5 + c1*s23*c5)*L56 + L34*c1*s23+L2*c1*c2,
		// (s1*c23*c4*s5 -c1*s4*s5 + s1*s23*c5)*L56+L34*s1*s23+ L2*c2*s1,(s23*c4*s5 - c23*c5)*L56 - L34*c23+L01+L2*s2};
		link_point[0] = tmp_after[0];
		link_point[1] = tmp_after[1];
		link_point[2] = tmp_after[2];
		tmp_before[0] =(c1*c23*c4*s5 + s1*s4*s5 + c1*s23*c5)*L56 + L34*c1*s23+L2*c1*c2;
		tmp_before[1] =(s1*c23*c4*s5 -c1*s4*s5 + s1*s23*c5)*L56+L34*s1*s23+ L2*c2*s1;
		tmp_before[2] =(s23*c4*s5 - c23*c5)*L56 - L34*c23+L01+L2*s2;
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after,Grip_Id);
		link_point[3] = tmp_after[0];
		link_point[4] = tmp_after[1];
		link_point[5] = tmp_after[2];
		Linkage.push_back(link_point);

		return;
	}
	else if(Grip_Id == 7)
	{
		double jrad[6];
		//关节弧度
		jrad[0]= - gdJPos[5]*PI_RAD;
		jrad[1]= - gdJPos[4]*PI_RAD;
		jrad[2]= - gdJPos[3]*PI_RAD;
		jrad[3]= - gdJPos[2]*PI_RAD;
		jrad[4]= - gdJPos[1]*PI_RAD;
		jrad[5]= - gdJPos[0]*PI_RAD; 

		// std::cout << "G2now" << std::endl;
		std::vector<double> link_point(6, 0);
		float tmp_link[6];
		double L01 = gdLen[5] + gdLen[6];
		double L2 = gdLen[3] + gdLen[4];
		double L34 = gdLen[2];
		double L56 = gdLen[0] + gdLen[1];

		double c1 = cos(jrad[0]);
		double s1 = sin(jrad[0]);
		double c2 = cos(jrad[1]);
		double s2 = sin(jrad[1]);
		double c3 = cos(jrad[2]);
		double s3 = sin(jrad[2]);
		double c4 = cos(jrad[3]);
		double s4 = sin(jrad[3]);
		double c5 = cos(jrad[4]);
		double s5 = sin(jrad[4]);

		float tmp_before[3];
		float tmp_after[3];

		//第一根连杆L01
		// link_point = {0, 0, 0, 0, 0, L01};
		tmp_before[0] = 0;tmp_before[1] = 0;tmp_before[2] = 0;
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after,Grip_Id);
		link_point[0] = tmp_after[0];
		link_point[1] = tmp_after[1];
		link_point[2] = tmp_after[2];
		tmp_before[0] = 0;tmp_before[1] = 0;tmp_before[2] = -L01;
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after,Grip_Id);
		link_point[3] = tmp_after[0];
		link_point[4] = tmp_after[1];
		link_point[5] = tmp_after[2];	
		Linkage.push_back(link_point);
		//第二根连杆L2
		link_point[0] = tmp_after[0];
		link_point[1] = tmp_after[1];
		link_point[2] = tmp_after[2];
		tmp_before[0] = - c1*s2*L2;
		tmp_before[1] = - s1*s2*L2 ;
		tmp_before[2] = - c2*L2 -L01;
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after,Grip_Id);
		link_point[3] = tmp_after[0];
		link_point[4] = tmp_after[1];
		link_point[5] = tmp_after[2];	
		Linkage.push_back(link_point);
		//第三根连杆L34
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after,Grip_Id);
		link_point[0] = tmp_after[0];
		link_point[1] = tmp_after[1];
		link_point[2] = tmp_after[2];
		tmp_before[0] = L34 * (c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c1*s2*L2 ;
		tmp_before[1] = - L34*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4)- s1*s2*L2;
		tmp_before[2] =  L34*(c2*s4 + c3*c4*s2) - c2*L2 -L01;
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after,Grip_Id);
		link_point[3] = tmp_after[0];
		link_point[4] = tmp_after[1];
		link_point[5] = tmp_after[2];	
		Linkage.push_back(link_point);
		//第四根连杆L56
		link_point[0] = tmp_after[0];
		link_point[1] = tmp_after[1];
		link_point[2] = tmp_after[2];
		tmp_before[0] =L2*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - L01*(c5*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) + s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4)) - c1*s2*L34;
		tmp_before[1] =(c5*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4))*L01 - L2*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - s1*s2*L34;
		tmp_before[2] =L01*(c5*(c2*c4 - c3*s2*s4) - s5*(c2*s4 + c3*c4*s2)) - L56 + L2*(c2*s4 + c3*c4*s2) - c2*L34;
		transPointToWorld(tmp_before, cur_truss, length, angle, tmp_after, Grip_Id);
		link_point[3] = tmp_after[0];
		link_point[4] = tmp_after[1];
		link_point[5] = tmp_after[2];
		Linkage.push_back(link_point);
		return;
	}
	else
	{
		std::cout << "Grip_ID error!" << std::endl;
		return;
	}

	return;
}


/******************************************************************************
 * 函数：Set_Length()
 * 功能：设置杆长
 *
 * 输入：gdLen - 杆长L1、L2、L3、L4、L5
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
 * 函数：FKine()
 * 功能：正解
 *
 * 输入：double gdJPos[] - 关节转角, 5关节
 * 输出：double gdCPos[] - 正解位姿, (x,y,z,w,p,r)
 *
 * 返回：int - 0成功,
 ******************************************************************************/
int Kine_CR_FiveDoF_G1::FKine(double gdJPos[], double gdCPos[])
{
	// 关节角度 - 关节弧度 - Tcp矩阵 - 位姿
	// id_jPos  -   jRad   - lm_Tcp  - id_cPos
	int i;
	double jrad[5]; // 关节弧度

	double ld_temp[8];     // 中间变量
	MtxKine lm_Tcp;        // 中间变量,TCP矩阵

	//--------------------- 关节弧度 -------------------------//
	for (i=0; i<5; i++)
	{
		jrad[i] = gdJPos[i] * PI_RAD;
	}

	//--------------------- Tcp矩阵 -------------------------//
	double c23 = cos(jrad[1] + jrad[2]);
	double s23 = sin(jrad[1] + jrad[2]);
	double c234 = cos(jrad[1] + jrad[2] + jrad[3]);
	double s234 = sin(jrad[1] + jrad[2] + jrad[3]);

	// 计算Tcp姿态矩阵的参数 //
	lm_Tcp.R11 =   cos(jrad[0]) * c234 * cos(jrad[4]) + sin(jrad[0]) * sin(jrad[4]);
	lm_Tcp.R12 = - cos(jrad[0]) * c234 * sin(jrad[4]) + sin(jrad[0]) * cos(jrad[4]);
	lm_Tcp.R13 =   cos(jrad[0]) * s234;
	lm_Tcp.R21 =   sin(jrad[0]) * c234 * cos(jrad[4]) - cos(jrad[0]) * sin(jrad[4]);
	lm_Tcp.R22 = - sin(jrad[0]) * c234 * sin(jrad[4]) - cos(jrad[0]) * cos(jrad[4]);
	lm_Tcp.R23 =   sin(jrad[0]) * s234;
	lm_Tcp.R31 =   s234 * cos(jrad[4]);
	lm_Tcp.R32 = - s234 * sin(jrad[4]);
	lm_Tcp.R33 = - c234;

	// 计算Tcp位置 //
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

	//--------------------- Tcp位姿 -------------------------//
	// 计算工具坐标系RPY角 - rad //
	ld_temp[5] = atan2(-lm_Tcp.R31, sqrt(lm_Tcp.R11 * lm_Tcp.R11 + lm_Tcp.R21 * lm_Tcp.R21));

	if (fabs(ld_temp[5] - PI / 2) < 0.00001)
	{
		ld_temp[4] = 0;
		ld_temp[6] = atan2(lm_Tcp.R12, lm_Tcp.R22);
	}
	else if (fabs(ld_temp[5] + PI / 2) < 0.00001)
	{
		ld_temp[4] = 0;
		ld_temp[6] = -atan2(lm_Tcp.R12, lm_Tcp.R22);
	}
	else
	{
		ld_temp[0] = 1 / cos(ld_temp[5]);

		ld_temp[1] = lm_Tcp.R21 * ld_temp[0];
		ld_temp[2] = lm_Tcp.R11 * ld_temp[0];
		if (fabs(ld_temp[1]) < 0.00001)
		{
			ld_temp[1] = 0;
		}
		if (fabs(ld_temp[2]) < 0.00001)
		{
			ld_temp[2] = 0;
		}
		ld_temp[4] = atan2(ld_temp[1], ld_temp[2]);

		ld_temp[1] = lm_Tcp.R32 * ld_temp[0];
		ld_temp[2] = lm_Tcp.R33 * ld_temp[0];
		if (fabs(ld_temp[1]) < 0.00001)
		{
			ld_temp[1] = 0;
		}
		if (fabs(ld_temp[2]) < 0.00001)
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

	return 0;

}

int Kine_CR_FiveDoF_G1::FKine_Inc(IN double gdJPos[5], IN double inc[3], OUT double gdCPos[3])
{
	// 关节角度 - 关节弧度 - Tcp矩阵 - 位姿
	// id_jPos  -   jRad   - lm_Tcp  - id_cPos
	int i;
	double jrad[5]; //关节弧度
	
//	double ld_temp[8];     // 中间变量
	MtxKine lm_Tcp;        // 中间变量,TCP矩阵
	
    //--------------------- 关节弧度 -------------------------//
	for (i=0; i<5; i++)
	{
		jrad[i] = gdJPos[i] * PI_RAD;
	}
	
    //--------------------- Tcp矩阵 -------------------------//
	double c23 = cos(jrad[1] + jrad[2]);
	double s23 = sin(jrad[1] + jrad[2]);
	double c234 = cos(jrad[1] + jrad[2] + jrad[3]);
	double s234 = sin(jrad[1] + jrad[2] + jrad[3]);
	
    // 计算Tcp姿态矩阵的参数 //
    lm_Tcp.R11 =   cos(jrad[0]) * c234 * cos(jrad[4]) + sin(jrad[0]) * sin(jrad[4]);
	lm_Tcp.R12 = - cos(jrad[0]) * c234 * sin(jrad[4]) + sin(jrad[0]) * cos(jrad[4]);
	lm_Tcp.R13 =   cos(jrad[0]) * s234;
	lm_Tcp.R21 =   sin(jrad[0]) * c234 * cos(jrad[4]) - cos(jrad[0]) * sin(jrad[4]);
	lm_Tcp.R22 = - sin(jrad[0]) * c234 * sin(jrad[4]) - cos(jrad[0]) * cos(jrad[4]);
	lm_Tcp.R23 =   sin(jrad[0]) * s234;
	lm_Tcp.R31 =   s234 * cos(jrad[4]);
	lm_Tcp.R32 = - s234 * sin(jrad[4]);
	lm_Tcp.R33 = - c234;
	
    // 计算Tcp位置 //
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

	
	return 0;
}

/******************************************************************************
 * 函数：IKine()
 * 功能：逆解
 *
 * 输入：double* gdCPos  - 位姿数组, (x,y,z,w,p,r)
 *       double* gdJCurr - 当前关节转角, 5关节
 * 输出：double* gdJPos  - 逆解关节转角, 5关节
 *
 * 返回：int - 0成功, 其他错误
 ******************************************************************************/
 int Kine_CR_FiveDoF_G1::IKine(double* gdCPos, double* gdJCurr, double* gdJPos)
 {
	//   位姿  - Tcp矩阵 - 关节弧度 - 关节角度
	// id_cPos - lm_Tcp  - ld_jRad  - id_jPos
	int i;
	int result;
	int li_flag[4] = {0};      // 四组解的情况,0为有解

	double s1,c1,s2,c2, s3, c3, s5, c5, s234, c234;

	double ld_temp[8];       // 中间变量
	MtxKine lm_Tcp;          // 中间变量,TCP矩阵
	double gd_rad[4][5];     // 中间变量，四组逆解

    //--------------------- TCP矩阵 -------------------------//
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

    //--------------------- 关节弧度 -------------------------//
    //------ 转角1 -------//
	ld_temp[0] = atan2(lm_Tcp.Y, lm_Tcp.X);
	ld_temp[1] = atan2(- lm_Tcp.Y, - lm_Tcp.X);
	RadInRange(&ld_temp[0], &gdJCurr[0]);
	RadInRange(&ld_temp[1], &gdJCurr[0]);

	gd_rad[0][0] = gd_rad[1][0] = ld_temp[0];
	gd_rad[2][0] = gd_rad[3][0] = ld_temp[1];

    //------ 转角5,3 ------//
    for (i=0; i<2; i++)
    {
        s1 = sin(gd_rad[2*i][0]);
        c1 = cos(gd_rad[2*i][0]);
	
		s5 = s1 * lm_Tcp.R11 - c1 * lm_Tcp.R21;
		c5 = s1 * lm_Tcp.R12 - c1 * lm_Tcp.R22;

		// 计算转角5 // 
		ld_temp[0] = atan2(s5, c5);
		RadInRange(&ld_temp[0], &gdJCurr[4]);

        gd_rad[2*i][4]   = ld_temp[0];
        gd_rad[2*i+1][4] = ld_temp[0];

		// 计算转角3 //
		ld_temp[2] = (
			  (lm_Tcp.X * c1 + lm_Tcp.Y * s1) * (lm_Tcp.X * c1 + lm_Tcp.Y * s1)
			+ (lm_Tcp.Z - m_dL1) * (lm_Tcp.Z - m_dL1) 
			- (m_dL2 * m_dL2 + m_dL3 * m_dL3)	) / (2 * m_dL2 * m_dL3);
		ld_temp[0] = 1 - ld_temp[2] * ld_temp[2];

		if (fabs(ld_temp[0]) < 0.00001)
		{
			ld_temp[1] = 0;
		}
		else if (ld_temp[0] < 0)
		{
			li_flag[2*i]   = 1;     // 标志 - 此组解无解
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
		return 1; //  腰关节无可用逆解值 //
    }
	
    //------ 转角2,4 ------//
    for (i=0; i<4; i++)
    {
		if (0 == li_flag[i])
		{
			s1 = sin(gd_rad[i][0]);
	        c1 = cos(gd_rad[i][0]);

			s3 = sin(gd_rad[i][2]);
			c3 = cos(gd_rad[i][2]);
		
			// 计算转角2 //
			ld_temp[1] = lm_Tcp.Z - m_dL1;
			ld_temp[2] = lm_Tcp.X * c1 + lm_Tcp.Y * s1;
			ld_temp[3] = ld_temp[1] * ld_temp[1] + ld_temp[2] * ld_temp[2];

			s2 = (ld_temp[1] * (m_dL3 * c3 + m_dL2) - ld_temp[2] * m_dL3 * s3) / ld_temp[3];
			c2 = (ld_temp[2] * (m_dL3 * c3 + m_dL2) + ld_temp[1] * m_dL3 * s3) / ld_temp[3];

			ld_temp[4] = atan2(s2, c2);
			RadInRange(&ld_temp[4], &gdJCurr[1]);

			gd_rad[i][1] = ld_temp[4];


			// 计算转角4 //
			s234 = lm_Tcp.R13 * c1 + lm_Tcp.R23 * s1;
			c234 = - lm_Tcp.R33;
			
			ld_temp[0] = atan2(s234, c234); // 234之和

			
			ld_temp[3] = ld_temp[0] - gd_rad[i][1] - gd_rad[i][2];
			RadInRange(&ld_temp[3], &gdJCurr[3]);
			
			gd_rad[i][3] = ld_temp[3];
		}
	}
	
	//------ 最佳结果 ------//
	for (i=0; i<4; i++)
	{
		if (0 == li_flag[i])  // 求取相对绝对值
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
			s1 = ld_temp[i]; // 用第一个有效值 来 初始化 中间变量s1
			result = i;
			break;           // 推出初始化
		}
	}
	for (i=0; i<4; i++)
	{
		if ((0 == li_flag[i]) && (ld_temp[i] <= s1))  // 有效值 | 相对绝对值最小
		{
			//ld_jRad = gd_rad[i];
			s1 = ld_temp[i];
			result = i;
		}
	}
	//------ 关节角度 ------//
	gdJPos[0] = gd_rad[result][0] * PI_DEG;
	gdJPos[1] = gd_rad[result][1] * PI_DEG;
	gdJPos[2] = gd_rad[result][2] * PI_DEG;
	gdJPos[3] = gd_rad[result][3] * PI_DEG;
	gdJPos[4] = gd_rad[result][4] * PI_DEG;


	return 0;
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

// 	jpos_G2[0] = - gdJPos[4];
// 	jpos_G2[1] = gdJPos[3];
// 	jpos_G2[2] = gdJPos[2];
// 	jpos_G2[3] = gdJPos[1];
// 	jpos_G2[4] = -gdJPos[0];

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
	
// 	jcurrpos_G2[0] = - gdJCurr[4];
// 	jcurrpos_G2[1] =   gdJCurr[3];
// 	jcurrpos_G2[2] =   gdJCurr[2];
// 	jcurrpos_G2[3] =   gdJCurr[1];
// 	jcurrpos_G2[4] = - gdJCurr[0];

	jcurrpos_G2[0] = gdJCurr[4];
	jcurrpos_G2[1] =   gdJCurr[3];
	jcurrpos_G2[2] =   gdJCurr[2];
	jcurrpos_G2[3] =   gdJCurr[1];
	jcurrpos_G2[4] = gdJCurr[0];

	flag = kine.IKine(gdCPos, jcurrpos_G2, jpos_G2);

	if (0 == flag)
	{
		gdJPos[0] = jpos_G2[4];
		gdJPos[1] =   jpos_G2[3];
		gdJPos[2] =   jpos_G2[2];
		gdJPos[3] =   jpos_G2[1];
		gdJPos[4] = jpos_G2[0];
	}
	else 
	{
		return flag;
	}

	return 0;
}

// int Linktest()
// {
// 	double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
//     double gdjPos[6] = {7.95526,147.273,157.015,-13.1725,57.9791,15.1831};
//     double curpos1[6],curpos2[6];
//     std::vector<std::vector<double>> link;
//     Linkage6D(len,gdjPos,truss1,60,28,0,link);	//#include"envirnoment.h"
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