#ifndef POLE_H__
#define POLE_H__

#include <iostream>
#include <vector>
#include "m_Matrix.h"
#include "Coordinate.h"
#include "Kine.h"

#define pole_rows 128
#define pole_cols 128

struct Poles
{   
    double p[6]; //端点坐标（世界坐标系下）
    double ***p_map ; //展开地图数组
    Vector pole_v;   //杆件向量
    double p_x, p_y, p_z;
    double p_w, p_p, p_r; //位姿数组
    std::vector<std::vector<int> > transflag;  //过渡标志数组
};


class Discretepole
{
    public:
        Discretepole();
        ~Discretepole();
    private:
        int DOF_flag;

    public:
        Poles *cur_pole = new Poles; //当前杆件
        Poles *target_pole = new Poles;  //目标杆件
    
    public:
        //结果打印，作为仿真输入
        int simulationResultPrint(std::vector<double> &p0,
                                std::vector<double> &p1,
                                int length_cur,int alpha_cur,
                                int length_tar,int alpha_tar,
                                const int GripId,
                                std::vector<double>& joint_val);

        //获取关节值
        int getJointResult(std::vector<double> &p0,
                                std::vector<double> &p1,
                                int length_cur,int alpha_cur,
                                int length_tar,int alpha_tar,
                                const int GripId,double *joint_val);

        //设置初始杆件上的起点和角度
        void setStartPos(Poles *cur_pole,
                        int length,int alpha,
                        float start_point[],double &res_angle);
        
        //杆件上离散点数据分配
        void AssignmentData(float temp[],Poles *cur_pole,Poles *target_pole);

        //杆件离散
        void poleTransition(std::vector<double> &p0,
                            std::vector<double> &p1,
                            int length,int alpha,
                            const int GripId,
                            const int DOFflag);      
        
        //可过渡性判断
        void transitionalTest(Poles *cur_pole, Poles *target_pole,const int GripId);

        //四叉树生成步骤：杆件离散处理
        Poles memberDisreteforQTree(std::vector<double> &p0, 
                                    std::vector<double> &p1, 
                                    int length, int alpha, const int GripId);
};

/***********************************
* 可过渡性判断
* 输入：p0[6]、p1[6] - 平面杆件端点数组
* 输出：1-可过渡，0-不可过渡
***********************************/
int transwithIK(const std::vector<double> &p0, const std::vector<double> &p1,const double length_p1, const double alpha_p1, const double length_p2, const double alpha_p2,const int GripId,double *out_gdjpos);

MtxKine getCurBaseMtx(int angle);

MtxKine getCurMtx(std::vector<double> &cur_truss,int length,int angle);

MtxKine getCurMtxfromJointAngle(double *joint_angle, const int Grip_ID);

void transPointToWorld(float* in_point,std::vector<double> &cur_truss,int length,int angle,float* out_point,const int Grip_ID);


#endif  //pole.h