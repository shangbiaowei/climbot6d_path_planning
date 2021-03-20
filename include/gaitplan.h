#ifndef Gaitplan_H__
#define Gaitplan_H__

#include <iostream>
#include <vector>
#include "Kine.h"



class gaitPlan
{
protected:
    Kine_CR_SixDoF_G1 climbot6d;

private:
    /* data */
public:


    /*
    * 功能：获取当前关节值机器人末端在世界坐标系下的位置
    * 参数：
    * - single_num：每根杆件需要步数
    * - opti_res：夹持点规划优化结果
    * - adj_mat：邻接矩阵
    * 返回值：1 - 成功, 0 - 失败, -1 - 程序运行异常
    */
    void curGripperPos(std::vector<double> &joint_angle,
                        std::vector<double> &cur_truss,
                        int length,int angle,
                        float* out_point);

    
};


#endif  //gaitplan.h