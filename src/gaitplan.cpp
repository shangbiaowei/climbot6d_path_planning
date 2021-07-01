#include "../include/gaitplan.h"


void gaitPlan::curGripperPos(std::vector<double> &joint_angle,
                            std::vector<double> &cur_truss,
                            int length,int angle,
                            float* out_point)
{
    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
    climbot6d.Set_Length(len);
    double gdjpos[6] = {joint_angle[0], joint_angle[1], joint_angle[2], joint_angle[3], joint_angle[4], joint_angle[5]};
    double out_pos[6];

    climbot6d.FKine(gdjpos, out_pos);   //求解出在基座坐标系下机器人末端位姿
    // std::cout << "fkine" << out_pos[0] << " " << out_pos[1] << " " << out_pos[2] << std::endl;

    float position[3] = {out_pos[0], out_pos[1], out_pos[2]};
    //转换到世界坐标系下
    transPointToWorld(position,cur_truss, length, angle, out_point,0);
}






    // std::vector<double> gdjpos = {29.3,83.1, 182.8,-105.8,112.1,61.7};
    // float out_point[3];
    // gaitPaln test;
    // test.curGripperPos(gdjpos,truss1,54,55,out_point);

    // std::cout << out_point[0] << " " << out_point[1] <<" " <<out_point[2] << std::endl;



