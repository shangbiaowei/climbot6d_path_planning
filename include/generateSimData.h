#ifndef GenerateSimData_H__
#define GenerateSimData_H__

#include<iostream>
#include<fstream>
#include"pole.h"

class SimDataGenerate
{
    protected:
        Discretepole test;

    public:
        void getJointAngle(std::vector<double> &cur_truss, std::vector<double> &tar_truss,
                           int length_cur, int alpha_cur,
                           int length_tar, int alpha_tar,
                           int flag,std::vector<double> &joint_val);

        void generatePathTxt(std::vector<std::vector<int> >& point_list,
                            std::vector<std::vector<double>>& truss,
                            std::vector<int>& truss_list,
                            std::vector<int>& single_num);

    public:
        SimDataGenerate();
        ~SimDataGenerate();
};

#endif  //generateSimData.h