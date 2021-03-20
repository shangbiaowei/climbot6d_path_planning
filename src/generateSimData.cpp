#include"../include/generateSimData.h"


SimDataGenerate::SimDataGenerate()
{

}

SimDataGenerate::~SimDataGenerate()
{

}

void SimDataGenerate::getJointAngle(std::vector<double> &cur_truss, std::vector<double> &tar_truss,
                                    int length_cur, int alpha_cur,
                                    int length_tar, int alpha_tar,
                                    int flag,std::vector<double> &joint_val)
{
    
    if(flag % 2 == 1)
    {
        test.simulationResultPrint(cur_truss, tar_truss, length_cur, alpha_cur, length_tar, alpha_tar, 0,joint_val);
    }
    else
    {
        test.simulationResultPrint(tar_truss, cur_truss, length_tar, alpha_tar, length_cur, alpha_cur, 0,joint_val);
    }
}


void SimDataGenerate::generatePathTxt(std::vector<std::vector<int> >& point_list,
                                        std::vector<std::vector<double>>& truss,
                                        std::vector<int>& truss_list,
                                        std::vector<int>& single_num)
{
    std::ofstream outfile;
    outfile.open("../src/jointvalue.txt");  //输出仿真用地图列表
    // outfile.open("/home/wei/ros/fourthClimbotSimulation/src/fourthClimbotSim/mr_simulation/config/climbot6d_path_list.txt");  //输出仿真用地图列表
    std::vector<double> joint_val;

    int step_counter = 0;
    int global_counter = 0;
    int i = 0;

    outfile << "# note: 每一次步态结束，"<< std::endl;
    outfile << "# 使用 'HALT' 标志位表示切换夹持器"<< std::endl;
    outfile << "P=0,0,0,0,0,0,;"<< std::endl;
    while( i < truss_list.size())
    {
        // std::cout << "cur_i" << i << std::endl;
        if(step_counter < single_num[i])
        {
            getJointAngle(truss[truss_list[i] - 1], truss[truss_list[i] - 1], point_list[global_counter][0], point_list[global_counter][1], point_list[global_counter + 1][0], point_list[global_counter + 1][1], global_counter,joint_val);
            outfile << "P=";
            for (size_t t = 0; t < 6;++t)
            {
                outfile << joint_val[t] << ",";
            }
            outfile << ";"<< std::endl;
            outfile << "HALT" << std::endl;
            outfile << "P=";
            for (size_t t = 0; t < 6;++t)
            {
                outfile << joint_val[t] << ",";
            }
            outfile << ";"<< std::endl;
            step_counter++;
            global_counter++;
        }
        else if(step_counter == single_num[i])
        {
            if(global_counter == point_list.size()-1)
            {
                break;  //最后一步不存在,跳出
            }
            getJointAngle(truss[truss_list[i] - 1], truss[truss_list[i + 1] - 1], point_list[global_counter][0], point_list[global_counter][1], point_list[global_counter + 1][0], point_list[global_counter + 1][1], global_counter, joint_val);
            outfile << "P=";
            for (size_t t = 0; t < 6;++t)
            {
                outfile << joint_val[t] << ",";
            }
            outfile << ";" << std::endl;
            outfile << "HALT" << std::endl;
            outfile << "P=";
            for (size_t t = 0; t < 6;++t)
            {
                outfile << joint_val[t] << ",";
            }
            outfile << ";" << std::endl;
            step_counter = 0;
            global_counter++;
            i++;
        }
    }

    outfile.close();
}
