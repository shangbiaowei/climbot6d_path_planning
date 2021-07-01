#ifndef Pathallocate_H__
#define Pathallocate_H__

#include <iostream>
#include <vector>
#include <map>
#include "gpoptidata.h"


class PathAllocate
{
    protected:
        int global_counter;
        std::vector<std::vector<int>> adj_mat;  //邻接矩阵
        std::vector<std::vector<int>> opti_res; //优化结果
        std::vector<int> single_num;   //优化结果对应步数

    public:
        std::vector<std::vector<int>> final_path_list;

        PathAllocate();
        ~PathAllocate();

        int PathAllocateInit(std::vector<std::vector<int>> &adj_mat_input,
                            std::vector<std::vector<int>> &opti_res_input,
                            std::vector<int> &single_num_input);

    public:
        /*
        * 功能：判断当前位置是否可行
        * 参数：
        * - cur_truss：机器人基座杆件
        * - length1\angle1：机器人基座的离散地图位置
        * - length2\angle2：机器人目标夹持点的离散地图位置
        * - single_truss_num:夹持点数量
        * - flag:标志位,用于判断目前基座手爪
        * 返回值：true - 成功, false - 失败(存在障碍), other - 程序运行异常
        */
        bool stepFeasibleVerify(const int length1, const int angle1,
                               const int length2, const int angle2,
                               std::vector<double> &cur_truss,
                               int cur_truss_num,
                               int flag);

        /*
        * 功能：生成所有路径点
        * 参数：
        * - single_num：每根杆件需要步数
        * - opti_res：夹持点规划优化结果
        * - adj_mat：邻接矩阵
        * 返回值：1 - 成功, 0 - 失败, -1 - 程序运行异常
        */
        int allocatePath(std::vector<std::vector<double>>& truss,std::vector<int>& truss_list);

        int stepAllocate(const int length1, const int angle1,
                                                    const int length2, const int angle2,
                                                    std::vector<double> &cur_truss,int cur_truss_num,
                                                    int single_truss_num,std::vector<std::vector<int>> &res);

        void inchwormStep(const int length1, const int angle1,
                                                    const int length2, const int angle2,
                                                    std::vector<double> &cur_truss,int cur_truss_num,
                                                    double len_scale,std::vector<std::vector<int>> &res);

        int otherStep(const int length1, const int angle1,
                                                const int length2, const int angle2,
                                                std::vector<double> &cur_truss, int cur_truss_num,
                                                int single_truss_num,double len_scale,
                                                std::vector<std::vector<int>> &res);

        void getPathMap(const int length1, const int angle1,
                        const int length2, const int angle2,
                        std::vector<double> &cur_truss,
                        int cur_truss_num,double len_scale,multimap<double, std::vector<int>> &path_map);

};


#endif //pathallocate.h