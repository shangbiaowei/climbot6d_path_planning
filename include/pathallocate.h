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
        * 返回值：1 - 成功, 0 - 失败(存在障碍), -1 - 程序运行异常
        */
        int stepFeasibleVerify(const int length1, const int angle1,
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

        std::vector<std::vector<int> > stepAllocate(const int length1, const int angle1,
                                                    const int length2, const int angle2,
                                                    std::vector<double> &cur_truss,int cur_truss_num,
                                                    int single_truss_num);

        std::vector<std::vector<int> > inchwormStep(const int length1, const int angle1,
                                                    const int length2, const int angle2,
                                                    std::vector<double> &cur_truss,int cur_truss_num,
                                                    double len_scale);

        std::vector<std::vector<int>> otherStep(const int length1, const int angle1,
                                                const int length2, const int angle2,
                                                std::vector<double> &cur_truss, int cur_truss_num,
                                                int single_truss_num,double len_scale);

        multimap<double, std::vector<int>> getPathMap(const int length1, const int angle1,
                                                        const int length2, const int angle2,
                                                        std::vector<double> &cur_truss,
                                                        int cur_truss_num,double len_scale);

        // static bool cmp(pair<std::vector<int>, double> a, pair<std::vector<int>, double> b) {
        //     return a.second < b.second;
        // };
};

    // /*
    //  * 功能：生成所有路径点
    //  * 参数：
    //  * - single_num：每根杆件需要步数
    //  * - opti_res：夹持点规划优化结果
    //  * - adj_mat：邻接矩阵
    //  * 返回值：0 - 成功, 1 - 失败, -1 - 程序运行异常
    //  */
    // int allocatePath(std::vector<int> &single_num,
    //              std::vector<std::vector<int>> &opti_res,
    //              std::vector<std::vector<int>> &adj_mat)
    // {
    //     std::vector<int> tmp_point(2, 0);

    //     for (size_t i = 0; i < single_num.size();++i)
    //     {
    //         if(single_num[i] == 0 || single_num[i] == 1)
    //         {
    //             tmp_point = opti_res[2 * i];
    //             final_path_list.push_back(tmp_point);
    //             tmp_point = opti_res[2 * i + 1];
    //             final_path_list.push_back(tmp_point);
    //             global_counter+=2;  //自身一步,过渡一步
    //         }
    //         else{
    //             tmp_point = {0, 0};

    //             final_path_list.push_back(tmp_point);
    //             final_path_list.push_back(tmp_point);
    //             global_counter += single_num[i] + 1;
    //         }
    //     }

    //     std::cout << global_counter << std::endl;
    //     // for (int i = 0; i < final_path_list.size();++i)
    //     // {
    //     //     for (int j = 0; j < final_path_list[0].size(); j++)
    //     //     {
    //     //         std::cout << final_path_list[i][j] << " ";
    //     //     }
    //     //     std::cout << std::endl;
    //     // }

    //     return -1;
    // }

    // /*
    //  * 功能：判断当前位置是否可行
    //  * 参数：
    //  * - cur_truss：机器人基座杆件
    //  * - length1\angle1：机器人基座的离散地图位置
    //  * - length2\angle2：机器人目标夹持点的离散地图位置
    //  * - single_truss_num:夹持点数量
    //  * - flag:标志位,用于判断目前基座手爪
    //  * 返回值：0 - 成功, 1 - 失败(存在障碍), -1 - 程序运行异常
    //  */
    // int stepFeasibleVerify(const int length1,const int angle1,
    //                 const int length2,const int angle2,
    //                 std::vector<double> &cur_truss,
    //                 int cur_truss_num,
    //                 int flag,
    //                 std::vector<std::vector<int>> &adj_mat)
    // {
    //     std::vector<std::vector<double>> link;
    //     std::vector<int> potential_truss;
    //     if (flag % 2 == 1)
    //     {
    //         link = getCurJointPoint(cur_truss, cur_truss, length1, angle1, length2, angle2);
    //         int tmp_flag = getPontentialObstacle(link, adj_mat, cur_truss_num, cur_truss_num, potential_truss);
    //         if(tmp_flag == 1)
    //         {
    //             if(potentialObsTest(link, potential_truss) == 1)
    //             {
    //                 return 1;
    //             }
    //             else{
    //                 return 0;
    //             }
    //         }
    //         else{
    //             return 0;
    //         }
    //     }
    //     else
    //     {

    //         link = getCurJointPoint(cur_truss, cur_truss, length2, angle2, length1, angle1);;
    //         int tmp_flag = getPontentialObstacle(link, adj_mat, cur_truss_num, cur_truss_num, potential_truss);
    //         if(tmp_flag == 1)
    //         {
    //             if(potentialObsTest(link, potential_truss) == 1)
    //             {
    //                 return 1;
    //             }
    //             else{
    //                 return 0;
    //             }
    //         }
    //         else{
    //             return 0;
    //         }
    //     }
    //     return -1;
    // }

    // int stepAllocate(const int length1,const int angle1,
    //                 const int length2,const int angle2,
    //                 std::vector<double> &cur_truss,
    //                 int single_truss_num,
    //                 std::vector<std::vector<int>> adj_mat)
    // {
    //     int length_scale = fabs(length1 - length2) / single_truss_num;
    //     int angle_scale = fabs(angle1 - angle2) / single_truss_num;

    //     int len_scale = (sqrt((cur_truss[0] - cur_truss[3]) * (cur_truss[0] - cur_truss[3]) +
    //                         (cur_truss[1] - cur_truss[4]) * (cur_truss[1] - cur_truss[4]) +
    //                         (cur_truss[2] - cur_truss[5]) * (cur_truss[2] - cur_truss[5]))) /
    //                         m_rows;

    //     std::vector<int> tmp_length_point(single_truss_num - 1, 0);
    //     std::vector<int> tmp_angle_point(single_truss_num - 1, 0);  //存储中间点

    //     //尺蠖步态
    //     if(fabs(length1 - length2) * len_scale <= 100 && single_truss_num == 2)
    //     {
    //         int tmp_point = 500 / len_scale;

    //     }

    //     //其他步态
    //     //先均分
    //     if(length1 > length2)
    //     {

    //     }
    // }

    // int inchwormStep()
    // {

    // }


#endif //pathallocate.h