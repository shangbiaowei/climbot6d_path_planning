/*****************************************************************************
 *        过渡夹持点优化
 *        GDUT, 2020
 *        输入:
 *        输出:
 *****************************************************************************/

#ifndef grippointopti_H__
#define grippointopti_H__

#include <iostream>
#include "QTree.h"
#include "gpoptidata.h"
#include "environment.h"
#include <nlopt.hpp>

using namespace std;

typedef struct 
{
    std::vector<int> truss_list;
    std::vector<std::vector<int> > grippoint_list;
    std::vector<std::vector<int>> adj_mat; //邻接矩阵
} my_function_data;


std::vector<std::vector<double>> robot_link;
std::vector<int> grip_num_single_member(30,0);    //杆件上夹持点数目：由杆件上运动距离dist计算得来

//输入：dist：单根杆件上首末两点距离
//     angle_start:单根杆件上首点角度
//     angle_end：单根杆件上末点角度
//输出：gripnum：单根杆件上夹持点数目
int getGripNumber(double dist,int length_start,int angle_start,int length_end,int angle_end,std::vector<double> &cur_truss)
{
    double gripnum = 0;
    // std::cout << dist<< std::endl;

    if(dist < 0.001 && angle_start == angle_end)
    {
        gripnum = 0;
    }
    else if(dist < 0.001 && angle_start != angle_end)
    {
        gripnum = 2;
    }
    else if(dist >= 0 && dist <=110) // && (fabs(angle_start - angle_end) < 45)
    {
        gripnum = 2;
    }
    else if(dist > 110 && dist <= 758.4)
    {
        double out_joint[6];
        if (transwithIK(cur_truss, cur_truss,length_start,angle_start,length_end,angle_end,0,out_joint) == 1)
        {
            gripnum = 1;
        }
        else
        {
            gripnum = 2;
        }     
    }
    else if(dist > 758.4)
    {
        gripnum = 1 + ceil(dist / 758.4);
    }
    return gripnum;
}


//目标函数
double objective_fun(const std::vector<double> &init_data,
                std::vector<double> &grad, void* truss_grippoint_list_ptr)
{
    my_function_data * function_data_ptr = (my_function_data *) truss_grippoint_list_ptr;
    std::vector<std::vector<int>> grippoint_list = function_data_ptr->grippoint_list;
    std::vector<int> truss_list = function_data_ptr->truss_list;

    int truss_num = truss_list.size();    //杆件数量
    int var_num = 4 * truss_num - 4;    //变量数量

    std::vector<double> dist(truss_num,0); //杆件上距离：末-首
    // std::vector<int> grip_num_single_member(100,0);    //杆件上夹持点数目：由杆件上运动距离dist计算得来
    std::vector<double> len_scale(truss_num,0); //长度尺寸
    int grip_num_all = 0;

    for (int i = 0; i < truss_num; ++i)
    {
        len_scale[i] = (sqrt((truss[truss_list[i] - 1][0] - truss[truss_list[i] - 1][3]) * (truss[truss_list[i] - 1][0] - truss[truss_list[i] - 1][3]) +
                        (truss[truss_list[i] - 1][1] - truss[truss_list[i] - 1][4]) * (truss[truss_list[i] - 1][1] - truss[truss_list[i] - 1][4]) +
                        (truss[truss_list[i] - 1][2] - truss[truss_list[i] - 1][5]) * (truss[truss_list[i] - 1][2] - truss[truss_list[i] - 1][5]))) /
                        pole_rows;
    }

    //第一根杆件
    dist[0] = fabs((init_data[0] - grippoint_list[0][0])) * len_scale[0]; //求解距离
    grip_num_single_member[0] = getGripNumber(dist[0],grippoint_list[0][0],grippoint_list[0][1],init_data[0],init_data[1],truss[truss_list[0] - 1]);     //获取步数
    grip_num_all += grip_num_single_member[0];  
    //中间n根杆
    for (int i = 1; i < truss_num - 1; ++i)
    {
        dist[i] = fabs((init_data[4 * i] - init_data[4 * i - 2])) * len_scale[i];   //求解距离
        grip_num_single_member[i] = getGripNumber(dist[i],init_data[4 * i - 2],init_data[4 * i - 1],init_data[4 * i],init_data[4 * i + 1],truss[truss_list[i] - 1]);  //获取步数
        grip_num_all += grip_num_single_member[i];
    }
    //最后一根杆件
    dist[dist.size() - 1] = fabs(((grippoint_list[grippoint_list.size() - 1][0]) - init_data[var_num - 2])) * len_scale[len_scale.size() - 1]; //获取步数
    grip_num_single_member[dist.size() - 1] = getGripNumber(dist[dist.size() - 1],grippoint_list[grippoint_list.size() - 1][0],grippoint_list[grippoint_list.size() - 1][1],init_data[var_num - 2],init_data[var_num - 1],truss[truss_list[truss_num - 1] - 1]);   //获取步数
    grip_num_all += grip_num_single_member[dist.size() - 1];

    // std::cout << grip_num_all + truss_num - 1 << "   ";
    // std::cout << grip_num_single_member[1] << " ";
    return grip_num_all + truss_num - 1;
}

//可过渡性约束
double constraint(const std::vector<double> &init_data,
                std::vector<double> &grad, void* consdata_list_ptr)
{
    my_function_data *constraint_data_ptr = (my_function_data*)consdata_list_ptr;
    std::vector<int> truss_list = constraint_data_ptr->truss_list;

    int flag = 0;
    std::vector<double> tmp_truss1, tmp_truss2; //用于表示过渡时两杆件
    int truss_num = truss_list.size();

    double joint_val[6];    //函数参数,此处无用
    int countnum = 0;
    for (int i = 0; i < truss_num - 1;++i)
    {
        countnum = countnum + grip_num_single_member[i] + 1;
        tmp_truss1 = truss[truss_list[i] - 1];
        tmp_truss2 = truss[truss_list[i+1] - 1];
        // auto test = new QTree;
        // test->construct(tmp_truss1, tmp_truss2, init_data[4 * i], init_data[4 * i + 1]);
        // Point point(init_data[4 * i + 2], init_data[4 * i + 3]);
        // test->findNode(point);
        // if(test->findNode(point)->state == 1)
        // {
        //     flag++;
        // }
        if(countnum % 2 == 0)
        {
            if(transwithIK(tmp_truss1, tmp_truss2, init_data[4 * i], init_data[4 * i + 1], init_data[4 * i + 2], init_data[4 * i + 3], 0,joint_val) == 1)
            {
                flag++;
            }
        }
        else if(countnum % 2 == 1)
        {
            if(transwithIK(tmp_truss2, tmp_truss1, init_data[4 * i + 2], init_data[4 * i + 3], init_data[4 * i], init_data[4 * i + 1], 0,joint_val) == 1)
            {
                flag++;
            }
        }
    }
    // std::cout << flag << "   ";
    return flag - init_data.size() / 4;
}

//过渡无碰约束
double collisioncon(const std::vector<double> &init_data,
                std::vector<double> &grad, void* consdata_list_ptr)
{
    my_function_data *constraint_data_ptr = (my_function_data*)consdata_list_ptr;
    std::vector<int> truss_list = constraint_data_ptr->truss_list;
    std::vector<std::vector<int>> adj_mat = constraint_data_ptr->adj_mat;

    int flag = 0;
    std::vector<double> tmp_truss1, tmp_truss2; //用于表示过渡时两杆件
    int truss_num = truss_list.size();
    int countnum = 0;
    double point[6];    //临时参数，用于minDistance，无用

    std::vector<int> poten_truss;   //潜在的碰撞杆件

    for (int i = 0; i < truss_num - 1;++i)
    {

        countnum = countnum + grip_num_single_member[i] + 1;
        tmp_truss1 = truss[truss_list[i] - 1];
        tmp_truss2 = truss[truss_list[i+1] - 1];

        if(countnum % 2 == 0)
        {
            getCurJointPoint(tmp_truss1, tmp_truss2,init_data[4 * i], init_data[4 * i + 1], init_data[4 * i + 2], init_data[4 * i + 3],robot_link);
            // // 机器人与夹持杆件不发生碰撞
            // if(minDistance(robot_link[2],tmp_truss1,point) < 90 || minDistance(robot_link[3],tmp_truss1,point) < 90 )
            // {
            //     flag--;
            // }
            // if(minDistance(robot_link[2],tmp_truss2,point) < 90 || minDistance(robot_link[3],tmp_truss2,point) < 90)
            // {
            //     flag--;
            // }

            int tmp_flag = getPontentialObstacle(robot_link, adj_mat, truss_list[i], truss_list[i + 1], poten_truss);
            if(tmp_flag == 1)
            {
                flag++;
            }
            else
            {
                if(potentialObsTest(robot_link,poten_truss) == 1)
                {
                    flag++;
                }
            }
            
        }
        else if(countnum % 2 == 1)
        {
            getCurJointPoint(tmp_truss2, tmp_truss1,init_data[4 * i + 2], init_data[4 * i + 3], init_data[4 * i], init_data[4 * i + 1],robot_link);
            int tmp_flag = getPontentialObstacle(robot_link, adj_mat, truss_list[i + 1], truss_list[i], poten_truss);

            // // 机器人与夹持杆件不发生碰撞
            // if(minDistance(robot_link[2],tmp_truss1,point) < 90 || minDistance(robot_link[3],tmp_truss1,point) < 90 )
            // {
            //     flag--;
            // }
            // if(minDistance(robot_link[2],tmp_truss2,point) < 90 || minDistance(robot_link[3],tmp_truss2,point) < 90)
            // {
            //     flag--;
            // }

            if(tmp_flag == 1)
            {
                flag++;
            }
            else
            {
                if(potentialObsTest(robot_link,poten_truss) == 1)
                {
                    flag++;
                }
            }
        }
    }

    // std::cout << flag << " ";
    return flag - init_data.size() / 4;
}


//单根杆件上步数为1时无碰
double singleCollision(const std::vector<double> &init_data,
                std::vector<double> &grad, void* consdata_list_ptr)
{
    my_function_data *constraint_data_ptr = (my_function_data*)consdata_list_ptr;
    std::vector<int> truss_list = constraint_data_ptr->truss_list;
    std::vector<std::vector<int>> adj_mat = constraint_data_ptr->adj_mat;
    std::vector<std::vector<int>> grippoint_list = constraint_data_ptr->grippoint_list;
    int truss_num = truss_list.size();

    std::vector<int> single_grip_num(truss_num,0);
    int single_step = 0;    //记录一根杆件上步数为1的情况
    for (size_t i = 0; i < truss_num;++i)
    {
        single_grip_num[i] = grip_num_single_member[i];
        if(grip_num_single_member[i] == 1)
        {
            single_step++;
        }
    }
    std::vector<double> tmp_truss1;
    int countnum = 0;
    std::vector<int> poten_truss;   //潜在的碰撞杆件

    if(single_step == 0)
    {
        return single_step;
    }
    else
    {
        if(single_grip_num[0] == 1) //起始杆件
        {
            tmp_truss1 = truss[truss_list[0] - 1];
            getCurJointPoint(tmp_truss1, tmp_truss1,grippoint_list[0][0], grippoint_list[0][1], init_data[0], init_data[1],robot_link);
            int tmp_flag = getPontentialObstacle(robot_link, adj_mat, truss_list[0], truss_list[0], poten_truss);
            if(tmp_flag == 1)
            {
                single_step--;
            }
            else
            {
                if(potentialObsTest(robot_link,poten_truss) == 1)
                {
                    single_step++;
                }
            }
        }
        for (int i = 1; i < truss_num - 1;++i)
        {
            if(single_grip_num[i] == 1)
            {
                tmp_truss1 = truss[truss_list[i] - 1];
                getCurJointPoint(tmp_truss1, tmp_truss1,init_data[2 * i], init_data[2 * i + 1], init_data[2 * i + 2], init_data[2 * i + 3],robot_link);
                int tmp_flag = getPontentialObstacle(robot_link, adj_mat, truss_list[i], truss_list[i], poten_truss);
                if(tmp_flag == 1)
                {
                    single_step--;
                }
                else
                {
                    if(potentialObsTest(robot_link,poten_truss) == 1)
                    {
                        single_step++;
                    }
                }
            }
        }
        if(single_grip_num[truss_num - 1] == 1) //最后一根杆件
        {
            tmp_truss1 = truss[truss_list[truss_num - 1] - 1];
            getCurJointPoint(tmp_truss1, tmp_truss1,grippoint_list[grippoint_list.size() - 1][0], grippoint_list[grippoint_list.size() - 1][1], init_data[0], init_data[1],robot_link);
            int tmp_flag = getPontentialObstacle(robot_link, adj_mat, truss_list[truss_num - 1], truss_list[truss_num - 1], poten_truss);
            if(tmp_flag == 1)
            {
                single_step--;
            }
            else
            {
                if(potentialObsTest(robot_link,poten_truss) == 1)
                {
                    single_step++;
                }
            }
        }

        return single_step;
    }

    return -1;
}


//将变量限制为整型
double int_constraint(const std::vector<double> &init_data,
                std::vector<double> &grad, void* data)
{
    double flag = 0;

    for (size_t i = 0; i < init_data.size();++i)
    {
        double tmp = pow(init_data[i], -6.0);
        if((init_data[i] - int(init_data[i]) < tmp) && (init_data[i] - int(init_data[i]) > -tmp))
        {
            flag++;
        }
    }

    return flag - init_data.size();
}


int GripOpti(std::vector<int> &truss_list,
            std::vector<std::vector<int>> &grippoint_list,
            std::vector<std::vector<int>> &adj_mat,
            std::vector<int> &single_num,
            std::vector<std::vector<int> > &opti_result,
            const int DOF_flag)
{
    double fmin = 0;
    double tol = 1e-1;

    int truss_num = truss_list.size();  //路径中包含的杆件数目

    std::vector<double> upper(4 * truss_num - 4,0);
    std::vector<double> lower(4 * truss_num - 4,0);   //变量上下边界
    for (size_t i = 0;i< 4 * truss_num - 4;++i)
    {
        upper[i] = pole_rows - 1;
        lower[i] = 0;
    }

    //设置变量初始值
    std::vector<double> init_data(4 * truss_num - 4,0); 
    for(size_t i = 0;i < 4 * truss_num - 4;++i)
    {
        init_data[i] = grippoint_list[(i+2) / 2][i % 2];
    }
    // std::cout << "初始值=" << std::endl;
    // for(size_t i = 0;i < 4 * truss_num - 4;++i)
    // {
    //     std::cout<<init_data[i]<<" ";
    // }
    //设置传递参数：truss_list和grippoint_list\adj_mat
    my_function_data consdata_list;
    consdata_list.grippoint_list = grippoint_list;
    consdata_list.truss_list = truss_list;
    consdata_list.adj_mat = adj_mat;
    my_function_data* consdata_list_ptr;
    consdata_list_ptr = &consdata_list;

    //设置使用算法
    nlopt::opt local_opter(nlopt::LN_COBYLA, 4 * truss_num - 4);
    //设置自变量界限
    local_opter.set_lower_bounds(lower);
    local_opter.set_upper_bounds(upper);
    // 目标函数
    local_opter.set_min_objective(objective_fun, consdata_list_ptr);

    // // // 不等式约束
    //local_opter.add_inequality_constraint(constraint, truss_grippoint_list_ptr, 0);

    // 等式约束
    local_opter.add_equality_constraint(constraint, consdata_list_ptr, 0);
    local_opter.add_equality_constraint(collisioncon, consdata_list_ptr, 0);
    local_opter.add_equality_constraint(singleCollision, consdata_list_ptr, 0);
    // local_opter.add_equality_constraint(int_constraint, NULL, 0);    //参数太多不生效

    //设置步长
    std::vector<double> int_step(4 * truss_num - 4,0);
    for (int i = 0; i < 4 * truss_num - 4;++i)
    {
        if(i%2 == 0)
        {
            int_step[i] = 10;
        }
        else{
            int_step[i] = 5;
        }

    }
    local_opter.set_initial_step(int_step);

    // 停止时需要的条件；
    local_opter.set_xtol_rel(tol);

    try{
        //开始优化
        auto local_result = local_opter.optimize(init_data, fmin);
        //优化结果输出
        if(local_result)
        {
            std::vector<int> tmp;
            tmp.push_back(grippoint_list[0][0]);
            tmp.push_back(grippoint_list[0][1]);
            opti_result.push_back(tmp);       
            tmp.pop_back();
            tmp.pop_back();
            for (int i = 0; i < 4 * truss_num - 4;++i)
            {
                if(i % 2 == 1)
                {
                    tmp.push_back(init_data[i]);
                    opti_result.push_back(tmp);
                    // std::cout << std::endl;
                    tmp.pop_back();
                    tmp.pop_back();
                }
                else
                {
                    tmp.push_back(init_data[i]);
                }
            }
            tmp.push_back(grippoint_list[grippoint_list.size() - 1][0]);
            tmp.push_back(grippoint_list[grippoint_list.size() - 1][1]);
            opti_result.push_back(tmp);

            std::cout <<"夹持点数目为"<< fmin << std::endl;

            // std::cout << grip_num_single_member[0] << " " << grip_num_single_member[1] << " " << grip_num_single_member[2] << std::endl;

            for (int i = 0; i < truss_num;++i)
            {
                single_num.push_back(grip_num_single_member[i]);
            }
        }
        // std::cout << local_result << std::endl;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    return 0;
}




// int GripOpti(std::vector<int> &truss_list,
//             std::vector<std::vector<int>> &grippoint_list)
// {
//     double fmin = 0;
//     double tol = 1e-1;

//     int truss_num = truss_list.size();  //路径中包含的杆件数目

//     std::vector<double> upper(4 * truss_num - 4,0);
//     std::vector<double> lower(4 * truss_num - 4,0);   //变量上下边界
//     for (size_t i = 0;i< 4 * truss_num - 4;++i)
//     {
//         upper[i] = pole_rows - 1;
//         lower[i] = 0;
//     }
//     for (int i = 0;i <  4 * truss_num - 4;++i)
//     {
//         if(i == 0)
//         {
//             lower[0] = grippoint_list[1][0] - 30;
//             upper[0] = grippoint_list[1][0] + 30;
//         }
//         else if(i == 1)
//         {
//             lower[i] = grippoint_list[1][1] - 30;
//             upper[i] = grippoint_list[1][1] + 30;   //限制过渡时转角基座范围
//         }
//         else if((i+2) % 2 == 0)
//         {
//             lower[i] = grippoint_list[i/2 + 1][0] - 30;
//             upper[i] = grippoint_list[i/2 + 1][0] + 30;     //过渡时目标杆件距离范围
//         }
//         else if(i>1 &&  i % 2 == 1)
//         {
//             lower[i] = grippoint_list[(i + 1) / 2][1] - 30;
//             upper[i] = grippoint_list[(i + 1) / 2][1] + 30; //限制转角范围
//         }
//     }
//     for (int i = 0;i <  4 * truss_num - 4;++i)
//     {
//         if(lower[i] < 0)
//         {
//             lower[i] = 0;
//         }
//         if(upper[i] > 127)
//         {
//             upper[i] = 127;
//         }
//     }

//     std::vector<double> init_data(4 * truss_num - 4,0); //设置变量初始值
//     for(size_t i = 0;i < 4 * truss_num - 4;++i)
//     {
//         init_data[i] = grippoint_list[(i+2) / 2][i % 2];
//     }
//     // std::cout << "初始值=" << std::endl;
//     // for(size_t i = 0;i < 4 * truss_num - 4;++i)
//     // {
//     //     std::cout<<init_data[i]<<" ";
//     // }
//     //设置传递参数：truss_list和grippoint_list
//     my_function_data truss_grippoint_list;
//     truss_grippoint_list.grippoint_list = grippoint_list;
//     truss_grippoint_list.truss_list = truss_list;
//     my_function_data* truss_grippoint_list_ptr;
//     truss_grippoint_list_ptr = &truss_grippoint_list;

//     //设置选用算法：LN_COBYLA(第二个参数为变量个数)
//     nlopt::opt global_opter(nlopt::GN_ISRES, 4 * truss_num - 4);

//     //设置自变量下限
//     global_opter.set_lower_bounds(lower);
//     global_opter.set_upper_bounds(upper);

//     // 目标函数
//     global_opter.set_min_objective(objective_fun, truss_grippoint_list_ptr);

//     // truss_grippoint_list.grip_num_single = 


//     // // // 不等式约束
//     // opter.add_inequality_constraint(constraint, truss_grippoint_list_ptr, 0);

//     // 等式约束
//     global_opter.add_equality_constraint(constraint, truss_grippoint_list_ptr, 0);

//     // 停止时需要的条件；
//     // opt.set_maxtime(2);
//     global_opter.set_xtol_rel(3);
//     global_opter.set_maxtime(truss_num-1);
//     // local_opter.set_xtol_rel(tol);

//     // 开始优化；
//     std::vector<double> local_init_data(4 * truss_num - 4, 0);
//     try
//     {
//         auto global_result = global_opter.optimize(init_data, fmin);
//         if(global_result)
//         {
//             for (size_t i = 0;i < 4 * truss_num - 4;++i)
//             {
//                 local_init_data[i] = init_data[i];
//             }
//         }
//         std::cout << global_result << std::endl;
//     }
//     catch(std::exception &e) 
//     {
//         std::cout << "nlopt failed: " << e.what() << std::endl;
//     }

//     nlopt::opt local_opter(nlopt::LN_COBYLA, 4 * truss_num - 4);
//     std::vector<double> local_upper(4 * truss_num - 4,0);
//     std::vector<double> local_lower(4 * truss_num - 4,0);   //变量上下边界
//     for (size_t i = 0;i< 4 * truss_num - 4;++i)
//     {
//         local_upper[i] = pole_rows - 1;
//         local_lower[i] = 0;
//     }
//     //设置自变量下限
//     local_opter.set_lower_bounds(local_lower);
//     local_opter.set_upper_bounds(local_upper);
//     // 目标函数
//     local_opter.set_min_objective(objective_fun, truss_grippoint_list_ptr);

//     // // // 不等式约束
//     // local_opter.add_inequality_constraint(constraint, truss_grippoint_list_ptr, 0);

//     // std::vector<double> int_step(4 * truss_num - 4,0);
//     // for (int i = 0; i < 4 * truss_num - 4;++i)
//     // {
//     //     int_step[i] = 10;
//     // }
//     // //设置步长
//     // local_opter.set_initial_step(int_step);

//     // 等式约束
//     local_opter.add_equality_constraint(constraint, truss_grippoint_list_ptr, 0);

//     // 停止时需要的条件；
//     local_opter.set_xtol_rel(tol);

//     try{
//         auto local_result = local_opter.optimize(local_init_data, fmin);
//         if(local_result)
//         {
//             std::cout << "optiresult = " << std::endl;
//             std::cout << grippoint_list[0][0] << " " << grippoint_list[0][1] << std::endl;
//             for (int i = 0; i < 4 * truss_num - 4;++i)
//             {
//                 std::cout << int(local_init_data[i]) << " ";
//                 if(i % 2 == 1)
//                 {
//                     std::cout << std::endl;
//                 }
//             }
//             std::cout << grippoint_list[grippoint_list.size() - 1][0] << " " << grippoint_list[grippoint_list.size() - 1][1] << std::endl;
//             std::cout <<"夹持点数目为"<< fmin << std::endl;
//             std::cout << "单点" << grip_num_single_member[0] << " " << grip_num_single_member[1] << " "<<grip_num_single_member[2]<<" "<<grip_num_single_member[3]<<std::endl;
//         }
//         // std::cout << local_result << std::endl;
//     }
//     catch(std::exception &e) {
//         std::cout << "nlopt failed: " << e.what() << std::endl;
//     }

//     return 0;
// }


#endif  //grippointopti.h