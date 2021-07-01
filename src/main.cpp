#include <iostream>
#include <vector>
#include <time.h>
#include <cmath>
#include "../include/globalpath.h"
#include "../include/grippointopti.h"
#include "../include/pathallocate.h"
// #include "../include/gaitplan.h"
#include "../include/generateSimData.h"
// #include "../include/motionplan.h"


using namespace std;

int main()
{
    clock_t global_start,global_finish;
    double globaltime;
    global_start=clock();    //计算程序运行时间

/******************************** 起点终点设置 ************************************/
    std::vector<int> start_point = {30, 32};
    std::vector<int> end_point = {50,32};

    int DOF_flag = 5;
/******************************** 起点终点设置 ************************************/

/******************************** 全局路径规划 ************************************/
    std::vector<std::vector<int>> adj_mat;
    globalPathPlanStart(start_point, end_point, 0, 10, adj_mat, DOF_flag);
/******************************** 全局路径规划 ************************************/

    // adj_mat = {
    //     {0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    //     {1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1},
    //     {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
    //     {1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1},
    //     {1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1},
    //     {1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1},
    //     {1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1},
    //     {0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0},
    //     {0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0},
    //     {0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0},
    //     {0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0},
    //     {0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1},
    //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
    //     {0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0},
    //     {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0}};

/******************************** 夹持点规划 ************************************/
    global_finish = clock();
    globaltime=(double)(global_finish-global_start)/CLOCKS_PER_SEC;
    std::cout<<"\n全局规划的运行时间为"<<globaltime<<"秒"<<std::endl;  //输出全局路径运行时间

    clock_t opti_start,opti_finish;
    double optitime;
    opti_start=clock();    //计算程序运行时间

    std::vector<int> truss_list = {1, 5, 8, 11};
    std::vector<std::vector<int> > grippoint_list;
    getTransTrussList(start_point,end_point,truss_list, grippoint_list,DOF_flag);
    std::cout << "before optimize" << std::endl;
    for (int i = 0; i < grippoint_list.size();++i)
    {
        for (int j = 0; j < grippoint_list[0].size();++j)
        {
            cout << grippoint_list[i][j] << " ";
        }
        cout << endl;
    }

    std::vector<int> single_num;
    std::vector<std::vector<int>> opti_res;
    GripOpti(truss_list,grippoint_list,adj_mat,single_num,opti_res,DOF_flag);

    opti_finish = clock();
    optitime=(double)(opti_finish-opti_start)/CLOCKS_PER_SEC;
    std::cout<<"\n夹持点优化模型的运行时间为"<<optitime<<"秒"<<std::endl;  //输出夹持点优化模型运行时间

    for (int i = 0; i < opti_res.size();++i)
    {
        for (int j = 0; j < opti_res[0].size();++j)
        {
            std::cout << opti_res[i][j] << " ";
        }
        std::cout << std::endl;
    }
    for (int i = 0; i < single_num.size();++i)
    {
        std::cout << single_num[i] << " ";
    }
    std::cout << std::endl;

//     clock_t grip_start,grip_finish;
//     double griptime;
//     grip_start=clock();    //计算程序运行时间

//     PathAllocate test1;
//     test1.PathAllocateInit(adj_mat, opti_res, single_num);
//     test1.allocatePath(truss,truss_list);

//     grip_finish = clock();
//     griptime=(double)(grip_finish-grip_start)/CLOCKS_PER_SEC;
//     std::cout<<"\n夹持点规划的运行时间为"<<griptime<<"秒"<<std::endl;  //输出夹持点规划运行时间
// /******************************** 夹持点规划 ************************************/

//     SimDataGenerate generater;
//     std::vector<std::vector<double>> joint_val_list;
//     generater.generatePathTxt(test1.final_path_list,truss,truss_list,single_num,joint_val_list);


// /******************************** 单步规划 ************************************/
//     // singleStepPlanner(joint_val_list,adj_mat,test1.final_path_list,truss_list,single_num);
// /******************************** 单步规划 ************************************/





    // double pos_start[6] = {0, 90, 90, 0, 0, 0};
    // double pos_end[6] = {0, 0, 0, 0, 0, 0};
    // motionPlanJointState joint_motion;
    // joint_motion.setStartPos(pos_start, 7);
    // joint_motion.setGoalPos(pos_end, 7);
    // std::vector<std::vector<double>> path_list;
    // joint_motion.plan(path_list, adj_mat, 7, 1, 1, truss1, truss1, 90, 32);

    // // double pos_start[6] = {0, 90, 90, 0, 0, 0};
    // // double pos_end[6] = {-3.0182,123.526,203.096,5.07633,33.63,-7.25194,}; //50 33  0.1s
    // // double pos_start[6] = {-3.0182,123.526,203.096,5.07633,33.63,-7.25194,};
    // // double pos_end[6] = {-3.01763,56.4739,-23.0959,5.07539,-33.6299,-7.25058,};   //70 34
    // double pos_start[6] = {-3.0,56.5,-23.1,5.1,-33.6,-7.3,};
    // double pos_end[6] = {-5.72739,125.795,198.972,9.51055,36.1735,-13.4567,};   //91 36
    // // double pos_start[6] = {-5.72739,125.795,198.972,9.51055,36.1735,-13.4567,};
    // // double pos_end[6] = {-100.362,7.26617,130.887,22.3788,67.3339,-16.9494,};   //30 26
    // // // double pos_start[6] = {806.5,184.7,-753.9,-72.2,16.1,27.0,};
    // // // double pos_end[6] = {-399.1,-0.0,-120.0,0.0,0.0,-163.1,};   //0  32
    // // // double pos_start[6] = {399.1,0.0,120.0,0.0,0.0,163.1,};
    // // // double pos_end[6] = {-834.6,-48.8,295.0,73.8,4.5,-130.0,};   //45  14
    // // // double pos_start[6] = {302.8,615.0,-628.9,65.0,48.6,165.5,};
    // // // double pos_end[6] = {391.9,-50.0,-100.0,0.0,0.0,140.6,};   //65 0
    // // // double pos_start[6] = {-391.9,-100.0,100.0,0.0,0.0,-140.6,};
    // // // double pos_end[6] = {-333.1,150.0,150.0,0.0,0.0,135.0,};   //48 112
    // // // double pos_start[6] = {333.1,150.0,-150.0,0.0,0.0,-135.0,};
    // // // double pos_end[6] = {17.0,131.7,-1179.6,-85.8,-16.6,-11.8,};   //57 29
    // // // double pos_start[6] = {460.9,-250,1064.9,85.9,-10.6,17.4,};
    // // // double pos_end[6] = {311.4,-100,100.0,0.0,0.0,-115.3,};   //84 52
    // // // double pos_start[6] = {-311.4,-150,0.0,0.0,0.0,115.3,};
    // // // double pos_end[6] = {300.0,150,-150.3,0.0,0.0,-146.3,};   //84 52
    // cout << "OMPL_VERSION:" << OMPL_VERSION << endl;
    // motionPlan planner;
    // double curpos[6] = {-93.8,-8.5,168.6,30.3,51.5,-34.7,};
    // planner.getAidPathPoint(curpos, 0);
    // planner.setStartPos(pos_start,7);
    // planner.setGoalPos(pos_end,7);
    // std::vector<std::vector<double>> path_list;
    // planner.plan(path_list, adj_mat, 7, 1, 1, truss1, truss1, 70, 34);
    // planner.getFinalJointValue(path_list, 7);

    return 0;
}