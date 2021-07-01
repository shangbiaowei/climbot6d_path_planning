#pragma once

#include <iostream>
#include <vector>
#include <cstdlib>
#include <list>
#include <iterator>
#include "globalpath.h"
#include "environment.h"
#include "QTree.h"
#include "miniball.h"

using namespace std;

/*
 * 功能：给定起点终点.输出全局规划路径
 * 参数：
 * - start_point：起点(离散杆件上)
 * - end_point：终点(离散杆件上)
 * - start_pole/end_pole：当前杆和目标杆
 * - res：邻接矩阵
 * 返回值：0表示正常，-1表示异常
 */
int globalPathPlanStart(std::vector<int> &start_point,
                        std::vector<int> &end_point,
                        int start_pole, int end_pole,
                        std::vector<std::vector<int>> &res,
                        const int DOF_flag);

/*
 * 功能：通过4个三维坐标点求包含这四个点的球心以及半径
 * 参数：
 * - p1：其中一个三维坐标点
 * - p2：其中一个三维坐标点
 * - p3：其中一个三维坐标点
 * - p4：其中一个三维坐标点
 * - center：球心对应的三维坐标点
 * - radius: 半径
 * 返回值：0表示正常，-1表示异常
 */
int findminiball(const vector<double> &p1, const vector<double> &p2,
                 const vector<double> &p3, const vector<double> &p4,
                 vector<double> &center, double &radius);


/*
 * 功能:选择一条全局路径并生成带可过渡区域的路径
 * GDUT, 2020
 * 输入:全局路径序列：vector<int> truss_list;
 *     起点start_point:(x,y)    x,y为展开地图上坐标
 *     终点end_point：（x,y)
 * 输出:全局路径序列 truss_list
 *     夹持点信息 grippoint_list
*/
void getTransTrussList(std::vector<int> start_point, std::vector<int> end_point, 
                        std::vector<int> &truss_list, 
                        std::vector<std::vector<int>> &grippoint_list,
                        const int DOF_flag);

/*
 * 功能：通过当前机器人位姿生成包络球体进行快速碰撞检测
 * 参数：
 * - link:机器人连杆点
 * - adj_mat：邻接地图
 * - cur_truss：机器人基座杆件
 * - tar_truss：机器人另一手爪杆件
 * - potential_truss：潜在的与机器人碰撞的杆件
 * 返回值：1表示正常，0表示存在碰撞杆件
 */
int getPontentialObstacle(std::vector<std::vector<double>> &link,
                          std::vector<std::vector<int>> adj_mat,
                          const int cur_truss, const int tar_truss,
                          std::vector<int> &pontential_truss);

/*
 * 功能：获取当前位置姿态下机器人连杆的点坐标
 * 参数：
 * - cur_truss：机器人基座杆件
 * - tar_truss：机器人另一手爪杆件
 * - length1\angle1：机器人基座的离散地图位置
 * - length2\angle2：机器人目标夹持点的离散地图位置
 * 返回值：连杆坐标数组
 */
void getCurJointPoint(std::vector<double> &cur_truss, std::vector<double> &tar_truss,
                    const int length1, const int angle1,
                    const int length2, const int angle2,
                    std::vector<std::vector<double>> &link);


/*
 * 功能：通过机器人当前连杆坐标点与潜在碰撞杆件进行碰撞检测
 * 参数：
 * - link：机器人当前连杆坐标数组
 * - potential_truss：潜在的碰撞杆件
 * 返回值：1表示正常，0表示碰撞
 */
int potentialObsTest(std::vector<std::vector<double>> &link,
                     std::vector<int> &pontential_truss); //注意:此处为杆件编码减1

/*
 * 功能：通过4个三维坐标点求过这四个点的球心以及半径
 * 参数：
 * - p1：其中一个三维坐标点
 * - p2：其中一个三维坐标点
 * - p3：其中一个三维坐标点
 * - p4：其中一个三维坐标点
 * - centre：球心对应的三维坐标点
 * - radius: 半径
 * 返回值：0表示正常，-1表示异常
 */
int calculateCentreAndRadius(const vector<double> &p1, const vector<double> &p2,
                             const vector<double> &p3, const vector<double> &p4,
                             vector<double> &centre, double &radius);
