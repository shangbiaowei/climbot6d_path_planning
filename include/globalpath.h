#ifndef globalpath_H__
#define globalpath_H__

#include <iostream>
#include <vector>
#include <cmath>
#include <stack>
#include <algorithm>
#include "m_Matrix.h"
#include "QTree.h"


class globalPathPlan
{
    public:
        int visited[100];//访问列表

    public:
        globalPathPlan();
        ~globalPathPlan();

    /*****************************************************************************
     *        全局路径规划
     *        GDUT, 2020
     *        输入:邻接矩阵，起始杆件，目标杆件
     *        输出:杆件序列
     *****************************************************************************/
        void outPut(std::vector<std::vector<int>> &all_pathway);
        void DFS(std::vector<std::vector<int>> &adj_mat, int start, int end, std::vector<int> a_pathway, std::vector<std::vector<int>> &all_pathway); //深搜遍历
        std::vector<std::vector<int>> findAllPath(std::vector<std::vector<int>> &adj_mat,int start_pole,int end_pole);
    /*****************************************************************************
     *        构建邻接矩阵
     *        GDUT, 2020
     *        输入:杆件信息 pole_mat
     *        输出:邻接矩阵 adj_mat
     *****************************************************************************/
        std::vector<std::vector<int> > adjMat(std::vector<std::vector<double> > &pole_mat);

    /*****************************************************************************
     *        可过渡性判断
     *        GDUT, 2020
     *        输入:杆件端点坐标数组
     *        输出:过渡标志，0-不可过渡，1-可过渡 
     *            夹持点：first_grippoint:同一杆件上第一个夹持点
     *                   sec_grippoint:同一杆件上最后一个夹持点（如果存在）
     *****************************************************************************/
        int transMat(std::vector<double> &pole1, std::vector<double> &pole2,std::vector<int> &first_grippoint,std::vector<int> &sec_grippoint);
        int fastTransMat(std::vector<double> &pole1, std::vector<double> &pole2);
};


/*****************************************************************************
 *        空间中两杆件的最短距离及最近点
 *        GDUT, 2020
 *        输入:杆件端点坐标数组 pole1,pole2
 *        输出:最近点坐标 point[0-2](第一个点) point[3-5](第二个点)
 *             最短距离 min_dis
 *****************************************************************************/
double minDistance(std::vector<double> &pole1, std::vector<double> &pole2, double *point);
//空间中点到直线的距离
double disPoint2Line(std::vector<double> &point, std::vector<double> &line);
//空间中点到点的距离
double disPoint2Point(std::vector<double> &point1, std::vector<double> &point2);

#endif //globalpath.h