#include "../include/globalpath.h"

using namespace std;

globalPathPlan::globalPathPlan()
{

}

globalPathPlan::~globalPathPlan()
{

}

//打印所有全局路径
void globalPathPlan::outPut(vector<vector<int>> &all_pathway)
{
	for(int i=0;i<all_pathway.size();i++)
	{
        // cout << "第" << i+1 << "条路径:	";
        for (int j = 0; j < all_pathway[i].size();++j)
        {
            if(all_pathway[i].size() < 5)
            {
                cout << all_pathway[i][j]+1<< " ";
            }
        }
        
    }
	cout<<endl;
}

//深度优先搜索所有路径
void globalPathPlan::DFS(vector<vector<int>> &adj_mat,int start,int end,vector<int> a_pathway,vector<vector<int>> &all_pathway) 
{
    a_pathway.push_back(start);//将当前节点入栈
	visited[start]=true;//标记为已经访问过
	int middle=0;//一个索引，用来找到下一个节点 
	
	while(a_pathway.empty()==false)//如果栈空则结束
	{
		if(start==end)//如果当前节点就是目标节点
		{
			all_pathway.push_back(a_pathway);//保存一条路径
			a_pathway.pop_back();//退栈，回溯
			visited[start]=false;//设置为没有被访问过
            break; //退出一个递归 
		} 
		
		while(middle<adj_mat.size())//如果没有到最后一个
		{
			if(adj_mat[start][middle]==1&&visited[middle]==false)
			{
				DFS(adj_mat,middle,end,a_pathway,all_pathway); //递归查询 
			} 
			middle++;
		}
		if(middle==adj_mat.size()) //找到最后一个
		{
			visited[a_pathway.at(a_pathway.size()-1)] = false;//先标记栈中最后一个元素为没有访问过
			a_pathway.pop_back();//退栈，回溯 
			
			break;  //退出一个递归 
		}
	} 
}

vector<vector<int>> globalPathPlan::findAllPath(vector<vector<int>>& adj_mat,int start_pole,int end_pole)
{
    int n = adj_mat.size();
    vector<vector<int>> res;
    DFS(adj_mat,start_pole,end_pole,{},res);//起点和终点

    return res;
}

//构建邻接矩阵
vector<vector<int>> globalPathPlan::adjMat(vector<vector<double> > &pole_mat,const int DOF_flag)
{
    int pole_num = pole_mat.size();
    vector<vector<int> > res(pole_num,vector<int>(pole_num,0));

    vector<int> first_grippoint,sec_grippoint;

    for (int i = 0; i < pole_num;i++)
    {
        for (int j = 0;j < pole_num;j++)
        {
            vector<double> temp1 = {pole_mat[i][0], pole_mat[i][1], pole_mat[i][2], pole_mat[i][3], pole_mat[i][4], pole_mat[i][5]};
            vector<double> temp2 = {pole_mat[j][0], pole_mat[j][1], pole_mat[j][2], pole_mat[j][3], pole_mat[j][4], pole_mat[j][5]};

            // if (transMat(temp1, temp2,first_grippoint,sec_grippoint) == 1)
            // {
            //     if(i == j)
            //     {
            //         res[i][j] = 0;
            //     }
            //     else
            //         res[i][j] = 1;
            // }
            // else
            //     res[i][j] = 0;

            if(fastTransMat(temp1,temp2,DOF_flag) == 1)
            {
                if(i == j)
                {
                    res[i][j] = 0;
                }
                else
                    res[i][j] = 1;
            }
            else
                res[i][j] = 0;
        }
    }
    return res;
}

//可过渡判断
int globalPathPlan::transMat(vector<double> &pole1,
                            vector<double> &pole2,
                            std::vector<int> &first_grippoint,
                            std::vector<int> &sec_grippoint,
                            std::vector<int> &base_point,
                            const int DOF_flag)
{
    double point1[6];
    double min_disp1p2 = minDistance(pole1, pole2, point1);
    vector<double> tmp = {point1[0], point1[1], point1[2]};

    int scale = 1;
    double alpha = 0;

    Vector pole1_x; //杆件1上坐标系x轴
    Vector pole1_y,pole1_z; //杆件1上坐标系y、z轴
    pole1_x.X = pole1[0] - pole1[3];
    pole1_x.Y = pole1[1] - pole1[4];
    pole1_x.Z = pole1[2] - pole1[5];
    pole1_x = Norm_Vec(pole1_x);

    Vector pole2_x; //杆件2上坐标系x轴
    pole2_x.X = pole2[0] - pole2[3];
    pole2_x.Y = pole2[1] - pole2[4];
    pole2_x.Z = pole2[2] - pole2[5];
    pole2_x = Norm_Vec(pole2_x);

    //定义世界坐标系
    Vector World_Z;
    World_Z.X = 0;World_Z.Y = 0;World_Z.Z = 1;

    if(pole1[0] == pole1[3] && pole1[1] == pole1[4])    //杆件平行于世界坐标系Z轴
    {
        pole1_y.X = 1;
        pole1_y.Y = 0;
        pole1_y.Z = 0;
        pole1_z.X = 0;
        pole1_z.Y = 1;
        pole1_z.Z = 0;
    }
    else
    {
        pole1_y = operator&(pole1_x, World_Z);
        pole1_y = Norm_Vec(pole1_y);//单位向量
        pole1_z = operator&(pole1_x,pole1_y);
        pole1_z = Norm_Vec(pole1_z);//单位向量
    }

    Vector tmp_p1base_p2 = {point1[0] - (pole2[0] + pole2[3]) / 2, point1[1] - (pole2[1] + pole2[4]) / 2, point1[2] - (pole2[2] + pole2[5]) / 2};
    Vector project_p1base_p2_y = vec_projection(tmp_p1base_p2, pole1_y);
    Vector project_p1base_p2_z = vec_projection(tmp_p1base_p2, pole1_z);
    Vector project_p1base_p2_yOz = operator+(project_p1base_p2_y, project_p1base_p2_z);
    Vector project_p2_x_p1_y = vec_projection(pole2_x, pole1_y);
    Vector project_p2_x_p1_z = vec_projection(pole2_x, pole1_z);
    alpha = v_angle(pole1_z, project_p1base_p2_yOz);

    if(alpha >= 1.56 && alpha < 3.13 &&
       fabs(project_p2_x_p1_y.X) < 0.1 && 
       fabs(project_p2_x_p1_y.Y) < 0.1 &&
       fabs(project_p2_x_p1_y.Z) < 0.1 &&
       project_p1base_p2_yOz.X !=0 &&
       project_p1base_p2_yOz.Y !=0 &&
       project_p1base_p2_yOz.Z !=0)
    {
        alpha = 1.5708;
    }
    else if(alpha < 1.56 &&
            fabs(project_p2_x_p1_y.X) < 0.1 && 
            fabs(project_p2_x_p1_y.Y) < 0.1 &&
            fabs(project_p2_x_p1_y.Z) < 0.1 &&
            project_p1base_p2_yOz.X !=0 &&
            project_p1base_p2_yOz.Y !=0 &&
            project_p1base_p2_yOz.Z !=0)
    {
        alpha = 0;
    }
    else if(alpha >= 3.13 &&
            fabs(project_p2_x_p1_y.X) < 0.1 && 
            fabs(project_p2_x_p1_y.Y) < 0.1 &&
            fabs(project_p2_x_p1_y.Z) < 0.1 &&
            project_p1base_p2_yOz.X !=0 &&
            project_p1base_p2_yOz.Y !=0 &&
            project_p1base_p2_yOz.Z !=0)
    {
        alpha = 3.1416;
    }

    if(alpha > 1.58 &&
       fabs(project_p2_x_p1_z.X) < 0.1 && 
       fabs(project_p2_x_p1_z.Y) < 0.1 &&
       fabs(project_p2_x_p1_z.Z) < 0.1 &&
       project_p1base_p2_yOz.X !=0 &&
       project_p1base_p2_yOz.Y !=0 &&
       project_p1base_p2_yOz.Z !=0)
    {
        alpha = 3.1415;
    }
    else if(alpha < 1.56 &&
            fabs(project_p2_x_p1_z.X) < 0.1 && 
            fabs(project_p2_x_p1_z.Y) < 0.1 &&
            fabs(project_p2_x_p1_z.Z) < 0.1 &&
            project_p1base_p2_yOz.X !=0 &&
            project_p1base_p2_yOz.Y !=0 &&
            project_p1base_p2_yOz.Z !=0)
    {
        alpha = 0;
    }
    else if(alpha > 1.56 && alpha < 1.58 &&
       fabs(project_p2_x_p1_z.X) < 0.1 && 
       fabs(project_p2_x_p1_z.Y) < 0.1 &&
       fabs(project_p2_x_p1_z.Z) < 0.1 &&
       project_p1base_p2_yOz.X !=0 &&
       project_p1base_p2_yOz.Y !=0 &&
       project_p1base_p2_yOz.Z !=0)
    {
        alpha = 1.5708;
    }

    //缺少方向，绕x轴，顺时针为正,pole1_z转向投影
    Vector tmp_pole1_z_project_pole2_x_pole1_yOz = operator&(pole1_z, project_p1base_p2_yOz);
    tmp_pole1_z_project_pole2_x_pole1_yOz = Norm_Vec(tmp_pole1_z_project_pole2_x_pole1_yOz);
    if(tmp_pole1_z_project_pole2_x_pole1_yOz.X == pole1_x.X &&
       tmp_pole1_z_project_pole2_x_pole1_yOz.Y == pole1_x.Y &&
       tmp_pole1_z_project_pole2_x_pole1_yOz.Z == pole1_x.Z)
    {
        alpha = -alpha;
   //     -3.1416; //20210219
    }
    else
    {
        alpha = alpha;
    }

    if((fabs(point1[0] - point1[3]) < 20) && 
        (fabs(point1[1] - point1[4]) < 20) && 
        (fabs(point1[2] - point1[5]) < 20))    //当两线段垂直时，最近点选取存在机器人极限情况导致无可过渡区域，但实际上存在
    {
        for (int i = 0; i < 3;++i)
        {
            if(pole1[i] != point1[i])
            {
                scale = fabs((point1[i] - pole1[i]) / ((pole1[i] - pole1[i+3]) / pole_rows));
                if(scale > pole_rows - 5)
                {
                    point1[i] = pole1[i] + (pole1[i + 3] - pole1[i]) / pole_rows * (scale - 5);
                }
                else
                {
                    point1[i] = pole1[i] + (pole1[i + 3] - pole1[i]) / pole_rows * (scale + 5);
                }
                break;
            }
        }
    }


    for (int i = 0; i < 3; ++i)
    {
        if(pole1[i] != point1[i])
        {
            scale = fabs((point1[i] - pole1[i]) / ((pole1[i] - pole1[i+3]) / pole_rows));
            if(scale >= 124)
            {
                scale = 124;
            }
            else if(scale <= 4)
            {
                scale = 4;
            }
            break;
        }
    }   //求解出基座位置

    if(scale < 4)
    {
        scale = 4;
    }

    int alpha_scale = (alpha + 3.1416) / (6.2832 / pole_cols);
    if(alpha_scale > 127)
    {
        alpha_scale = 0;
    }


    if(DOF_flag == 6)
    {
        first_grippoint = {scale, alpha_scale};
    }
    else if(DOF_flag == 5)
    {
        alpha_scale = base_point[1];
        first_grippoint = {scale, alpha_scale};
    }
    else
    {
        std::cout << "DOF setting error!" << std::endl;
    }
    



    // std::cout << "test   ";
    // for (int i = 0; i < first_grippoint.size();++i)
    // {
    //     std::cout << first_grippoint[i] << " ";
    // }
    // std::cout << std::endl;

    auto tree_pole1 = new QTree;
    // std::cout <<"result = "<< scale << " " << alpha_scale << std::endl;
    tree_pole1->construct(pole1,pole2,scale,alpha_scale,DOF_flag);

    tree_pole1->findTreeNode(tree_pole1->t_root,sec_grippoint);

    if (tree_pole1->t_root->isLeaf == 0 && (pole1 != pole2)) 
    {
        // std::cout << "ok" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "无解" << std::endl;
        return 0;
    }

    return 0;
}

//快速可过渡判断
int globalPathPlan::fastTransMat(std::vector<double> &pole1, std::vector<double> &pole2,const int DOF_flag)
{
    double point1[6];

    Vector pole1_v, pole2_v;
    pole1_v.X = pole1[3] - pole1[0];
    pole1_v.Y = pole1[4] - pole1[1];
    pole1_v.Z = pole1[5] - pole1[2];
    pole2_v.X = pole2[3] - pole2[0];
    pole2_v.Y = pole2[4] - pole2[1];
    pole2_v.Z = pole2[5] - pole2[2];


    if(DOF_flag == 5)
    {
        if(pole1_v*pole2_v == 0)
        {
            if(minDistance(pole1, pole2, point1) < 840) //1267.8-204-136.7
            {
                return 1; //两杆最短距离大于机器人连杆长度，直接判定不可过渡
            } 
            else
                return 0;
        }
        
        if (minDistance(pole1, pole2, point1) < 1140) //1267.8 * 0.9
        {
            return 1; //两杆最短距离大于机器人连杆长度，直接判定不可过渡
        }
        return 0;
    }
    else if(DOF_flag == 6)
    {
        if(pole1_v*pole2_v == 0)
        {
            if(minDistance(pole1, pole2, point1) < 1057)  //(1611 - 269.3 - 167.2) * 0.9
            {
                return 1; //两杆最短距离大于机器人连杆长度，直接判定不可过渡
            } 
            else
                return 0;
        }
        
        if (minDistance(pole1, pole2, point1) < 1450) //1611 * 0.9(经验系数)
        {
            return 1; //两杆最短距离大于机器人连杆长度，直接判定不可过渡
        }
        return 0;
    }
}

//空间直线最短距离求解
double minDistance(std::vector<double> &pole1,std::vector<double> &pole2,double *point)
{
    double min_dis;
    Vector p1, p2;
    p1.X = pole1[0] - pole1[3];
    p1.Y = pole1[1] - pole1[4];
    p1.Z = pole1[2] - pole1[5];
    p2.X = pole2[0] - pole2[3];
    p2.Y = pole2[1] - pole2[4];
    p2.Z = pole2[2] - pole2[5]; //求出方向向量

    

    point[0] = (pole1[0] + pole1[3]) / 2;
    point[1] = (pole1[1] + pole1[4]) / 2;
    point[2] = (pole1[2] + pole1[5]) / 2;
    point[3] = (pole2[0] + pole2[3]) / 2;
    point[4] = (pole2[1] + pole2[4]) / 2;
    point[5] = (pole2[2] + pole2[5]) / 2;  //最近点对默认赋值

    Vector vertical_vec = operator&(p1, p2);

    // //精度判断
    // if(vertical_vec.X < 0.000001)
    // {
    //     vertical_vec.X = 0;
    // }
    // if(vertical_vec.Y < 0.000001)
    // {
    //     vertical_vec.Y = 0;
    // }
    // if(vertical_vec.Z < 0.000001)
    // {
    //     vertical_vec.Z = 0;
    // }

    //杆件平行
    if(vertical_vec.X == 0 && vertical_vec.Y == 0 && vertical_vec.Z == 0)
    {
        min_dis = sqrt((pole1[0] - pole2[0]) * (pole1[0] - pole2[0]) + (pole1[1] - pole2[1]) * (pole1[1] - pole2[1]) + (pole1[2] - pole2[2]) * (pole1[2] - pole2[2]));
        return min_dis;
    }
    else
    {
        Vector temp_p1p2;   //p1和p2杆上任意取一点组成向量
        temp_p1p2.X = pole1[0] - pole2[0];
        temp_p1p2.Y = pole1[1] - pole2[1];
        temp_p1p2.Z = pole1[2] - pole2[2];

        min_dis = fabs(operator*(vertical_vec, temp_p1p2) / sqrt(operator*(vertical_vec, vertical_vec)));

        Vector temp_vec1 = -p1;
        Vector temp_vec2 = -p2;
        Vector vertical_vec_temp = operator&(temp_vec1, temp_vec2);

        double t1, t2;  //中间变量

        t1 = operator*(operator&(-temp_p1p2, temp_vec2), vertical_vec_temp);
        t2 = operator*(operator&(-temp_p1p2, temp_vec1), vertical_vec_temp);
        double temp_dis = sqrt(operator*(vertical_vec_temp,vertical_vec_temp));

        t1 /= temp_dis * temp_dis;
        t2 /= temp_dis * temp_dis;

        // cout << t1 << " " << t2 << endl;
        // cout << sqrt(p1.X * p1.X + p1.Y * p1.Y + p1.Z * p1.Z) << sqrt(p2.X * p2.X + p2.Y * p2.Y + p2.Z * p2.Z) << endl;
        //判断t1，t2是否使点在线段范围内
        if(t1 < 0 || t1 > 1 || t2 < 0 ||t2 > 1)
        {
            vector<double> temp_point1 = {pole1[0], pole1[1], pole1[2]};
            vector<double> temp_point2 = {pole1[3], pole1[4], pole1[5]};
            vector<double> temp_point3 = {pole2[0], pole2[1], pole2[2]};
            vector<double> temp_point4 = {pole2[3], pole2[4], pole2[5]};

            double a1 = disPoint2Line(temp_point1, pole2);
            double a2 = disPoint2Line(temp_point2, pole2);
            double a3 = disPoint2Line(temp_point3, pole1);
            double a4 = disPoint2Line(temp_point4, pole1);

            int tmp = a1;
            point[0] = pole1[0];
            point[1] = pole1[1];
            point[2] = pole1[2];

            if(tmp > a2)
            {
                tmp = a2;
                point[0] = pole1[3];
                point[1] = pole1[4];
                point[2] = pole1[5];
            }
            if(tmp > a3)
            {
                tmp = a3;
                point[0] = pole2[0];
                point[1] = pole2[1];
                point[2] = pole2[2];
            }
            if(tmp > a4)
            {
                tmp = a4;
                point[0] = pole2[3];
                point[1] = pole2[4];
                point[2] = pole2[5];
            }
            min_dis = tmp;

            double k;   //临时参数
            //求垂足
            if(point[0] == pole1[0] || point[0] == pole1[3]) //垂足在杆件2上
            {
                k = -((pole2[0] - point[1]) * (pole2[3] - pole2[0]) + (pole2[1] - point[1]) * (pole2[4] - pole2[1]) + (pole2[2]  - point[2]) * (pole2[5] - pole2[2])) 
                / ((pole2[3] - pole2[0]) * (pole2[3] - pole2[0]) + (pole2[4] - pole2[1]) * (pole2[4] - pole2[1]) + (pole2[5] - pole2[2]) * (pole2[5] - pole2[2]));
                point[3] = k * (pole2[3] - pole2[0]) + pole2[0];
                point[4] = k * (pole2[4] - pole2[1]) + pole2[1];
                point[5] = k * (pole2[5] - pole2[2]) + pole2[2];              
            }
            else    //垂足在杆件1上
            {
                k = -((pole1[0] - point[0]) * (pole1[3] - pole1[0]) + (pole1[1] - point[1]) * (pole1[4] - pole1[1]) + (pole1[2] - point[2]) * (pole1[5] - pole1[2])) 
                / ((pole1[3] - pole1[0]) * (pole1[3] - pole1[0]) + (pole1[4] - pole1[1]) * (pole1[4] - pole1[1]) + (pole1[5] - pole1[2]) * (pole1[5] - pole1[2]));
                point[3] = point[0];
                point[4] = point[1];
                point[5] = point[2];    //杆件1的点在前三位
                point[0] = k * (pole1[3] - pole1[0]) + pole1[0];
                point[1] = k * (pole1[4] - pole1[1]) + pole1[1];
                point[2] = k * (pole1[5] - pole1[2]) + pole1[2];
            }
            return min_dis;
        }
        else
        {
            point[0] = (pole1[0] + (pole1[3] - pole1[0]) * t1);
            point[1] = (pole1[1] + (pole1[4] - pole1[1]) * t1);
            point[2] = (pole1[2] + (pole1[5] - pole1[2]) * t1);
            point[3] = (pole2[0] + (pole2[3] - pole2[0]) * t2);
            point[4] = (pole2[1] + (pole2[4] - pole2[1]) * t2);
            point[5] = (pole2[2] + (pole2[5] - pole2[2]) * t2);
            return min_dis;
        }
    }



    return min_dis;
}

//点到空间直线的最短距离求解
double disPoint2Line(vector<double> &point,vector<double> &line)
{
    // double ab = sqrt(pow((point[0] - line[0]), 2.0) + pow((point[1] - line[1]), 2.0) + pow((point[2] - line[2]), 2.0));
    // double as = sqrt(pow((point[0] - line[3]), 2.0) + pow((point[1] - line[4]), 2.0) + pow((point[2] - line[5]), 2.0));
    // double bs = sqrt(pow((point[3] - line[0]), 2.0) + pow((point[4] - line[1]), 2.0) + pow((point[5] - line[2]), 2.0));
    // double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
	// double sin_A = sqrt(1 - pow(cos_A, 2.0));
	// return as*sin_A;

    //矢量算法
    double cross = (line[3] - line[0]) * (point[0] - line[0]) + (line[4] - line[1]) * (point[1] - line[1]) + (line[5] - line[2]) * (point[2] - line[2]);
    if(cross<=0)
    {
        return sqrt((point[0] - line[0]) * (point[0] - line[0]) + (point[1] - line[1]) * (point[1] - line[1]) + (point[2] - line[2]) * (point[2] - line[2]));
    }

    double d2 = ((line[3] - line[0]) * (line[3] - line[0]) + (line[4] - line[1]) * (line[4] - line[1]) + (line[5] - line[2]) * (line[5] - line[2]));
    if(cross >= d2)
    {
        return sqrt((point[0] - line[3]) * (point[0] - line[3]) + (point[1] - line[4]) * (point[1] - line[4]) + (point[2] - line[5]) * (point[2] - line[5]));
    }

    double r = cross / d2;
    double px = line[0] + (line[3] - line[0]) * r;
    double py = line[1] + (line[4] - line[1]) * r;
    double pz = line[2] + (line[5] - line[2]) * r;
    return sqrt((px - point[0]) * (px - point[0]) + (py - point[1]) * (py - point[1]) + (pz - point[2]) * (pz - point[2]));
}

//空间两点的最短距离求解
double disPoint2Point(std::vector<double>& point1,std::vector<double> &point2)
{
    double tmp_x = point1[0] - point2[0];
    double tmp_y = point1[1] - point2[1];
    double tmp_z = point1[2] - point2[2];

    return sqrt(pow(tmp_x,2) + pow(tmp_y,2) +  pow(tmp_z,2));
}
