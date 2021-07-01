#include "../include/gpoptidata.h"


static globalPathPlan globaltmp;

/*
 * 功能：给定起点终点.输出全局规划路径
 * 参数：
 * - start_point：起点(离散杆件上)
 * - end_point：终点(离散杆件上)
 * - start_pole/end_pole：当前杆和目标杆
 * - res：邻接矩阵
 * 返回值：0表示正常，-1表示异常
 */
int globalPathPlanStart(std::vector<int>& start_point,
                                    std::vector<int>& end_point,
                                    int start_pole,int end_pole,
                                    std::vector<std::vector<int>> &res,
                                    const int DOF_flag)
{
    std::vector<double> grippoint;
    // globalPathPlan globaltest1;
    res = globaltmp.adjMat(truss,DOF_flag);
    std::vector<std::vector<int>> path = globaltmp.findAllPath(res,start_pole,end_pole);
    globaltmp.outPut(path);
    std::cout << "邻接矩阵：" << std::endl;
    for (int i = 0; i < res.size(); ++i)
    {
        for (int j = 0; j < res[0].size();++j)
        {
            cout << res[i][j] << ",";
        }
        cout << endl;
    }
    return 0;
}

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
int findminiball(const vector<double> &p1,const vector<double> &p2,
            const vector<double> &p3, const vector<double> &p4,
            vector<double> &center, double &radius)
{ 
    typedef double mytype;             // coordinate type

    const int       d = 3;            // dimension
    int             n = 4;      // number of points:1000000

    double seed;                      // initialize random number generator
    //   if (argc != 2) {
    //     seed = 0;
    //   } else
    //     seed = std::atoi(argv[1]);
    //   std::srand (seed);

    // generate random points and store them in a list of vectors
    // ----------------------------------------------------------
    std::list<std::vector<mytype> > lp;
    //   for (int i=0; i<n; ++i) {			
    //     std::vector<mytype> p(d);
    //     for (int j=0; j<d; ++j) 
    //       p[j] = rand();
    //     lp.push_back(p);
    //   }
		
    std::vector<mytype> p(d);
    lp.push_back(p1);
    lp.push_back(p2);
    lp.push_back(p3);
    lp.push_back(p4);
    
    // define the types of iterators through the points and their coordinates
    // ----------------------------------------------------------------------
    typedef std::list<std::vector<mytype> >::const_iterator PointIterator; 
    typedef std::vector<mytype>::const_iterator CoordIterator;

    // create an instance of Miniball
    // ------------------------------
    typedef Miniball::
        Miniball <Miniball::CoordAccessor<PointIterator, CoordIterator> > 
        MB;
    MB mb (d, lp.begin(), lp.end());
    
    // output results
    // --------------
    // center
    // std::cout << "Center:\n  ";
    // const mytype* center = mb.center();
    center = {mb.center()[0],mb.center()[1],mb.center()[2]};
    // for(int i=0; i<d; ++i, ++center)
    //     std::cout << *center << " ";
    // std::cout << std::endl;

    // // squared radius
    // std::cout << "Squared radius:\n  ";
    // std::cout << mb.squared_radius() <<  std::endl;
    radius = sqrt(mb.squared_radius());

    // // number of support points
    // std::cout << "Number of support points:\n  ";
    // std::cout << mb.nr_support_points() << std::endl;

    // // support points on the boundary determine the smallest enclosing ball
    // std::cout << "Support point indices (numbers refer to the input order):\n  ";
    // MB::SupportPointIterator it = mb.support_points_begin();
    // PointIterator first = lp.begin();
    // for (; it != mb.support_points_end(); ++it) {
    //     std::cout << std::distance(first, *it) << " "; // 0 = first point
    // }
    // std::cout << std::endl;
    
    // // relative error: by how much does the ball fail to contain all points? 
    // //                 tiny positive numbers come from roundoff and are ok
    // std::cout << "Relative error:\n  ";
    // mytype suboptimality;
    // std::cout << mb.relative_error (suboptimality) <<  std::endl;
    
    // // suboptimality: by how much does the ball fail to be the smallest
    // //                enclosing ball of its support points? should be 0 
    // //                in most cases, but tiny positive numbers are again ok
    // std::cout << "Suboptimality:\n  ";
    // std::cout << suboptimality <<  std::endl;

    // // validity: the ball is considered valid if the relative error is tiny
    // //           (<= 10 times the machine epsilon) and the suboptimality is zero
    // std::cout << "Validity:\n  ";
    // std::cout << (mb.is_valid() ? "ok" : "possibly invalid") << std::endl;

    // // computation time
    // std::cout << "Computation time was "<< mb.get_time() << " seconds\n";

    return 0;
}

/*
 * 功能:选择一条全局路径并生成带可过渡区域的路径
 * GDUT, 2020
 * 输入:全局路径序列：vector<int> truss_list;
 *     起点start_point:(x,y)    x,y为展开地图上坐标
 *     终点end_point：（x,y)
 * 输出:全局路径序列 truss_list
 *     夹持点信息 grippoint_list
*/
void getTransTrussList(std::vector<int> start_point,
                        std::vector<int> end_point,
                        std::vector<int> &truss_list, 
                        std::vector<std::vector<int>> &grippoint_list,
                        const int DOF_flag)
{
    if(truss_list.size() == 0 || truss_list.size() == 1)
    {
        std::cout << "error" << std::endl;
        return;
    }

    int truss_num = truss_list.size();
    grippoint_list.push_back(start_point);  //起点压入夹持点列表
    grippoint_list.resize(2 * truss_num-1,{0,0});

    std::vector<int> grippoint,sec_grippoint,base_point;
    base_point.push_back(grippoint_list[0][0]);
    base_point.push_back(grippoint_list[0][1]);
    int tmp = 1; //临时参数，用于标志grippoint_list
    for (int i = 0; i < truss_list.size()-1;++i)
    {
        std::cout << "base point " << base_point[0] << " " << base_point[1] << std::endl;
        globaltmp.transMat(truss[truss_list[i] - 1], truss[truss_list[i + 1] - 1], grippoint, sec_grippoint,base_point, DOF_flag);
        grippoint_list[tmp][0] = grippoint[0];
        grippoint_list[tmp][1] = grippoint[1];
        grippoint_list[tmp + 1][0] = sec_grippoint[0];
        grippoint_list[tmp + 1][1] = sec_grippoint[1];
        base_point[0] = sec_grippoint[0];
        base_point[1] = sec_grippoint[1];
        grippoint.pop_back();
        sec_grippoint.pop_back();
        sec_grippoint.pop_back();
        tmp += 2;
    }

    grippoint_list.push_back(end_point);  //终点压入夹持点列表
}

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
                        const int cur_truss,const int tar_truss,
                        std::vector<int> &pontential_truss)
{
    // double distance;
    // std::vector<double> tmp_point(3, 0);
    // std::vector<double> res_point(3, 0);
    // std::vector<double> link_point = {link[2][0], link[2][1], link[2][2]};

    // tmp_point = {link[0][0], link[0][1], link[0][2]};
    // res_point = tmp_point;
    // distance = disPoint2Point(link_point, tmp_point);

    // tmp_point = {link[1][0], link[1][1], link[1][2]};
    // if(disPoint2Point(link_point, tmp_point) > distance)
    // {
    //     res_point = tmp_point;
    //     distance = disPoint2Point(link_point, tmp_point);
    // }
    // tmp_point = {link[3][3], link[3][4], link[3][5]};
    // if(disPoint2Point(link_point, tmp_point) > distance)
    // {
    //     res_point = tmp_point;
    //     distance = disPoint2Point(link_point, tmp_point);
    // }

    std::vector<double> point1 = {link[0][0], link[0][1], link[0][2]};
    std::vector<double> point2 = {link[1][0], link[1][1], link[1][2]};
    std::vector<double> point3 = {link[2][0], link[2][1], link[2][2]};
    std::vector<double> point4 = {link[3][3], link[3][4], link[3][5]};
    std::vector<double> center;
    double radius;
    //构建包络圆球
    findminiball(point1,point2,point3,point4,center,radius);

    std::vector<int> cur_adjmat = adj_mat[cur_truss - 1];

    for (int i = 0; i < cur_adjmat.size(); ++i)
    {
        if (cur_adjmat[i] == 1 && i != tar_truss - 1)
        {
            if (disPoint2Line(center, truss[i]) < radius + 100)
            {
                pontential_truss.push_back(i);
            }
        }
    }

    pontential_truss.push_back(cur_truss);
    pontential_truss.push_back(tar_truss);

    if(pontential_truss.size() != 2)
    {
        return 0;
    }

    return 1;
}

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
                    std::vector<std::vector<double>> &link)
{
    double joint_val[6];
    transwithIK(cur_truss, tar_truss, length1, angle1, length2, angle2,0,joint_val);

    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
    Linkage6D(len, joint_val, cur_truss, length1, angle1,0,link);
    return;
}

/*
 * 功能：通过机器人当前连杆坐标点与潜在碰撞杆件进行碰撞检测
 * 参数：
 * - link：机器人当前连杆坐标数组
 * - potential_truss：潜在的碰撞杆件
 * 返回值：1表示正常，0表示碰撞
 */
int potentialObsTest(std::vector<std::vector<double>> &link,
                    std::vector<int> &pontential_truss) //注意:此处为杆件编码减1
{
    double threshold = 90;
    std::vector<double> tmp_link(6, 0);
    double point[6];    //函数参数,此处无用
    for (int i = 0; i < pontential_truss.size()-2; ++i)
    {
        for (int j = 0; j < 4;++j)
        {
            tmp_link = link[j];
            double distance = minDistance(tmp_link, truss[pontential_truss[i]], point);
            // std::cout << distance << " ";
            if (distance < threshold)  //碰撞检测阈值设置  //85
            {
                // std::cout << "err01 " << distance <<std::endl;
                return 0;
            }
        }
    }

    for (int i = pontential_truss.size()-2; i < pontential_truss.size(); ++i)
    {
        //机器人与夹持杆件不发生碰撞
        if(minDistance(link[1],truss[pontential_truss[i]],point) < threshold+40 || 
        minDistance(link[2],truss[pontential_truss[i]],point) < threshold+40 || 
        minDistance(link[3],truss[pontential_truss[i]],point) < threshold+40 )
        {
            // std::cout << "err02 " <<minDistance(link[1],truss[pontential_truss[i]],point)<<" "<<
            // minDistance(link[2],truss[pontential_truss[i]],point)<<" "<<
            // minDistance(link[3],truss[pontential_truss[i]],point) <<std::endl;
            return 0;
        }
    }

    return 1;
}

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
int calculateCentreAndRadius(const vector<double> &p1,const vector<double> &p2,
        const vector<double> &p3, const vector<double> &p4,
        vector<double> &centre, double &radius)
{
    double a = p1[0] - p2[0], b = p1[1] - p2[1], c = p1[2] - p2[2];
    double a1 = p3[0] - p4[0], b1 = p3[1] - p4[1], c1 = p3[2] - p3[2];
    double a2 = p2[0] - p3[0], b2 = p2[1] - p3[1], c2 = p2[2] - p3[2];
    double A = p1[0] * p1[0] - p2[0] * p2[0];
    double B = p1[1] * p1[1] - p2[1] * p2[1];
    double C = p1[2] * p1[2] - p2[2] * p2[2];
    double A1 = p3[0] * p3[0] - p4[0] * p4[0];
    double B1 = p3[1] * p3[1] - p4[1] * p4[1];
    double C1 = p3[2] * p3[2] - p4[2] * p4[2];
    double A2 = p2[0] * p2[0] - p3[0] * p3[0];
    double B2 = p2[1] * p2[1] - p3[1] * p3[1];
    double C2 = p2[2] * p2[2] - p3[2] * p3[2];
    double P = (A + B + C) /2;
    double Q = (A1 + B1 + C1) / 2;
    double R = (A2 + B2 + C2) / 2;

    // D是系数行列式，利用克拉默法则
    double D = a*b1*c2 + a2*b*c1 + c*a1*b2 - (a2*b1*c + a1*b*c2 + a*b2*c1);
    double Dx = P*b1*c2 + b*c1*R + c*Q*b2 - (c*b1*R + P*c1*b2 + Q*b*c2);
    double Dy = a*Q*c2 + P*c1*a2 + c*a1*R - (c*Q*a2 + a*c1*R + c2*P*a1);
    double Dz = a*b1*R + b*Q*a2 + P*a1*b2 - (a2*b1*P + a*Q*b2 + R*b*a1);

    if(D == 0){
        cerr << "四点共面" << endl;
        return -1;
    }else{
        centre.push_back(Dx/D);
        centre.push_back(Dy/D);
        centre.push_back(Dz/D);
        radius = sqrt((p1[0]-centre[0])*(p1[0]-centre[0]) +
                              (p1[1]-centre[1])*(p1[1]-centre[1]) +
                              (p1[2]-centre[2])*(p1[2]-centre[2]));
        return 0;
    }
}