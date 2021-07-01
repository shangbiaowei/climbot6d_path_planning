#include <math.h>
#include <iomanip>
#include <fstream>
#include "../include/pole.h"


#define PI      3.1415926535898 // 圆周率
#define PI_RAD  0.0174532925199 // 角度转换为弧度参数
#define PI_DEG  57.2957795130823 // 弧度转换为角度参数



//构造函数
Discretepole::Discretepole()
{
    //展开数组初始化
    cur_pole->p_map = new double**[pole_rows] ;
    target_pole->p_map = new double**[pole_rows] ;
    for( int i = 0 ; i < pole_rows ; ++i )
    {
        cur_pole->p_map[i] = new double*[pole_cols] ;
        target_pole->p_map[i] = new double*[pole_cols] ;
    }
    for( int i = 0 ; i < pole_rows ; ++i )
    for( int j = 0 ; j < pole_cols ; ++j )
    {
        cur_pole->p_map[i][j] = new double[6] ;
        target_pole->p_map[i][j] = new double[6] ;
    }

    //过渡标志数组初始化
    cur_pole->transflag.resize(pole_rows);
    target_pole->transflag.resize(pole_rows);
    for (int i = 0; i < pole_rows;++i)
    {
        cur_pole->transflag[i].resize(pole_cols);
        target_pole->transflag[i].resize(pole_cols);
    }
}

//析构函数
Discretepole::~Discretepole()
{
    //释放三维数组
    for(int  i = 0 ; i < pole_rows ; ++i )
    {
        for(int  j = 0 ; j < pole_cols ; ++j )
        {
            delete[] target_pole->p_map[i][j];
            delete[] cur_pole->p_map[i][j];
        }
    }
    for(int  i = 0 ; i < pole_rows ; ++i )
    {
        delete[] target_pole->p_map[i];
        delete[] cur_pole->p_map[i];
    }
    delete[] target_pole->p_map;
    delete[] cur_pole->p_map;

    delete cur_pole;
    delete target_pole;
}

//起点设定
void Discretepole::setStartPos(Poles *cur_pole,
                            int length,int alpha,
                            float start_point[],double &res_angle)
{
    for(int i = 0;i<3;++i)
    {
        start_point[i] = cur_pole->p[i] + (cur_pole->p[i+3] - cur_pole->p[i]) / pole_rows * (length);
        // std::cout << "start_p[" << i << "]" << start_point[i] << " ";
    }   //起点位置设置

    //double alpha = 0.0;   //弧度
    res_angle = -3.1416 + 6.2832 / pole_cols * (alpha);
    // std::cout << "res_angle=" << res_angle << std::endl;
}

//离散杆件赋值
void Discretepole::AssignmentData(float temp[],Poles *cur_pole,Poles *target_pole)
{
    //赋值
    for (int i = 0; i < pole_rows; ++i)
    {
        for (int j = 0; j < pole_cols; ++j)
        {
            cur_pole->transflag[i][j] = 0;
            target_pole->transflag[i][j] = 0; //过渡标志位默认置0
            for (int k = 0; k < 6; ++k)
            {
                switch (k)
                {
                case 0:
                    cur_pole->p_map[i][j][k] = cur_pole->p_x + i * temp[0];
                    target_pole->p_map[i][j][k] = target_pole->p_x + i * temp[4];
                    break;
                case 1:
                    cur_pole->p_map[i][j][k] = cur_pole->p_y + i * temp[1];
                    target_pole->p_map[i][j][k] = target_pole->p_y + i * temp[5];
                    break;
                case 2:
                    cur_pole->p_map[i][j][k] = cur_pole->p_z + i * temp[2];
                    target_pole->p_map[i][j][k] = target_pole->p_z + i * temp[6];
                    break;
                case 3:
                    cur_pole->p_map[i][j][k] = cur_pole->p_w;
                    target_pole->p_map[i][j][k] = target_pole->p_w;
                    break;
                case 4:
                    // if (GripId == 0)
                    // {
                        cur_pole->p_map[i][j][k] = cur_pole->p_p;
                        target_pole->p_map[i][j][k] = target_pole->p_p;
                    // }
                    // else
                    // {
                    //     cur_pole->p_map[i][j][k] = cur_pole->p_p - j * temp[3];
                    //     target_pole->p_map[i][j][k] = target_pole->p_p - j * temp[7];
                    //     if (cur_pole->p_map[i][j][k] < -180)
                    //     {
                    //         cur_pole->p_map[i][j][k] = cur_pole->p_map[i][j][k] + 360;
                    //     }
                    //     if (target_pole->p_map[i][j][k] < -180)
                    //     {
                    //         target_pole->p_map[i][j][k] = target_pole->p_map[i][j][k] + 360;
                    //     }
                    // }
                    // std::cout<<target_pole->p_map[i][j][k]<<" ";
                    break;
                case 5:
                    // if (GripId == 0)
                    // {
                        cur_pole->p_map[i][j][k] = cur_pole->p_r + j * temp[3];
                        target_pole->p_map[i][j][k] = target_pole->p_r + j * temp[7];
                        if (cur_pole->p_map[i][j][k] > 180)
                        {
                            cur_pole->p_map[i][j][k] = cur_pole->p_map[i][j][k] - 360;
                        }
                        if (target_pole->p_map[i][j][k] > 180)
                        {
                            target_pole->p_map[i][j][k] = target_pole->p_map[i][j][k] - 360;
                        }
                    // }
                    // else
                    // {
                    //     cur_pole->p_map[i][j][k] = cur_pole->p_r;
                    //     target_pole->p_map[i][j][k] = target_pole->p_r;
                    // }
                    break;
                }
            }
        } 
    }          
}

//过渡测试
void Discretepole::transitionalTest(Poles *cur_pole, Poles *target_pole,const int GripId)
{
    std::ofstream outfile;
    outfile.open("../src/myfile.txt");  //输出栅格地图

    if(DOF_flag == 6)
    {
        //初始化机器人
        double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
        Kine_CR_SixDoF_G1 climbot6d_G1;
        Kine_CR_SixDoF_G2 climbot6d_G2;
        climbot6d_G1.Set_Length(len);
        climbot6d_G2.Set_Length(len);

        //距离初步筛选，过滤不可到达点（不考虑姿态）
        //工作空间球形方程为x * x + y * y + (z - c) * (z - c) = r * r;
        double curgdj[6] = {0,90,90,0,0,0},out_gdjpos[6];
    
        /*
        //遍历杆件1
        for (int i = 0;i < pole_rows;++i)
        { 
            //std::cout<<"pole1["<<i<<"]="<<"\t";
            for (int j = 0;j < pole_cols;++j)
                {
                    double gdcpos[6] = {cur_pole->p_map[i][j][0],cur_pole->p_map[i][j][1],cur_pole->p_map[i][j][2],cur_pole->p_map[i][j][3],cur_pole->p_map[i][j][4],cur_pole->p_map[i][j][5]};
                    if(cur_pole->p_map[i][j][0] * cur_pole->p_map[i][j][0] + cur_pole->p_map[i][j][1] * cur_pole->p_map[i][j][1] + 
                    (cur_pole->p_map[i][j][2] - len[0] - len[1]) * (cur_pole->p_map[i][j][2] - len[0] - len[1]) - 
                    (len[2] + len[3] + len[4] + len[5] + len[6]) * (len[2] + len[3] + len[4] + len[5] + len[6]) > 0)
                    {
                        break; //标志位设0,不可过渡
                    }
                    else if(climbot6d_G1.IKine(gdcpos,curgdj,out_gdjpos) == 0)
                    {
                        cur_pole->transflag[i][j] = 1; //标志位设1,可过渡
                    }  
                    else
                    {
                        cur_pole->transflag[i][j] = 0; //标志位设0,不可过渡
                    }  //判断逆解是否存在
                    //std::cout<<cur_pole->p_map[i][j][6]<<" ";
                    //outfile << cur_pole->p_map[i][j][6] << " ";
                } 
        //std::cout<<std::endl;
        //outfile << std::endl;
        }
        //outfile.close();
        */

        //遍历杆件2
        //g0基座
        if(GripId == 0)
        {
            // std::cout << "g0";
            for (int i = 0;i < pole_rows;++i)
            {
                for (int j = 0;j < pole_cols;++j)
                {
                    double gdcpos[6] = {target_pole->p_map[i][j][0],target_pole->p_map[i][j][1],target_pole->p_map[i][j][2],target_pole->p_map[i][j][3],target_pole->p_map[i][j][4],target_pole->p_map[i][j][5]};
                    if(target_pole->p_map[i][j][0] * target_pole->p_map[i][j][0] + target_pole->p_map[i][j][1] * target_pole->p_map[i][j][1] + 
                    (target_pole->p_map[i][j][2] - len[0] - len[1]) * (target_pole->p_map[i][j][2] - len[0] - len[1]) - 
                    (len[2] + len[3] + len[4] + len[5] + len[6]) * (len[2] + len[3] + len[4] + len[5] + len[6]) > 0)
                    {
                        target_pole->transflag[i][j] = 0;
                        // break; //标志位设0,不可过渡
                    }
                    else if(climbot6d_G1.IKine(gdcpos,curgdj,out_gdjpos) == 0)
                    {
                        //target_pole->p_map[i][j][6] = 1; //标志位设1,可过渡
                        target_pole->transflag[i][j] = 1;
                    }  
                    else
                    {
                        //target_pole->p_map[i][j][6] = 0; //标志位设0,不可过渡
                        target_pole->transflag[i][j] = 0;
                    }  //判断逆解是否存在
                    outfile << target_pole->transflag[i][j]<< " ";
                } 
            outfile << std::endl;
            }
            outfile.close();
        }
        if(GripId == 7)
        {
            // std::cout << "g7";
            for (int i = 0;i < pole_rows;++i)
            {
                for (int j = 0;j < pole_cols;++j)
                {
                    double gdcpos[6] = {target_pole->p_map[i][j][0],target_pole->p_map[i][j][1],target_pole->p_map[i][j][2],target_pole->p_map[i][j][3],target_pole->p_map[i][j][4],target_pole->p_map[i][j][5]};
                    if(target_pole->p_map[i][j][0] * target_pole->p_map[i][j][0] + target_pole->p_map[i][j][1] * target_pole->p_map[i][j][1] + 
                    (target_pole->p_map[i][j][2] - len[0] - len[1]) * (target_pole->p_map[i][j][2] - len[0] - len[1]) - 
                    (len[2] + len[3] + len[4] + len[5] + len[6]) * (len[2] + len[3] + len[4] + len[5] + len[6]) > 0)
                    {
                        target_pole->transflag[i][j] = 0;
                        // break; //标志位设0,不可过渡
                    }
                    else if(climbot6d_G2.IKine(gdcpos,curgdj,out_gdjpos) == 0)
                    {
                        //target_pole->p_map[i][j][6] = 1; //标志位设1,可过渡
                        target_pole->transflag[i][j] = 1;
                    }  
                    else
                    {
                        //target_pole->p_map[i][j][6] = 0; //标志位设0,不可过渡
                        target_pole->transflag[i][j] = 0;
                    }  //判断逆解是否存在
                    outfile << target_pole->transflag[i][j]<< " ";
                } 
            outfile << std::endl;
            }
            outfile.close();
        }
    }
    else if(DOF_flag == 5)
    {
        //初始化机器人
        double len[6] = {204, 136.7, 293.2, 293.2, 136.7, 204};
        Kine_CR_FiveDoF_G1 climbot5d_G1;
        Kine_CR_FiveDoF_G2 climbot5d_G2;
        climbot5d_G1.Set_Length(len);
        climbot5d_G2.Set_Length(len);

        //距离初步筛选，过滤不可到达点（不考虑姿态）
        //工作空间球形方程为x * x + y * y + (z - c) * (z - c) = r * r;
        double curgdj[5] = {0,90,0,90,0},out_gdjpos[6];
    
        //遍历杆件2
        //g0基座
        if(GripId == 0)
        {
            // std::cout << "g0";
            for (int i = 0;i < pole_rows;++i)
            {
                for (int j = 0;j < pole_cols;++j)
                {
                    double gdcpos[6] = {target_pole->p_map[i][j][0],target_pole->p_map[i][j][1],target_pole->p_map[i][j][2],target_pole->p_map[i][j][3],target_pole->p_map[i][j][4],target_pole->p_map[i][j][5]};
                    if(target_pole->p_map[i][j][0] * target_pole->p_map[i][j][0] + target_pole->p_map[i][j][1] * target_pole->p_map[i][j][1] + 
                    (target_pole->p_map[i][j][2] - len[0] - len[1]) * (target_pole->p_map[i][j][2] - len[0] - len[1]) - 
                    (len[2] + len[3] + len[4] + len[5] + len[6]) * (len[2] + len[3] + len[4] + len[5] + len[6]) > 0)
                    {
                        target_pole->transflag[i][j] = 0;
                        // break; //标志位设0,不可过渡
                    }
                    else if(climbot5d_G1.IKine(gdcpos,curgdj,out_gdjpos) == 0)
                    {
                        //target_pole->p_map[i][j][6] = 1; //标志位设1,可过渡
                        target_pole->transflag[i][j] = 1;
                    }  
                    else
                    {
                        //target_pole->p_map[i][j][6] = 0; //标志位设0,不可过渡
                        target_pole->transflag[i][j] = 0;
                    }  //判断逆解是否存在
                    outfile << target_pole->transflag[i][j]<< " ";
                } 
            outfile << std::endl;
            }
            outfile.close();
        }
        if(GripId == 7)
        {
            // std::cout << "g7";
            for (int i = 0;i < pole_rows;++i)
            {
                for (int j = 0;j < pole_cols;++j)
                {
                    double gdcpos[6] = {target_pole->p_map[i][j][0],target_pole->p_map[i][j][1],target_pole->p_map[i][j][2],target_pole->p_map[i][j][3],target_pole->p_map[i][j][4],target_pole->p_map[i][j][5]};
                    if(target_pole->p_map[i][j][0] * target_pole->p_map[i][j][0] + target_pole->p_map[i][j][1] * target_pole->p_map[i][j][1] + 
                    (target_pole->p_map[i][j][2] - len[0] - len[1]) * (target_pole->p_map[i][j][2] - len[0] - len[1]) - 
                    (len[2] + len[3] + len[4] + len[5] + len[6]) * (len[2] + len[3] + len[4] + len[5] + len[6]) > 0)
                    {
                        target_pole->transflag[i][j] = 0;
                        // break; //标志位设0,不可过渡
                    }
                    else if(climbot5d_G2.IKine(gdcpos,curgdj,out_gdjpos) == 0)
                    {
                        //target_pole->p_map[i][j][6] = 1; //标志位设1,可过渡
                        target_pole->transflag[i][j] = 1;
                    }  
                    else
                    {
                        //target_pole->p_map[i][j][6] = 0; //标志位设0,不可过渡
                        target_pole->transflag[i][j] = 0;
                    }  //判断逆解是否存在
                    outfile << target_pole->transflag[i][j]<< " ";
                } 
            outfile << std::endl;
            }
            outfile.close();
        }
    }
    else
    {
        std::cout << "DOF setting is error!" << std::endl;
    }
 
}

int Discretepole::simulationResultPrint(std::vector<double> &p0,
                                        std::vector<double> &p1,
                                        int length_cur,int alpha_cur,
                                        int length_tar,int alpha_tar,
                                        const int GripId,
                                        std::vector<double>& joint_val)
{
    poleTransition(p0, p1, length_cur, alpha_cur, GripId, DOF_flag);
    
    //初始化机器人
    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
    Kine_CR_SixDoF_G1 climbot6d_G1;
    Kine_CR_SixDoF_G2 climbot6d_G2;
    climbot6d_G1.Set_Length(len);
    climbot6d_G2.Set_Length(len);
    double curgdj[6] = {0,90,90,0,0,0},out_gdjpos[6];

    if(GripId == 0)
    {
        // std::cout << "g0";
        double gdcpos[6] = {target_pole->p_map[length_tar][alpha_tar][0],target_pole->p_map[length_tar][alpha_tar][1],
                            target_pole->p_map[length_tar][alpha_tar][2],target_pole->p_map[length_tar][alpha_tar][3],
                            target_pole->p_map[length_tar][alpha_tar][4],target_pole->p_map[length_tar][alpha_tar][5]};
        if(climbot6d_G1.IKine(gdcpos,curgdj,out_gdjpos) == 0)
        {
            std::cout << "位姿 =";
            for (size_t r = 0; r < 6;++r)
            {
                std::cout << out_gdjpos[r]<< ",";
                // std::cout << target_pole->p_map[length_tar][alpha_tar][r] << " ";
            }
            std::cout << std::endl;
            //仿真用输出值
            std::cout << "P=";
            std::cout <<std::fixed<<std::setprecision(1)<< -out_gdjpos[5] << ",";     
            std::cout << std::fixed<<std::setprecision(1)<<out_gdjpos[4]<< ",";  
            std::cout << std::fixed<<std::setprecision(1)<<-out_gdjpos[3] << ",";   
            std::cout <<std::fixed<<std::setprecision(1)<<(out_gdjpos[2] -  90)<< ",";      
            std::cout <<std::fixed<<std::setprecision(1)<< -(out_gdjpos[1] - 90) << ",";
            std::cout <<std::fixed<<std::setprecision(1)<< out_gdjpos[0]<< ",";
            std::cout << ";"<<std::endl;

            joint_val = {-out_gdjpos[5], out_gdjpos[4], -out_gdjpos[3], (out_gdjpos[2] - 90), -(out_gdjpos[1] - 90), out_gdjpos[0]};

            return 1;
        }
        else
        {
            return 0;
        }
    }

        //g7基座
    if(GripId == 7)
    {
        // std::cout << "g7";
        double gdcpos[6] = {target_pole->p_map[length_tar][alpha_tar][0],target_pole->p_map[length_tar][alpha_tar][1],
                            target_pole->p_map[length_tar][alpha_tar][2],target_pole->p_map[length_tar][alpha_tar][3],
                            target_pole->p_map[length_tar][alpha_tar][4],target_pole->p_map[length_tar][alpha_tar][5]};

        if(climbot6d_G1.IKine(gdcpos,curgdj,out_gdjpos) == 0)
        {
            std::cout << "位姿 =";
            for (size_t r = 0; r < 6;++r)
            {
                // std::cout << out_gdjpos[r] << " ";
                std::cout << target_pole->p_map[length_tar][alpha_tar][r] << " ";
            }                    
            std::cout << std::endl;
            std::cout << "P=";
            std::cout <<std::fixed<<std::setprecision(1)<< -out_gdjpos[5] << ",";     
            std::cout << std::fixed<<std::setprecision(1)<<out_gdjpos[4]<< ",";  
            std::cout << std::fixed<<std::setprecision(1)<<-out_gdjpos[3] << ",";   
            std::cout <<std::fixed<<std::setprecision(1)<<(out_gdjpos[2] -  90)<< ",";      
            std::cout <<std::fixed<<std::setprecision(1)<< -(out_gdjpos[1] - 90) << ",";
            std::cout <<std::fixed<<std::setprecision(1)<< out_gdjpos[0]<< ",";
            std::cout << ";"<<std::endl;
            joint_val = {-out_gdjpos[5], out_gdjpos[4], -out_gdjpos[3], (out_gdjpos[2] - 90), -(out_gdjpos[1] - 90), out_gdjpos[0]};
            return 1;
        } 
        else
        {
            return 0;
        }
    }
    return -1;
}

int Discretepole::getJointResult(std::vector<double> &p0,
                        std::vector<double> &p1,
                        int length_cur,int alpha_cur,
                        int length_tar,int alpha_tar,
                        const int GripId,double *joint_val)
{

    poleTransition(p0, p1, length_cur, alpha_cur, GripId, DOF_flag);
    
    //初始化机器人
    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
    Kine_CR_SixDoF_G1 climbot6d_G1;
    Kine_CR_SixDoF_G2 climbot6d_G2;
    climbot6d_G1.Set_Length(len);
    climbot6d_G2.Set_Length(len);
    double curgdj[6] = {0,90,90,0,0,0},out_gdjpos[6];

    if(GripId == 0)
    {
        double gdcpos[6] = {target_pole->p_map[length_tar][alpha_tar][0],target_pole->p_map[length_tar][alpha_tar][1],
                            target_pole->p_map[length_tar][alpha_tar][2],target_pole->p_map[length_tar][alpha_tar][3],
                            target_pole->p_map[length_tar][alpha_tar][4],target_pole->p_map[length_tar][alpha_tar][5]};
        if(climbot6d_G1.IKine(gdcpos,curgdj,out_gdjpos) == 0)
        {
            for (size_t r = 0; r < 6;++r)
            {
                joint_val[r] = out_gdjpos[r];
            }
        }
    }

    return 0;
}

void Discretepole::poleTransition(std::vector<double> &p0,
                                std::vector<double> &p1,
                                int length,int alpha,
                                const int GripId,
                                const int DOFflag)
{
    Discretepole();

    DOF_flag = DOFflag;

    for(int i = 0;i<6;i++)
    {
        cur_pole->p[i] = p0[i];
        target_pole->p[i] = p1[i];
    }

    //定义世界坐标系
    Vector World_X,World_Y, World_Z; //世界坐标系向量
    World_X.X = 1;World_X.Y = 0;World_X.Z = 0;
    World_Y.X = 0;World_Y.Y = 1;World_Y.Z = 0;
    World_Z.X = 0;World_Z.Y = 0;World_Z.Z = 1;

    //float r = 24.0;
    float temp[8];  //中间变量

    //杆件向量初始化
    cur_pole->pole_v.X = cur_pole->p[3] - cur_pole->p[0];
    cur_pole->pole_v.Y = cur_pole->p[4] - cur_pole->p[1];
    cur_pole->pole_v.Z = cur_pole->p[5] - cur_pole->p[2];
    target_pole->pole_v.X = target_pole->p[3] - target_pole->p[0];
    target_pole->pole_v.Y = target_pole->p[4] - target_pole->p[1];
    target_pole->pole_v.Z = target_pole->p[5] - target_pole->p[2];
//**************************************************************************************************
    //起点设定
    float start_p[6];
    double res_angle;
    setStartPos(cur_pole, length, alpha, start_p, res_angle);

    // std::cout <<"res_angle =" <<res_angle << std::endl;

    MtxKine mtx_rotx;
    mtx_rotx.R11 = 1.0;mtx_rotx.R12 = 0.0;mtx_rotx.R13 = 0.0;
    mtx_rotx.R21 = 0.0;mtx_rotx.R22 = cos(res_angle);mtx_rotx.R23 = sin(res_angle);
    mtx_rotx.R31 = 0.0;mtx_rotx.R32 = -sin(res_angle);mtx_rotx.R33 = cos(res_angle);
    mtx_rotx.X = 0.0;            mtx_rotx.Y = 0.0;            mtx_rotx.Z = 0.0;  //起点姿态设置
    //**************************************************************************************************
    //20201011修改，以沿重力方向的矢量作为假设
    Vector Pole1_X = Norm_Vec(cur_pole->pole_v);  //20200723修正错误，没有考虑所有情况
    Vector Pole1_Y,Pole1_Z;

    if(cur_pole->p[0] == cur_pole->p[3] && cur_pole->p[1] == cur_pole->p[4])    //杆件平行于世界坐标系Z轴
    {
        Pole1_Y.X = 1;
        Pole1_Y.Y = 0;
        Pole1_Y.Z = 0;
        Pole1_Z.X = 0;
        Pole1_Z.Y = 1;
        Pole1_Z.Z = 0;
    }
    else
    {
        Pole1_Y = operator&(Pole1_X, World_Z);
        Pole1_Y = Norm_Vec(Pole1_Y);//单位向量
        Pole1_Z = operator&(Pole1_X,Pole1_Y);
        Pole1_Z = Norm_Vec(Pole1_Z);//单位向量
    }
    //基座坐标系→杆件坐标系
    MtxKine mtx_base2pole1;
    mtx_base2pole1.R11 = Pole1_X.X;mtx_base2pole1.R12 = Pole1_X.Y;mtx_base2pole1.R13 = Pole1_X.Z;
    mtx_base2pole1.R21 = Pole1_Y.X;mtx_base2pole1.R22 = Pole1_Y.Y;mtx_base2pole1.R23 = Pole1_Y.Z;
    mtx_base2pole1.R31 = Pole1_Z.X;mtx_base2pole1.R32 = Pole1_Z.Y;mtx_base2pole1.R33 = Pole1_Z.Z;
    //以起点为基座坐标系
    mtx_base2pole1.X = -start_p[0];mtx_base2pole1.Y = -start_p[1];mtx_base2pole1.Z = -start_p[2];
    Vector Pole2_X = Norm_Vec(target_pole->pole_v);
    Vector Pole2_Y,Pole2_Z;
    if(target_pole->p[0] == target_pole->p[3] && target_pole->p[1] == target_pole->p[4])
    {
        Pole2_Y.X = 1;
        Pole2_Y.Y = 0;
        Pole2_Y.Z = 0;
        Pole2_Z.X = 0;
        Pole2_Z.Y = 1;
        Pole2_Z.Z = 0;
    }
    else
    {
        //Pole2_Y.X = 100; Pole2_Y.Y = 100;
        //Pole2_Y.Z = -(Pole2_Y.X * Pole2_X.X + Pole2_Y.Y * Pole2_X.Y) / Pole2_X.Z;
        Pole2_Y = operator&(Pole2_X, World_Z);
        Pole2_Y = Norm_Vec(Pole2_Y);//单位向量
        //std::cout<<"Pole2_Y x="<<Pole2_Y.X<<"y"<<Pole2_Y.Y<<"z"<<Pole2_Y.Z<<std::endl;
        Pole2_Z = operator&(Pole2_X,Pole2_Y);
        Pole2_Z = Norm_Vec(Pole2_Z);//单位向量

    }


    // std::cout << mtx_base2pole1.R11 << " " << mtx_base2pole1.R12 << " "<<mtx_base2pole1.R13 << std::endl;
    // std::cout << mtx_base2pole1.R21 << " " << mtx_base2pole1.R22 << " "<<mtx_base2pole1.R23 << std::endl;
    // std::cout << mtx_base2pole1.R31 << " " << mtx_base2pole1.R32 << " "<<mtx_base2pole1.R33 << std::endl;
    // std::cout << mtx_base2pole1.X << " " << mtx_base2pole1.Y << " "<<mtx_base2pole1.Z << std::endl;
   
    float in_p1[3],in_p2[3]; //杆件1端点在基座坐标系坐标
    float in_p3[3],in_p4[3]; //杆件2端点在基座坐标系坐标

    //姿态求解，此时以杆件1上夹持器所在坐标系为坐标系，求解出杆2上夹持器坐标系的旋转矩阵
    float in_vecx[3] = {float(Pole2_X.X), float(Pole2_X.Y), float(Pole2_X.Z)},
          in_vecy[3] = {float(Pole2_Y.X), float(Pole2_Y.Y), float(Pole2_Y.Z)},
          in_vecz[3] = {float(Pole2_Z.X), float(Pole2_Z.Y), float(Pole2_Z.Z)};
    float out_vecx[3],out_vecy[3],out_vecz[3];
    float world_zero[3]={0.0,0.0,0.0};

    PointCoordinateC(&mtx_base2pole1,world_zero,world_zero);
    PointCoordinateC(&mtx_base2pole1,in_vecx,in_vecx);
    PointCoordinateC(&mtx_base2pole1,in_vecy,in_vecy);
    PointCoordinateC(&mtx_base2pole1,in_vecz,in_vecz);
    PointCoordinateC(&mtx_rotx,world_zero,world_zero);
    PointCoordinateC(&mtx_rotx,in_vecx,in_vecx);
    PointCoordinateC(&mtx_rotx,in_vecy,in_vecy);
    PointCoordinateC(&mtx_rotx,in_vecz,in_vecz); //左乘变换矩阵顺序

    for(int i = 0;i<3;++i)
    {
        out_vecx[i] = in_vecx[i] - world_zero[i];
        out_vecy[i] = in_vecy[i] - world_zero[i];
        out_vecz[i] = in_vecz[i] - world_zero[i];
    }

    MtxKine mtx_pole2pole1; //此时杆件1上夹持点固结基座标系
    double pos_p2top1[6] = {0,0,0,0,0,0};
    // mtx_pole2pole1.R11 = out_vecx[0];mtx_pole2pole1.R12 = out_vecx[1];mtx_pole2pole1.R13 = out_vecx[2];
    // mtx_pole2pole1.R21 = out_vecy[0];mtx_pole2pole1.R22 = out_vecy[1];mtx_pole2pole1.R23 = out_vecy[2];
    // mtx_pole2pole1.R31 = out_vecz[0];mtx_pole2pole1.R32 = out_vecz[1];mtx_pole2pole1.R33 = out_vecz[2];
    mtx_pole2pole1.R11 = out_vecx[0];mtx_pole2pole1.R12 = out_vecy[0];mtx_pole2pole1.R13 = out_vecz[0];
    mtx_pole2pole1.R21 = out_vecx[1];mtx_pole2pole1.R22 = out_vecy[1];mtx_pole2pole1.R23 = out_vecz[1];
    mtx_pole2pole1.R31 = out_vecx[2];mtx_pole2pole1.R32 = out_vecy[2];mtx_pole2pole1.R33 = out_vecz[2];
    mtx_pole2pole1.X = 0;            mtx_pole2pole1.Y = 0;            mtx_pole2pole1.Z = 0;  //xyz不作设置
    Trans_MtxToPos(&mtx_pole2pole1,pos_p2top1); //杆件2点在基座坐标系下表示

    // std::cout<<"mtx.r11="<<mtx_pole2pole1.R11<<"mtx.r12="<<mtx_pole2pole1.R12<<"mtx.r13="<<mtx_pole2pole1.R13<<std::endl;
    // std::cout<<"mtx.r21="<<mtx_pole2pole1.R21<<"mtx.r22="<<mtx_pole2pole1.R22<<"mtx.r23="<<mtx_pole2pole1.R23<<std::endl;
    // std::cout<<"mtx.r31="<<mtx_pole2pole1.R31<<"mtx.r32="<<mtx_pole2pole1.R32<<"mtx.r33="<<mtx_pole2pole1.R33<<std::endl;
  
    // std::cout<< "pos";
    // for(int k=0;k<6;++k)
    // {
    //     std::cout << pos_p2top1[k] << " ";
    // }
    // std::cout << std::endl;

    //处理杆件点
    for(int i=0;i<3;++i)
    {
        in_p1[i] = cur_pole->p[i];
        in_p2[i] = cur_pole->p[i+3];
        in_p3[i] = target_pole->p[i];
        in_p4[i] = target_pole->p[i+3];
    }
    float out_point[3]; //中间变量，暂存数据

    PointCoordinateC(&mtx_base2pole1,in_p1,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    cur_pole->p_x = out_point[0];
    cur_pole->p_y = out_point[1];
    cur_pole->p_z = out_point[2];
    PointCoordinateC(&mtx_base2pole1,in_p2,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    temp[0] = (out_point[0] - cur_pole->p_x) / pole_rows;
    temp[1] = (out_point[1] - cur_pole->p_y) / pole_rows;
    temp[2] = (out_point[2] - cur_pole->p_z) / pole_rows;
    temp[3] = 360.0 / pole_cols;
    cur_pole->p_w = 0;
    cur_pole->p_p = 0;
    cur_pole->p_r = 0;

    PointCoordinateC(&mtx_base2pole1,in_p3,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    target_pole->p_x = out_point[0];
    target_pole->p_y = out_point[1];
    target_pole->p_z = out_point[2];
    PointCoordinateC(&mtx_base2pole1,in_p4,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    temp[4] = (out_point[0] - target_pole->p_x) / pole_rows;
    temp[5] = (out_point[1] - target_pole->p_y) / pole_rows;
    temp[6] = (out_point[2] - target_pole->p_z) / pole_rows;
    temp[7] = 360.0 / pole_cols;
    target_pole->p_w = pos_p2top1[3];
    target_pole->p_p = pos_p2top1[4];
    target_pole->p_r = pos_p2top1[5];

    AssignmentData(temp, cur_pole, target_pole);

    // std::cout << "赋值后起点 = " << target_pole->p_map[70][64][0] << " " << target_pole->p_map[70][64][1] << " " << target_pole->p_map[70][64][2] << " ";
    // std::cout << target_pole->p_map[70][64][3] << " " << target_pole->p_map[70][64][4] << " " << target_pole->p_map[70][64][5] << std::endl;
    // std::cout << "赋值后终点 = " << target_pole->p_map[127][127][0] << " " << target_pole->p_map[127][127][1] << " " << target_pole->p_map[127][127][2] <<" ";
    // std::cout <<target_pole->p_map[127][127][3] << " " << target_pole->p_map[127][127][4] << " " << target_pole->p_map[127][127][5] << std::endl;

    transitionalTest(cur_pole, target_pole, GripId);
}

Poles Discretepole::memberDisreteforQTree(std::vector<double> &p0,
                                        std::vector<double> &p1,
                                        int length, int alpha, 
                                        const int GripId)
{
    Discretepole();
    for(int i = 0;i<6;i++)
    {
        cur_pole->p[i] = p0[i];
        target_pole->p[i] = p1[i];
    }

    //定义世界坐标系
    Vector World_X,World_Y, World_Z; //世界坐标系向量
    World_X.X = 1;World_X.Y = 0;World_X.Z = 0;
    World_Y.X = 0;World_Y.Y = 1;World_Y.Z = 0;
    World_Z.X = 0;World_Z.Y = 0;World_Z.Z = 1;

    //G7为基座，y轴和z轴反转
    MtxKine mtx_G0_G7;
    mtx_G0_G7.R11 = 1.0;mtx_G0_G7.R12 = 0.0;mtx_G0_G7.R13 = 0.0;
    mtx_G0_G7.R21 = 0.0;mtx_G0_G7.R22 = -1.0;mtx_G0_G7.R23 = 0.0;
    mtx_G0_G7.R31 = 0.0;mtx_G0_G7.R32 = 0.0;mtx_G0_G7.R33 = -1.0;
    mtx_G0_G7.X =0.0;   mtx_G0_G7.Y = 0.0;     mtx_G0_G7.Z = 0.0;  //基座交替旋转矩阵

    //float r = 24.0;
    float temp[8];  //中间变量

    //杆件向量初始化
    cur_pole->pole_v.X = cur_pole->p[3] - cur_pole->p[0];
    cur_pole->pole_v.Y = cur_pole->p[4] - cur_pole->p[1];
    cur_pole->pole_v.Z = cur_pole->p[5] - cur_pole->p[2];
    target_pole->pole_v.X = target_pole->p[3] - target_pole->p[0];
    target_pole->pole_v.Y = target_pole->p[4] - target_pole->p[1];
    target_pole->pole_v.Z = target_pole->p[5] - target_pole->p[2];
//**************************************************************************************************
    //起点设定
    float start_p[6];
    double res_angle;
    setStartPos(cur_pole, length, alpha, start_p, res_angle);

    // std::cout <<"res_angle =" <<res_angle << std::endl;

    MtxKine mtx_rotx;
    mtx_rotx.R11 = 1.0;mtx_rotx.R12 = 0.0;mtx_rotx.R13 = 0.0;
    mtx_rotx.R21 = 0.0;mtx_rotx.R22 = cos(res_angle);mtx_rotx.R23 = sin(res_angle);
    mtx_rotx.R31 = 0.0;mtx_rotx.R32 = -sin(res_angle);mtx_rotx.R33 = cos(res_angle);
    mtx_rotx.X = 0.0;            mtx_rotx.Y = 0.0;            mtx_rotx.Z = 0.0;  //起点姿态设置
//**************************************************************************************************
    //20201011修改，以沿重力方向的矢量作为假设
    Vector Pole1_X = Norm_Vec(cur_pole->pole_v);  //20200723修正错误，没有考虑所有情况
    Vector Pole1_Y,Pole1_Z;

    if(cur_pole->p[0] == cur_pole->p[3] && cur_pole->p[1] == cur_pole->p[4])    //杆件平行于世界坐标系Z轴
    {
        Pole1_Y.X = 1;
        Pole1_Y.Y = 0;
        Pole1_Y.Z = 0;
        Pole1_Z.X = 0;
        Pole1_Z.Y = 1;
        Pole1_Z.Z = 0;
    }
    else
    {
        Pole1_Y = operator&(Pole1_X, World_Z);
        Pole1_Y = Norm_Vec(Pole1_Y);//单位向量
        Pole1_Z = operator&(Pole1_X,Pole1_Y);
        Pole1_Z = Norm_Vec(Pole1_Z);//单位向量
    }
    Vector Pole2_X = Norm_Vec(target_pole->pole_v);
    Vector Pole2_Y,Pole2_Z;
    if(target_pole->p[0] == target_pole->p[3] && target_pole->p[1] == target_pole->p[4])
    {
        Pole2_Y.X = 1;
        Pole2_Y.Y = 0;
        Pole2_Y.Z = 0;
        Pole2_Z.X = 0;
        Pole2_Z.Y = 1;
        Pole2_Z.Z = 0;
    }
    else
    {
        //Pole2_Y.X = 100; Pole2_Y.Y = 100;
        //Pole2_Y.Z = -(Pole2_Y.X * Pole2_X.X + Pole2_Y.Y * Pole2_X.Y) / Pole2_X.Z;
        Pole2_Y = operator&(Pole2_X, World_Z);
        Pole2_Y = Norm_Vec(Pole2_Y);//单位向量
        //std::cout<<"Pole2_Y x="<<Pole2_Y.X<<"y"<<Pole2_Y.Y<<"z"<<Pole2_Y.Z<<std::endl;
        Pole2_Z = operator&(Pole2_X,Pole2_Y);
        Pole2_Z = Norm_Vec(Pole2_Z);//单位向量

    }
    //基座坐标系→杆件坐标系
    MtxKine mtx_base2pole1;
    mtx_base2pole1.R11 = Pole1_X.X;mtx_base2pole1.R12 = Pole1_X.Y;mtx_base2pole1.R13 = Pole1_X.Z;
    mtx_base2pole1.R21 = Pole1_Y.X;mtx_base2pole1.R22 = Pole1_Y.Y;mtx_base2pole1.R23 = Pole1_Y.Z;
    mtx_base2pole1.R31 = Pole1_Z.X;mtx_base2pole1.R32 = Pole1_Z.Y;mtx_base2pole1.R33 = Pole1_Z.Z;
    // mtx_base2pole1.R11 = Pole1_X.X;mtx_base2pole1.R12 = Pole1_Y.X;mtx_base2pole1.R13 = Pole1_Z.X;
    // mtx_base2pole1.R21 = Pole1_X.Y;mtx_base2pole1.R22 = Pole1_Y.Y;mtx_base2pole1.R23 = Pole1_Z.Y;
    // mtx_base2pole1.R31 = Pole1_X.Z;mtx_base2pole1.R32 = Pole1_Y.Z;mtx_base2pole1.R33 = Pole1_Z.Z;
    // mtx_base2pole1.X = -pole1.p[0];mtx_base2pole1.Y = -pole1.p[1];mtx_base2pole1.Z = -pole1.p[2];
    /*
    MtxKine mtx_base2pole2;
    mtx_base2pole2.R11 = Pole2_X.X;mtx_base2pole2.R12 = Pole2_X.Y;mtx_base2pole2.R13 = Pole2_X.Z;
    mtx_base2pole2.R21 = Pole2_Y.X;mtx_base2pole2.R22 = Pole2_Y.Y;mtx_base2pole2.R23 = Pole2_Y.Z;
    mtx_base2pole2.R31 = Pole2_Z.X;mtx_base2pole2.R32 = Pole2_Z.Y;mtx_base2pole2.R33 = Pole2_Z.Z;
    mtx_base2pole2.X = -pole2.p[0];mtx_base2pole2.Y = -pole2.p[1];mtx_base2pole2.Z = -pole2.p[2];
    */

    //Trans_MtxToPos(&mtx_rotx,pos_p1tobase);
    //以起点为基座坐标系
    mtx_base2pole1.X = -start_p[0];mtx_base2pole1.Y = -start_p[1];mtx_base2pole1.Z = -start_p[2];
   
    float in_p1[3],in_p2[3]; //杆件1端点在基座坐标系坐标
    float in_p3[3],in_p4[3]; //杆件2端点在基座坐标系坐标

    //姿态求解，此时以杆件1上夹持器所在坐标系为坐标系，求解出杆2上夹持器坐标系的旋转矩阵
    float in_vecx[3] = {float(Pole2_X.X), float(Pole2_X.Y), float(Pole2_X.Z)},
          in_vecy[3] = {float(Pole2_Y.X), float(Pole2_Y.Y), float(Pole2_Y.Z)},
          in_vecz[3] = {float(Pole2_Z.X), float(Pole2_Z.Y), float(Pole2_Z.Z)};
    float out_vecx[3],out_vecy[3],out_vecz[3];
    float world_zero[3]={0.0,0.0,0.0};

    PointCoordinateC(&mtx_base2pole1,world_zero,world_zero);
    PointCoordinateC(&mtx_base2pole1,in_vecx,in_vecx);
    PointCoordinateC(&mtx_base2pole1,in_vecy,in_vecy);
    PointCoordinateC(&mtx_base2pole1,in_vecz,in_vecz);
    PointCoordinateC(&mtx_rotx,world_zero,world_zero);
    PointCoordinateC(&mtx_rotx,in_vecx,in_vecx);
    PointCoordinateC(&mtx_rotx,in_vecy,in_vecy);
    PointCoordinateC(&mtx_rotx,in_vecz,in_vecz); //左乘变换矩阵顺序

    for(int i = 0;i<3;++i)
    {
        out_vecx[i] = in_vecx[i] - world_zero[i];
        out_vecy[i] = in_vecy[i] - world_zero[i];
        out_vecz[i] = in_vecz[i] - world_zero[i];
    }

    MtxKine mtx_pole2pole1; //此时杆件1上夹持点固结基座标系
    double pos_p2top1[6] = {0,0,0,0,0,0};
    // mtx_pole2pole1.R11 = out_vecx[0];mtx_pole2pole1.R12 = out_vecx[1];mtx_pole2pole1.R13 = out_vecx[2];
    // mtx_pole2pole1.R21 = out_vecy[0];mtx_pole2pole1.R22 = out_vecy[1];mtx_pole2pole1.R23 = out_vecy[2];
    // mtx_pole2pole1.R31 = out_vecz[0];mtx_pole2pole1.R32 = out_vecz[1];mtx_pole2pole1.R33 = out_vecz[2];
    mtx_pole2pole1.R11 = out_vecx[0];mtx_pole2pole1.R12 = out_vecy[0];mtx_pole2pole1.R13 = out_vecz[0];
    mtx_pole2pole1.R21 = out_vecx[1];mtx_pole2pole1.R22 = out_vecy[1];mtx_pole2pole1.R23 = out_vecz[1];
    mtx_pole2pole1.R31 = out_vecx[2];mtx_pole2pole1.R32 = out_vecy[2];mtx_pole2pole1.R33 = out_vecz[2];
    mtx_pole2pole1.X = 0;            mtx_pole2pole1.Y = 0;            mtx_pole2pole1.Z = 0;  //xyz不作设置
    Trans_MtxToPos(&mtx_pole2pole1,pos_p2top1); //杆件2点在基座坐标系下表示
/*
    std::cout<<"mtx.r11="<<mtx_pole2cur_pole->R11<<"mtx.r12="<<mtx_pole2cur_pole->R12<<"mtx.r13="<<mtx_pole2cur_pole->R13<<std::endl;
    std::cout<<"mtx.r21="<<mtx_pole2cur_pole->R21<<"mtx.r22="<<mtx_pole2cur_pole->R22<<"mtx.r23="<<mtx_pole2cur_pole->R23<<std::endl;
    std::cout<<"mtx.r31="<<mtx_pole2cur_pole->R31<<"mtx.r32="<<mtx_pole2cur_pole->R32<<"mtx.r33="<<mtx_pole2cur_pole->R33<<std::endl;
*/    
    // std::cout<< "pos";
    // for(int k=0;k<6;++k)
    // {
    //     std::cout << pos_p2top1[k] << " ";
    // }
    // std::cout << std::endl;

    //处理杆件点
    for(int i=0;i<3;++i)
    {
        in_p1[i] = cur_pole->p[i];
        in_p2[i] = cur_pole->p[i+3];
        in_p3[i] = target_pole->p[i];
        in_p4[i] = target_pole->p[i+3];
    }
    float out_point[3]; //中间变量，暂存数据
    PointCoordinateC(&mtx_base2pole1,in_p1,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    if(GripId == 7)
    {
        PointCoordinateC(&mtx_G0_G7,out_point,out_point);
    }

    cur_pole->p_x = out_point[0];
    cur_pole->p_y = out_point[1];
    cur_pole->p_z = out_point[2];
    PointCoordinateC(&mtx_base2pole1,in_p2,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    if(GripId == 7)
    {
        PointCoordinateC(&mtx_G0_G7,out_point,out_point);
    }

    temp[0] = (out_point[0] - cur_pole->p_x) / pole_rows;
    temp[1] = (out_point[1] - cur_pole->p_y) / pole_rows;
    temp[2] = (out_point[2] - cur_pole->p_z) / pole_rows;
    temp[3] = 360.0 / pole_cols;
    cur_pole->p_w = 0;
    cur_pole->p_p = 0;
    cur_pole->p_r = 0;

    PointCoordinateC(&mtx_base2pole1,in_p3,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    if(GripId == 7)
    {
        PointCoordinateC(&mtx_G0_G7,out_point,out_point);
    }

    target_pole->p_x = out_point[0];
    target_pole->p_y = out_point[1];
    target_pole->p_z = out_point[2];

    PointCoordinateC(&mtx_base2pole1,in_p4,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    if(GripId == 7)
    {
        PointCoordinateC(&mtx_G0_G7,out_point,out_point);
    }

    temp[4] = (out_point[0] - target_pole->p_x) / pole_rows;
    temp[5] = (out_point[1] - target_pole->p_y) / pole_rows;
    temp[6] = (out_point[2] - target_pole->p_z) / pole_rows;
    temp[7] = 360.0 / pole_cols;
    target_pole->p_w = pos_p2top1[3];
    target_pole->p_p = pos_p2top1[4];
    target_pole->p_r = pos_p2top1[5];

    AssignmentData(temp, cur_pole, target_pole);

    return *target_pole;
}


int transwithIK(const std::vector<double> &p0,const std::vector<double> &p1,const double length_p1,const double alpha_p1,const double length_p2,const double alpha_p2,const int GripId,double *out_gdjpos)
{
    Poles pole1; 
    Poles pole2;

    for(int i = 0;i<6;i++)
    {
        pole1.p[i] = p0[i];
        pole2.p[i] = p1[i];
    }
    //展开数组初始化
    pole1.p_map = new double**[pole_rows] ;
    pole2.p_map = new double**[pole_rows] ;
    for( int i = 0 ; i < pole_rows ; ++i )
    {
        pole1.p_map[i] = new double*[pole_cols] ;
        pole2.p_map[i] = new double*[pole_cols] ;
    }
    for( int i = 0 ; i < pole_rows ; ++i )
    {
        for( int j = 0 ; j < pole_cols ; ++j )
        {
            pole1.p_map[i][j] = new double[6] ;
            pole2.p_map[i][j] = new double[6] ;
        }
    }

    //定义世界坐标系
    Vector World_X,World_Y, World_Z; //世界坐标系向量
    World_X.X = 1;World_X.Y = 0;World_X.Z = 0;
    World_Y.X = 0;World_Y.Y = 1;World_Y.Z = 0;
    World_Z.X = 0;World_Z.Y = 0;World_Z.Z = 1;
    float temp[8];  //中间变量

    //杆件向量初始化
    pole1.pole_v.X = pole1.p[3] - pole1.p[0];
    pole1.pole_v.Y = pole1.p[4] - pole1.p[1];
    pole1.pole_v.Z = pole1.p[5] - pole1.p[2];
    pole2.pole_v.X = pole2.p[3] - pole2.p[0];
    pole2.pole_v.Y = pole2.p[4] - pole2.p[1];
    pole2.pole_v.Z = pole2.p[5] - pole2.p[2];

    // pole1.p_length = sqrt(pole1.pole_v.X * pole1.pole_v.X + pole1.pole_v.Y * pole1.pole_v.Y + pole1.pole_v.Z * pole1.pole_v.Z);  //展开地图长度
    // pole2.p_length = sqrt(pole2.pole_v.X * pole2.pole_v.X + pole2.pole_v.Y * pole2.pole_v.Y + pole2.pole_v.Z * pole2.pole_v.Z);
    
//**************************************************************************************************
    //起点设定
    float start_p[6];
    for(int i = 0;i<3;++i)
    {
        start_p[i] = pole1.p[i] + (pole1.p[i+3] - pole1.p[i]) / pole_rows * length_p1;  
        //std::cout<<"start_p["<<i<<"]"<<start_p[i]<<std::endl;
    }   //起点位置设置

    //double alpha = 0.0;   //弧度

    double alpha = -3.1416 + 6.2832 / pole_cols * alpha_p1;

    MtxKine mtx_rotx;
    mtx_rotx.R11 = 1.0;mtx_rotx.R12 = 0.0;mtx_rotx.R13 = 0.0;
    mtx_rotx.R21 = 0.0;mtx_rotx.R22 = cos(alpha);mtx_rotx.R23 = sin(alpha);
    mtx_rotx.R31 = 0.0;mtx_rotx.R32 = -sin(alpha);mtx_rotx.R33 = cos(alpha);
    mtx_rotx.X = 0.0;            mtx_rotx.Y = 0.0;            mtx_rotx.Z = 0.0;  //起点姿态设置
//**************************************************************************************************
    //20201011修改，以沿重力方向的矢量作为假设
    Vector Pole1_X = Norm_Vec(pole1.pole_v);  //20200723修正错误，没有考虑所有情况
    Vector Pole1_Y,Pole1_Z;
    if(pole1.p[0] == pole1.p[3] && pole1.p[1] == pole1.p[4])    //杆件平行于世界坐标系Z轴
    {
        Pole1_Y.X = 1;
        Pole1_Y.Y = 0;
        Pole1_Y.Z = 0;
        Pole1_Z.X = 0;
        Pole1_Z.Y = 1;
        Pole1_Z.Z = 0;
    }
    else
    {
        Pole1_Y = operator&(Pole1_X, World_Z);
        Pole1_Y = Norm_Vec(Pole1_Y);//单位向量
        Pole1_Z = operator&(Pole1_X,Pole1_Y);
        Pole1_Z = Norm_Vec(Pole1_Z);//单位向量
    }
    Vector Pole2_X = Norm_Vec(pole2.pole_v);
    Vector Pole2_Y,Pole2_Z;
    if(pole2.p[0] == pole2.p[3] && pole2.p[1] == pole2.p[4])
    {
        Pole2_Y.X = 1;
        Pole2_Y.Y = 0;
        Pole2_Y.Z = 0;
        Pole2_Z.X = 0;
        Pole2_Z.Y = 1;
        Pole2_Z.Z = 0;
    }
    else
    {
        Pole2_Y = operator&(Pole2_X, World_Z);
        Pole2_Y = Norm_Vec(Pole2_Y);//单位向量
        Pole2_Z = operator&(Pole2_X,Pole2_Y);
        Pole2_Z = Norm_Vec(Pole2_Z);//单位向量
    }
    
    //基座坐标系→杆件坐标系
    MtxKine mtx_base2pole1;
    mtx_base2pole1.R11 = Pole1_X.X;mtx_base2pole1.R12 = Pole1_X.Y;mtx_base2pole1.R13 = Pole1_X.Z;
    mtx_base2pole1.R21 = Pole1_Y.X;mtx_base2pole1.R22 = Pole1_Y.Y;mtx_base2pole1.R23 = Pole1_Y.Z;
    mtx_base2pole1.R31 = Pole1_Z.X;mtx_base2pole1.R32 = Pole1_Z.Y;mtx_base2pole1.R33 = Pole1_Z.Z;
    mtx_base2pole1.X = -pole1.p[0];mtx_base2pole1.Y = -pole1.p[1];mtx_base2pole1.Z = -pole1.p[2];

    //以起点为基座坐标系
    mtx_base2pole1.X = -start_p[0];mtx_base2pole1.Y = -start_p[1];mtx_base2pole1.Z = -start_p[2];
   
    float in_p1[3],in_p2[3]; //杆件1端点在基座坐标系坐标
    float in_p3[3],in_p4[3]; //杆件2端点在基座坐标系坐标

    //姿态求解，此时以杆件1上夹持器所在坐标系为坐标系，求解出杆2上夹持器坐标系的旋转矩阵
    float in_vecx[3] = {float(Pole2_X.X),float(Pole2_X.Y),float(Pole2_X.Z)},in_vecy[3]={float(Pole2_Y.X),float(Pole2_Y.Y),float(Pole2_Y.Z)},in_vecz[3]={float(Pole2_Z.X),float(Pole2_Z.Y),float(Pole2_Z.Z)},out_vecx[3],out_vecy[3],out_vecz[3];
    float world_zero[3]={0.0,0.0,0.0};

    PointCoordinateC(&mtx_base2pole1,world_zero,world_zero);
    PointCoordinateC(&mtx_base2pole1,in_vecx,in_vecx);
    PointCoordinateC(&mtx_base2pole1,in_vecy,in_vecy);
    PointCoordinateC(&mtx_base2pole1,in_vecz,in_vecz);
    PointCoordinateC(&mtx_rotx,world_zero,world_zero);
    PointCoordinateC(&mtx_rotx,in_vecx,in_vecx);
    PointCoordinateC(&mtx_rotx,in_vecy,in_vecy);
    PointCoordinateC(&mtx_rotx,in_vecz,in_vecz); //左乘变换矩阵顺序

    for(int i = 0;i<3;++i)
    {
        out_vecx[i] = in_vecx[i] - world_zero[i];
        out_vecy[i] = in_vecy[i] - world_zero[i];
        out_vecz[i] = in_vecz[i] - world_zero[i];
    }

    MtxKine mtx_pole2pole1; //此时杆件1上夹持点固结基座标系
    double pos_p2top1[6] = {0,0,0,0,0,0};
    mtx_pole2pole1.R11 = out_vecx[0];mtx_pole2pole1.R12 = out_vecy[0];mtx_pole2pole1.R13 = out_vecz[0];
    mtx_pole2pole1.R21 = out_vecx[1];mtx_pole2pole1.R22 = out_vecy[1];mtx_pole2pole1.R23 = out_vecz[1];
    mtx_pole2pole1.R31 = out_vecx[2];mtx_pole2pole1.R32 = out_vecy[2];mtx_pole2pole1.R33 = out_vecz[2];
    mtx_pole2pole1.X = 0;            mtx_pole2pole1.Y = 0;            mtx_pole2pole1.Z = 0;  //xyz不作设置
    Trans_MtxToPos(&mtx_pole2pole1,pos_p2top1); //杆件2点在基座坐标系下表示

    double alpha_G0_G7 = 0;
    MtxKine mtx_G0_G7;
    mtx_G0_G7.R11 = 1.0;mtx_G0_G7.R12 = 0.0;mtx_G0_G7.R13 = 0.0;
    mtx_G0_G7.R21 = 0.0;mtx_G0_G7.R22 = cos(alpha_G0_G7);mtx_G0_G7.R23 = sin(alpha_G0_G7);
    mtx_G0_G7.R31 = 0.0;mtx_G0_G7.R32 = -sin(alpha_G0_G7);mtx_G0_G7.R33 = cos(alpha_G0_G7);
    mtx_G0_G7.X = 0.0;   mtx_G0_G7.Y = 0.0;     mtx_G0_G7.Z = 0.0;  //基座交替旋转矩阵

    //处理杆件点
    for(int i=0;i<3;++i)
    {
        in_p1[i] = pole1.p[i];
        in_p2[i] = pole1.p[i+3];
        in_p3[i] = pole2.p[i];
        in_p4[i] = pole2.p[i+3];
    }
    float out_point[3]; //中间变量，暂存数据
    PointCoordinateC(&mtx_base2pole1,in_p1,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    if(GripId == 7)
    {
        PointCoordinateC(&mtx_G0_G7,out_point,out_point);
    }
    pole1.p_x = out_point[0];
    pole1.p_y = out_point[1];
    pole1.p_z = out_point[2];
    PointCoordinateC(&mtx_base2pole1,in_p2,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    if(GripId == 7)
    {
        PointCoordinateC(&mtx_G0_G7,out_point,out_point);
    }
    temp[0] = (out_point[0] - pole1.p_x) / pole_rows;
    temp[1] = (out_point[1] - pole1.p_y) / pole_rows;
    temp[2] = (out_point[2] - pole1.p_z) / pole_rows;
    temp[3] = 360.0 / pole_cols;
    pole1.p_w = 0;
    pole1.p_p = 0;
    pole1.p_r = 0;

    PointCoordinateC(&mtx_base2pole1,in_p3,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    if(GripId == 7)
    {
        PointCoordinateC(&mtx_G0_G7,out_point,out_point);
    }
    pole2.p_x = out_point[0];
    pole2.p_y = out_point[1];
    pole2.p_z = out_point[2];
    PointCoordinateC(&mtx_base2pole1,in_p4,out_point);
    PointCoordinateC(&mtx_rotx,out_point,out_point);
    if(GripId == 7)
    {
        PointCoordinateC(&mtx_G0_G7,out_point,out_point);
    }
    temp[4] = (out_point[0] - pole2.p_x) / pole_rows;
    temp[5] = (out_point[1] - pole2.p_y) / pole_rows;
    temp[6] = (out_point[2] - pole2.p_z) / pole_rows;
    temp[7] = 360.0 / pole_cols;
    pole2.p_w = pos_p2top1[3];
    pole2.p_p = pos_p2top1[4];
    pole2.p_r = pos_p2top1[5];

    //赋值
     for(int i = 0;i < pole_rows;++i)
    {
        for(int j = 0;j < pole_cols;++j)
        {
            for(int k = 0;k < 6;++k)
            {
                pole2.p_map[i][j][0] = pole2.p_x + i * temp[4];
                pole2.p_map[i][j][1] = pole2.p_y + i * temp[5];
                pole2.p_map[i][j][2] = pole2.p_z + i * temp[6];
                pole2.p_map[i][j][3] = pole2.p_w;
                pole2.p_map[i][j][4] = pole2.p_p;
                pole2.p_map[i][j][5] = pole2.p_r + j * temp[7];
                if(pole2.p_map[i][j][5] > 180)
                {
                    pole2.p_map[i][j][5] = pole2.p_map[i][j][5] - 360;
                }
            }
        } 
    }

    //初始化机器人
    // double len[7] = {174.85, 167.2, 379.2, 178.4, 200.8, 167.2, 174.85};
    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
    Kine_CR_SixDoF_G1 climbot6d_G1;
    Kine_CR_SixDoF_G2 climbot6d_G2;
    climbot6d_G1.Set_Length(len);
    climbot6d_G2.Set_Length(len);
    double curgdj[6] = {0,90,90,0,0,0};
    double gdcpos[6] = {pole2.p_map[int(length_p2)][int(alpha_p2)][0],pole2.p_map[int(length_p2)][int(alpha_p2)][1],pole2.p_map[int(length_p2)][int(alpha_p2)][2],pole2.p_map[int(length_p2)][int(alpha_p2)][3],pole2.p_map[int(length_p2)][int(alpha_p2)][4],pole2.p_map[int(length_p2)][int(alpha_p2)][5]};
    
    if(GripId == 7 && climbot6d_G2.IKine(gdcpos,curgdj,out_gdjpos) == 0)
    {
        return 1;
    }
    else if(GripId == 0 && climbot6d_G1.IKine(gdcpos,curgdj,out_gdjpos) == 0)
    {
        return 1;
    }
    else
        return 0;

    return 0;
}

MtxKine getCurBaseMtx(int angle)
{
    double res_angle = -3.1416 + 6.2832 / pole_cols * (angle);
    MtxKine mtx_rotx;       //基座矩阵
    mtx_rotx.R11 = 1.0;mtx_rotx.R21 = 0.0;mtx_rotx.R31 = 0.0;
    mtx_rotx.R12 = 0.0;mtx_rotx.R22 = cos(res_angle);mtx_rotx.R32 = sin(res_angle);
    mtx_rotx.R13 = 0.0;mtx_rotx.R23 = -sin(res_angle);mtx_rotx.R33 = cos(res_angle);
    mtx_rotx.X = 0;            mtx_rotx.Y = 0.0;            mtx_rotx.Z = 0;
    return mtx_rotx;
}

MtxKine getCurMtx(std::vector<double> &cur_truss,int length,int angle)
{
    double start_p[3];
    for(int i = 0;i<3;++i)
    {
        start_p[i] = cur_truss[i] + (cur_truss[i+3] - cur_truss[i]) / pole_rows * (length);
    }   //起点位置设置
    Vector pole_v;
    pole_v.X = cur_truss[3] - cur_truss[0];
    pole_v.Y = cur_truss[4] - cur_truss[1];
    pole_v.Z = cur_truss[5] - cur_truss[2];

    //定义世界坐标系
    Vector World_X,World_Y, World_Z; //世界坐标系向量
    World_X.X = 1;World_X.Y = 0;World_X.Z = 0;
    World_Y.X = 0;World_Y.Y = 1;World_Y.Z = 0;
    World_Z.X = 0;World_Z.Y = 0;World_Z.Z = 1;

    Vector Pole1_X = Norm_Vec(pole_v);  //20200723修正错误，没有考虑所有情况
    Vector Pole1_Y,Pole1_Z;

    if(cur_truss[0] == cur_truss[3] && cur_truss[1] == cur_truss[4])    //杆件平行于世界坐标系Z轴
    {
        Pole1_Y.X = 1;
        Pole1_Y.Y = 0;
        Pole1_Y.Z = 0;
        Pole1_Z.X = 0;
        Pole1_Z.Y = 1;
        Pole1_Z.Z = 0;
    }
    else
    {
        Pole1_Y = operator&(Pole1_X, World_Z);
        Pole1_Y = Norm_Vec(Pole1_Y);//单位向量
        Pole1_Z = operator&(Pole1_X,Pole1_Y);
        Pole1_Z = Norm_Vec(Pole1_Z);//单位向量
    }
    //基座坐标系→杆件坐标系
    MtxKine mtx_base2pole1;
    mtx_base2pole1.R11 = Pole1_X.X;mtx_base2pole1.R21 = Pole1_X.Y;mtx_base2pole1.R31 = Pole1_X.Z;
    mtx_base2pole1.R12 = Pole1_Y.X;mtx_base2pole1.R22 = Pole1_Y.Y;mtx_base2pole1.R32 = Pole1_Y.Z;
    mtx_base2pole1.R13 = Pole1_Z.X;mtx_base2pole1.R23 = Pole1_Z.Y;mtx_base2pole1.R33 = Pole1_Z.Z;
    //以起点为基座坐标系
    mtx_base2pole1.X = start_p[0];mtx_base2pole1.Y = start_p[1];mtx_base2pole1.Z = start_p[2];


    // std::cout << mtx_base2pole1.R11 << " " << mtx_base2pole1.R12 << " "<<mtx_base2pole1.R13 << std::endl;
    // std::cout << mtx_base2pole1.R21 << " " << mtx_base2pole1.R22 << " "<<mtx_base2pole1.R23 << std::endl;
    // std::cout << mtx_base2pole1.R31 << " " << mtx_base2pole1.R32 << " "<<mtx_base2pole1.R33 << std::endl;
    // std::cout << mtx_base2pole1.X << " " << mtx_base2pole1.Y << " "<<mtx_base2pole1.Z << std::endl;

    return mtx_base2pole1;
}

MtxKine getCurMtxfromJointAngle(double* joint_angle,const int Grip_ID)
{
    MtxKine cur_mtx;
    Kine_CR_SixDoF_G1 climbot6d_g1;
    Kine_CR_SixDoF_G2 climbot6d_g2;
    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};

    if(Grip_ID == 0)
    {
        climbot6d_g1.Set_Length(len);
        double cur_pos[6];
        climbot6d_g1.FKine(joint_angle, cur_pos);
        Trans_PosToMtx(cur_pos, &cur_mtx, 0);
        return cur_mtx;
    }
    else if(Grip_ID == 7)
    {
        climbot6d_g2.Set_Length(len);
        double cur_pos[6];
        climbot6d_g2.FKine(joint_angle, cur_pos);
        Trans_PosToMtx(cur_pos, &cur_mtx, 0);
        return cur_mtx;
    }
    else
    {
        std::cout << "Grip_ID error!" << std::endl;
    }
    
    return cur_mtx;
}

void transPointToWorld(float* in_point,std::vector<double> &cur_truss,int length,int angle,float* out_point,const int Grip_ID)
{
    MtxKine mtxbase, curmtx;
    if(Grip_ID == 0)
        mtxbase = getCurBaseMtx(angle);
    else if(Grip_ID == 7)
        mtxbase = getCurBaseMtx(angle - pole_cols / 2);
    else
        std::cout << "Grip_ID error!" << std::endl;


    // std::cout << curmtx.R11 << " " << curmtx.R12 << " "<<curmtx.R13 << std::endl;
    // std::cout << curmtx.R21 << " " << curmtx.R22 << " "<<curmtx.R23 << std::endl;
    // std::cout << curmtx.R31 << " " << curmtx.R32 << " "<<curmtx.R33 << std::endl;
    // std::cout << curmtx.X << " " << curmtx.Y << " "<<curmtx.Z << std::endl;
    curmtx = getCurMtx(cur_truss, length, angle);


    PointCoordinateC_(&mtxbase, in_point, out_point);
    PointCoordinateC_(&curmtx, out_point, out_point);   
    // std::cout << "trans"<<out_point[0] << " " << out_point[1] << " " << out_point[2] << std::endl;
}
