// #include <iostream>
// #include <vector>
// #include <time.h>
// #include <cmath>
// #include "../include/globalpath.h"
// #include "../include/grippointopti.h"
// #include "../include/pathallocate.h"
// #include "../include/gaitplan.h"
// #include "../include/generateSimData.h"
// // #include <boost/test/included/unit_test.hpp>


// using namespace std;

// int main()
// {
//     clock_t t_start,t_finish;
//     double totaltime;
//     t_start=clock();    //计算程序运行时间
//     //起点：0号杆，50,64
//     //终点：5号杆，50,32
//     std::vector<int> start_point = {30, 32};
//     std::vector<int> end_point = {50, 32};
//     // // 全局路径
//     // std::vector<std::vector<int>> adj_mat;
//     // globalPathPlanStart(start_point, end_point, 0, 6,adj_mat);

//     // std::vector<int> truss_list = {1,2,7};
//     // std::vector<std::vector<int> > grippoint_list;
//     // getTransTrussList(start_point,end_point,truss_list, grippoint_list);
//     // // for (int i = 0; i < grippoint_list.size();++i)
//     // // {
//     // //     for (int j = 0; j < grippoint_list[0].size();++j)
//     // //     {
//     // //         cout << grippoint_list[i][j] << " ";
//     // //     } 
//     // //     cout << endl;
//     // // }
//     // std::vector<int> single_num;
//     // std::vector<std::vector<int>> opti_res;
//     // GripOpti(truss_list,grippoint_list,adj_mat,single_num,opti_res);
//     // // for (int i = 0; i < opti_res.size();++i)
//     // // {
//     // //     for (int j = 0; j < opti_res[0].size();++j)
//     // //     {
//     // //         std::cout << opti_res[i][j] << " ";
//     // //     }
//     // //     std::cout << std::endl;
//     // // }
//     // for (int i = 0; i < single_num.size();++i)
//     // {
//     //     std::cout << single_num[i] << " ";
//     // }
//     // std::cout << std::endl;



//     // PathAllocate test1;
//     // test1.PathAllocateInit(adj_mat, opti_res, single_num);
//     // test1.allocatePath(truss,truss_list);

//     // SimDataGenerate generater;
//     // generater.generatePathTxt(test1.final_path_list,truss,truss_list,single_num);

    

//     // std::vector<double> joint_val;
//     // Discretepole test;
//     // // // // test.poleTransition(truss1, truss3, 30, 32,0);
//     // test.simulationResultPrint(truss1, truss1,58,29,30,32, 0,joint_val);    //反(第一步)
 
 


//     // //  初始化机器人
//     // std::cout<<"机器人正逆运动学测试"<<std::endl;
//     // double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
//     // Kine_CR_SixDoF_G1 climbot6d_G1;
//     // Kine_CR_SixDoF_G2 climbot6d_G2;
//     // climbot6d_G1.Set_Length(len);
//     // climbot6d_G2.Set_Length(len);
//     // // double gdcPos[6] = {0, -500, -900, 90, 0, -90 };
//     // double gdPos[6] = {0,0,0,0,0,0};
//     // double out_gdjpos[6];

//     // climbot6d_G2.FKine(gdPos, out_gdjpos);
//     // for (int i = 0; i < 6;++i)
//     // {
//     //     std::cout << out_gdjpos[i] << " ";
//     // }


//     // climbot6d_G1.IKine(gdcPos,gdPos,out_gdjpos);
//     // // for (int i = 0; i < 6;++i)
//     // // {
//     // //     std::cout << out_gdjpos[i] << " ";
//     // // }

//     // std::cout << -out_gdjpos[5] << ",";     
//     // std::cout <<out_gdjpos[4]<< ",";  
//     // std::cout <<-out_gdjpos[3] << ",";   
//     // std::cout <<(out_gdjpos[2] -  90)<< ",";      
//     // std::cout << -(out_gdjpos[1] - 90) << ",";
//     // std::cout << out_gdjpos[0]<< ",";
//     // std::cout << ";"<<std::endl;
//     // // test.simulationResultPrint(truss3, truss1,70,64,30,32, 0);    //反(第一步) z轴往后退 -100
//     // // 位姿 =745.036 491.436 400.463 0 72.9276 -79.3862 




//     t_finish = clock();
//     totaltime=(double)(t_finish-t_start)/CLOCKS_PER_SEC;
//     std::cout<<"\n此程序的运行时间为"<<totaltime<<"秒"<<std::endl;  //输出程序运行时间


//     return 0;
// }


#define BOOST_TEST_MODULE stringtest
#include <boost/test/included/unit_test.hpp>
#include "./str.h"

BOOST_AUTO_TEST_SUITE (stringtest) //测试套件开始


BOOST_AUTO_TEST_CASE (test1)       //测试用例1
{
  mystring s;
  BOOST_CHECK(s.size() == 0);
}

BOOST_AUTO_TEST_CASE (test2)       //测试用例2
{
  mystring s;
  s.setbuffer("hello world");
  BOOST_REQUIRE_EQUAL ('h', s[0]); // basic test

}

BOOST_AUTO_TEST_SUITE_END( )     //测试套件结束