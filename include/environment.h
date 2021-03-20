#ifndef ENVIRONMENT_H__
#define ENVIRONMENT_H__

/*****************************************************************************
 *        桁架测试环境1
 *        GDUT, 2020
 *****************************************************************************/
#include <vector>

// std::vector<double> truss1 = {-510.0,0.0,-490.4,-510.0,3000.0,-490.4};
// std::vector<double> truss2 = {-510.0, 0.0, 2579.2, -510.0, 3000.0, 2579.2};
// std::vector<double> truss3 = {1071.9,0.0,2550.0,1071.9,3000.0,2550.0};
// std::vector<double> truss4 = {1034.2,0.0,-490.4,1034.2,3000.0,-490.4};
// std::vector<double> truss5 = {-510.0,104.4,-490.4,-510.0,104.4,2579.2};
// std::vector<double> truss6 = {-510.0,104.4,2550.0,1071.9,104.4,2550.0};
// std::vector<double> truss7 = {1071.9,121.8,2550.0,1034.2,110.9,-490.4};
// std::vector<double> truss8 = {1034.2,110.9,-490.4,-510.0,96.1,-490.4};
// std::vector<double> truss9 = {-510.0,1650.7,-476.9,-510.0,1656.4,2574.8};
// std::vector<double> truss10 = {-510.0,1656.4,2574.8,1055.3,1672.4,2546.1};
// std::vector<double> truss11 = {1055.3,1672.4,2546.1,1042.0,1662.8,-490.5};
// std::vector<double> truss12 = {1042.0,1662.8,-490.5,-510.0,1650.7,-490.5};
// std::vector<double> truss13 = {-510.0,116.4,1051.1,-510.0,1649.3,1051.1};
// std::vector<double> truss14 = {-510.0,1649.3,1051.1,1054.9,1649.3,1029.7};
// std::vector<double> truss15 = {1054.9,1649.3,1029.7,1054.9,116.4,1034.1};
// std::vector<double> truss16 = {1054.9,116.4,1031.1,-510.0,116.4,1050.6};
// std::vector<std::vector<double>> truss = {truss1, truss2, truss3, truss4, truss5, truss6, truss7, truss8,truss9,truss10,truss11, truss12, truss13, truss14,truss15, truss16 };

// // 测试环境0
// std::vector<double> truss1 = {-510.0,0.0,-490.4,-510.0,3000.0,-490.4};
// std::vector<double> truss2 = {-510.0, 0.0, 2579.2, -510.0, 3000.0, 2579.2};
// std::vector<double> truss3 = {1071.9,0.0,2550.0,1071.9,3000.0,2550.0};
// std::vector<double> truss4 = {1034.2,0.0,-490.4,1034.2,3000.0,-490.4};
// std::vector<double> truss5 = {-510.0,1650.7,-476.9,-510.0,1656.4,2574.8};
// std::vector<double> truss6 = {-510.0,1656.4,2574.8,1055.3,1672.4,2546.1};
// std::vector<double> truss7 = {1042.0,1662.8,-490.5,1055.3,1660,2546.1};
// std::vector<double> truss8 = {-510.0,1650.7,-490.5,1042.0,1662.8,-490.5};
// std::vector<std::vector<double>> truss = {truss1, truss2, truss3, truss4, truss5, truss6, truss7, truss8};

// //测试环境1
// std::vector<double> truss1 = {500,0,-300,500,0,2300};
// std::vector<double> truss2 = {2000,0,2500,-500,300,1800};
// std::vector<double> truss3 = {1500,750 ,1100 ,700, 550, 1300};
// std::vector<double> truss4 = {-200,500,100,1800,1800,1800};
// std::vector<double> truss5 = {1400,-300,1800,1600,2200,2200};
// std::vector<double> truss6 = {1300,1600,200,1000,0,700};
// std::vector<double> truss7 = {700,1800,0,700,1800,2500};
// std::vector<std::vector<double>> truss = {truss1, truss2, truss3, truss4, truss5, truss6, truss7};


// //测试环境2
// std::vector<double> truss1 = {400,100,-300,400,100,2300};
// std::vector<double> truss2 = {1500,200,300,-500,300,1800};
// std::vector<double> truss3 = {1500,50 ,1100 ,1500, 550, 1100};
// std::vector<double> truss4 = {-200,500,100,1800,1800,1800};
// std::vector<double> truss5 = {-500,0,500,-500,2000,500};
// std::vector<double> truss6 = {500,500,200,1000,1600,700};
// std::vector<double> truss7 = {0,1800,1200,1500,1800,1200};
// std::vector<double> truss8 = {700,2000,0,700,2000,2500};
// std::vector<std::vector<double>> truss = {truss1, truss2, truss3, truss4, truss5, truss6, truss7, truss8};

// //测试环境3
// std::vector<double> truss1 = {400,100,-300,400,100,2300};
// std::vector<double> truss2 = {1000,0,2500,1000,3000,2500};
// std::vector<double> truss3 = {-200,500,100,1800,1800,1800};
// std::vector<double> truss4 = {1300,1600,200,1000,0,700};
// std::vector<double> truss5 = {1000,1600,1000,1000,100,1000};
// std::vector<double> truss6 = {-500,1600,-500,-500,1600,2500};
// std::vector<double> truss7 = {0,1800,1200,1500,1800,1200};
// std::vector<double> truss8 = {1200,2000,0,1200,2000,2500};
// std::vector<double> truss9 = {1000,0,500,1000,2000,500};
// std::vector<std::vector<double>> truss = {truss1, truss2, truss3, truss4, truss5, truss6, truss7, truss8, truss9};

//测试环境4
static std::vector<double> truss1 = {500,0,-300,500,0,2300};
static std::vector<double> truss2 = {1500,0,2200,1500,2500,2200};
static std::vector<double> truss3 = {1300,1600,200,1000,0,700};
static std::vector<double> truss4 = {700,2000,0,700,2000,2500};
static std::vector<double> truss5 = {0,1800,700,1500,1800,700};
static std::vector<double> truss6 = {0,2300,1700,1500,2300,1700};
static std::vector<double> truss7 = {2000,2000,0,2000,2000,2000};
static std::vector<double> truss8 = {1000,-500,1000,1000,500,1000};
static std::vector<std::vector<double>> truss = {truss1, truss2, truss3, truss4, truss5, truss6, truss7,truss8};



#endif  //environment.h