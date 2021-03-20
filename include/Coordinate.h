#ifndef Coordinate_H__
#define Coordinate_H__

#include<cmath>
#include"m_Matrix.h"

//点的坐标转换
void PointCoordinateC(MtxKine *PUntifyM, float *INPoint, float *OUTPoint);
void PointCoordinateC_(MtxKine *PUntifyM, float *INPoint, float *OUTPoint);

//向量的坐标转换
void VectorCoordinateC(MtxKine *PUntifyM, float INVector[], float OUTVector[]);

//平移转换
//void transl(float *INPoint,float *OUTPoint);


#endif