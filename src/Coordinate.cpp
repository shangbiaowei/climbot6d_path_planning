#include "../include/Coordinate.h"
#include <iostream>
#include <math.h>

//点的坐标转换
void PointCoordinateC(MtxKine *PUntifyM, float *INPoint, float *OUTPoint)
{
	float temp[3];
	INPoint[0] = PUntifyM->X + INPoint[0];
	INPoint[1] = PUntifyM->Y + INPoint[1];
	INPoint[2] = PUntifyM->Z + INPoint[2]; //先平移，再旋转

	temp[0] = PUntifyM->R11*INPoint[0]+PUntifyM->R12*INPoint[1]+PUntifyM->R13*INPoint[2];//+PUntifyM->X;
	temp[1] = PUntifyM->R21*INPoint[0]+PUntifyM->R22*INPoint[1]+PUntifyM->R23*INPoint[2];//+PUntifyM->Y;
	temp[2] = PUntifyM->R31*INPoint[0]+PUntifyM->R32*INPoint[1]+PUntifyM->R33*INPoint[2];//+PUntifyM->Z;

	OUTPoint[0] = temp[0];
	OUTPoint[1] = temp[1];
	OUTPoint[2] = temp[2];
}

//点的坐标转换(基座->世界)
void PointCoordinateC_(MtxKine *PUntifyM, float *INPoint, float *OUTPoint)
{
	float temp[3];

	temp[0] = PUntifyM->R11*INPoint[0]+PUntifyM->R12*INPoint[1]+PUntifyM->R13*INPoint[2]+PUntifyM->X;
	temp[1] = PUntifyM->R21*INPoint[0]+PUntifyM->R22*INPoint[1]+PUntifyM->R23*INPoint[2]+PUntifyM->Y;
	temp[2] = PUntifyM->R31*INPoint[0]+PUntifyM->R32*INPoint[1]+PUntifyM->R33*INPoint[2]+PUntifyM->Z;

	OUTPoint[0] = temp[0];
	OUTPoint[1] = temp[1];
	OUTPoint[2] = temp[2];
}


//向量的坐标转换
void VectorCoordinateC(MtxKine *PUntifyM, float INVector[], float OUTVector[])
{
	float temp[3];
	INVector[0] = PUntifyM->X + INVector[0];
	INVector[1] = PUntifyM->Y + INVector[1];
	INVector[2] = PUntifyM->Z + INVector[2]; //先平移，再旋转
	temp[0] = PUntifyM->R11*INVector[0]+PUntifyM->R12*INVector[1]+PUntifyM->R13*INVector[2];//+PUntifyM->X;
	temp[1] = PUntifyM->R21*INVector[0]+PUntifyM->R22*INVector[1]+PUntifyM->R23*INVector[2];//+PUntifyM->Y;
	temp[2] = PUntifyM->R31*INVector[0]+PUntifyM->R32*INVector[1]+PUntifyM->R33*INVector[2];//+PUntifyM->Z;
	OUTVector[0] = temp[0];
	OUTVector[1] = temp[1];
	OUTVector[2] = temp[2];
}
