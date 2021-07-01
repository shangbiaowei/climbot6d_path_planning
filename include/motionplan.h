#ifndef MOTIONPLAN_H__
#define MOTIONPLAN_H__

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>


#include <iostream>
#include <fstream>
#include <ostream>
#include <boost/bind.hpp> //绑定函数

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "Kine.h"
#include "environment.h"
#include "gpoptidata.h"

using namespace std;

//声明全局变量
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

class motionPlan
{
	public:
		void setStartPos(double *start_jpos,const int GripID);
		void setGoalPos(double *goal_jpos,const int GripID);

		motionPlan();
		~motionPlan();


		void replan(std::vector<std::vector<double> > &path_list,const int GripID);
		void plan(std::vector<std::vector<double> > &path_list,std::vector<std::vector<int>>& adjmat,const int GripID,
				int cur_truss_number,int tar_truss_number,
				std::vector<double> &cur_truss_input,std::vector<double> &tar_truss_input,
				int length,int angle);

		void getFinalJointValue(std::vector<std::vector<double>> &path_list,const int Grip_ID,std::vector<std::vector<double>> &joint_val);

		void getAidPathPoint(double *gdPos,const int Grip_ID,double z_distance,double *pos_res);

		double pos_first[6];	//添加路径辅助点前的第一个点
		double pos_final[6];	//添加路径辅助点后的最后一个点

		bool replan_flag = false;

	protected:
		void paraInit(std::vector<std::vector<int>>& adj_mat,const int GripID,
				            int cur_truss_number,int tar_truss_number,
				            std::vector<double> &cur_truss_input,std::vector<double> &tar_truss_input,
							int length,int angle);

		double cur_jpos[6] = {0,90,90,0,0,0};	//机器人当前关节角度，初始值为0，90，90，0，0，0

		int Grip_ID;	//夹持端标志位，0为G1，7为G2

		std::vector<std::vector<int>> adjmat;	//邻接矩阵

		int cur_truss_num;
		int tar_truss_num;	//夹持端杆件编号和目标杆件编号

		int cur_length;
		int cur_angle;	//当前机器人基座和转角

		std::vector<double> cur_truss;
		std::vector<double> tar_truss;	//夹持端杆件参数和目标杆件参数

		int global_count = 0;	//全局计数器

	private:
		//声明我们规划所在的空间维度
		ob::StateSpacePtr space;
		//从此状态空间构造空间信息的实例
		ob::SpaceInformationPtr si;
		//生成问题实例
		ob::ProblemDefinitionPtr pdef;

		og::PathGeometric *path_smooth;


		bool isStateValid(const ob::State *state);

		bool jointconstrain(std::vector<std::vector<double>> &path_list, const int Grip_ID);

		ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
		{
			ob::OptimizationObjectivePtr obj(new ob::MechanicalWorkOptimizationObjective(si));
			obj->setCostThreshold(ob::Cost(10));
			return obj;
		};


		ob::OptimizationObjectivePtr getMechWork(const ob::SpaceInformationPtr &si)
		{
			ob::OptimizationObjectivePtr obj(new ob::MechanicalWorkOptimizationObjective(si));
			// obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
			return obj;
		};

		Kine_CR_SixDoF_G1 climbot6d_G1;
		Kine_CR_SixDoF_G2 climbot6d_G2;
};

class motionPlanJointState
{
	public:
		motionPlanJointState();
		~motionPlanJointState();

		void setStartPos(double *start_jpos,const int GripID);
		void setGoalPos(double *goal_jpos,const int GripID);

		void plan(std::vector<std::vector<double> > &path_list,std::vector<std::vector<int>>& adjmat,const int GripID,
				int cur_truss_number,int tar_truss_number,
				std::vector<double> &cur_truss_input,std::vector<double> &tar_truss_input,
				int length,int angle);

		void propagate(const ob::State *start_state, const oc::Control *control, const double duration, ob::State *result_state);

	protected:
		void paraInit(std::vector<std::vector<int>>& adj_mat,const int GripID,
				            int cur_truss_number,int tar_truss_number,
				            std::vector<double> &cur_truss_input,std::vector<double> &tar_truss_input,
							int length,int angle);


		double cur_jpos[6] = {0,90,90,0,0,0};	//机器人当前关节角度，初始值为0，90，90，0，0，0

		int Grip_ID;	//夹持端标志位，0为G1，7为G2

		std::vector<std::vector<int>> adjmat;	//邻接矩阵

		int cur_truss_num;
		int tar_truss_num;	//夹持端杆件编号和目标杆件编号

		int cur_length;
		int cur_angle;	//当前机器人基座和转角

		std::vector<double> cur_truss;
		std::vector<double> tar_truss;	//夹持端杆件参数和目标杆件参数

	private:
		bool isStateValid(const oc::SpaceInformation *si, const ob::State *state);

		Kine_CR_SixDoF_G1 climbot6d_G1;
		Kine_CR_SixDoF_G2 climbot6d_G2;

		//声明我们规划所在的空间维度
		ob::StateSpacePtr space;

		//从此状态空间构造空间信息的实例
		// ob::SpaceInformationPtr si;
		oc::SpaceInformationPtr si;
		//生成问题实例
		ob::ProblemDefinitionPtr pdef;

		og::PathGeometric *path_smooth;
};

std::vector<std::vector<double>> getCurJointPoint_ompl(std::vector<double> &cur_truss, std::vector<double> &tar_truss,
													   const int length1, const int angle1, double *cur_pos,double *cur_jpos,double *out_gdjpos,const int Grip_ID);


void singleStepPlanner(std::vector<std::vector<double>> &joint_val_list,
						std::vector<std::vector<int>> &adj_mat,
						std::vector<std::vector<int>> &final_path_list,
						std::vector<int> &truss_list,
						std::vector<int> &single_num);	//单步规划器

#endif	//motionplan.h