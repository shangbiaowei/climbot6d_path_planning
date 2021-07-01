#include "../include/motionplan.h"


motionPlan::motionPlan()
{
    //初始化机器人
    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
    climbot6d_G1.Set_Length(len);
    climbot6d_G2.Set_Length(len);

    //状态空间
    space = ob::StateSpacePtr(new ob::SE3StateSpace());

    //设置起点终点
    ob::ScopedState<ob::SE3StateSpace> start(space);
    ob::ScopedState<ob::SE3StateSpace> goal(space);
    
    //搜索的三维范围设置
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1630);
    bounds.setHigh(1630);
    space->as<ob::SE3StateSpace>()->setBounds(bounds);

    //从此状态空间构造空间信息的实例
    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

    // setStartPos(pos_start);
    // setGoalPos(pos_end);

    start->setXYZ(0,0,0);
    start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    // start.random();
    goal->setXYZ(0,0,0);
    goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    // goal.random();

    si->setStateValidityChecker(std::bind(&motionPlan::isStateValid, this, std::placeholders::_1));

    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

    pdef->setStartAndGoalStates(start, goal);

    // pdef->setOptimizationObjective(motionPlan::getPathLengthObjWithCostToGo(si));

    // pdef->setOptimizationObjective(motionPlan::getMechWork(si));

    std::cout << "Initialize: " << std::endl;

}

motionPlan::~motionPlan()
{
    
}

void motionPlan::setStartPos(double *start_jpos,const int GripID)
{
    double gdcpos[6];

    if (GripID == 0)
    {
        climbot6d_G1.FKine(start_jpos, gdcpos);
    }
    else if(GripID == 7)
    {
        climbot6d_G2.FKine(start_jpos, gdcpos);
    }

    ob::ScopedState<ob::SE3StateSpace> start(space);
    start->setXYZ(gdcpos[0], gdcpos[1], gdcpos[2]);	
	double yaw = gdcpos[3] * 0.0174532925199,pitch = gdcpos[4] * 0.0174532925199,droll = gdcpos[5] * 0.0174532925199;
    //EulerAngles to RotationMatrix
    ::Eigen::Vector3d ea0(yaw, pitch, droll);
    	::Eigen::Matrix3d R; 
    	R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d ::UnitZ())
        * ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
        * ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
    ::Eigen::Quaterniond q; 
    q = R;
	// start->as<ob::SO3StateSpace::StateType>(1)
    // cout << q.x() << endl << endl;
    // cout << q.y() << endl << endl;
    // cout << q.z() << endl << endl;
    // cout << q.w() << endl << endl;

	start->as<ob::SO3StateSpace::StateType>(1)->x = q.x();
	start->as<ob::SO3StateSpace::StateType>(1)->y = q.y();
	start->as<ob::SO3StateSpace::StateType>(1)->z = q.z();
	start->as<ob::SO3StateSpace::StateType>(1)->w = q.w();

    pdef->clearStartStates();
    pdef->addStartState(start);
}

void motionPlan::setGoalPos(double *goal_jpos,const int GripID)
{
    double gdcpos[6];
    if (GripID == 0)
    {
        climbot6d_G1.FKine(goal_jpos, gdcpos);
    }
    else if(GripID == 7)
    {
        climbot6d_G2.FKine(goal_jpos, gdcpos);
    }

    ob::ScopedState<ob::SE3StateSpace> goal(space);
	goal->setXYZ(gdcpos[0], gdcpos[1], gdcpos[2]);	
	double yaw = gdcpos[3] * 0.0174532925199,pitch = gdcpos[4] * 0.0174532925199,droll = gdcpos[5] * 0.0174532925199;
    //EulerAngles to RotationMatrix
    ::Eigen::Vector3d ea0(yaw, pitch, droll);
    ::Eigen::Matrix3d R;
    R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
        * ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
        * ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
    ::Eigen::Quaterniond q; 
    q = R;
	// start->as<ob::SO3StateSpace::StateType>(1)
    // cout << q.x() << endl << endl;
    // cout << q.y() << endl << endl;
    // cout << q.z() << endl << endl;
    // cout << q.w() << endl << endl;

	goal->as<ob::SO3StateSpace::StateType>(1)->x = q.x();
	goal->as<ob::SO3StateSpace::StateType>(1)->y = q.y();
	goal->as<ob::SO3StateSpace::StateType>(1)->z = q.z();
	goal->as<ob::SO3StateSpace::StateType>(1)->w = q.w();

    pdef->clearGoal();
    pdef->setGoalState(goal);

    // std::cout << "goal set to: " << goal << std::endl;
}


bool motionPlan::jointconstrain(std::vector<std::vector<double>> &path_list, const int Grip_ID)
{
    double pre_joint[6] = {0,90,90,0,0,0};

    for (size_t i = 0; i < path_list.size();++i)
    {
        ::Eigen::Quaterniond q;
        q.x() = path_list[i][3];
        q.y() = path_list[i][4];
        q.z() = path_list[i][5];
        q.w() = path_list[i][6];
        Eigen::Vector3d eulerAngle=q.matrix().eulerAngles(2,1,0);	//四元数转换成欧拉角zyx

        if(Grip_ID == 0)
        {
            double gdcPos[6] = {path_list[i][0],path_list[i][1],path_list[i][2], eulerAngle[0] * 57.2957795130823, eulerAngle[1] * 57.2957795130823, eulerAngle[2] * 57.2957795130823 };
            double out_gdjpos[6];
            climbot6d_G1.IKine(gdcPos,pre_joint,out_gdjpos);

            for (size_t i = 0; i < 6;++i)
            {
                if(fabs(out_gdjpos[i] - pre_joint[i]) > 120)
                {
                    return true;
                }
            }

            for (size_t i = 0; i < 6;++i)
            {
                pre_joint[i] = out_gdjpos[i];
            }
        }
        else if(Grip_ID == 7)
        {
            double gdcPos[6] = {path_list[i][0],path_list[i][1],path_list[i][2], eulerAngle[0] * 57.2957795130823, eulerAngle[1] * 57.2957795130823, eulerAngle[2] * 57.2957795130823 };
            double out_gdjpos[6];
            climbot6d_G2.IKine(gdcPos,pre_joint,out_gdjpos);

            for (size_t i = 0; i < 6;++i)
            {
                if(fabs(out_gdjpos[i] - pre_joint[i]) > 90)
                {
                    return true;
                }
            }

            for (size_t i = 0; i < 6;++i)
            {
                pre_joint[i] = out_gdjpos[i];
            }
        }
    }
}



void motionPlan::replan(std::vector<std::vector<double> > &path_list,const int GripID)
{
    // std::cout << std::endl;
    // std::cout << "Total Points:" << path_smooth->getStateCount() << std::endl;

    // for (int i = 0; i < path_list.size();++i)
    // {
    //     for (int j = 0; j < path_list[i].size();++j)
    //     {
    //         std::cout << path_list[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // if(jointconstrain(path_list,GripID) == 1)
    // {
    //     replan_flag = 1;
    //     // plan(path_list,adjmat,Grip_ID,cur_truss_num,tar_truss_num,cur_truss,tar_truss,cur_length,cur_angle);
    // }

    if(path_smooth->getStateCount () <= 5)
    {
        cur_jpos[0] = 0;
        cur_jpos[1] = 90;
        cur_jpos[2] = 90;
        cur_jpos[3] = 0;
        cur_jpos[4] = 0;
        cur_jpos[5] = 0;
        path_list.clear();
        // motionPlan();
        plan(path_list,adjmat,Grip_ID,cur_truss_num,tar_truss_num,cur_truss,tar_truss,cur_length,cur_angle);
    }
    else
    {
        cur_jpos[0] = 0;
        cur_jpos[1] = 90;
        cur_jpos[2] = 90;
        cur_jpos[3] = 0;
        cur_jpos[4] = 0;
        cur_jpos[5] = 0;

        for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
        {
            if (!replan_flag)
            {
                replan_flag = !isStateValid(path_smooth->getState(idx));
            }
            else
                break;
        }
        if(replan_flag)
        {
            std::cout << "Replanning required" << std::endl;
            path_list.clear();
            // motionPlan();
            plan(path_list,adjmat,Grip_ID,cur_truss_num,tar_truss_num,cur_truss,tar_truss,cur_length,cur_angle);
        }
        else
            std::cout << "Replanning not required" << std::endl;
    }   
}

void motionPlan::paraInit(std::vector<std::vector<int>>& adj_mat,const int GripID,
				            int cur_truss_number,int tar_truss_number,
				            std::vector<double> &cur_truss_input,std::vector<double> &tar_truss_input,
                            int length,int angle)
{
    Grip_ID = GripID;
    adjmat = adj_mat;
    cur_truss_num = cur_truss_number;
    tar_truss_num = tar_truss_number;
    cur_truss = cur_truss_input;
    tar_truss = tar_truss_input;
    cur_length = length;
    cur_angle = angle;
}

void motionPlan::plan(std::vector<std::vector<double> > &path_list,std::vector<std::vector<int>>& adj_mat,const int GripID,
				        int cur_truss_number,int tar_truss_number,
				        std::vector<double> &cur_truss_input,std::vector<double> &tar_truss_input,
                        int length,int angle)
{
    paraInit(adj_mat, GripID, cur_truss_number, tar_truss_number, cur_truss_input, tar_truss_input,length,angle);

    // create a planner for the defined space
    og::RRTConnect *rrt = new og::RRTConnect(si);
    // og::LazyPRM *rrt = new og::LazyPRM(si);
    // og::TRRT *rrt = new og::TRRT(si);
    //设置rrt的参数range
    // rrt->setRange(500);
    ob::PlannerPtr plan(rrt);
    // set the problem we are trying to solve for the planner
    plan->setProblemDefinition(pdef);
    // perform setup steps for the planner
    plan->setup();
    // print the settings for this space
    // si->printSettings(std::cout);
    std::cout << "problem setting\n";
    // print the problem settings
    pdef->print(std::cout);
    // attempt to solve the problem within ten second of planning time
    ob::PlannerStatus solved = plan->solve(5.0);
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        
        ob::PathPtr path = pdef->getSolutionPath();
        og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();

        // pdef->getOptimizationObjective()->print(std::cout);

        ofstream osf0("../plotpath/path0.txt");
		pth->printAsMatrix(osf0);

        // std::cout << "Found solution:" << std::endl;
        // pth->printAsMatrix(std::cout);

        //Path smoothing using bspline
        //B样条曲线优化
        og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
        path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
        pathBSpline->smoothBSpline(*path_smooth,3);

        // std::cout << "Smoothed Path" << std::endl;
        // path_smooth->printAsMatrix(std::cout);

        // std::cout << path_smooth->getStateCount() << std::endl;

        ofstream osf1("../plotpath/path1.txt");
		path_smooth->printAsMatrix(osf1);
        
        for (size_t i = 0; i < path_smooth->getStateCount();++i)
        {
            std::vector<double> tmp_path_list;
            //抽象类型转换为我们期望类型
            const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(i)->as<ob::SE3StateSpace::StateType>();
            //提取第1、2状态的组成，并转换为我们期望的
            const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
            const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

            // std::cout << isStateValid(path_smooth->getState(i)) << std::endl;

            tmp_path_list.push_back(pos->values[0]);
            tmp_path_list.push_back(pos->values[1]);
            tmp_path_list.push_back(pos->values[2]);
            tmp_path_list.push_back(rot->x);
            tmp_path_list.push_back(rot->y);
            tmp_path_list.push_back(rot->z);
            tmp_path_list.push_back(rot->w);

            path_list.push_back(tmp_path_list);

        }

        // replan(path_list,GripID);

        // Clear memory
        pdef->clearSolutionPaths();
        replan_flag = false;
    }
    else
        std::cout << "No solution found" << std::endl;
}

bool motionPlan::isStateValid(const ob::State *state)
{
    //抽象类型转换为我们期望类型
	const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
	//提取第1、2状态的组成，并转换为我们期望的
	const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

	::Eigen::Quaterniond q;
    q.x() = rot->x;
	q.y() = rot->y;
	q.z() = rot->z;
	q.w() = rot->w;
	Eigen::Vector3d eulerAngle=q.matrix().eulerAngles(2,1,0);	//四元数转换成欧拉角zyx

	double cur_pos[6] = {pos->values[0], pos->values[1], pos->values[2],eulerAngle[0] * 57.2957795130823,eulerAngle[1] * 57.2957795130823,eulerAngle[2] * 57.2957795130823};	//机器人当前位姿

	std::vector<std::vector<double>> robot_link;
	std::vector<int> poten_truss;   //潜在的碰撞杆件


    double curjpos[6];
    climbot6d_G2.IKine(cur_pos, cur_jpos, curjpos);


    // for (int i = 0; i < 6;++i)
    // {
    //     if(fabs(cur_jpos[i] - curjpos[i]) > 180 && global_count > 1)
    //     {
    //         // std::cout << "error1" << std::endl;
    //         return false;
    //     }
    // }
    // global_count++;

    //获取当前机器人连杆位置
    robot_link = getCurJointPoint_ompl(cur_truss, tar_truss, cur_length,cur_angle, cur_pos,cur_jpos,cur_jpos,Grip_ID);

    //机器人运动学逆解不存在
    if(robot_link.size() == 0)
    {
        // std::cout << "error2" << std::endl;
        return false;
    }

    double min_distance = 0;
    double point[3];    //此处无用，满足函数参数输入
    //机器人连杆不干涉
    if(minDistance(robot_link[0],robot_link[2],point) < 100 || minDistance(robot_link[0],robot_link[3],point) < 100 || minDistance(robot_link[1],robot_link[3],point) < 100)
    {
        // std::cout << "error3" << std::endl;
        return false;
    }   //机器人连杆不互相碰撞

    // //机器人与夹持杆件不发生碰撞
    // if(minDistance(robot_link[2],cur_truss,point) < 90 || minDistance(robot_link[3],cur_truss,point) < 90 )
    // {
    //     std::cout << "error3 1" << std::endl;
    //     return false;
    // }
    // if(minDistance(robot_link[2],tar_truss,point) < 90 || minDistance(robot_link[3],tar_truss,point) < 90)
    // {
    //     std::cout << "error3 2" << std::endl;
    //     return false;
    // }
    
    //机器人与环境无碰
    int tmp_flag = getPontentialObstacle(robot_link, adjmat, cur_truss_num, tar_truss_num, poten_truss);
	if(tmp_flag == 1)
	{
		return true;
	}
	else
	{
		if(potentialObsTest(robot_link,poten_truss) == 1)
		{
			return true;
		}
        else
        {
            // std::cout << "error4" << std::endl;
            return false;
        }
            
    }

    return false;
}


/*
 * 功能：获取当前位置姿态下机器人连杆的点坐标
 * 参数：
 * - cur_truss：机器人基座杆件
 * - tar_truss：机器人另一手爪杆件
 * - length1\angle1：机器人基座的离散地图位置
 * - cur_pos:机器人当前位姿
 * 返回值：连杆坐标数组
 */
std::vector<std::vector<double>> getCurJointPoint_ompl(std::vector<double> &cur_truss, std::vector<double> &tar_truss,
                    const int length1, const int angle1,double *cur_pos,double *cur_jpos,double *out_gdjpos,const int Grip_ID)
{
	// std::cout << "cur_pos = ";
	// for (int i = 0; i < 6;++i)
	// {
	// 	std::cout << cur_pos[i] << " ";
	// }
	// std::cout << std::endl;
    Kine_CR_SixDoF_G1 climbot6d_g1;
	Kine_CR_SixDoF_G2 climbot6d_g2;
    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
	// double gdPos[6] = {0,90,90,0,0,0};
    // double out_gdjpos[6];
    climbot6d_g1.Set_Length(len);
	climbot6d_g2.Set_Length(len);

    std::vector<std::vector<double>> link;
    if(Grip_ID == 0)
    {
        double flag = climbot6d_g1.IKine(cur_pos, cur_jpos, out_gdjpos);

        // std::cout << "flag = " << flag << std::endl;

        if(flag != 0)
        {
            return {};
        }
        /////////////////////////////////////////////////////////////
        Linkage6D(len, out_gdjpos, cur_truss, length1, angle1,Grip_ID,link);
        ///////////////////////////////////
        return link;
    }
    else if(Grip_ID == 7)
    {
        double flag = climbot6d_g2.IKine(cur_pos, cur_jpos, out_gdjpos);

        // std::cout << "flag = " << flag << std::endl;

        if(flag != 0)
        {
            return {};
        }
        
        Linkage6D(len, out_gdjpos, cur_truss, length1, angle1,Grip_ID,link);
        return link;
    }

    return link;
}


void motionPlan::getFinalJointValue(std::vector<std::vector<double>> &path_list,const int Grip_ID,std::vector<std::vector<double>> &joint_val)
{
    // 初始化机器人
    // std::cout<<"机器人正逆运动学测试"<<std::endl;
    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};

    double pre_joint[6] = {0,90,90,0,0,0};

    //添加第一个点
    double gdjpos[6];
    if(Grip_ID == 0)
    {
        double tmp_pos[6];
        climbot6d_G1.FKine(pos_first, tmp_pos);
        climbot6d_G1.IKine(tmp_pos, pre_joint, gdjpos);
        std::vector<double> vec_gdjpos = {-gdjpos[5], gdjpos[4], -gdjpos[3], (gdjpos[2] - 90), -(gdjpos[1] - 90), gdjpos[0]};
        joint_val.push_back(vec_gdjpos);
        // std::cout << "P=";
        // std::cout << std::fixed << std::setprecision(1) << -gdjpos[5] << ",";
        // std::cout << std::fixed << std::setprecision(1) << gdjpos[4] << ",";
        // std::cout << std::fixed << std::setprecision(1) << -gdjpos[3] << ",";
        // std::cout << std::fixed << std::setprecision(1) << (gdjpos[2] - 90) << ",";
        // std::cout << std::fixed << std::setprecision(1) << -(gdjpos[1] - 90) << ",";
        // std::cout << std::fixed << std::setprecision(1) << gdjpos[0] << ",";
        // std::cout << ";" << std::endl;
    }

    else if(Grip_ID == 7)
    {
        double tmp_pos[6];
        climbot6d_G2.FKine(pos_first, tmp_pos);
        climbot6d_G2.IKine(tmp_pos, pre_joint, gdjpos);
        std::vector<double> vec_gdjpos = {-gdjpos[5], gdjpos[4], -gdjpos[3], (gdjpos[2] - 90), -(gdjpos[1] - 90), gdjpos[0]};
        joint_val.push_back(vec_gdjpos);
        // std::cout << "P=";
        // std::cout << std::fixed << std::setprecision(1) << -gdjpos[5] << ",";
        // std::cout << std::fixed << std::setprecision(1) << gdjpos[4] << ",";
        // std::cout << std::fixed << std::setprecision(1) << -gdjpos[3] << ",";
        // std::cout << std::fixed << std::setprecision(1) << (gdjpos[2] - 90) << ",";
        // std::cout << std::fixed << std::setprecision(1) << -(gdjpos[1] - 90) << ",";
        // std::cout << std::fixed << std::setprecision(1) << gdjpos[0] << ",";
        // std::cout << ";" << std::endl;  
    }

    for (size_t i = 0; i < path_list.size();++i)
    {
        ::Eigen::Quaterniond q;
        q.x() = path_list[i][3];
        q.y() = path_list[i][4];
        q.z() = path_list[i][5];
        q.w() = path_list[i][6];
        Eigen::Vector3d eulerAngle=q.matrix().eulerAngles(2,1,0);	//四元数转换成欧拉角zyx

        double out_gdjpos[6];
        if(Grip_ID == 0)
        {
            double gdcPos[6] = {path_list[i][0],path_list[i][1],path_list[i][2], eulerAngle[0] * 57.2957795130823, eulerAngle[1] * 57.2957795130823, eulerAngle[2] * 57.2957795130823 };
            climbot6d_G1.IKine(gdcPos,pre_joint,out_gdjpos);

            for (size_t i = 0; i < 6;++i)
            {
                pre_joint[i] = out_gdjpos[i];
            }
            std::vector<double> vec_gdjpos = {-out_gdjpos[5], out_gdjpos[4], -out_gdjpos[3], (out_gdjpos[2] - 90), -(out_gdjpos[1] - 90), out_gdjpos[0]};
            joint_val.push_back(vec_gdjpos);

            // std::cout << "P=";
            // std::cout <<std::fixed<<std::setprecision(1)<< -out_gdjpos[5] << ",";     
            // std::cout << std::fixed<<std::setprecision(1)<<out_gdjpos[4]<< ",";  
            // std::cout << std::fixed<<std::setprecision(1)<<-out_gdjpos[3] << ",";   
            // std::cout <<std::fixed<<std::setprecision(1)<<(out_gdjpos[2] -  90)<< ",";      
            // std::cout <<std::fixed<<std::setprecision(1)<< -(out_gdjpos[1] - 90) << ",";
            // std::cout <<std::fixed<<std::setprecision(1)<< out_gdjpos[0]<< ",";
            // std::cout << ";"<<std::endl;
        }
        else if(Grip_ID == 7)
        {
            double gdcPos[6] = {path_list[i][0],path_list[i][1],path_list[i][2], eulerAngle[0] * 57.2957795130823, eulerAngle[1] * 57.2957795130823, eulerAngle[2] * 57.2957795130823 };
            climbot6d_G2.IKine(gdcPos,pre_joint,out_gdjpos);

            for (size_t i = 0; i < 6;++i)
            {
                pre_joint[i] = out_gdjpos[i];
            }

            std::vector<double> vec_gdjpos = {-out_gdjpos[5], out_gdjpos[4], -out_gdjpos[3], (out_gdjpos[2] - 90), -(out_gdjpos[1] - 90), out_gdjpos[0]};
            joint_val.push_back(vec_gdjpos);
            // std::cout << "P=";
            // std::cout <<std::fixed<<std::setprecision(1)<< -out_gdjpos[5] << ",";     
            // std::cout << std::fixed<<std::setprecision(1)<<out_gdjpos[4]<< ",";  
            // std::cout << std::fixed<<std::setprecision(1)<<-out_gdjpos[3] << ",";   
            // std::cout <<std::fixed<<std::setprecision(1)<<(out_gdjpos[2] -  90)<< ",";      
            // std::cout <<std::fixed<<std::setprecision(1)<< -(out_gdjpos[1] - 90) << ",";
            // std::cout <<std::fixed<<std::setprecision(1)<< out_gdjpos[0]<< ",";
            // std::cout << ";"<<std::endl;
        }
    }

    //添加最后一个点
    if(Grip_ID == 0)
    {
        double tmp_pos[6];
        climbot6d_G1.FKine(pos_final, tmp_pos);
        climbot6d_G1.IKine(tmp_pos, pre_joint, gdjpos);
        std::vector<double> vec_gdjpos = {-gdjpos[5], gdjpos[4], -gdjpos[3], (gdjpos[2] - 90), -(gdjpos[1] - 90), gdjpos[0]};
        joint_val.push_back(vec_gdjpos);
        // std::cout << "P=";
        // std::cout << std::fixed << std::setprecision(1) << -gdjpos[5] << ",";
        // std::cout << std::fixed << std::setprecision(1) << gdjpos[4] << ",";
        // std::cout << std::fixed << std::setprecision(1) << -gdjpos[3] << ",";
        // std::cout << std::fixed << std::setprecision(1) << (gdjpos[2] - 90) << ",";
        // std::cout << std::fixed << std::setprecision(1) << -(gdjpos[1] - 90) << ",";
        // std::cout << std::fixed << std::setprecision(1) << gdjpos[0] << ",";
        // std::cout << ";" << std::endl;  
    }
    else if(Grip_ID == 7)
    {
        double tmp_pos[6];
        climbot6d_G2.FKine(pos_final, tmp_pos);
        climbot6d_G2.IKine(tmp_pos, pre_joint, gdjpos);
        std::vector<double> vec_gdjpos = {-gdjpos[5], gdjpos[4], -gdjpos[3], (gdjpos[2] - 90), -(gdjpos[1] - 90), gdjpos[0]};
        joint_val.push_back(vec_gdjpos);
        // std::cout << "P=";
        // std::cout << std::fixed << std::setprecision(1) << -gdjpos[5] << ",";
        // std::cout << std::fixed << std::setprecision(1) << gdjpos[4] << ",";
        // std::cout << std::fixed << std::setprecision(1) << -gdjpos[3] << ",";
        // std::cout << std::fixed << std::setprecision(1) << (gdjpos[2] - 90) << ",";
        // std::cout << std::fixed << std::setprecision(1) << -(gdjpos[1] - 90) << ",";
        // std::cout << std::fixed << std::setprecision(1) << gdjpos[0] << ",";
        // std::cout << ";" << std::endl;  
    }
}


void motionPlan::getAidPathPoint(double *gdPos,const int Grip_ID,double z_distance,double *pos_res)
{
    double curpos[6];
    if (Grip_ID == 7)
    {
        climbot6d_G2.FKine(gdPos, curpos);
    }
    else if(Grip_ID == 0)
    {
        climbot6d_G1.FKine(gdPos, curpos);
    }

    ::Eigen::Vector3d eulerangle;
    double yaw = curpos[3] * 0.0174532925199,pitch = curpos[4] * 0.0174532925199,droll = curpos[5] * 0.0174532925199;
    //EulerAngles to RotationMatrix
    ::Eigen::Vector3d ea0(yaw, pitch, droll);
    ::Eigen::Matrix3d curmtx;
    curmtx = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
        * ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
        * ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
    // std::cout << curmtx << std::endl;

    ::Eigen::Matrix4d R;
    R << curmtx(0,0), curmtx(0,1), curmtx(0,2), curpos[0],
        curmtx(1,0),  curmtx(1,1), curmtx(1,2), curpos[1], 
        curmtx(2,0),  curmtx(2,1), curmtx(2,2), curpos[2],
        0, 0 ,0 ,1;
	//逆运动学
    ::Eigen::Matrix4d Trx;
    Trx << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;
    ::Eigen::Matrix4d tarmtx;
    tarmtx = Trx * (R * Trx).inverse();

    double x_distance = 0;  //调整量
    int aid_flag = 0;
    int add_flag = 0;

    ::Eigen::Matrix4d testpoint;

    while(aid_flag == 0)
    {
        testpoint << 1, 0, 0, x_distance, 
                    0, 1, 0, 0, 
                    0, 0, 1, z_distance,
                    0, 0, 0, 1;
        ::Eigen::Matrix4d res;
        res = tarmtx.inverse() * testpoint;
        // std::cout <<  res << std::endl;

        curpos[0] = res(12);
        curpos[1] = res(13);
        curpos[2] = res(14);

        double res_gdjpos[6];

        if(Grip_ID == 7)
        {
            gdPos[0] = 0;gdPos[1] = 90;gdPos[2] = -41;gdPos[3] = 17;gdPos[4] = 50;gdPos[5] = 33;
            climbot6d_G2.IKine(curpos, gdPos, res_gdjpos);
            if(climbot6d_G2.IKine(curpos, gdPos, res_gdjpos) == 0)
            {
                for (size_t i = 0; i < 6;++i)
                {
                    pos_res[i] = res_gdjpos[i];
                }
                // std::cout << "P=";
                // std::cout <<std::fixed<<std::setprecision(1)<< -res_gdjpos[5] << ",";     
                // std::cout << std::fixed<<std::setprecision(1)<<res_gdjpos[4]<< ",";  
                // std::cout << std::fixed<<std::setprecision(1)<<-res_gdjpos[3] << ",";   
                // std::cout <<std::fixed<<std::setprecision(1)<<(res_gdjpos[2] -  90)<< ",";      
                // std::cout <<std::fixed<<std::setprecision(1)<< -(res_gdjpos[1] - 90) << ",";
                // std::cout <<std::fixed<<std::setprecision(1)<< res_gdjpos[0]<< ",";
                // std::cout << ";"<<std::endl;

                aid_flag = 1;
            }
            else
            {
                if(add_flag < 10)
                {
                    x_distance -= 15;
                    add_flag++;
                }
                else if(add_flag == 10)
                {
                    x_distance = 15;
                    add_flag++;
                }
                else
                {
                    x_distance += 15;
                    add_flag++;
                }

                if(x_distance > 150)
                {
                    std::cout << "aid path point did not found!" << std::endl;
                    return;
                }
                std::cout << x_distance << std::endl;
            }
        }
        else if(Grip_ID == 0)
        {
            climbot6d_G1.IKine(curpos, gdPos, res_gdjpos);
            if(climbot6d_G1.IKine(curpos, gdPos, res_gdjpos) == 0)
            {
                for (size_t i = 0; i < 6;++i)
                {
                    pos_res[i] = res_gdjpos[i];
                }
                // std::cout << "P=";
                // std::cout <<std::fixed<<std::setprecision(1)<< -res_gdjpos[5] << ",";     
                // std::cout << std::fixed<<std::setprecision(1)<<res_gdjpos[4]<< ",";  
                // std::cout << std::fixed<<std::setprecision(1)<<-res_gdjpos[3] << ",";   
                // std::cout <<std::fixed<<std::setprecision(1)<<(res_gdjpos[2] -  90)<< ",";      
                // std::cout <<std::fixed<<std::setprecision(1)<< -(res_gdjpos[1] - 90) << ",";
                // std::cout <<std::fixed<<std::setprecision(1)<< res_gdjpos[0]<< ",";
                // std::cout << ";"<<std::endl;

                aid_flag = 1;
            } 
            else
            {
                if(add_flag < 10)
                {
                    x_distance -= 15;
                    add_flag++;
                }
                else if(add_flag == 10)
                {
                    x_distance = 15;
                    add_flag++;
                }
                else
                {
                    x_distance += 15;
                    add_flag++;
                }

                if(x_distance > 150)
                {
                    std::cout << "aid path point did not found!" << std::endl;
                    return;
                }
                std::cout << x_distance << std::endl;
            }      
        }
    }
}

void singleStepPlanner(std::vector<std::vector<double>> &joint_val_list,
                        std::vector<std::vector<int>> &adj_mat,
                        std::vector<std::vector<int>> &final_path_list,
                        std::vector<int> &truss_list,
                        std::vector<int> &single_num)
{
    int step_counter = 0;   //单根杆件上夹持点累计
    int global_counter = 0;     //全局夹持点累计

    std::cout << single_num.size() << std::endl
              << std::endl;

    std::ofstream outfile;
    outfile.open("../src/singleStepJointValue.txt");  //输出仿真用地图列表

    double pos_start[6];
    double pos_end[6];
    double pos_end_aid[6];
    double pos_start_aid[6];
    int t = 0;

    double aid_point_dis = 100;

    while(step_counter <= single_num[t])
    {
        std::cout << "cur_t" << t <<"   ";
        std::cout << "cur_step_counter" << step_counter << std::endl 
        << std::endl;
        std::cout << "global_counter" << global_counter << std::endl;

        if(global_counter == joint_val_list.size())
        {
            return;  //最后一步不存在,跳出
        }
       
        if(step_counter == single_num[t])
        {
            motionPlan planner;
            for (size_t i = 0; i < 6; ++i)
            {
                planner.pos_first[i] = joint_val_list[global_counter - 1][i];
                pos_start[i] = joint_val_list[global_counter - 1][i];
                pos_end[i] = joint_val_list[global_counter][i];
                planner.pos_final[i] = joint_val_list[global_counter][i];
            }
            if (global_counter % 2 == 1)
            {
                planner.getAidPathPoint(pos_start, 0, -aid_point_dis,pos_start_aid);
                planner.getAidPathPoint(pos_end, 0, -aid_point_dis,pos_end_aid);
                planner.setStartPos(pos_start_aid, 0);
                planner.setGoalPos(pos_end_aid, 0);
                std::vector<std::vector<double>> path_list;
                planner.plan(path_list, adj_mat, 0, truss_list[t], truss_list[t + 1], truss[truss_list[t] - 1], truss[truss_list[t + 1] - 1], final_path_list[global_counter][0], final_path_list[global_counter][1]);               
                // while(path_list.size() == 0)
                // {
                //     planner.replan_flag = true;
                //     planner.replan(path_list, 0);
                // }
                std::vector<std::vector<double>> single_joint_val;
                planner.getFinalJointValue(path_list, 0, single_joint_val);

                //打印路径点到文档
                for (size_t i = 0; i < single_joint_val.size();++i)
                {
                    outfile << "P=";
                    for (size_t j = 0; j < 6;++j)
                    {
                        outfile << single_joint_val[i][j] << ",";
                    }   
                    outfile << ";"<< std::endl; 
                }
                outfile << "HALT" << std::endl;

                step_counter = 0;
                t++;
                global_counter++;
            }
            else
            {
                planner.getAidPathPoint(pos_start, 7, aid_point_dis,pos_start_aid);
                planner.getAidPathPoint(pos_end, 7, aid_point_dis,pos_end_aid);
                planner.setStartPos(pos_start_aid, 7);
                planner.setGoalPos(pos_end_aid, 7);
                std::vector<std::vector<double>> path_list;
                planner.plan(path_list, adj_mat, 7, truss_list[t], truss_list[t + 1], truss[truss_list[t] - 1], truss[truss_list[t + 1] - 1], final_path_list[global_counter][0], final_path_list[global_counter][1]);
                // while(path_list.size() == 0)
                // {
                //     planner.replan_flag = true;
                //     planner.replan(path_list, 7);
                // }
                std::vector<std::vector<double>> single_joint_val;
                planner.getFinalJointValue(path_list, 7,single_joint_val);

                //打印路径点到文档
                for (size_t i = 0; i < single_joint_val.size();++i)
                {
                    outfile << "P=";
                    for (size_t j = 0; j < 6;++j)
                    {
                        outfile << single_joint_val[i][j] << ",";
                    }   
                    outfile << ";"<< std::endl; 
                }
                outfile << "HALT" << std::endl;

                step_counter = 0;
                t++;
                global_counter++;
            }
        }
        else
        {
            if(t == 0 && step_counter == 0) //第一步
            {
                motionPlan planner;
                pos_start[0] = 0;pos_start[1] = 90;pos_start[2] = 90;pos_start[3] = 0;pos_start[4] = 0;pos_start[5] = 0;
                planner.pos_first[0] = 0;planner.pos_first[1] = 90;planner.pos_first[2] = 90;planner.pos_first[3] = 0;planner.pos_first[4] = 0;planner.pos_first[5] = 0;
                for (size_t i = 0; i < 6;++i)
                {
                    pos_end[i] = joint_val_list[0][i];
                    planner.pos_final[i] = joint_val_list[0][i];
                }
                planner.setStartPos(pos_start, 7);
                planner.getAidPathPoint(pos_end, 7, 100,pos_end_aid);
                planner.setGoalPos(pos_end_aid, 7);
                std::vector<std::vector<double>> path_list;
                planner.plan(path_list, adj_mat, 7, truss_list[t], truss_list[t], truss[truss_list[t]-1], truss[truss_list[t]-1] , final_path_list[global_counter][0], final_path_list[global_counter][1]);
                // while(path_list.size() == 0)
                // {
                //     planner.replan_flag = true;
                //     planner.replan(path_list, 7);
                // }
                std::vector<std::vector<double>> single_joint_val;
                planner.getFinalJointValue(path_list, 7 ,single_joint_val);
                step_counter++;
                global_counter++;

                //打印路径点到文档
                for (size_t i = 0; i < single_joint_val.size();++i)
                {
                    outfile << "P=";
                    for (size_t j = 0; j < 6;++j)
                    {
                        outfile << single_joint_val[i][j] << ",";
                    }   
                    outfile << ";"<< std::endl; 
                }
                outfile << "HALT" << std::endl;
            }
            else
            {
                motionPlan planner;
                for (size_t i = 0; i < 6; ++i)
                {
                    planner.pos_first[i] = joint_val_list[global_counter - 1][i];
                    pos_start[i] = joint_val_list[global_counter - 1][i];
                    pos_end[i] = joint_val_list[global_counter][i];
                    planner.pos_final[i] = joint_val_list[global_counter][i];
                }
                if(global_counter % 2 == 1)
                {
                    planner.getAidPathPoint(pos_start, 0, -aid_point_dis,pos_start_aid);
                    planner.getAidPathPoint(pos_end, 0, -aid_point_dis,pos_end_aid);
                    planner.setStartPos(pos_start_aid, 0);
                    planner.setGoalPos(pos_end_aid, 0);
                    std::vector<std::vector<double>> path_list;
                    planner.plan(path_list, adj_mat, 0, truss_list[t], truss_list[t], truss[truss_list[t]-1], truss[truss_list[t]-1] , final_path_list[global_counter][0], final_path_list[global_counter][1]);
                    // while (path_list.size() == 0)
                    // {
                    //     planner.replan_flag = true;
                    //     planner.replan(path_list, 0);
                    // }
                    std::vector<std::vector<double>> single_joint_val;
                    planner.getFinalJointValue(path_list, 0, single_joint_val);
                    step_counter++;
                    global_counter++;

                    //打印路径点到文档
                    for (size_t i = 0; i < single_joint_val.size();++i)
                    {
                        outfile << "P=";
                        for (size_t j = 0; j < 6;++j)
                        {
                            outfile << single_joint_val[i][j] << ",";
                        }   
                        outfile << ";"<< std::endl; 
                    }
                    outfile << "HALT" << std::endl;
                }
                else
                {
                    planner.getAidPathPoint(pos_start, 7, aid_point_dis,pos_start_aid);
                    planner.getAidPathPoint(pos_end, 7, aid_point_dis,pos_end_aid);
                    planner.setStartPos(pos_start_aid, 7);
                    planner.setGoalPos(pos_end_aid, 7);
                    std::vector<std::vector<double>> path_list;
                    planner.plan(path_list, adj_mat, 7, truss_list[t], truss_list[t], truss[truss_list[t]-1], truss[truss_list[t]-1], final_path_list[global_counter][0], final_path_list[global_counter][1]);
                    // while(path_list.size() == 0)
                    // {
                    //     planner.replan_flag = true;
                    //     planner.replan(path_list, 7);
                    // }
                    std::vector<std::vector<double>> single_joint_val;
                    planner.getFinalJointValue(path_list, 7,single_joint_val);
                    step_counter++;
                    global_counter++;

                    //打印路径点到文档
                    for (size_t i = 0; i < single_joint_val.size();++i)
                    {
                        outfile << "P=";
                        for (size_t j = 0; j < 6;++j)
                        {
                            outfile << single_joint_val[i][j] << ",";
                        }   
                        outfile << ";"<< std::endl; 
                    }
                    outfile << "HALT" << std::endl;
                }
            }
            
        }
    }
    outfile.close();
}



motionPlanJointState::motionPlanJointState()
{
    //初始化机器人
    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
    climbot6d_G1.Set_Length(len);
    climbot6d_G2.Set_Length(len);

    //状态空间
    space = ob::StateSpacePtr(new ob::RealVectorStateSpace(6));

    //设置起点终点
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);

    //搜索的三维范围设置
    ob::RealVectorBounds bounds(6);
    bounds.high[0] = 360;bounds.low[0] = -360;
    bounds.high[1] = 210;bounds.low[1] = -30;
    bounds.high[2] = 210;bounds.low[2] = -30;
    bounds.high[3] = 360;bounds.low[3] = -360;
    bounds.high[4] = 120;bounds.low[4] = -120;
    bounds.high[5] = 360;bounds.low[5] = -360;
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    ob::RealVectorBounds vel_bounds(6);
    vel_bounds.setHigh(40);
    vel_bounds.setLow(-40);
    

    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 6));
    ob::RealVectorBounds cbounds(6);
    cbounds.setLow(-40);
    cbounds.setHigh(40);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    //创建状态空间
    si = oc::SpaceInformationPtr(new oc::SpaceInformation(space, cspace));
    si->setStatePropagator(boost::bind(&motionPlanJointState::propagate,this,_1,_2,_3,_4));
    si->setStateValidityChecker(boost::bind(&motionPlanJointState::isStateValid, this, si.get(), _1));

    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    std::cout << "Initialize finish: " << std::endl;

    //从此状态空间构造空间信息的实例
    // si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

    // start->setXYZ(0,0,0);
    // start->as<ob::CompoundStateSpace::StateType>(1)->setIdentity();
    // goal->setXYZ(0,0,0);
    // goal->as<ob::CompoundStateSpace::StateType>(1)->setIdentity();

    // si->setStateValidityChecker(std::bind(&motionPlan::isStateValid, this, std::placeholders::_1));

    // pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

    // pdef->setStartAndGoalStates(start, goal);

    // // pdef->setOptimizationObjective(motionPlan::getPathLengthObjWithCostToGo(si));

    // // pdef->setOptimizationObjective(motionPlan::getMechWork(si));

    // std::cout << "Initialize: " << std::endl;

}

motionPlanJointState::~motionPlanJointState()
{

}

bool motionPlanJointState::isStateValid(const oc::SpaceInformation *si,const ob::State *state)
{
    if(!si->satisfiesBounds(state))
    {
        return false;
    }

    const ob::CompoundStateSpace::StateType *compound_state = state->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType *joint_state = compound_state->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *vel_state = compound_state->as<ob::RealVectorStateSpace::StateType>(1);

    double cur_pos[6];
    std::cout << "statevalid" << std::endl;

    std::cout << joint_state->values[0]<< std::endl;

    // std::cout<< joint_state->values << " " << vel_state->values << std::endl;
    // std::cout << joint_state->values << std::endl;
    // cur_pos[0] = joint_state->values[0];
    // cur_pos[1] = joint_state->values[1];
    // cur_pos[2] = joint_state->values[2];
    // cur_pos[3] = joint_state->values[3];
    // cur_pos[4] = joint_state->values[4];
    // cur_pos[5] = joint_state->values[5];
    // cur_pos = joint_state->values;

    std::vector<std::vector<double>> robot_link;
    std::vector<int> poten_truss;   //潜在的碰撞杆件
    // //获取当前机器人连杆位置
    robot_link = getCurJointPoint_ompl(cur_truss, tar_truss, cur_length,cur_angle, cur_pos,cur_jpos,cur_jpos,Grip_ID);

    double min_distance = 0;
    double point[3];    //此处无用，满足函数参数输入
    //机器人连杆不干涉
    if(minDistance(robot_link[0],robot_link[2],point) < 100 || minDistance(robot_link[0],robot_link[3],point) < 100 || minDistance(robot_link[1],robot_link[3],point) < 100)
    {
        // std::cout << "error3" << std::endl;
        return false;
    }   //机器人连杆不互相碰撞

    //机器人与环境无碰
    int tmp_flag = getPontentialObstacle(robot_link, adjmat, cur_truss_num, tar_truss_num, poten_truss);
	if(tmp_flag == 1)
	{
		return true;
	}
	else
	{
		if(potentialObsTest(robot_link,poten_truss) == 1)
		{
			return true;
		}
        else
        {
            // std::cout << "error4" << std::endl;
            return false;
        }
            
    }

    return false;
}

void motionPlanJointState::setStartPos(double *start_jpos,const int GripID)
{
    ob::ScopedState<> start_state(space);
    start_state[0] = start_jpos[0];
    start_state[1] = start_jpos[1];
    start_state[2] = start_jpos[2];
    start_state[3] = start_jpos[3];
    start_state[4] = start_jpos[4];
    start_state[5] = start_jpos[5];

    pdef->clearStartStates();
    pdef->addStartState(start_state);
    std::cout << "start pos success" << std::endl;
}

void motionPlanJointState::setGoalPos(double *goal_jpos,const int GripID)
{
    ob::ScopedState<> goal_state(space);
    goal_state[0] = goal_jpos[0];
    goal_state[1] = goal_jpos[1];
    goal_state[2] = goal_jpos[2];
    goal_state[3] = goal_jpos[3];
    goal_state[4] = goal_jpos[4];
    goal_state[5] = goal_jpos[5];

    pdef->clearGoal();
    pdef->setGoalState(goal_state);
    std::cout << "goal pos success" << std::endl;
}

void motionPlanJointState::propagate(const ob::State *start_state, const oc::Control *control, const double duration, ob::State *result_state)
{
    std::cout << "propagate" << std::endl;
    const ob::CompoundStateSpace::StateType *compound_state = start_state->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType *joint_state = compound_state->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *vel_state = compound_state->as<ob::RealVectorStateSpace::StateType>(1);

    const double *ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double acc[6];
    double velocity[6];
    *velocity = *vel_state->values;
    *acc = *ctrl;

    double res_joint[6], res_vel[6];
    for (int i = 0; i < 6;++i)
    {
        res_vel[i] = acc[i] * duration + velocity[i];
        res_joint[i] = joint_state->values[i] + velocity[i] * duration;
    }

    ob::CompoundStateSpace::StateType *res_compound_state = result_state->as<ob::CompoundStateSpace::StateType>();
    ob::RealVectorStateSpace::StateType *res_joint_state = res_compound_state->as<ob::RealVectorStateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType *res_vel_state = res_compound_state->as<ob::RealVectorStateSpace::StateType>(1);

    res_joint_state->values = res_joint;
    res_vel_state->values = res_vel;
}

void motionPlanJointState::paraInit(std::vector<std::vector<int>>& adj_mat,const int GripID,
                    int cur_truss_number,int tar_truss_number,
                    std::vector<double> &cur_truss_input,std::vector<double> &tar_truss_input,
                    int length,int angle)
{
    Grip_ID = GripID;
    adjmat = adj_mat;
    cur_truss_num = cur_truss_number;
    tar_truss_num = tar_truss_number;
    cur_truss = cur_truss_input;
    tar_truss = tar_truss_input;
    cur_length = length;
    cur_angle = angle;    
}

void motionPlanJointState::plan(std::vector<std::vector<double> > &path_list,std::vector<std::vector<int>>& adjmat,const int GripID,
                                int cur_truss_number,int tar_truss_number,
                                std::vector<double> &cur_truss_input,std::vector<double> &tar_truss_input,
                                int length,int angle)
{
    paraInit(adjmat, GripID, cur_truss_number, tar_truss_number, cur_truss_input, tar_truss_input, length, angle);
    // create a planner for the defined space
    og::RRTConnect *rrt = new og::RRTConnect(si);
    // og::LazyPRM *rrt = new og::LazyPRM(si);
    // og::TRRT *rrt = new og::TRRT(si);
    ob::PlannerPtr plan(rrt);
    // set the problem we are trying to solve for the planner
    plan->setProblemDefinition(pdef);
    // perform setup steps for the planner
    plan->setup();
    // print the settings for this space
    // si->printSettings(std::cout);
    std::cout << "problem setting\n";
    // print the problem settings
    pdef->print(std::cout);
    // attempt to solve the problem within ten second of planning time
    ob::PlannerStatus solved = plan->solve(5.0);
    
    if (solved)
    {
        
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path

        ob::PathPtr path = pdef->getSolutionPath();
        og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();

        // pdef->getOptimizationObjective()->print(std::cout);

        ofstream osf0("../plotpath/path0.txt");
		pth->printAsMatrix(osf0);

        // std::cout << "Found solution:" << std::endl;
        // pth->printAsMatrix(std::cout);

        //Path smoothing using bspline
        //B样条曲线优化
        og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
        path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
        pathBSpline->smoothBSpline(*path_smooth,3);

        // std::cout << "Smoothed Path" << std::endl;
        // path_smooth->printAsMatrix(std::cout);

        // std::cout << path_smooth->getStateCount() << std::endl;

        ofstream osf1("../plotpath/path1.txt");
		path_smooth->printAsMatrix(osf1);
        
        for (size_t i = 0; i < path_smooth->getStateCount();++i)
        {
            std::vector<double> tmp_path_list;
            //抽象类型转换为我们期望类型
            const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(i)->as<ob::SE3StateSpace::StateType>();
            //提取第1、2状态的组成，并转换为我们期望的
            const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
            const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

            // std::cout << isStateValid(path_smooth->getState(i)) << std::endl;

            tmp_path_list.push_back(pos->values[0]);
            tmp_path_list.push_back(pos->values[1]);
            tmp_path_list.push_back(pos->values[2]);
            tmp_path_list.push_back(rot->x);
            tmp_path_list.push_back(rot->y);
            tmp_path_list.push_back(rot->z);
            tmp_path_list.push_back(rot->w);

            path_list.push_back(tmp_path_list);

        }

        // replan(path_list,GripID);

        // Clear memory
        pdef->clearSolutionPaths();
        // replan_flag = false;
    }
    else
        std::cout << "No solution found" << std::endl;
}
