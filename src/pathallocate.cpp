#include "../include/pathallocate.h"


PathAllocate::PathAllocate()
{

}

PathAllocate::~PathAllocate()
{
    
}

int PathAllocate::PathAllocateInit(std::vector<std::vector<int>> &adj_mat_input,
                                    std::vector<std::vector<int>> &opti_res_input,
                                    std::vector<int> &single_num_input)
{
    adj_mat = adj_mat_input;
    opti_res = opti_res_input;
    single_num = single_num_input;

    global_counter = 0;

    return 0;
}

/* 
* 功能：判断当前位置是否可行
* 参数：
* - cur_truss：机器人基座杆件
* - length1\angle1：机器人基座的离散地图位置
* - length2\angle2：机器人目标夹持点的离散地图位置
* - single_truss_num:夹持点数量
* - flag:标志位,用于判断目前基座手爪
* 返回值：1 - 成功, 0 - 失败(存在障碍), -1 - 程序运行异常
*/
bool PathAllocate::stepFeasibleVerify(const int length1, const int angle1,
                                    const int length2, const int angle2,
                                    std::vector<double> &cur_truss,
                                    int cur_truss_num,
                                    int flag)
{
    std::vector<std::vector<double>> link;
    std::vector<int> potential_truss;
    double out_joint[6];
    if (flag % 2 == 1)
    {
        if(transwithIK(cur_truss,cur_truss,length1, angle1, length2, angle2,0,out_joint) == 0)
        {
            return false;   //过渡点不可行
        }
        getCurJointPoint(cur_truss, cur_truss, length1, angle1, length2, angle2,link);
        int tmp_flag = getPontentialObstacle(link, adj_mat, cur_truss_num, cur_truss_num, potential_truss);
        if(tmp_flag == 0)
        {
            if(potentialObsTest(link, potential_truss) == 1)
            {
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return true;
        }
    }
    else
    {
        if(transwithIK(cur_truss,cur_truss,length2, angle2, length1, angle1,0,out_joint) == 0)
        {
            return false;   //过渡点不可行
        }
        getCurJointPoint(cur_truss, cur_truss, length2, angle2, length1, angle1,link);;
        int tmp_flag = getPontentialObstacle(link, adj_mat, cur_truss_num, cur_truss_num, potential_truss);
        if(tmp_flag == 0)
        {
            if(potentialObsTest(link, potential_truss) == 1)
            {
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return true;
        }
    }
    return -1;
}

/*
* 功能：生成所有路径点
* 参数：
* - single_num：每根杆件需要步数
* - opti_res：夹持点规划优化结果
* - adj_mat：邻接矩阵
* 返回值：1 - 成功, 0 - 失败, -1 - 程序运行异常
*/
int PathAllocate::allocatePath(std::vector<std::vector<double>>& truss,std::vector<int>& truss_list)
{
    std::vector<int> tmp_point(2, 0);

    int res_flag;  

    for (size_t i = 0; i < single_num.size();++i)
    {
        if(single_num[i] == 0)
        {
            tmp_point = opti_res[2 * i];
            final_path_list.push_back(tmp_point);
            tmp_point = opti_res[2 * i + 1];
            final_path_list.push_back(tmp_point);
            global_counter++;
        }
        else if(single_num[i] == 1)
        {
            tmp_point = opti_res[2 * i];
            final_path_list.push_back(tmp_point);
            tmp_point = opti_res[2 * i + 1];
            final_path_list.push_back(tmp_point);
            global_counter = global_counter + 2;
        }
        else
        {
            final_path_list.push_back({opti_res[2 * i][0], opti_res[2 * i][1]});
            std::vector<std::vector<int>> path_point;

            res_flag = stepAllocate(opti_res[2 * i][0], opti_res[2 * i][1], opti_res[2 * i + 1][0], opti_res[2 * i + 1][1], truss[truss_list[i] - 1], truss_list[i], single_num[i],path_point);
            if(res_flag == 0)
            {
                std::cout << "无可行解" << std::endl;
                return 0;
            }
            for (size_t j = 0; j < path_point.size();++j)
            {
                final_path_list.push_back(path_point[j]);
            }
            final_path_list.push_back({opti_res[2 * i + 1][0], opti_res[2 * i + 1][1]});
        }
    }

    // std::cout <<"globalcounter="<< global_counter << std::endl;
    std::cout << "final result = " << std::endl;
    for (int i = 0; i < final_path_list.size(); ++i)
    {
        for (int j = 0; j < final_path_list[0].size(); j++)
        {
            std::cout << final_path_list[i][j] << " ";
        }
        std::cout << std::endl;
    }

    return 1;
}

int PathAllocate::stepAllocate(const int length1, const int angle1,
                                const int length2, const int angle2,
                                std::vector<double> &cur_truss,int cur_truss_num,
                                int single_truss_num,std::vector<std::vector<int>> &res)
{
    int length_scale = fabs(length1 - length2) / single_truss_num;
    int angle_scale = fabs(angle1 - angle2) / single_truss_num;

    double len_scale = (sqrt((cur_truss[0] - cur_truss[3]) * (cur_truss[0] - cur_truss[3]) +
                        (cur_truss[1] - cur_truss[4]) * (cur_truss[1] - cur_truss[4]) +
                        (cur_truss[2] - cur_truss[5]) * (cur_truss[2] - cur_truss[5]))) /
                        pole_rows;

    std::vector<int> tmp_length_point(single_truss_num - 1, 0);
    std::vector<int> tmp_angle_point(single_truss_num - 1, 0);  //存储中间点

    int res_flag;   //结果标志位

    //尺蠖步态
    if(fabs(length1 - length2) * len_scale <= 110 && single_truss_num == 2)
    {
        
        inchwormStep(length1, angle1, length2, angle2, cur_truss, cur_truss_num,len_scale,res);
        global_counter++;
    }
    else
    {
        //其他步态
        res_flag = otherStep(length1, angle1, length2, angle2, cur_truss, cur_truss_num, single_truss_num,len_scale,res);
        if(res_flag == 0)
        {
            return 0;   //无解
        }
        global_counter++;
    }

    return 1;
}

void PathAllocate::inchwormStep(const int length1, const int angle1,
                                const int length2, const int angle2,
                                std::vector<double> &cur_truss,int cur_truss_num,
                                double len_scale,std::vector<std::vector<int>> &res)
{
    //尺蠖步态一定是两步
    std::vector<int> tmp_length_point(1, 0);
    std::vector<int> tmp_angle_point(1, 0);  //存储中间点

    if(angle1 - angle2 > 64)
    {
        tmp_angle_point[0] = angle1 - (angle1 + (127 - angle2)) / 2;    //一定是两步,所以除以2
        if(tmp_angle_point[0] < 0)
        {
            tmp_angle_point[0] += 127;
        }
    }
    else if(angle2 - angle1 > 64)
    {
        tmp_angle_point[0] = angle2 - (angle2 + (127 - angle1)) / 2;
        if(tmp_angle_point[0] < 0)
        {
            tmp_angle_point[0] += 127;
        }
    }
    else
    {
        tmp_angle_point[0] = (angle2 - angle1) / 2 + angle1;
    }


    //第二个方向
    for (size_t i = 408; i < 758.4; i += 50)
    {
        int tmp_point = i / len_scale;
        tmp_length_point[0] = length1 - tmp_point;  //取反方向
        
        std::cout << tmp_length_point[0] << std::endl;

        if(stepFeasibleVerify(tmp_length_point[0],tmp_angle_point[0],length1,angle1,cur_truss,cur_truss_num,global_counter) == 1)
        {
            //当前点可行
            res.push_back({tmp_length_point[0], tmp_angle_point[0]});
            global_counter++;
            return;
        }       
    }

    //第一个方向
    for (size_t i = 408; i < 758.4; i += 50)
    {
        int tmp_point = i / len_scale;
        tmp_length_point[0] = length1 + tmp_point;

        if(tmp_length_point[0] > 125)
        {
            break;
        }

        if(stepFeasibleVerify(tmp_length_point[0],tmp_angle_point[0],length1,angle1,cur_truss,cur_truss_num,global_counter) == 1)
        {
            //当前点可行
            res.push_back({tmp_length_point[0], tmp_angle_point[0]});
            global_counter++;
            return;
        }       
    }
    //缺点:角度限定,万一出现所有距离内都存在障碍,但通过调整角度可行(概率不大)

}


int PathAllocate::otherStep(const int length1, const int angle1,
                            const int length2, const int angle2,
                            std::vector<double> &cur_truss,int cur_truss_num,
                            int single_truss_num,double len_scale,
                            std::vector<std::vector<int>> &res)
{
    std::vector<int> tmp_length_point(single_truss_num - 1, 0);
    std::vector<int> tmp_angle_point(single_truss_num -1, 0);  //存储中间点

    int per_length = 0;
    int per_angle = 0;

/******************************初始值设定*************************************/
    //距离
    per_length = (length2 - length1) / single_truss_num;
    for (size_t i = 0; i < single_truss_num - 1;++i)
    {
        tmp_length_point[i] = length1 + per_length * (i + 1);
    }

    //角度
    if (angle1 - angle2 > 64)
    {
        per_angle = (angle1 + (127 - angle2)) / single_truss_num;
        tmp_angle_point[0] = angle1 - per_angle;
        if (tmp_angle_point[0] < 0)
        {
            tmp_angle_point[0] += 127;
        }
    }
    else if (angle2 - angle1 > 64)  
    {
        per_angle = ((127 + angle1)-angle2) / single_truss_num;
        tmp_angle_point[0] = angle1 - per_angle;
        if (tmp_angle_point[0] < 0)
        {
            tmp_angle_point[0] += 127;
        }
    }
    else
    {
        per_angle = (angle2 - angle1) / single_truss_num;
        tmp_angle_point[0] = per_angle + angle1;
    }
    for (size_t i = 1; i < single_truss_num - 1;++i)
    {
        tmp_angle_point[i] = angle1 + per_angle * (i + 1);
    }
/******************************初始值设定*************************************/

    std::cout <<"当前夹持点数目 "<< single_truss_num << std::endl;
    for (int i = 0; i < single_truss_num-1; ++i)
    {
        std::cout << "初始值设置为" << tmp_length_point[i] << " " << tmp_angle_point[i] << std::endl;
    }

    double path_count = 0;
    int backtrack_flag = 0; //为1时执行回溯操作
    int t = 0;
    while(t < single_truss_num - 1)
    {
        std::cout << "global step = " << global_counter << std::endl;
        std::cout << "global cur_t" << t << std::endl;
        if(t == 0)  //第一个夹持点
        {
            if (stepFeasibleVerify(length1, angle1, tmp_length_point[t], tmp_angle_point[t], cur_truss, cur_truss_num, global_counter) == true)
            {
                if(single_truss_num == 2)   //该杆件上夹持点数目为2，第一步分配完就是最后一步,还需确保能到达终点
                {
                    if(stepFeasibleVerify(tmp_length_point[t],tmp_angle_point[t],length2,angle2,cur_truss,cur_truss_num,global_counter+1) == true)
                    {
                        res.push_back({tmp_length_point[t], tmp_angle_point[t]});
                        global_counter = global_counter + 2;
                        t++;
                    }
                    else
                    {
                        // backtrack_flag++;
                        // multimap<double, std::vector<int>> cur_path_map;
                        // getPathMap(length1, angle1, tmp_length_point[t], tmp_angle_point[t], cur_truss, cur_truss_num, len_scale,cur_path_map);
                        // if(cur_path_map.size() == 0)
                        // {
                        //     return 0;
                        // }
                        // multimap<double,std::vector<int>>::iterator iter;
                        // iter = cur_path_map.begin();
                        // for (size_t r = 0; r < backtrack_flag;++r)
                        // {
                        //     iter++;
                        // }
                        // tmp_length_point[0] = (*iter).second[0];
                        // tmp_angle_point[0] = (*iter).second[1];
                        // if(stepFeasibleVerify(tmp_length_point[t],tmp_angle_point[t],length2,angle2,cur_truss,cur_truss_num,global_counter+1) == true)
                        // {
                        //     res.push_back({tmp_length_point[t], tmp_angle_point[t]});
                        //     global_counter = global_counter + 2;
                        //     t++;
                        // }


                        multimap<double, std::vector<int>> cur_path_map;
                        getPathMap(length1, angle1, tmp_length_point[t], tmp_angle_point[t], cur_truss, cur_truss_num, len_scale,cur_path_map);
                        while(stepFeasibleVerify(tmp_length_point[t],tmp_angle_point[t],length2,angle2,cur_truss,cur_truss_num,global_counter+1) != true)
                        {
                            backtrack_flag++;
                            multimap<double,std::vector<int>>::iterator iter;
                            iter = cur_path_map.begin();
                            for (size_t r = 0; r < backtrack_flag;++r)
                            {
                                iter++;
                            }
                            tmp_length_point[0] = (*iter).second[0];
                            tmp_angle_point[0] = (*iter).second[1];

                            // if()
                        }
                        res.push_back({tmp_length_point[t], tmp_angle_point[t]});
                        global_counter = global_counter + 2;
                        t++;
                    }
                }
                else    //步数不为2，且该点满足约束
                {
                    res.push_back({tmp_length_point[t],tmp_angle_point[t]});
                    global_counter++;
                    t++;
                }
            }
            else    //该点不满足约束
            {
                multimap<double, std::vector<int>> cur_path_map;
                getPathMap(length1, angle1, tmp_length_point[t], tmp_angle_point[t], cur_truss, cur_truss_num, len_scale,cur_path_map);
                //当前基座无解
                if(cur_path_map.size() == 0)    //第一步无解，错误，返回
                {
                    std::cout << "error,current point did not have results" << std::endl;
                    return 0;
                }
                multimap<double,std::vector<int>>::iterator iter;
                iter = cur_path_map.begin();
                for (size_t r = 0; r < backtrack_flag;++r)
                {
                    iter++;
                }
                tmp_length_point[0] = (*iter).second[0];
                tmp_angle_point[0] = (*iter).second[1];
                std::cout << tmp_length_point[0] << " " << tmp_angle_point[0] << std::endl;
                if (single_truss_num == 2)
                {
                    while (stepFeasibleVerify(tmp_length_point[t], tmp_angle_point[t], length2, angle2, cur_truss, cur_truss_num, global_counter + 1) == false)
                    {
                        iter++;
                        tmp_length_point[0] = (*iter).second[0];
                        tmp_angle_point[0] = (*iter).second[1];
                    }
                    res.push_back({tmp_length_point[t], tmp_angle_point[t]});
                    global_counter = global_counter + 2;
                    t++;
                }
                else
                {
                    res.push_back({tmp_length_point[0],tmp_angle_point[0]});
                    global_counter++;
                    backtrack_flag = 0;
                    t++;
                }
            }
        }
        else    //不是第一个夹持点
        {
            for (int i = 1; i < single_truss_num - 1; ++i)
            {
                if (stepFeasibleVerify(tmp_length_point[i - 1], tmp_angle_point[i - 1], tmp_length_point[i], tmp_angle_point[i], cur_truss, cur_truss_num, global_counter) == true)
                {
                    if(i + 1 == single_truss_num - 1)   //最后一步,还需确保能到达终点
                    {
                        if(stepFeasibleVerify(tmp_length_point[i],tmp_angle_point[i],length2,angle2,cur_truss,cur_truss_num,global_counter+1) == true)
                        {
                            std::cout << "cur t = " << t << " 最后一步是否满足约束" << std::endl;
                            res.push_back({tmp_length_point[t], tmp_angle_point[t]});
                            global_counter = global_counter + 2;
                            t++;
                        }
                        else
                        {
                            std::cout << "无可行解";
                            std::cout << "cur t = " << t << " 回溯上一步" << std::endl;
                            backtrack_flag++;
                            res.pop_back();
                            t--;
                        }
                    }
                    else
                    {
                        std::cout << "cur t = " << t << " 不是最后一步 ";
                        if(backtrack_flag != 0)
                        {
                            std::cout << "执行回溯操作" << std::endl;
                            multimap<double, std::vector<int>> cur_path_map;
                            getPathMap(tmp_length_point[t - 1], tmp_angle_point[t - 1], tmp_length_point[t], tmp_angle_point[t], cur_truss, cur_truss_num, len_scale,cur_path_map);
                            multimap<double, std::vector<int>>::iterator iter;

                            iter = cur_path_map.begin();
                            for (size_t r = 0; r < backtrack_flag;++r)
                            {
                                iter++;
                            }
                            tmp_length_point[t] = (*iter).second[0];
                            tmp_angle_point[t] = (*iter).second[1];
                            res.push_back({tmp_length_point[t],tmp_angle_point[t]});
                            global_counter++;
                            backtrack_flag = 0;
                            std::cout << tmp_length_point[t] << " " << tmp_angle_point[t] << std::endl;
                            t++;
                        }
                        else
                        {
                            std::cout << "找到结果" << std::endl;
                            res.push_back({tmp_length_point[i], tmp_angle_point[i]});
                            t++;
                            global_counter++;                            
                        }
                    }
                }
                else
                {
                    multimap<double, std::vector<int>> cur_path_map;
                    getPathMap(tmp_length_point[t - 1], tmp_angle_point[t - 1], tmp_length_point[t], tmp_angle_point[t], cur_truss, cur_truss_num, len_scale,cur_path_map);
                    if(cur_path_map.size() == 0)
                    {
                        std::cout << "cur t = " << t << " 回溯上一步" << std::endl;
                        backtrack_flag++;
                        t--;
                    }
                    else
                    {
                        std::cout << "cur t = " << t << " 求解成功" << std::endl;
                        multimap<double,std::vector<int>>::iterator iter;
                        iter = cur_path_map.begin();
                        for (size_t r = 0; r < backtrack_flag;++r)
                        {
                            iter++;
                        }
                        tmp_length_point[t] = (*iter).second[0];
                        tmp_angle_point[t] = (*iter).second[1];
                        res.push_back({tmp_length_point[t],tmp_angle_point[t]});
                        global_counter++;
                        backtrack_flag = 0;
                        t++;                        
                    }
                }
            }            
        }
    }

    return 1;
}


void PathAllocate::getPathMap(const int length1, const int angle1,
                            const int length2, const int angle2,
                            std::vector<double> &cur_truss,
                            int cur_truss_num,double len_scale,multimap<double, std::vector<int>> &path_map)
{
    double path_count;

    for (int i = length2; i < length2 + 8; ++i) //长度上两个方向
    {
        for (int j = angle2; j < angle2 + 10;++j)
        {
            if(stepFeasibleVerify(length1,angle1,i,j,cur_truss,cur_truss_num,global_counter) == true)
            {
                path_count = sqrt(pow(i - length2,2) + pow(j - angle2, 2));
                // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
            }
        }
        for (int j = angle2-1; j > angle2 - 10;--j)
        {
            int tmp_j = j;
            if(j < 0)
            {
                tmp_j = j + 128;
            }
            if(stepFeasibleVerify(length1,angle1,i,tmp_j,cur_truss,cur_truss_num,global_counter) == true)
            {
                path_count = sqrt(pow(i - length2,2) + pow(j - angle2, 2));
                // path_point.insert(pair<std::vector<int>, double>({i, j}, path_count));
                path_map.insert(pair<double, std::vector<int>>(path_count, {i, tmp_j}));
            }
        }
    }

    for (int i = length2 -1; i > length2 - 8;--i) //长度上两个方向
    {
        for (int j = angle2; j < angle2 + 10;++j)
        {
            if(stepFeasibleVerify(length1,angle1,i,j,cur_truss,cur_truss_num,global_counter) == 1)
            {
                path_count = sqrt(pow(i - length2,2) + pow(j - angle2, 2));
                // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
            }
        }
        for (int j = angle2-1; j > angle2 - 10;--j)
        {
            int tmp_j = j;
            if(j < 0)
            {
                tmp_j = j + 128;
            }
            if(stepFeasibleVerify(length1,angle1,i,tmp_j,cur_truss,cur_truss_num,global_counter) == 1)
            {
                path_count = sqrt(pow(i - length2,2) + pow(j - angle2, 2));
                // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                path_map.insert(pair<double, std::vector<int>>(path_count, {i, tmp_j}));
            }
        }
    }
}
