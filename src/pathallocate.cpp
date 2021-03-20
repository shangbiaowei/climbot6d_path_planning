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
int PathAllocate::stepFeasibleVerify(const int length1, const int angle1,
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
            return 0;   //过渡点不可行
        }
        link = getCurJointPoint(cur_truss, cur_truss, length1, angle1, length2, angle2);
        int tmp_flag = getPontentialObstacle(link, adj_mat, cur_truss_num, cur_truss_num, potential_truss);
        if(tmp_flag == 0)
        {
            if(potentialObsTest(link, potential_truss) == 1)
            {
                return 1;
            }
            else{
                return 0;
            }
        }
        else{
            return 1;
        }
    }
    else
    {
        if(transwithIK(cur_truss,cur_truss,length2, angle2, length1, angle1,0,out_joint) == 0)
        {
            return 0;   //过渡点不可行
        }
        link = getCurJointPoint(cur_truss, cur_truss, length2, angle2, length1, angle1);;
        int tmp_flag = getPontentialObstacle(link, adj_mat, cur_truss_num, cur_truss_num, potential_truss);
        if(tmp_flag == 0)
        {
            if(potentialObsTest(link, potential_truss) == 1)
            {
                return 1;
            }
            else{
                return 0;
            }
        }
        else{
            return 1;
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

            path_point = stepAllocate(opti_res[2 * i][0], opti_res[2 * i][1], opti_res[2 * i + 1][0], opti_res[2 * i + 1][1], truss[truss_list[i] - 1], truss_list[i], single_num[i]);
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

    return -1;
}

std::vector<std::vector<int> > PathAllocate::stepAllocate(const int length1, const int angle1,
                                const int length2, const int angle2,
                                std::vector<double> &cur_truss,int cur_truss_num,
                                int single_truss_num)
{
    std::vector<std::vector<int>> res;

    int length_scale = fabs(length1 - length2) / single_truss_num;
    int angle_scale = fabs(angle1 - angle2) / single_truss_num;

    double len_scale = (sqrt((cur_truss[0] - cur_truss[3]) * (cur_truss[0] - cur_truss[3]) +
                        (cur_truss[1] - cur_truss[4]) * (cur_truss[1] - cur_truss[4]) +
                        (cur_truss[2] - cur_truss[5]) * (cur_truss[2] - cur_truss[5]))) /
                        m_rows;

    std::vector<int> tmp_length_point(single_truss_num - 1, 0);
    std::vector<int> tmp_angle_point(single_truss_num - 1, 0);  //存储中间点

    //尺蠖步态
    if(fabs(length1 - length2) * len_scale <= 110 && single_truss_num == 2)
    {
        
        res = inchwormStep(length1, angle1, length2, angle2, cur_truss, cur_truss_num,len_scale);
        global_counter++;
        // std::cout << res[0][0] << " " << res[0][1] << std::endl;
        return res;
    }
    else
    {
        
        //其他步态
        res = otherStep(length1, angle1, length2, angle2, cur_truss, cur_truss_num, single_truss_num,len_scale);
        global_counter++;
    }
}

std::vector<std::vector<int> > PathAllocate::inchwormStep(const int length1, const int angle1,
                                const int length2, const int angle2,
                                std::vector<double> &cur_truss,int cur_truss_num,
                                double len_scale)
{
    // std::cout << "inc curglo=" << global_counter << std::endl;
    std::vector<std::vector<int>> res;

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

    //第一个方向
    for (size_t i = 400; i < 758.4; i += 50)
    {
        int tmp_point = i / len_scale;
        tmp_length_point[0] = length1 + tmp_point;

        if(stepFeasibleVerify(tmp_length_point[0],tmp_angle_point[0],length1,angle1,cur_truss,cur_truss_num,global_counter) == 1)
        {
            //当前点可行
            res.push_back({tmp_length_point[0], tmp_angle_point[0]});
            global_counter++;
            return res;
        }       
    }
    //第二个方向
    for (size_t i = 400; i < 758.4; i += 50)
    {
        int tmp_point = i / len_scale;
        tmp_length_point[0] = length1 - tmp_point;  //取反方向
        if(stepFeasibleVerify(tmp_length_point[0],tmp_angle_point[0],length1,angle1,cur_truss,cur_truss_num,global_counter) == 1)
        {
            //当前点可行
            res.push_back({tmp_length_point[0], tmp_angle_point[0]});
            global_counter++;
            return res;
        }       
    }


    //缺点:角度限定,万一出现所有距离内都存在障碍,但通过调整角度可行(概率不大)
    return res;
}


std::vector<std::vector<int> > PathAllocate::otherStep(const int length1, const int angle1,
                                        const int length2, const int angle2,
                                        std::vector<double> &cur_truss,int cur_truss_num,
                                        int single_truss_num,double len_scale)
{
    std::vector<std::vector<int>> res;

    std::vector<int> tmp_length_point(single_truss_num - 1, 0);
    std::vector<int> tmp_angle_point(single_truss_num -1, 0);  //存储中间点

    int per_length = 0;
    int per_angle = 0;

    //初始值设定
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
        per_angle = (angle2 + (127 - angle1)) / single_truss_num;
        tmp_angle_point[0] = angle2 - per_angle;
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

    // std::cout << "init_data =" <<tmp_length_point[0]<<" "<<tmp_angle_point[0] << std::endl;
    // std::cout << "init_data =" <<tmp_length_point[1]<<" "<<tmp_angle_point[1] << std::endl;

    // multimap<double,std::vector<int>> path_map;
    double path_count = 0;
    int backtrack_flag = 0; //为1时执行回溯操作
    int t = 0;
    while(t < single_truss_num - 1)
    {
        // std::cout <<"cur_t="<< t << std::endl;
        if(t == 0)
        {
            if(stepFeasibleVerify(length1,angle1,tmp_length_point[0],tmp_angle_point[0],cur_truss,cur_truss_num,global_counter) == 1)
            {
                res.push_back({tmp_length_point[0], tmp_angle_point[0]});
                global_counter++;
                t++;
            }
            else
            {
                /*   //代码封装
                // for (int i = tmp_length_point[0]; i < tmp_length_point[0] + 150 / len_scale;++i) //长度上两个方向
                // {
                //     for (int j = tmp_angle_point[0]; j < tmp_angle_point[0] + 15;++j)
                //     {
                //         if(stepFeasibleVerify(length1,angle1,i,j,cur_truss,cur_truss_num,global_counter) == 1)
                //         {
                //             path_count = sqrt(pow(i - tmp_length_point[0],2) + pow(j - tmp_angle_point[0], 2));
                //             // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                //             path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                //             break;
                //         }
                //     }
                //     for (int j = tmp_angle_point[0]; j > tmp_angle_point[0] - 15;--j)
                //     {
                //         if(stepFeasibleVerify(length1,angle1,i,j,cur_truss,cur_truss_num,global_counter) == 1)
                //         {
                //             path_count = sqrt(pow(i - tmp_length_point[0],2) + pow(j - tmp_angle_point[0], 2));
                //             // path_point.insert(pair<std::vector<int>, double>({i, j}, path_count));
                //             path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                //             break;
                //         }
                //     }
                // }

                // for (int i = tmp_length_point[0]-1; i > tmp_length_point[0] - 150 / len_scale;--i) //长度上两个方向
                // {
                //     for (int j = tmp_angle_point[0]; j < tmp_angle_point[0] + 15;++j)
                //     {
                //         if(stepFeasibleVerify(length1,angle1,i,j,cur_truss,cur_truss_num,global_counter) == 1)
                //         {
                //             path_count = sqrt(pow(i - tmp_length_point[0],2) + pow(j - tmp_angle_point[0], 2));
                //             // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                //             path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                //             break;
                //         }
                //     }
                //     for (int j = tmp_angle_point[0]-1; j > tmp_angle_point[0] - 15;--j)
                //     {
                //         if(stepFeasibleVerify(length1,angle1,i,j,cur_truss,cur_truss_num,global_counter) == 1)
                //         {
                //             path_count = sqrt(pow(i - tmp_length_point[0],2) + pow(j - tmp_angle_point[0], 2));
                //             // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                //             path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                //             break;
                //         }
                //     }
                // }
                */
                multimap<double,std::vector<int>> cur_path_map;
                cur_path_map = getPathMap(length1, angle1, tmp_length_point[0], tmp_angle_point[0], cur_truss, cur_truss_num, len_scale);

                if(cur_path_map.size() == 0)
                {
                    backtrack_flag++;
                    t--;
                }
                else
                {
                    multimap<double,std::vector<int>>::iterator iter;
                    //输出map内所有值
                    // for (iter = path_map.begin(); iter != path_map.end(); ++iter) {
                    //     std::cout << (*iter).first <<" "<<(*iter).second[0] << "     "<<(*iter).second[1]<<endl;
                    // }
                    // std::cout << endl;
                    // std::cout << "sizeofmap = " << path_map.size() << std::endl;

                    // //排序 (耗时)
                    // std::vector<pair<std::vector<int>, double> > path_point_vec; //把map中元素转存到vector中
                    // for(map<std::vector<int>, double>::iterator it = path_point.begin(); it != path_point.end(); it++){
                    //     path_point_vec.push_back( pair<std::vector<int>, double>(it->first,it->second));
                    // }
                    // sort(path_point_vec.begin(), path_point_vec.end(),cmp);

                    // tmp_length_point[0] = path_point_vec[0].first[0];
                    // tmp_angle_point[0] = path_point_vec[0].first[1];
                    iter = cur_path_map.begin();
                    for (size_t r = 0; r < backtrack_flag;++r)
                    {
                        iter++;
                    }
                    tmp_length_point[0] = (*iter).second[0];
                    tmp_angle_point[0] = (*iter).second[1];
                    res.push_back({tmp_length_point[0],tmp_angle_point[0]});
                    global_counter++;
                    backtrack_flag = 0;
                    t++;
                }
            }
        }
        else
        {
            for (int i = 1; i < single_truss_num - 1; ++i)
            {
                if(stepFeasibleVerify(tmp_length_point[i-1],tmp_angle_point[i-1],tmp_length_point[i],tmp_angle_point[i],cur_truss,cur_truss_num,global_counter) == 1)
                {
                    if(i + 1 == single_truss_num - 1)   //最后一步,还需确保能到达终点
                    {
                        if(stepFeasibleVerify(tmp_length_point[t],tmp_angle_point[t],length2,angle2,cur_truss,cur_truss_num,global_counter+1) == 1)
                        {
                            res.push_back({tmp_length_point[t],tmp_angle_point[t]});
                            global_counter = global_counter + 2;
                            t++;
                        }
                        // else
                        // {
                        //     std::cout << "huisu" << std::endl;
                        //     //执行回溯操作
                        //     t--;
                        // }
                    }
                    else
                    {
                        res.push_back({tmp_length_point[i],tmp_angle_point[i]});
                        global_counter++;
                        t++;
                    }
                }
                else
                {
                    // path_map.clear();    //map直接被删除报错
                    // for (auto iter = path_map.begin(); iter != path_map.end();++iter)
                    // {
                    //     path_map.erase(iter);
                    // }
                    // std::cout << path_map.size() << std::endl;
                    // // path_map.insert(pair<double, std::vector<int>>(100000, {1, 1}));
                    // for (auto iter = path_map.begin(); iter != path_map.end(); ++iter)
                    // {
                    //     std::cout << (*iter).first << " " << (*iter).second[0] << " " << (*iter).second[1] << std::endl;
                    // }

                    /*  //代码封装
                    // for (int i = tmp_length_point[t]; i < tmp_length_point[t] + 200 / len_scale; ++i) //长度上两个方向
                    // {
                    //     for (int j = tmp_angle_point[t]; j < tmp_angle_point[t] + 15;++j)
                    //     {
                    //         if(stepFeasibleVerify(tmp_length_point[t-1],tmp_angle_point[t-1],i,j,cur_truss,cur_truss_num,global_counter) == 1)
                    //         {
                    //             path_count = sqrt(pow(i - tmp_length_point[t],2) + pow(j - tmp_angle_point[t], 2));
                    //             // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                    //             path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                    //             break;
                    //         }
                    //     }
                    //     for (int j = tmp_angle_point[t]; j > tmp_angle_point[t] - 15;--j)
                    //     {
                    //         if(stepFeasibleVerify(tmp_length_point[t-1],tmp_angle_point[t-1],i,j,cur_truss,cur_truss_num,global_counter) == 1)
                    //         {
                    //             path_count = sqrt(pow(i - tmp_length_point[t],2) + pow(j - tmp_angle_point[t], 2));
                    //             // path_point.insert(pair<std::vector<int>, double>({i, j}, path_count));
                    //             path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                    //             break;
                    //         }
                    //     }
                    // }

                    // for (int i = tmp_length_point[t]-1; i > tmp_length_point[t] - 200 / len_scale;--i) //长度上两个方向
                    // {
                    //     for (int j = tmp_angle_point[t]; j < tmp_angle_point[t] + 15;++j)
                    //     {
                    //         if(stepFeasibleVerify(tmp_length_point[t-1],tmp_angle_point[t-1],i,j,cur_truss,cur_truss_num,global_counter) == 1)
                    //         {
                    //             path_count = sqrt(pow(i - tmp_length_point[t],2) + pow(j - tmp_angle_point[t], 2));
                    //             // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                    //             path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                    //             break;
                    //         }
                    //     }
                    //     for (int j = tmp_angle_point[t]-1; j > tmp_angle_point[t] - 15;--j)
                    //     {
                    //         if(stepFeasibleVerify(tmp_length_point[t-1],tmp_angle_point[t-1],i,j,cur_truss,cur_truss_num,global_counter) == 1)
                    //         {
                    //             path_count = sqrt(pow(i - tmp_length_point[t],2) + pow(j - tmp_angle_point[t], 2));
                    //             // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                    //             path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                    //             break;
                    //         }
                    //     }
                    // }
                    */
                    multimap<double,std::vector<int>> cur_path_map;
                    cur_path_map = getPathMap(tmp_length_point[t - 1], tmp_angle_point[t - 1], tmp_length_point[t], tmp_angle_point[t], cur_truss, cur_truss_num, len_scale);
                    
                    if(cur_path_map.size() == 0)
                    {
                        backtrack_flag++;
                        t--;
                    }
                    else
                    {
                        multimap<double,std::vector<int>>::iterator iter;
                        //输出map内所有值
                        // for (iter = path_map.begin(); iter != path_map.end(); ++iter) {
                        //     std::cout << (*iter).first <<" "<<(*iter).second[0] << "     "<<(*iter).second[1]<<endl;
                        // }
                        // std::cout << endl;
                        // std::cout << "sizeofmap = " << path_map.size() << std::endl;

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
                        // std::cout << tmp_length_point[t] << " " << tmp_angle_point[t] << std::endl;
                        t++;                        
                    }
                    
                }
            }            
        }
    }

    // for (int i = 0; i < tmp_length_point.size();++i)
    // {
    //     std::cout << tmp_length_point[i] << " " << tmp_angle_point[i] << std::endl;
    // }

    // std::cout <<"globalcounter="<< global_counter << std::endl;

    return res;
}


multimap<double, std::vector<int>> PathAllocate::getPathMap(const int length1, const int angle1,
                                                const int length2, const int angle2,
                                                std::vector<double> &cur_truss,
                                                int cur_truss_num,double len_scale)
{
    multimap<double, std::vector<int>> path_map;
    double path_count;

    for (int i = length2; i < length2 + 150 / len_scale; ++i) //长度上两个方向
    {
        for (int j = angle2; j < angle2 + 15;++j)
        {
            if(stepFeasibleVerify(length1,angle1,i,j,cur_truss,cur_truss_num,global_counter) == 1)
            {
                path_count = sqrt(pow(i - length2,2) + pow(j - angle2, 2));
                // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                break;
            }
        }
        for (int j = angle2; j > angle2 - 15;--j)
        {
            if(stepFeasibleVerify(length1,angle1,i,j,cur_truss,cur_truss_num,global_counter) == 1)
            {
                path_count = sqrt(pow(i - length2,2) + pow(j - angle2, 2));
                // path_point.insert(pair<std::vector<int>, double>({i, j}, path_count));
                path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                break;
            }
        }
    }

    for (int i = length2 -1; i > length2 - 150 / len_scale;--i) //长度上两个方向
    {
        for (int j = angle2; j < angle2 + 15;++j)
        {
            if(stepFeasibleVerify(length1,angle1,i,j,cur_truss,cur_truss_num,global_counter) == 1)
            {
                path_count = sqrt(pow(i - length2,2) + pow(j - angle2, 2));
                // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                break;
            }
        }
        for (int j = angle2-1; j > angle2 - 15;--j)
        {
            if(stepFeasibleVerify(length1,angle1,i,j,cur_truss,cur_truss_num,global_counter) == 1)
            {
                path_count = sqrt(pow(i - length2,2) + pow(j - angle2, 2));
                // path_point.insert(pair<std::vector<int>, double> ({i, j}, path_count));
                path_map.insert(pair<double, std::vector<int>>(path_count, {i, j}));
                break;
            }
        }
    }

    return path_map;
}
