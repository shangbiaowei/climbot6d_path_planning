#include <cmath>
#include "../include/QTree.h"

QTreeNode::QTreeNode() 
{
    state = 0;
    isLeaf = false;
    for (int i = 0; i < Child_Num; ++i)
    {
        Child[i] = NULL;
    }
    treedepth = 0;
}
    
QTreeNode::QTreeNode(int state_, bool isLeaf_) 
{
    state = state_;
	isLeaf = isLeaf_;
    for (int i = 0; i < Child_Num; ++i)
    {
        Child[i] = NULL;
    }
}
    
QTreeNode::QTreeNode(int state_, bool isLeaf_, QTreeNode *Child[Child_Num])
{
    state = state_;
    isLeaf = isLeaf_;
    for (int i = 0; i < Child_Num; ++i)
    {
        Child[i] = NULL;
    }
}

//构建
QTreeNode *QTree::construct(std::vector<double> &p0,std::vector<double> &p1,int length,int alpha){
    Poles temp_pole;
    Discretepole pole_QTree;
    temp_pole = pole_QTree.memberDisreteforQTree(p0, p1,length,alpha,0);
    t_root = split(0,0,m_rows,m_cols,0,temp_pole);
    t_root = merge(t_root);
    // for (int i = 0;i < m_rows;i++)
    // {
    //     std::cout<<"pole["<<i<<"]="<<"\t";
    //     for (int j = 0;j < m_cols;j++)
    //         {
    //             std::cout<<temp_pole.transflag[i][j];
    //             if(temp_pole.transflag[i][j] == 1){
    //                 count += 1;
    //             }
    //         }
    //     std::cout<<count<<std::endl;
    // }
    // std::cout<<std::endl;
    return t_root;
}

//分裂
QTreeNode *QTree::split(int t_rows, int t_cols, int M, int N ,int depth,Poles &temp_pole){
    QTreeNode *result = new QTreeNode();    //中间结点
    int temp_rows, temp_cols; //地图临时坐标

    //初始化机器人
    // double len[7] = {174.85, 167.2, 379.2, 178.4, 200.8, 167.2, 174.85};
    double len[7] = {269.3, 167.2, 369, 167.2, 201.8, 167.2, 269.3};
    Kine_CR_SixDoF_G1 climbot6d_G1;
    //Kine_CR_SixDoF_G2 climbot6d_G2;
    climbot6d_G1.Set_Length(len);
    //climbot6d_G2.Set_Length(len);

    result->isLeaf = false;
    result->bottom_left.x = t_rows;
    result->bottom_left.y = t_cols;
    result->top_right.x = t_rows + M;
    result->top_right.y = t_cols + N;
    M *= 0.5;
    N *= 0.5;

    double curgdj[6] = {0,0.0,0,0,0,0},out_gdjpos[6];
    temp_rows = 0.5 * (result->bottom_left.x + result->top_right.x);
    temp_cols = 0.5 * (result->top_right.y + result->bottom_left.y);
    double gdcpos[6] = {temp_pole.p_map[temp_rows][temp_cols][0],temp_pole.p_map[temp_rows][temp_cols][1],temp_pole.p_map[temp_rows][temp_cols][2],temp_pole.p_map[temp_rows][temp_cols][3],temp_pole.p_map[temp_rows][temp_cols][4],temp_pole.p_map[temp_rows][temp_cols][5]};

    //根结点分裂，然后进行判断
    if(depth == 0){
        result->state = 0;
        result->Child[0] = split(t_rows, t_cols, M, N,result->treedepth = depth + 1,temp_pole);
        result->Child[1] = split(t_rows, t_cols+N, M, N,result->treedepth = depth + 1,temp_pole);
        result->Child[2] = split(t_rows + M, t_cols, M, N,result->treedepth = depth + 1,temp_pole);
        result->Child[3] = split(t_rows + M, t_cols+N, M, N,result->treedepth = depth + 1,temp_pole);
    }
    else if(0 < depth && depth < 3)
    {
        //判断是否在机器人工作空间，超出工作空间范围不再分裂
        if(temp_pole.p_map[result->bottom_left.x][0][0] * temp_pole.p_map[result->bottom_left.x][0][0] + temp_pole.p_map[result->bottom_left.x][0][1] * temp_pole.p_map[result->bottom_left.x][0][1] +
        (temp_pole.p_map[result->bottom_left.x][0][2] - len[0] - len[1]) * (temp_pole.p_map[result->bottom_left.x][0][2] - len[0] - len[1]) -
        (len[2] + len[3] + len[4] + len[5] + len[6]) * (len[2] + len[3] + len[4] + len[5] + len[6]) > 0 &&
        temp_pole.p_map[result->top_right.x-1][0][0] * temp_pole.p_map[result->top_right.x-1][0][0] + temp_pole.p_map[result->top_right.x-1][0][1] * temp_pole.p_map[result->top_right.x-1][0][1] +
        (temp_pole.p_map[result->top_right.x-1][0][2] - len[0] - len[1]) * (temp_pole.p_map[result->top_right.x-1][0][2] - len[0] - len[1]) -
        (len[2] + len[3] + len[4] + len[5] + len[6]) * (len[2] + len[3] + len[4] + len[5] + len[6]) > 0){
            result->state = 2;     
            result->isLeaf = true;
            result->treedepth = depth + 1;
            result->Child[0] = NULL;
            result->Child[1] = NULL;
            result->Child[2] = NULL;
            result->Child[3] = NULL;
        }
        else
        {
            result->state = 0;     //默认内部有可过渡点
            result->Child[0] = split(t_rows, t_cols, M, N,result->treedepth = depth + 1,temp_pole);
            result->Child[1] = split(t_rows, t_cols+N, M, N,result->treedepth = depth + 1,temp_pole);
            result->Child[2] = split(t_rows + M, t_cols, M, N,result->treedepth = depth + 1,temp_pole);
            result->Child[3] = split(t_rows + M, t_cols+N, M, N,result->treedepth = depth + 1,temp_pole);
        }
    }
    else if(depth >=3 && depth < MAX_TREEDEPTH -1)
    {
        result->state = 0;     //默认内部有可过渡点
        result->isLeaf = false;
        result->Child[0] = split(t_rows, t_cols, M, N,result->treedepth = depth + 1,temp_pole);
        result->Child[1] = split(t_rows, t_cols+N, M, N,result->treedepth = depth + 1,temp_pole);
        result->Child[2] = split(t_rows + M, t_cols, M, N,result->treedepth = depth + 1,temp_pole);
        result->Child[3] = split(t_rows + M, t_cols+N, M, N,result->treedepth = depth + 1,temp_pole);
    }
    else if(depth == MAX_TREEDEPTH-1)
    {
        result->isLeaf = true;
        result->bottom_left.x = t_rows;
        result->bottom_left.y = t_cols;
        result->top_right.x = t_rows + M;
        result->top_right.y = t_cols + N;
        result->treedepth = MAX_TREEDEPTH;
        result->Child[0] = NULL;
        result->Child[1] = NULL;
        result->Child[2] = NULL;
        result->Child[3] = NULL;
        if(climbot6d_G1.IKine(gdcpos,curgdj,out_gdjpos) == 0){
            result->state = 1;
            //temp_pole.transflag[temp_rows][temp_cols] = 1;
        }
        else{
            result->state = 2;
            //temp_pole.transflag[temp_rows][temp_cols] = 0;
        }
    }

    return result;
}

//节点合并
QTreeNode *QTree::merge(QTreeNode *p_node){
    if (!p_node)
	{
		return p_node;
	}
    //后序遍历，合并结点
    for(auto i : p_node->Child)
    {
        merge(i);
        if(p_node->isLeaf == false)
        {
            if(p_node->Child[0] == NULL && p_node->Child[1] == NULL && p_node->Child[2] == NULL && p_node->Child[3] == NULL)
            {
                return p_node;  //空结点
            }
            else if(p_node->Child[0]->state == 2 && p_node->Child[1]->state == 2 && p_node->Child[2]->state == 2 && p_node->Child[3]->state == 2)
            {  
                //所有的都是不可过渡,合并
                for (int i = 0; i < 4;++i)
                {
                    p_node->Child[i] = NULL;
                }
                p_node->isLeaf = true;
                p_node->state = 2;
            }
            else if(p_node->Child[0]->state == 1 && p_node->Child[1]->state == 1 && p_node->Child[2]->state == 1 && p_node->Child[3]->state == 1)
            {
                //都可以过渡
                p_node->state = 1;
            }
        }
    }
    return p_node;
}

QTreeNode *QTree::findNode(const Point& point)
{
    QTreeNode* node = t_root;
    while (!node->isLeaf) {
        int x_mean = node->bottom_left.x + node->width() / 2;
        int y_mean = node->bottom_left.y + node->height() / 2;

        if (point.x < x_mean) {
            if (point.y < y_mean) {
                node = node->Child[0];
            } else {
                node = node->Child[1];
            }
        } else {
            if (point.y < y_mean) {
                node = node->Child[2];
            } else {
                node = node->Child[3];
            }
        }
    }
    // std::cout << node->bottom_left.x << " " << node->bottom_left.y << std::endl;
    // std::cout << node->top_right.x << " " << node->top_right.y << std::endl;
    return node;
}






//打印叶子节点
void QTree::PrintAllQTreeNode(QTreeNode *p_node){
    if (p_node == NULL)
	{
		return;
	}
    if(p_node->isLeaf == true && p_node->treedepth == 8)
    {
        if(p_node->state == 1)
        {
            std::cout << "QTNode[" << p_node->bottom_left.x << "][" << p_node->bottom_left.y <<"]="<< p_node->state<<" ";
            //count++;
        }
    }
    else
    {
        for (int i = 0; i < Child_Num;++i)
        {
            PrintAllQTreeNode(p_node->Child[i]);
        }
    std::cout << std::endl;
    }
}

void QTree::PrintObsQTreeNode(QTreeNode *p_node)
{
    if (p_node == NULL)
	{
		return;
	}

    if(p_node->state == 2)
    {
        std::cout << "QTNode[" << p_node->bottom_left.x << "][" << p_node->bottom_left.y <<"]->"<<  "QTNode[" << p_node->top_right.x << "][" << p_node->top_right.y <<"]"<<std::endl;
        //count++;
    }
    else
    {
        for (int i = 0; i < Child_Num;++i)
        {
            PrintObsQTreeNode(p_node->Child[i]);
        }
    }
}

void QTree::findTreeNode(QTreeNode *p_node,std::vector<int> &gpoint)
{
    if(gpoint.size() != 0)
    {
        return;
    }
    if (p_node == NULL)
	{
		return;
	}
    if(p_node->state == 1 && p_node->treedepth == 4)
    {
        // std::cout << "QTREE"<<p_node->top_right.x << " " << p_node->top_right.y << "   ";
        gpoint.push_back((p_node->bottom_left.x + p_node->top_right.x)/2);
        gpoint.push_back((p_node->bottom_left.y + p_node->top_right.y)/2);
        return;
    }
    else if(p_node->state == 1 && p_node->treedepth == 5)
    {
        gpoint.push_back((p_node->bottom_left.x + p_node->top_right.x)/2);
        gpoint.push_back((p_node->bottom_left.y + p_node->top_right.y)/2);
        return;
    }
    else if(p_node->state == 1 && p_node->treedepth == 6)
    {
        gpoint.push_back(p_node->bottom_left.x);
        gpoint.push_back(p_node->bottom_left.y);
        return;
    }  
    else
    {
        for(int i = 0; i < Child_Num;++i)
        {
            if(p_node->state == 0)
            {
                findTreeNode(p_node->Child[i],gpoint);
            }
        }
    }
}

QTree::QTree()
{

}

QTree::~QTree()
{
    t_root = NULL;
}







// //测试杆件展开代码
// std::vector<double> p0={0.0,0.0,0.0,0.0,0.0,2800.0};
// std::vector<double> p1={-115.0,-75.0,1410.0,2885.0,-75.0,1410.0};
// std::vector<double> p2={-100.0,1000.0,0.0,300.0,-50.0,2800.0};
// std::vector<double> p3={500.0,40.0,1300.0,2885.0,35.0,200.0};
// float p4[6]={0,0,1410.0,0,3000,1410.0};
// int count = 0;
// auto pole1 = new Poles;
// *pole1 = pole_transition(p0,p0);
// for(int i =0 ;i<m_rows;++i){
//     std::cout<<i<<" ";
//     for(int j = 0;j<m_cols;++j){
//         std::cout<<pole1->transflag[i][j];
//     }
//     std::cout<<std::endl;
// }
// auto tree_pole1 = new QTree;
// auto tree_pole2 = new QTree;
// auto tree_pole3 = new QTree;
// tree_pole1->construct(p0,p0);
// tree_pole2->construct(p0,p1);
// tree_pole3->construct(p0,p2);
// tree_pole2->PrintAllQTreeNode(tree_pole2->t_root);