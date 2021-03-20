#ifndef QUADTREE_H
#define QUADTREE_H

//#include <iostream>
#include "pole.h"

/***********************************************************
* 四叉树类（栅格地图作为输入）
* GDUT,2020
***********************************************************/

class QuadTreeNode 
{
public:
    bool val;    //储存叶子结点所代表的区域的值。1 对应 True，0 对应 False；
    bool isLeaf;	//当这个节点是一个叶子结点时为 True，如果它有 4 个子节点则为 False 。
    int treedepth;  //结点深度
    int range_x, range_y; //区域的范围

    QuadTreeNode* topLeft;
    QuadTreeNode* topRight;
    QuadTreeNode* bottomLeft;
    QuadTreeNode* bottomRight;  //四个区域

	QuadTreeNode();   //节点构造函数
	QuadTreeNode(bool val_, bool isLeaf_);
	QuadTreeNode(bool val_, bool isLeaf_, QuadTreeNode *topLeft_, QuadTreeNode *topRight_, QuadTreeNode *bottomLeft_, QuadTreeNode *bottomRight_);
};


/*
当前矩阵由三个参数确定：左上角元素的行号 row 列号 col，以及矩阵规模M,N

判断当前矩阵是否为叶子节点，方法为比较左上角元素和其他元素的关系，
若出现不等则为非叶节点，需要继续划分，每个划分后的矩阵规模为 N/2 * N/2，
这4个矩阵递归地生成当前节点的4个子节点。
若元素全相等则返回一个叶子节点 (true, grid[row][col])。
*/
class QuadTree
{
    public:
    QuadTreeNode *t_root;   //四叉树根结点

    //int count = 0;
    //int pos = 0;
    //int neg = 0;

    QuadTree();
    ~QuadTree();

    QuadTreeNode *split(std::vector<std::vector<int> > grid, int t_row, int t_col, int M, int N , int depth);

    QuadTreeNode *construct(std::vector<std::vector<int> > grid);

    // void test()
    // {
    // std::cout << pos << std::endl;
    // std::cout << neg << std::endl;
    // }

};


#endif  //QuadTree.h