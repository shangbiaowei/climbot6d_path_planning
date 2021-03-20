#include <iostream>
#include "../include/QuadTree.h"

QuadTreeNode::QuadTreeNode() 
{
        val = false;
        isLeaf = false;
        topLeft = NULL;
        topRight = NULL;
        bottomLeft = NULL;
        bottomRight = NULL;
        treedepth = 0;
}
    
QuadTreeNode::QuadTreeNode(bool val_, bool isLeaf_) 
{
    val = val_;
	isLeaf = isLeaf_;
    topLeft = NULL;
    topRight = NULL;
    bottomLeft = NULL;
    bottomRight = NULL;
}
    
QuadTreeNode::QuadTreeNode(bool val_, bool isLeaf_, QuadTreeNode* topLeft_, QuadTreeNode* topRight_, QuadTreeNode* bottomLeft_, QuadTreeNode* bottomRight_)
{
    val = val_;
    isLeaf = isLeaf_;
    topLeft = topLeft_;
    topRight = topRight_;
    bottomLeft = bottomLeft_;
    bottomRight = bottomRight_;
}

QuadTree::QuadTree()
{
    t_root = NULL;
}

QuadTree::~QuadTree()
{
    //t_root = NULL;
    delete t_root;
}

QuadTreeNode * QuadTree::construct(std::vector<std::vector<int> > grid)
{
    int M = (int)grid.size();
    if (M == 0) return NULL;
    t_root = split(grid, 0, 0, m_rows, m_rows,0);
    return t_root;
}

QuadTreeNode * QuadTree::split(std::vector<std::vector<int> > grid, int t_rows, int t_cols, int M, int N , int depth)
{
    int first = grid[t_rows][t_cols];

    QuadTreeNode *result = new QuadTreeNode();
    bool isLeaf = true;
    int m, n;
    m = t_rows + M;
    n = t_cols + N;


    // 遍历比较每个元素与左上角元素的值
    for (int i = t_rows; i < m; ++i)
    {
        for (int j = t_cols; j < n; ++j)
        {
            if (grid[i][j] != first)
            {
                isLeaf = false;
                break;
            }
            if (!isLeaf)
            {
                break;
            }
        }
    }
    if (isLeaf)
    {
        result->val = first;
        result->isLeaf = isLeaf;
        result->treedepth = depth + 1;
        result->range_x = t_rows + M;
        result->range_y = t_cols + N;
        //count += 1;
        //if(result->val == true)
        //{
        //    pos += 1;
        //}
        //else{
        //    neg += 1;
        //}
    }
    else
    {
        result->range_x = t_rows +M;
        result->range_y = t_cols +N;
        // 存在不一样的元素，递归计算子节点，每个子矩阵的行列数减半
        M /= 2;
        N /= 2;
        result->isLeaf = isLeaf;
        result->topLeft = split(grid, t_rows, t_cols, M, N,result->treedepth = depth +  1);
        result->topRight = split(grid, t_rows, t_cols + N, M, N,result->treedepth = depth + 1);
        result->bottomLeft = split(grid, t_rows + M, t_cols, M, N,result->treedepth = depth + 1);
        result->bottomRight = split(grid, t_rows + M, t_cols + N, M, N,result->treedepth = depth + 1);
    }   
    
    return result;
}