#ifndef QTree_H
#define QTree_H

//#include <iostream>
#include <iostream>
#include <memory>   //shared_ptr
#include <vector>
#include <queue>
#include "m_Matrix.h"
#include "Coordinate.h"
#include "pole.h"

#define MAX_TREEDEPTH 8 //满四叉树最大深度，对应栅格地图分辨率，2^(n-1)
#define Child_Num 4 //四叉树，四个结点



//! \brief A 2d point abstraction.
struct Point {
    Point() :
        x(0), y(0) {
    }

    Point(int x, int y) :
        x(x), y(y) {
    }

    bool operator==(const Point& other) {
        return x == other.x && y == other.y;
    }

    int x, y;
};

//! \brief Obstacle abstraction.
class Obstacle {
protected:
    Point top_right;   /**< The obstacle top right corner's coordinates. */
    Point bottom_left; /**< The obstacle bottom left corner's coordinates. */

public:
    //! \brief Obstacle constructor.
    //! \brief origine Bottom left corner's coordinates.
    //! \brief width Obstacle's width.
    //! \brief height Obstacle's height.
    Obstacle(Point origine, int width, int height) :
        bottom_left(origine),
        top_right(Point(origine.x + width, origine.y + height)) {
    }

    //! \brief Obstacle constructor.
    //! \brief bottom_left Bottom left corner's coordinates.
    //! \brief top_right Top right corner's coordinates.
    Obstacle(Point bottom_left, Point top_right):
        bottom_left(bottom_left), top_right(top_right) {
    }

    //! \brief Given two points representing the bottom left and top right
    //! corners of an object, check if the obstacle intersect them.
    //! \brief bottom_left The object bottom left corner's coordinates.
    //! \brief top_right The object top right corner's coordinates.
    //! \return True if the obstacle intersect the object.
    bool intersect(const Point& bottom_left, const Point& top_right) const {

        // First, check if there is an overlap on the x-axis.
        bool x_overlap = this->bottom_left.x < top_right.x &&
                this->top_right.x > bottom_left.x;

        // Second, check if there is an overlap on the y-axis.
        bool y_overlap = this->bottom_left.y < top_right.y &&
                this->top_right.y > bottom_left.y;

        // If there is an overlap in both the y and x axis, we can safely
        // conclude that there is an overlap between the two objects.
        return x_overlap && y_overlap;
    }

    //! \brief Given two points representing the bottom left and top right
    //! corners of an object, check if the obstacle obstruct them.
    //! \brief bottom_left The object bottom left corner's coordinates.
    //! \brief top_right The object top right corner's coordinates.
    //! \return True if the obstacle obstruct the object.
    bool obstruct(const Point& bottom_left, const Point& top_right) const {

        // First, check if there is an obstruction on the x-axis.
        bool x_obstruct = this->bottom_left.x <= bottom_left.x &&
                this->top_right.x >= top_right.x;

        // Second, check if there is an obstruction on the y-axis.
        bool y_obstruct = this->bottom_left.y <= bottom_left.y &&
                this->top_right.y >= top_right.y;

        // If there is an obstruction in both the y and x axis, we can safely
        // conclude that there is an overlap between the two objects.
        return x_obstruct && y_obstruct;
    }
};

//! \brief QTreeNode abstraction.
class QTreeNode 
{
    protected:
        friend class QTree;

        int treedepth;  //结点深度
        Point top_right;        /**< QTreeNode top right corner coordinates. */
        Point bottom_left;      /**< QTreeNode bottom left corner coordinates. */

    public:
        bool isLeaf;	//当这个节点是一个叶子结点时为 True，如果它有 4 个子节点则为 False
        int state;    //储存叶子结点所代表的区域的值。1对应可过渡，2对应不可过渡,0对应混合状态
        QTreeNode *Child[Child_Num];  //指向孩子结点的指针

        //        North
        //      .---.---.
        //      | 1 | 3 |
        // West '---'---' East
        //      | 0 | 2 |
        //      '---'---'
        //        South

        QTreeNode();   //节点构造函数
        ~QTreeNode();
        QTreeNode(int state_, bool isLeaf_);
        QTreeNode(int state_, bool isLeaf_, QTreeNode *Child[Child_Num]);


        //! \brief Calculate the width of the region represented by this node.
        //! \return The width of the region represented by this node.
        inline int width() const {
            return top_right.x - bottom_left.x;
        }

        //! \brief Calculate the height of the region represented by this node.
        //! \return The height of the region represented by this node.
        inline int height() const {
            return top_right.y - bottom_left.y;
        }
};

//! \brief QTree abstraction.
class QTree
{
    public:
        QTreeNode *t_root;   //四叉树根结点

        QTree();
        ~QTree();

        QTreeNode *split(int t_rows, int t_cols, int M, int N, int depth, Poles &temp_pole,const int DOF_flag);           //树结点分裂
        QTreeNode *construct(std::vector<double> &p0, std::vector<double> &p1, int length, int alpha,const int DOF_flag); //构建树
        QTreeNode *merge(QTreeNode *p_node);                                                           //树结点合并
        void PrintAllQTreeNode(QTreeNode *p_node);                                                     //输出叶子结点
        void PrintObsQTreeNode(QTreeNode *p_node);                                                     //输出不可过渡结点
        void findTreeNode(QTreeNode *p_node, std::vector<int> &gpoint);   //寻找值为1的所有节点

        QTreeNode *findNode(const Point& point);

        void getNodeCount(QTreeNode *p_node);
        int count = 0;

    protected:
        // std::shared_ptr<QTreeNode> t_root;  //四叉树根结点
        
};



#endif  //QTree.h