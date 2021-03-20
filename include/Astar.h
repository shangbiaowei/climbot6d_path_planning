#include <iostream>
#include <cstdlib>      // NULL, EXIT_SUCCESS, size_t
#include <stdexcept>
#include <algorithm>    // find_if, bind2nd, etc.
#include <vector>
#include <memory>       // shared_ptr
#include <cmath>        // sqrt, abs, etc.
#include "QTree.h"


//! \brief Quadtree node implementation.
class Quadnode {
protected:

    friend class Quadtree;

    enum Side { NORTH, EAST, SOUTH, WEST };
    enum Quadrant { NORTHWEST, NORTHEAST, SOUTHWEST, SOUTHEAST };
    enum State { FREE, MIXED, OBSTRUCTED };

    //         North
    //      .----.----.
    //      | NW | NE |
    // West '----'----' East
    //      | SW | SE |
    //      '----'----'
    //         South

    Quadnode* parent;       /**< Pointer to the parent node. */
    Quadnode* northwest;    /**< Pointer to the northwest child. */
    Quadnode* southwest;    /**< Pointer to the southwest child. */
    Quadnode* northeast;    /**< Pointer to the northeast child. */
    Quadnode* southeast;    /**< Pointer to the southeast child. */

    Point top_right;        /**< Quadnode top right corner coordinates. */
    Point bottom_left;      /**< Quadnode bottom left corner coordinates. */

    State status;           /**< Quadnode status. */

    //! \brief Quadtree constructor.
    //! \param obstacles List of obstacles.
    //! \param bottom_left Quadnode bottom left corner's coordinates.
    //! \param top_right Quadnode top right corner's coordinates.
    //! \param resolution The Quadtree resolution which is the minimal
    //! Quadnode size.
    //!
    //! This constructor is called by the Quadtree's constructor. It will
    //! instanciate the Quadtree's root node and launch the sub-dividing
    //! process.
    Quadnode(const std::vector<Obstacle>& obstacles, Point bottom_left,
            Point top_right, int resolution) :
        status(MIXED), northwest(NULL), southwest(NULL), northeast(NULL),
                southeast(NULL), parent(NULL), bottom_left(bottom_left),
                top_right(top_right) {

        subdivide(obstacles, resolution);
    }

    //! \brief Quadnode children constructor.
    //! \param obstacles List of obstacles.
    //! \param bottom_left Quadnode bottom left corner's coordinates.
    //! \param top_right Quadnode top right corner's coordinates.
    //! \param resolution The Quadtree resolution which is the minimal
    //! Quadnode size.
    //! \param parent A pointer to the parent Quadnode.
    //!
    //! This constructor is called when building inner Quadtree's nodes.
    //! It will only subdivide itself if it intersects with an obstacle and
    //! if the minimal resolution isn't reached yet.
    Quadnode(const std::vector<Obstacle>& obstacles, Point bottom_left,
            Point top_right, int resolution, Quadnode* parent) :
        status(MIXED), northwest(NULL), southwest(NULL), northeast(NULL),
                southeast(NULL), parent(parent), bottom_left(bottom_left),
                top_right(top_right) {

        // Loop over all obstacles checking if we intersect something.
        for (std::vector<Obstacle>::const_iterator it = obstacles.begin();
                it != obstacles.end(); ++it) {
            if (it->intersect(bottom_left, top_right)) {
                // We're intersecting something, i.e. the node is either
                // mixed or obstructed. If the node is obstructed by this
                // obstacle, no subdivision is needed, otherwise we subdivide.
                if (it->obstruct(bottom_left, top_right) ||
                        width() <= resolution || height() <= resolution) {
                    status = OBSTRUCTED;
                } else {
                    subdivide(obstacles, resolution);
                }
                return; // We're done.
            }
        }

        status = FREE;  // This node intersects no obstacle
                        // and is therefore marked as free.
    }

    // Both assignment operator and copy constructor declared
    // protected in order to disallow raw copy or assignment.
    void operator=(const Quadnode&) {}
    Quadnode(const Quadnode&) {}

    //! \brief Subdivide the current node into four children.
    //! \param obstacles The obstacle list.
    //! \param resolution The Quadtree resolution.
    //!
    //! This methode should be called once by the constructor if
    //! the current node intersect with an obstacle and its
    //! width and height are both greater than the resolution.
    void subdivide(const std::vector<Obstacle>& obstacles, int resolution) {

        status = MIXED; // This is the default.

        int x0 = bottom_left.x;                     //  y2 .----.-------.
        int x1 = bottom_left.x + width() / 2;       //     |    |       |
        int x2 = top_right.x;                       //     | NW |  NE   |
                                                    //     |    |       |
        int y0 = bottom_left.y;                     //  y1 '----'-------'
        int y1 = bottom_left.y + height() / 2;      //     | SW |  SE   |
        int y2 = top_right.y;                       //  y0 '----'-------'
                                                    //     x0   x1     x2

        northwest = new Quadnode(obstacles,
                Point(x0, y1), Point(x1, y2), resolution, this);
        southwest = new Quadnode(obstacles,
                Point(x0, y0), Point(x1, y1), resolution, this);
        northeast = new Quadnode(obstacles,
                Point(x1, y1), Point(x2, y2), resolution, this);
        southeast = new Quadnode(obstacles,
                Point(x1, y0), Point(x2, y1), resolution, this);
    }

    //! \brief Check if a quadrant is adjacent to a given side of this node.
    //! \param side The direction.
    //! \param quad The quadrant.
    //! \return True if the quadrant is adjacent to the given side of this node.
    bool adjacent(Side side, Quadrant quad) const {

        static const bool ADJACENT[5][5] = {
                        /* NW     NE     SW     SE  */
                /* N */ { true,  true,  false, false },
                /* E */ { false, true,  false, true  },
                /* S */ { false, false, true,  true  },
                /* W */ { true,  false, true,  false }
        };

        return ADJACENT[side][quad];
    }

    //! \brief Obtain the mirror image of a quadrant on a given side.
    //! \param side The direction.
    //! \param quad The quadrant.
    //! \return The mirror quadrant on the given side.
    Quadrant reflect(Side side, Quadrant quad) const {

        static const Quadrant REFLECT[5][5] = {
                        /*   NW         NE         SW         SE    */
                /* N */ { SOUTHWEST, SOUTHEAST, NORTHWEST, NORTHEAST },
                /* E */ { NORTHEAST, NORTHWEST, SOUTHEAST, SOUTHWEST },
                /* S */ { SOUTHWEST, SOUTHEAST, NORTHWEST, NORTHEAST },
                /* W */ { NORTHEAST, NORTHWEST, SOUTHEAST, SOUTHWEST }
        };

        return REFLECT[side][quad];
    }

    //! \brief Given a direction, obtain its opposite.
    //! \param side The direction.
    //! \return The opposite direction.
    Side opposite(Side side) const {
                                        /*  N     E      S     W  */
        static const Side OPPOSITE[4] = { SOUTH, WEST, NORTH, EAST };

        return OPPOSITE[side];
    }

    //! \brief Obtain this node's quadrant relative to its parent.
    //! \pre The current node isn't the tree root.
    //! \return This node's quadrant relative to its parent.
    Quadrant quadrant() const //throw(std::logic_error) 
    {
        // if (!parent) {
        //     throw std::logic_error("The root node's quadrant isn't defined.");
        // }

        if (parent->northwest == this) {
            return NORTHWEST;
        } else if (parent->southwest == this) {
            return SOUTHWEST;
        } else if (parent->northeast == this) {
            return NORTHEAST;
        } else {
            return SOUTHEAST;
        }
    }

    //! \brief Given a quadrant, obtain a pointer to the associated child node.
    //! \param quad The quadrant.
    //! \return A pointer to the child node associated to this quadrant.
    Quadnode* child(Quadrant quad) const {
        switch (quad) {
        case NORTHWEST:
            return northwest;
        case SOUTHWEST:
            return southwest;
        case NORTHEAST:
            return northeast;
        case SOUTHEAST:
            return southeast;
        }
    }

    //! \brief Append all the leaf children of this node, i.e. either free or
    //! obstructed node, that can be found in a given direction to a vector.
    //! \param direction The side where to look for leaf children.
    //! \param[out] children The vector to which the children will be appended.
    void children(Side direction, std::vector<Quadnode*>& children) const {

        if (is_leaf())
            return;

        Quadnode* s1;
        Quadnode* s2;

        switch (direction) {
        case NORTH:
            s1 = northeast;
            s2 = northwest;
            break;
        case EAST:
            s1 = northeast;
            s2 = southeast;
            break;
        case SOUTH:
            s1 = southeast;
            s2 = southwest;
            break;
        case WEST:
            s1 = northeast;
            s2 = southwest;
        }

        if (s1->is_leaf()) {
            children.push_back(s1);
        } else {
            s1->children(direction, children);
        }

        if (s2->is_leaf()) {
            children.push_back(s2);
        } else {
            s2->children(direction, children);
        }
    }

    //! \brief Locate an equal-sized neighbor of the current node in the
    //! vertical or horizontal direction; cf. Hanan Samet 1981 article Neighbor
    //! Finding in Quadtrees.
    //! \param direction The direction where to look for the neighbor.
    //! \return A pointer to the neighbor or NULL if it can't be found.
    Quadnode* neighbor(Side direction) const {
        Quadnode* neighbor = NULL;

        // Ascent the tree up to a common ancestor.
        if (parent and adjacent(direction, quadrant())) {
            neighbor = parent->neighbor(direction);
        } else {
            neighbor = parent;
        }

        // Backtrack mirroring the ascending moves.
        if (neighbor and !neighbor->is_leaf()) {
            return neighbor->child(reflect(direction, quadrant()));
        } else {
            return neighbor;
        }
    }

    //! \brief Locate a neighbor of the current quadnode in the horizontal or
    //! vertical direction which is adjacent to one of its corners. The
    //! neighboring node must be adjacent to this corner.
    //! \param direction The direction where to look for the neighbor.
    //! \param corner The corner to which the neighboring node must be adjacent.
    //! \return A pointer to the neighbor or NULL if it can't be found.
    Quadnode* neighbor(Side direction, Quadrant corner) const {

        // If no neighbor can be found in the given
        // direction, quadnode will be null.
        Quadnode* quadnode = neighbor(direction);

        if (!quadnode)
            return NULL;

        // Go down until we reach either a free or
        // an obstructed node, i.e. a leaf node.
        while (!quadnode->is_leaf()) {
            quadnode = quadnode->child(reflect(direction, corner));
        }

        return quadnode;
    }

    //! \brief Locate all leaf neighbours of the current node in the given
    //! direction, appending them to a given vector.
    //! \param direction The side where to look for neighbours.
    //! \param[out] neighbours The vector to which the neighbours will be appended.
    void neighbours(Side direction, std::vector<Quadnode*>& neighbours) const {

        // If no neighbor can be found in the given
        // direction, quadnode will be null.
        Quadnode* quadnode = neighbor(direction);

        if (quadnode != NULL) {
            if (quadnode->is_leaf()) {
                // Neighbor is already a leaf node, we're done.
                neighbours.push_back(quadnode);
            } else {
                // The neighbor isn't a leaf node so we need to
                // go further down matching its children, but in
                // the opposite direction from where we came.
                quadnode->children(opposite(direction), neighbours);
            }
        }
    }

public:

    //! \brief Retrieve a list of all leaf neighbours of the current node.
    //! \return A vector containing pointers to each node's neighbor.
    std::vector<Quadnode*> neighbours() const {
        std::vector<Quadnode*> _neighbours;
        neighbours(NORTH, _neighbours);
        neighbours(SOUTH, _neighbours);
        neighbours(EAST, _neighbours);
        neighbours(WEST, _neighbours);
        return _neighbours;
    }

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

    Point origine() const {
        return bottom_left;
    }

    //! \brief Check if this node is a leaf, i.e. is either obstructed or free.
    //! \return True if this node is a leaf.
    inline bool is_leaf() const {
        return status != MIXED;
    }

    //! \brief Check if this node is a free.
    //! \return True if this node is a free.
    inline bool is_free() const {
        return status == FREE;
    }

    //! \brief Obtain the straight-line distance between this node and another.
    //! \param other A pointer to the other node.
    //! \return The straight-line distance between the two nodes.
    float distance(const Quadnode* other) {

        // Pythagore is your friend !
        //
        //         a    (x1,y1)
        //    .-----------.
        //    |        .-'
        //  b |     .-'
        //    |  .-'
        //    '-'
        // (x2, y2)

        float x1 = bottom_left.x + width() / 2.0;
        float y1 = bottom_left.y + height() / 2.0;

        float x2 = other->bottom_left.x + other->width() / 2.0;
        float y2 = other->bottom_left.y + other->height() / 2.0;

        float a = abs(x1 - x2);
        float b = abs(y1 - y2);

        return sqrt(pow(a, 2) + pow(b, 2));
    }

    //! \brief Check if a given point is inside the region represented by this node.
    //! \param point The point to check for containment.
    //! \return True if the point is inside the region represented by this node.
    bool inbound(const Point& point) const {
        return (bottom_left.x <= point.x && point.x <= top_right.x)
                && (bottom_left.y <= point.y && point.y <= top_right.y);
    }

    //! \brief Destructor.
    ~Quadnode() {
        // Only mixed node have children.
        if (status == MIXED) {
            delete northwest;
            delete southwest;
            delete northeast;
            delete southeast;
        }
    }
};

//! \brief Quadtree implementation.
class Quadtree {
protected:
    std::shared_ptr<Quadnode> root; /**< Quadtree's root node. */

public:
    Quadtree(const std::vector<Obstacle>& obstacles, int size = 2,
            int resolution = 1) //throw (std::logic_error) 
            {

        // To ensure a consistent behavior and eliminate corner cases, the
        // Quadtree's root node need to have children, i.e. it can't
        // be a leaf node. Thus, the first instantiated Quadnode need to
        // always be subdivided. These two conditions make sure that
        // even with this subdivision the resolution will be respected.
        // if (resolution < 1) {
        //     throw std::logic_error("Resolution must be greater than 0.");
        // } else if (size < resolution * 2) {
        //     throw std::logic_error("Grid size must be greater or equal "
        //                            "to twice the resolution value.");
        // }

        // Can't use make_shared since the Quadnode constructor is protected.
        root = std::shared_ptr<Quadnode>(new Quadnode(obstacles, Point(0, 0),
                Point(size, size), resolution));
    }

    //! \brief Given a point, find the leaf Quadnode associated containing it.
    //! \param point The point searched for.
    //! \return A pointer to the containing Quadnode.
    Quadnode* find_node(const Point& point) //throw(std::logic_error) 
    {

        // Before doing anything else, make sure
        // that the requested point is inbound.
        // if (!root->inbound(point)) {
        //     throw std::logic_error("Input point is out of bounds.");
        // }

        Quadnode* node = root.get();
        while (!node->is_leaf()) {
            int x_mean = node->bottom_left.x + node->width() / 2;
            int y_mean = node->bottom_left.y + node->height() / 2;

            if (point.x < x_mean) {
                if (point.y < y_mean) {
                    node = node->southwest;
                } else {
                    node = node->northwest;
                }
            } else {
                if (point.y < y_mean) {
                    node = node->southeast;
                } else {
                    node = node->northeast;
                }
            }
        }

        return node;
    }
};

//! \brief Abstraction of A* node.
//!
//! The AStarNode is mainly used to store A* specific informations
//! associated to a given Quadnode while executing the A* algorithm.
struct AStarNode {
    //! \brief AStarNode constructor.
    //! \param ancestor A pointer to the AStarNode from where we came from.
    //! \param quadnode A pointer to the Quadnode that we're representing.
    //! \param path The actual cost from the stored Quadnode to the start one.
    //! \param estimate The estimated cost from the stored Quadnode to the
    //! destination one.
    AStarNode(AStarNode* ancestor, Quadnode* quadnode,
            float path = 0, float estimate = 0) :
        ancestor(ancestor), quadnode(quadnode), total_cost(path + estimate),
        path_cost(path), estimated_cost(estimate) {
    }

    //! \brief Check if this AStarNode contains (represents) a given Quadnode.
    //! \param quadnode The query quadnode.
    //! \return True if this AStarNode contains the quadnode.
    bool contains(const Quadnode* quadnode) const {
        return this->quadnode == quadnode;
    }

    //! \brief Wrapper around the stored Quadnode neighbours method.
    //! \return A vector containing the stored Quadnode neighbours.
    std::vector<Quadnode*> neighbours() const {
        return quadnode->neighbours();
    }

    //! \brief Calculate the distance between this AStarNode and another.
    //! \param other A pointer to another AStarNode.
    //! \return The distance between the two AStarNode.
    float distance(AStarNode* other) const {
        return quadnode->distance(other->quadnode);
    }

    //! \brief Calculate the distance between this AStarNode and a given Quadnode.
    //! \param other A pointer to a Quadnode.
    //! \return The distance between the two AStarNode.
    float distance(Quadnode* other) const {
        return quadnode->distance(other);
    }

    AStarNode* ancestor;    /**< The node ancestor, used to rebuild path. */
    Quadnode* quadnode;     /**< Corresponding quadnode. */
    float total_cost;       /**< (F) Total cost. */
    float path_cost;        /**< (G) Path cost from the starting point. */
    float estimated_cost;   /**< (H) Estimated path cost to the goal. */
};

//! \brief A* heap comparison functor.
struct AStarHeapComparator {

    //! \brief Given two AStarNode, check if the total cost induced by the
    //! first one is smaller than the total cost induced by the second one.
    //! \param node1 A pointer to an AStarNode
    //! \param node2 A pointer to an AStarNode
    //! \return True if the total cost of node1 is smaller than the total cost
    //! of node2.
    bool operator()(const AStarNode* node1, const AStarNode* node2) {
        // Sort in reverse order since pop_heap removes the largest element.
        return node1->total_cost > node2->total_cost;
    }
};

//! \brief A* open and closed set search functor.
//!
//! This binary functor is used to search both the A* algorithm open and closed
//! set. In order to use it in conjuction with the std::find_if algorithm, which
//! is expecting a unary predicate, bind the second argument with std::bind2nd.
struct AStarNodeFinder : public std::binary_function<AStarNode*, Quadnode*, bool> {
    //! \brief Check if a given AStarNode contains a given Quadnode.
    //! \param anode A pointer to an AStarNode
    //! \param qnode A pointer to a Quadnode
    //! \return True if the AStarNode contains the given Quadnode.
    bool operator()(const AStarNode* anode, const Quadnode* qnode) const {
       return anode->contains(qnode);
    }
};

//!Â \brief Implementation of the A* algorithm.
//! \param start The starting node.
//! \param destination The destination (goal) node.
//! \return A vector containg pointers to node forming the path in ascendant
//! order, i.e. the start node is at position 0 and the goal node is at
//! position length - 1. If there is no existing path from the start node
//! to the destination node, the returned vector will be empty.
std::vector<Quadnode*> astar(Quadnode* start, Quadnode* destination) {

    std::vector<AStarNode*> open;   // Vector used as a heap, cf. Steve Rabin
    std::vector<AStarNode*> closed;

    // Prime the open set with the start node.
    open.push_back(new AStarNode(NULL, start, 0, start->distance(destination)));

    // Loop until either the destination Quadnode is reached or the
    // open set is exhausted, meaning that no solution where found.
    while (!open.empty() && !open.front()->contains(destination)) {

        // Retrieve the best node from the open set, i.e. the
        // one with the lowest total cost (path + estimate).
        AStarNode* current = open.front();
        std::pop_heap(open.begin(), open.end(), AStarHeapComparator());
        open.pop_back();

        // Retrieve the current (stored) quadnode neighbours.
        std::vector<Quadnode*> qneighbours = current->neighbours();

        // Go over each quadnode neighbor.
        for (std::vector<Quadnode*>::const_iterator iterator = qneighbours.begin();
                iterator != qneighbours.end(); ++iterator) {

            Quadnode* qneighbor = *iterator;

            // If this neighbor isn't free, then
            // don't waste time considering it.
            if (!qneighbor->is_free())
                continue;

            // Search the AStarNode of the closed list for this quadnode neighbor.
            // If do we find it, simply skip it since it has already been computed.
            if (std::find_if(closed.begin(), closed.end(),
                    std::bind2nd(AStarNodeFinder(), qneighbor)) != closed.end()) {
                continue;
            }

            // Search the AStarNode of the open list for this quadnode neighbor.
            std::vector<AStarNode*>::iterator open_set_iterator;
            open_set_iterator = std::find_if(open.begin(), open.end(),
                    std::bind2nd(AStarNodeFinder(), qneighbor));

            // If this neighbor is in the open list and our current g value
            // is lower; update the neighbor with the new, lower, g value
            // and change the neighbor's parent to our current node.
            if (open_set_iterator != open.end()) {

                AStarNode* astarnode = *open_set_iterator;

                // We'll need the total path cost from our current position
                // to this neighbor quadnode for the following operations.
                float path_cost = current->path_cost + current->distance(astarnode);

                if (path_cost < astarnode->path_cost) {

                    float estimated_cost = astarnode->estimated_cost;
                    astarnode->path_cost = path_cost;
                    astarnode->total_cost = estimated_cost + path_cost;
                    astarnode->ancestor = current;

                    std::make_heap(open.begin(), open.end(), AStarHeapComparator());
                }
            } else {
                // This quadnode neighbor is neither in the open nor in the closed
                // list; so add it to the open list and set its path value.
                float path_cost = current->path_cost + current->distance(qneighbor);
                float estimated_cost = qneighbor->distance(destination);

                open.push_back(new AStarNode(current, qneighbor, path_cost, estimated_cost));
                push_heap(open.begin(), open.end(), AStarHeapComparator());
            }
        }

        // We're done with this node.
        // Put it into the closed set.
        closed.push_back(current);
    }

    std::vector<Quadnode*> path;

    // If the open set isn't empty, it means that we've reached the
    // destination. From it, rebuild the path to the starting point.
    if (!open.empty()) {
        AStarNode* current = open.front();
        while (current) {
            path.push_back(current->quadnode);
            current = current->ancestor;
        }
        std::reverse(path.begin(), path.end());
    }

    // Before returning, clean up dynamically allocated assets.
    for (std::vector<AStarNode*>::iterator iter = open.begin();
            iter != open.end(); ++iter) {
        delete *iter; // Cleaning open set.
    }

    for (std::vector<AStarNode*>::iterator iter = closed.begin();
            iter != closed.end(); ++iter) {
        delete *iter; // Cleaning closed set.
    }

    return path;
}


int astartest()
{
    std::vector<Obstacle> obstacles;
    obstacles.push_back(Obstacle(Point(0, 48), Point(32, 64)));
    obstacles.push_back(Obstacle(Point(32, 16), Point(64, 32)));
    obstacles.push_back(Obstacle(Point(32, 32), Point(48, 64)));
    obstacles.push_back(Obstacle(Point(48, 48), Point(64, 112)));
    obstacles.push_back(Obstacle(Point(80, 32), Point(96, 64)));

    Quadtree quadtree(obstacles, 128);

    Quadnode* q00 = quadtree.find_node(Point(0, 0));
    Quadnode* q04 = quadtree.find_node(Point(90, 40));

    std::vector<Quadnode*> path = astar(q00, q04);
    std::cout << path.size() << std::endl;

    for (std::vector<Quadnode*>::iterator it = path.begin();
            it != path.end(); ++it) {
        Point origine = (*it)->origine();
        std::cout << origine.x << ":" << origine.y << std::endl;
    }
}
