#pragma once
#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>

using Cost = int;

enum Direction {
    UP, DOWN, LEFT, RIGHT, STAY
};

struct GridPoint {
    int x{0};
    int y{0};
    Direction direction{UP};
    GridPoint() : x{0}, y{0}, direction{UP} {}
    GridPoint(int xx, int yy) : x{xx}, y{yy} {}
    GridPoint(int xx, int yy, Direction direc) : x{xx}, y{yy}, direction{direc} {}
    bool operator==(const GridPoint &p) const { return p.x == x && p.y == y; }
};

inline std::ostream &operator<<(std::ostream &s, const GridPoint &point) {
    return s << "(" << point.x << "," << point.y << ")";
}

struct Node {
    GridPoint point;
    Cost g;
    Cost f;
    GridPoint parent;
    int timeStamp;
    Node(GridPoint p, Cost gg, Cost ff, GridPoint pa)
        : point(p), g(gg), f(ff), parent(pa), timeStamp(0) {}
    Node(GridPoint p, Cost gg, Cost ff, GridPoint pa, int t)
        : point(p), g(gg), f(ff), parent(pa), timeStamp(t) {}
    Node(GridPoint p, Cost gg, Cost ff,GridPoint pa ,int t, Direction new_direction)
        : point(p), g(gg),f(ff), parent(pa), timeStamp(t) {
            point.direction = new_direction;
        }
};
// used to sort Nodes, very slow, but easy to use
struct greater {
    bool operator()(const Node &a, const Node &b) { return a.f > b.f; }
};

struct Constraint {
    int agent{};
    GridPoint point;
    int constraintTimeStamp{};
    Constraint() = default;
    Constraint(int a, GridPoint p, int t)
        : agent(a), point(p), constraintTimeStamp(t) {}
    Constraint(GridPoint p, int t) : point(p), constraintTimeStamp(t) {}
    bool operator==(const Constraint &cc) const {
        return cc.agent == agent && cc.point == point &&
               cc.constraintTimeStamp == constraintTimeStamp;
    }
};

class AStar {
   private:
    int dimX, dimY;
    std::vector<Node> openSet;
    std::vector<Node> closedSet;
    std::vector<GridPoint> obstacles;
    std::vector<Constraint> constraints;
    int timeStamp;
    Cost finalCost;

   public:
    AStar(int xx, int yy, std::vector<GridPoint> &o,
          std::vector<Constraint> &cc)
        : dimX(xx),
          dimY(yy),
          obstacles(o),
          constraints(cc),
          timeStamp(0),
          finalCost(0) {}
    std::vector<GridPoint> search(GridPoint &start, GridPoint &goal);
    static Cost getCost(GridPoint p1, GridPoint p2);
    std::vector<GridPoint> getAdjacentGridPoints(GridPoint &p);
    static Cost heuristic(GridPoint s, GridPoint g);
    Cost getDirectionCost(Direction old, Direction New){
        return (old != New)? 1: 0;
    }
    Node getParent(GridPoint p);
    Cost getFinalCost() const;
    int Rotationcost(Direction form, Direction to);
};


inline int AStar::Rotationcost(Direction from, Direction to){
    return (from == to) ? 0 : 1;
}



// inline std::vector<GridPoint> AStar::search(GridPoint &start, GridPoint &goal) {
//     std::vector<GridPoint> path;
//     Cost h = heuristic(start, goal);
//     Node startNode(start, 0, h, start, 0);
//     openSet.push_back(startNode);

//     while (!openSet.empty()) {
//         std::sort(openSet.begin(), openSet.end(), greater());
//         Node currentNode = openSet.back();
//         openSet.pop_back();
//         closedSet.push_back(currentNode);
//         timeStamp = currentNode.timeStamp;

//         if (currentNode.point == goal) {
//             finalCost = currentNode.f;
//             Node n = currentNode;
//             while (!(n.point == start)) {
//                 path.push_back(n.point);
//                 n = getParent(n.point);
//             }
//             path.push_back(start);
//             std::reverse(path.begin(), path.end());
//             return path;
//         }

//         std::vector<GridPoint> childrenGridPoints = getAdjacentGridPoints(currentNode.point);

//         for (const GridPoint &childGridPoint : childrenGridPoints) {
//             bool inClosedSet = std::find_if(closedSet.begin(), closedSet.end(),
//                 [&childGridPoint](const Node &n) { return n.point == childGridPoint; }) != closedSet.end();

//             if (inClosedSet) continue;

//             // Calculate cost including direction change
//             int directionCost = getDirectionCost(currentNode.point.direction, childGridPoint.direction);
//             Cost newg = currentNode.g + getCost(childGridPoint, currentNode.point) + directionCost;
//             Cost newf = newg + heuristic(childGridPoint, goal);
//             Node child(childGridPoint, newg, newf, currentNode.point, currentNode.timeStamp + 1);

//             // Check if the node with the same point but different direction is in openSet
//             auto it = std::find_if(openSet.begin(), openSet.end(),
//                 [&childGridPoint](const Node &n) { return n.point.x == childGridPoint.x && n.point.y == childGridPoint.y; });

//             if (it != openSet.end()) {
//                 if (child.f < it->f) {
//                     it->g = child.g;
//                     it->f = child.f;
//                     it->timeStamp = child.timeStamp;
//                     it->parent = child.parent;
//                     it->point.direction = child.point.direction; // Update direction as well
//                 }
//             } else {
//                 openSet.push_back(child);
//             }
//         }
//     }

//     return path;
// }




inline std::vector<GridPoint> AStar::search(GridPoint &start, GridPoint &goal) {
    std::vector<GridPoint> path;
    Cost h = heuristic(start, goal);
    Node startNode(start, 0, h, start, 0);
    openSet.push_back(startNode);

    while (!openSet.empty()) {
        std::sort(openSet.begin(), openSet.end(), greater());
        Node currentNode = openSet.back();
        openSet.pop_back();
        closedSet.push_back(currentNode);
        timeStamp = currentNode.timeStamp;

        if (currentNode.point == goal) {
            finalCost = currentNode.f;
            Node n = currentNode;
            while (!(n.point == start)) {
                path.push_back(n.point);
                n = getParent(n.point);
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        std::vector<GridPoint> childrenGridPoints = getAdjacentGridPoints(currentNode.point);

        for (const GridPoint &childGridPoint : childrenGridPoints) {
            // Check if the childGridPoint is in closedSet
            bool inClosedSet = std::find_if(closedSet.begin(), closedSet.end(),
                [&childGridPoint](const Node &n) { return n.point == childGridPoint; }) != closedSet.end();

            if (inClosedSet) continue;

            // Calculate cost including direction change
            int directionCost = getDirectionCost(currentNode.point.direction, childGridPoint.direction);
            Cost newg = currentNode.g + getCost(childGridPoint, currentNode.point) + directionCost;
            Cost newf = newg + heuristic(childGridPoint, goal);
            Node child(childGridPoint, newg, newf, currentNode.point, currentNode.timeStamp + 1);

            // Check if the node with the same point but different direction is in openSet
            auto it = std::find_if(openSet.begin(), openSet.end(),
                [&childGridPoint](const Node &n) { return n.point.x == childGridPoint.x && n.point.y == childGridPoint.y; });

            if (it != openSet.end()) {
                // Node with same coordinates but potentially different direction
                if (child.f < it->f) {
                    it->g = child.g;
                    it->f = child.f;
                    it->timeStamp = child.timeStamp;
                    it->parent = child.parent;
                    it->point.direction = child.point.direction; // Update direction as well
                }
            } else {
                // Handle direction changes separately
                if (directionCost > 0) {
                    // Add the node with updated direction to openSet
                    Node directionChangedNode(childGridPoint, newg, newf, currentNode.point, currentNode.timeStamp + 1);
                    openSet.push_back(directionChangedNode);
                } else {
                    // Standard node addition
                    openSet.push_back(child);
                }
            }
        }
    }

    return path;
}









inline std::vector<GridPoint> AStar::getAdjacentGridPoints(GridPoint &p) {
    std::vector<GridPoint> adjGridPoints;
    std::vector<std::pair<int, int>> moves{ {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {0, 0} };
    std::vector<Direction> directions = {UP, RIGHT, DOWN, LEFT, STAY};
    unsigned int i = 0;
    for (const auto &m : moves) {
        Direction new_direction = directions[i];

        if((p.direction == LEFT && new_direction == RIGHT) || (p.direction == RIGHT && new_direction == LEFT) ){
            new_direction = p.direction;
        }

        if((p.direction == UP && new_direction == DOWN) || (p.direction == DOWN && new_direction == UP)){
            new_direction = p.direction;
        }

        int dx = m.first, dy = m.second;
        int x = dx + p.x;
        int y = dy + p.y;
        if (x >= 0 && x < dimX && y >= 0 && y < dimY) {
            GridPoint newPoint(x, y, new_direction);

            if (std::find(obstacles.begin(), obstacles.end(), newPoint) == obstacles.end()) {
                bool flag = true;
                for (Constraint c : constraints)
                    if (c.point == newPoint && c.constraintTimeStamp == timeStamp + 1)
                    // not allowed to go to newPoint
                    {
                        std::cout << "constrained, not adding point "
                                  << newPoint << "\n";
                        flag = false;
                        break;
                    }
                if (flag) adjGridPoints.push_back(newPoint);
            }
        }

        ++i;
    }

    return adjGridPoints;
}

inline Cost AStar::getCost(GridPoint p1, GridPoint p2) {
    return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
}

inline Cost AStar::heuristic(GridPoint s, GridPoint g) {
    return std::abs(s.x - g.x) + std::abs(s.y - g.y);
}

inline Node AStar::getParent(GridPoint p) {
    Node newNode(GridPoint(-1, -1), -1, -1, GridPoint(-1, -1));
    for (Node n : closedSet) {
        if (n.point == p) {
            newNode = n;
            break;
        }
    }
    if (newNode.f == -1) {
        return newNode;
    }
    for (Node n : closedSet) {
        if (n.point == newNode.parent) {
            return n;
        }
    }
    return newNode;
}

inline Cost AStar::getFinalCost() const { return finalCost; }
