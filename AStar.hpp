#pragma once
#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>

using Cost = int;


enum  Direction {
    None,
    Up,
    Down,
    Left,
    Right
};



struct GridPoint {
    int x{0};
    int y{0};
    Direction direction;
    GridPoint() : x{0}, y{0}, direction{None} {}
    GridPoint(int xx, int yy): x{xx}, y{yy} {
        direction = Up;
    }
    GridPoint(int xx, int yy, Direction dir) : x{xx}, y{yy}, direction{dir} {}

    size_t operator()(const GridPoint& pointToHash) const noexcept {
        size_t hash = pointToHash.x + 10 * pointToHash.y;
        return hash;
    };

    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const GridPoint& other) const {
        return !(*this == other);
    }

    bool operator<(const GridPoint& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }

    bool operator>(const GridPoint& other) const {
        return other < *this;
    }

    bool operator<=(const GridPoint& other) const {
        return !(other < *this);
    }

    bool operator>=(const GridPoint& other) const {
        return !(*this < other);
    }
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
    Node(GridPoint p, Cost gg, Cost ff, GridPoint pa, int t, Direction direc)
        : point(p), g(gg), f(ff), parent(pa),timeStamp(t) {
            point.direction = direc;
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
        return cc.agent == agent && cc.point == point && cc.constraintTimeStamp == constraintTimeStamp;
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
    Node getParent(GridPoint p);
    Cost getFinalCost() const;
    Direction getDirection(GridPoint& current, GridPoint& child);
};

Direction AStar::getDirection(GridPoint& current, GridPoint& child) {
    if (child.x > current.x) {
        return Direction::Right;
    } else if (child.x < current.x) {
        return Direction::Left;
    } else if (child.y > current.y) {
        return Direction::Down;
    } else if (child.y < current.y) {
        return Direction::Up;
    } else {
        return Direction::None; // No movement
    }
}


inline std::vector<GridPoint> AStar::search(GridPoint& start, GridPoint& goal) {
    std::vector<GridPoint> path;
    Cost h = heuristic(start, goal);
    Node startNode = Node(start, 0, h, start, 0, Direction::None); // Initial direction is None
    openSet.push_back(startNode);

    while (!openSet.empty()) {
        std::sort(openSet.begin(), openSet.end(), greater());
        Node currentNode = openSet.back();
        openSet.pop_back();
        closedSet.push_back(currentNode);
        timeStamp = currentNode.timeStamp;

        if (currentNode.point.x == goal.x && currentNode.point.y == goal.y) {
            // Goal point found
            finalCost = currentNode.f;
            Node n = currentNode;
            while (n.point.x != start.x || n.point.y != start.y) {
                if (n.parent == n.point) break;
                path.push_back(n.point);
                n = getParent(n.point);
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        std::vector<GridPoint> childrenGridPoints = getAdjacentGridPoints(currentNode.point);
        std::vector<GridPoint> closedGridPoints;
        closedGridPoints.resize(closedSet.size());
        std::transform(closedSet.begin(), closedSet.end(), closedGridPoints.begin(), [](Node n) { return n.point; });

        for (GridPoint childGridPoint : childrenGridPoints) {
            auto it = std::find(closedGridPoints.begin(), closedGridPoints.end(), childGridPoint);
            if (it != closedGridPoints.end()) continue;

            Cost newg = currentNode.g + getCost(childGridPoint, currentNode.point);
            Cost newf = newg + heuristic(childGridPoint, goal);
            Direction newDirection = getDirection(currentNode.point, childGridPoint);

            // Handle direction change
            if (newDirection != currentNode.point.direction) {
                // Add an intermediate state with the new direction but old coordinates
                Node intermediateNode = Node(currentNode.point, newg, newf, currentNode.point, currentNode.timeStamp + 1, newDirection);
                openSet.push_back(intermediateNode);
            }

            Node child = Node(childGridPoint, newg, newf, currentNode.point, currentNode.timeStamp + 1, newDirection);
            bool flag = true;

            for (Node n : openSet) {
                if (n.point == childGridPoint) {
                    flag = false;
                    if (child.f < n.f) {
                        n.g = child.g;
                        n.f = child.f;
                        n.timeStamp = child.timeStamp;
                        n.parent = child.parent;
                        n.point.direction = child.point.direction; // Update direction
                    }
                }
            }
            if (flag) {
                openSet.push_back(child);
            }
        }
    }
    return path;
}


inline std::vector<GridPoint> AStar::getAdjacentGridPoints(GridPoint& currentPoint) {
    std::vector<GridPoint> adjGridPoints;
    std::vector<std::pair<int, int>> moves{
        {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {0, 0} // Stay
    };

    // Prioritize movement in the same direction
    switch (currentPoint.direction) {
        case Direction::Left:
            moves = {{-1, 0}, {0, 1}, {0, -1}, {1, 0}, {0, 0}};
            break;
        case Direction::Right:
            moves = {{1, 0}, {0, 1}, {0, -1}, {-1, 0}, {0, 0}};
            break;
        case Direction::Up:
            moves = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {0, 0}};
            break;
        case Direction::Down:
            moves = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {0, 0}};
            break;
    }

    for (const auto& m : moves) {
        int dx = m.first, dy = m.second;
        int x = dx + currentPoint.x;
        int y = dy + currentPoint.y;
        if (x >= 0 && x < dimX && y >= 0 && y < dimY) {
            GridPoint newPoint(x, y);
            if (std::find(obstacles.begin(), obstacles.end(), newPoint) == obstacles.end()) {
                bool flag = true;
                for (Constraint c : constraints)
                    if (c.point == newPoint && c.constraintTimeStamp == timeStamp + 1) {
                        flag = false;
                        break;
                    }
                if (flag) {
                    // Add new point with the same direction as current node if possible
                    adjGridPoints.push_back(newPoint);
                } else {
                    // If movement is blocked, add current point with a stay direction
                    adjGridPoints.push_back(currentPoint);
                }
            }
        }
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
        std::cout << "node not found";
        return newNode;
    }
    for (Node n : closedSet) {
        if (n.point == newNode.parent) {
            return n;
        }
    }
    std::cout << "node not found";
    return newNode;
}

inline Cost AStar::getFinalCost() const { return finalCost; }























































































// inline std::vector<GridPoint> AStar::search(GridPoint &start, GridPoint &goal) {

//     std::vector<GridPoint> path;
//     Cost h = heuristic(start, goal);
//     Node startNode = Node(start, 0, h, start, 0);
//     openSet.push_back(startNode);
    
//     while (!openSet.empty()) {
//         std::sort(openSet.begin(), openSet.end(), greater());
//         Node currentNode = openSet.back();
//         openSet.pop_back();  // delete the smallest element
//         // push into closedSet
//         closedSet.push_back(currentNode);
//         timeStamp = currentNode.timeStamp;
//         if (currentNode.point.x == goal.x && currentNode.point.y == goal.y) {
//             // goal point found
//             finalCost = currentNode.f;
//             Node n = currentNode;
//             while (n.point.x != start.x || n.point.y != start.y) {
//                 if (n.parent == n.point) break;
//                 path.push_back(n.point);
//                 n = getParent(n.point);
//             }
//             path.push_back(start);
//             std::reverse(path.begin(), path.end());
//             return path;
//         }
//         std::vector<GridPoint> childrenGridPoints = getAdjacentGridPoints(currentNode.point);
//         std::vector<GridPoint> closedGridPoints;
//         closedGridPoints.resize(closedSet.size());
//         std::transform(closedSet.begin(), closedSet.end(), closedGridPoints.begin(), [](Node n) { return n.point; });

//         for (GridPoint childGridPoint : childrenGridPoints) {
//             auto it = std::find(closedGridPoints.begin(), closedGridPoints.end(), childGridPoint);
//             // if point is in closed GridPoints
//             if (it != closedGridPoints.end()) continue;
//             Cost newg = currentNode.g + getCost(childGridPoint, currentNode.point);
//             Cost newf = newg + heuristic(childGridPoint, goal);
//             Node child = Node(childGridPoint, newg, newf, currentNode.point,currentNode.timeStamp + 1);
//             bool flag = true;

//             for (Node n : openSet) {
//                 if (n.point == childGridPoint) {
//                     flag = false;
//                     if (child.f < n.f) {
//                         n.g = child.g;
//                         n.f = child.f;
//                         n.timeStamp = child.timeStamp;
//                         n.parent = child.parent;
//                     }
//                 }
//             }
//             if (flag) {
//                 openSet.push_back(child);
//             }
//         }
//     }
//     return path;
// }

// inline std::vector<GridPoint> AStar::getAdjacentGridPoints(GridPoint &p) {
//     std::vector<GridPoint> adjGridPoints;
//     std::vector<std::pair<int, int>> moves{
//         {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {0, 0}};
//     for (const auto &m : moves) {
//         int dx = m.first, dy = m.second;
//         int x = dx + p.x;
//         int y = dy + p.y;
//         if (x >= 0 && x < dimX && y >= 0 && y < dimY) {
//             GridPoint newPoint(x, y);
//             if (std::find(obstacles.begin(), obstacles.end(), newPoint) == obstacles.end()) {
//                 bool flag = true;
//                 for (Constraint c : constraints)
//                     if (c.point == newPoint && c.constraintTimeStamp == timeStamp + 1)
//                     // not allowed to go to newPoint
//                     {
//                         flag = false;
//                         break;
//                     }
//                 if (flag) adjGridPoints.push_back(newPoint);
//             }
//         }
//     }

//     return adjGridPoints;
// }