#pragma once
#include <algorithm>
#include <iostream>
#include <map>
#include <queue>
#include <vector>
#include <cassert>
using Cost = int;

enum Direction {
    Up,
    Down,
    Left,
    Right
};

struct GridPoint {
    int x{0};
    int y{0};
    Direction direction;
    GridPoint() : x{0}, y{0}, direction{Up} {}
    GridPoint(int xx, int yy, Direction dir) : x{xx}, y{yy}, direction{dir} {}

    size_t operator()(const GridPoint& pointToHash) const noexcept {
        size_t hash = pointToHash.x + 10 * pointToHash.y + 10 *pointToHash.direction;
        return hash;
    }

    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y && direction == other.direction;
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
    return s << "(" << point.x << ", " << point.y << ", " << point.direction << ")";
}

struct Node {
    GridPoint point;
    Cost g;
    Cost f;
    int timeStamp;
    Node(GridPoint p, Cost gg, Cost ff, int t)
        : point(p), g(gg), f(ff), timeStamp(t) {}
    Node(GridPoint p, Cost gg, Cost ff, int t, Direction direc)
        : point(p), g(gg), f(ff), timeStamp(t) {
            point.direction = direc;
        }
};

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

typedef std::pair<int, GridPoint> GridPointT;
class AStar {
   private:
    int dimX, dimY;
    std::vector<GridPoint> closedSet;
    std::vector<GridPoint> obstacles;
    std::vector<Constraint> constraints;
    int timeStamp;
    Cost finalCost;
    std::map<GridPointT, GridPointT> parentMap; // Map to track parent nodes

   public:
    AStar(int xx, int yy, std::vector<GridPoint> &o, std::vector<Constraint> &cc)
        : dimX(xx), dimY(yy), obstacles(o), constraints(cc), timeStamp(0), finalCost(0) {}

    std::vector<GridPoint> search(GridPoint &start, GridPoint &goal);
    static Cost getCost(GridPoint p1, GridPoint p2);
    std::vector<GridPoint> getAdjacentGridPoints(GridPoint &p);
    static Cost heuristic(GridPoint s, GridPoint g);
    GridPointT getParent(const GridPointT& p) const;
    Cost getFinalCost() const;
};

inline std::vector<GridPoint> AStar::search(GridPoint& start, GridPoint& goal) {
    std::vector<GridPoint> path;
    Cost h = heuristic(start, goal);
    Node startNode = Node(start, 0, h, 0, start.direction); // Initial direction is Up

    std::priority_queue<Node, std::vector<Node>, greater> openQueue;
    openQueue.push(startNode);
    parentMap[{0, start}] = {0, start}; // Start node's parent is itself

    while (!openQueue.empty()) {
        Node currentNode = openQueue.top();
        openQueue.pop();
        closedSet.push_back(currentNode.point);
        timeStamp = currentNode.timeStamp;

        if (currentNode.point.x == goal.x && currentNode.point.y == goal.y) {
            finalCost = currentNode.f;
            GridPointT n = {currentNode.timeStamp, currentNode.point};
            while (!(n.second == start)) {
                std::cout << n.second << std::endl;
                path.push_back(n.second);
                n = getParent(n); // Get parent from the map
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        std::vector<GridPoint> childrenGridPoints = getAdjacentGridPoints(currentNode.point);

        for (const GridPoint& childGridPoint : childrenGridPoints) {
            if (std::find(closedSet.begin(), closedSet.end(), childGridPoint) != closedSet.end()) continue;

            Cost newg = currentNode.g + getCost(childGridPoint, currentNode.point);
            Cost newf = newg + heuristic(childGridPoint, goal);
            Direction newDirection = childGridPoint.direction;

            Node child = Node(childGridPoint, newg, newf, currentNode.timeStamp + 1, newDirection);

            bool isInOpenSet = false;
            std::vector<Node> updatedOpenSet;
            while (!openQueue.empty()) {
                Node n = openQueue.top();
                openQueue.pop();
                if (n.point == childGridPoint && n.timeStamp == child.timeStamp) {
                    isInOpenSet = true;
                    if (child.f < n.f) {
                        updatedOpenSet.push_back(child);
                    } else {
                        updatedOpenSet.push_back(n);
                    }
                } else {
                    updatedOpenSet.push_back(n);
                }
            }

            if (!isInOpenSet) {
                openQueue.push(child);
                parentMap[{child.timeStamp, childGridPoint}] = {currentNode.timeStamp, currentNode.point}; // Update parent map
            }

            // Re-add remaining nodes to the priority queue
            for (const Node& node : updatedOpenSet) {
                openQueue.push(node);
            }
        }
    }
    return path;
}

inline Cost AStar::getCost(GridPoint p1, GridPoint p2) {
    return 1;
    //return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
}

inline Cost AStar::heuristic(GridPoint s, GridPoint g) {
    return std::abs(s.x - g.x) + std::abs(s.y - g.y);
}

inline GridPointT AStar::getParent(const GridPointT& p) const {
    auto it = parentMap.find(p);
    if (it != parentMap.end()) {
        return it->second;
    }
    assert(false);
}

inline Cost AStar::getFinalCost() const {
    return finalCost;
}


inline std::vector<GridPoint> AStar::getAdjacentGridPoints(GridPoint& currentPoint) {
    std::vector<GridPoint> adjGridPoints;
    // Define possible moves based on direction
    std::vector<std::tuple<int, int, Direction>> moves;
    
    if (currentPoint.direction == Direction::Left || currentPoint.direction == Direction::Right) {
        moves = {
            {currentPoint.x - 1, currentPoint.y, currentPoint.direction}, 
            {currentPoint.x + 1, currentPoint.y, currentPoint.direction}, 
            {currentPoint.x, currentPoint.y, currentPoint.direction}, 
            {currentPoint.x, currentPoint.y, Direction::Up}, 
            {currentPoint.x, currentPoint.y, Direction::Down}
        };
    } else if (currentPoint.direction == Direction::Up || currentPoint.direction == Direction::Down) {
        moves = {
            {currentPoint.x, currentPoint.y - 1, currentPoint.direction}, 
            {currentPoint.x, currentPoint.y + 1, currentPoint.direction}, 
            {currentPoint.x, currentPoint.y, currentPoint.direction}, 
            {currentPoint.x, currentPoint.y, Direction::Left}, 
            {currentPoint.x, currentPoint.y, Direction::Right}
        };
    }
    // Process each move
    for (const auto& move : moves) {
        int x = std::get<0>(move);
        int y = std::get<1>(move);
        Direction dir = std::get<2>(move);
        // Boundary check
        if (x >= 0 && x < dimX && y >= 0 && y < dimY) {
            GridPoint newPoint(x, y, dir); // Include direction in newPoint
            // Obstacle check
            if (std::find(obstacles.begin(), obstacles.end(), newPoint) == obstacles.end()) {
                bool flag = true;
                // Constraint check
                for (Constraint c : constraints) {
                    if ((c.point.x == newPoint.x && c.point.y == newPoint.y) && c.constraintTimeStamp == timeStamp + 1) {
                        flag = false;
                        break;
                    }
                }
                if (flag) {
                    // Add new point with the same direction as current node if possible
                    adjGridPoints.push_back(newPoint);
                }
            }
        }
    }

    return adjGridPoints;
}