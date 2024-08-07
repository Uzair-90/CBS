#pragma once
#include <memory>
#include <utility>
#include <unordered_set>
#include<unordered_map>
#include<bits/stdc++.h>
#include "AStar.hpp"

//structure to catch conflict
struct Conflict {
    int agent1, agent2;
    GridPoint point;
    int timeStamp;
    Conflict(int a1, int a2, GridPoint p, int t)
        : agent1(a1), agent2(a2), point(p), timeStamp(t) {}
};

// Constraint Tree Node
struct CTNode {
    Cost cost{};
    std::vector<Cost> costs;
    std::vector<Constraint> constraints;
    std::vector<std::vector<GridPoint>> solution;
    Conflict getFirstConflict();
    bool checkForEdgeConflict(const std::vector<GridPoint>& pathOfOneAgent, const std::vector<GridPoint>& pathOfAnotherAgent, int i);
    std::shared_ptr<CTNode> leftChild, rightChild, parent;
    CTNode() = default;
    CTNode(Cost c, std::vector<Constraint> &con,
           std::vector<std::vector<GridPoint>> &s, std::vector<Cost> cc)
        : cost(c),
          costs(std::move(cc)),
          constraints(con),
          solution(s),
          leftChild(nullptr),
          rightChild(nullptr),
          parent(nullptr) {}
};

//comparator for CTNode 
struct CompareCTNode {
    bool operator()(const std::shared_ptr<CTNode>& lhs, const std::shared_ptr<CTNode>& rhs) const {
        return lhs->cost > rhs->cost; // Min-heap (lower cost has higher priority)
    }
};


class CBS {
   private:
    int numberOfAgents;
    int dimX, dimY;
    std::vector<GridPoint> obstacles;
    std::vector<GridPoint> starts;
    std::vector<GridPoint> goals;
    std::shared_ptr<CTNode> root;
    std::shared_ptr<CTNode> solutionNode;
    int cnt;

   public:
    CBS(int n, int xx, int yy, std::vector<GridPoint> &o,
        std::vector<GridPoint> &s, std::vector<GridPoint> &g)
        : numberOfAgents(n),
          dimX(xx),
          dimY(yy),
          obstacles(o),
          starts(s),
          goals(g),
          root(nullptr),
          solutionNode(nullptr),
          cnt(0) {}
    void splitOnConflict(Conflict con, std::shared_ptr<CTNode>& node);
    void search();
    const std::shared_ptr<CTNode>& getSolutionNode() { return solutionNode; }
    void onNewNode();
};


//will give a msg for checking 100 nodes.
inline void CBS::onNewNode() {
    cnt++;
    if (cnt % 100 == 0) std::cout << "Checked " << cnt << " nodes.\n";
}

inline bool CTNode::checkForEdgeConflict(const std::vector<GridPoint>& pathOfOneAgent, const std::vector<GridPoint>& pathOfAnotherAgent, int i) {
    // Ensure indices are within bounds
    if (i + 1 >= pathOfOneAgent.size() || i + 1 >= pathOfAnotherAgent.size()) {
        return false;
    }

    // Check if agents swap positions at time i
    return (((pathOfOneAgent[i].x == pathOfAnotherAgent[i + 1].x) && (pathOfOneAgent[i].y == pathOfAnotherAgent[i + 1].y ) && (pathOfOneAgent[i + 1].x == pathOfAnotherAgent[i].x) && (pathOfOneAgent[i].y == pathOfAnotherAgent[i + 1].y)));
    // && pathOfOneAgent[i] != pathOfOneAgent[i + 1]); // Avoid self-loop
}

//getting first conflict
inline Conflict CTNode::getFirstConflict() {
    Conflict con(-1, -1, GridPoint(-1, -1,Direction::Down), -1);
    std::vector<int> lengths;
    lengths.resize(solution.size());
    std::transform(
        solution.begin(), solution.end(), lengths.begin(),
        [](const std::vector<GridPoint> &path) { return path.size(); });

    std::vector<GridPoint> pointsAtTimei;
    int longestLength = *std::max_element(lengths.begin(), lengths.end());

        for (int i = 0; i < longestLength; i++) {
        // i is the timeStamp
        pointsAtTimei.clear();
        int agent2 = -1;

        for (std::vector<GridPoint> pathOfOneAgent : solution) {
                agent2++;
                if (i < pathOfOneAgent.size()) {
                    auto it=pointsAtTimei.begin();
                    for(; it != pointsAtTimei.end(); it++) if(it->x == pathOfOneAgent[i].x && it->y == pathOfOneAgent[i].y) break;
                    if(it == pointsAtTimei.end()){
                        // no conflict
                        std::cout << "NO CON" << std::endl;
                        std::cout << "C " << pathOfOneAgent[i].x << " " << pathOfOneAgent[i].y << std::endl;
                        for(auto tt:pointsAtTimei){
                            std::cout << tt.x << " " << tt.y << " " << tt.direction << std::endl;
                        }
                    }else{
                        int agent = it - pointsAtTimei.begin();
                        std::cout << it->x << " " << it->y << std::endl;
                        return {agent, agent2, *it, i};
                    }
                    pointsAtTimei.push_back(pathOfOneAgent[i]);
                }
            }
        }

        for (int i = 0; i < longestLength - 1; ++i) { // Check up to the second last position
            for (int agent1 = 0; agent1 < solution.size(); ++agent1) {
                for (int agent2 = agent1 + 1; agent2 < solution.size(); ++agent2) {
                    if (checkForEdgeConflict(solution[agent1], solution[agent2], i)) {
                        return Conflict(agent1, agent2, solution[agent1][i], i);
                    }
                }
            }
        }

    return con;
}


inline void CBS::splitOnConflict(Conflict con, std::shared_ptr<CTNode>& node) {
    // split current node into two nodes
    // each has a new set of constraints
    int agent1, agent2;
    agent1 = con.agent1;
    agent2 = con.agent2;
    int conflictTimeStamp = con.timeStamp;
    GridPoint conflictPoint = con.point;
    // generate new contraints;
    Constraint c1(agent1, conflictPoint, conflictTimeStamp);
    Constraint c2(agent2, conflictPoint, conflictTimeStamp);
    std::vector<Constraint> new1;
    std::vector<Constraint> new2;
    // TODO: for each agent, maintain a constraint list
    for (Constraint c : node->constraints)
        if (c.agent == agent1)
            new1.push_back(c);
        else if (c.agent == agent2)
            new2.push_back(c);

    if (std::find(new1.begin(), new1.end(), c1) == new1.end()) {

        new1.push_back(c1);
        auto newNode1 = std::make_shared<CTNode>(node->cost, new1,node->solution, node->costs);
        AStar lowLevelSearchObj1(dimX, dimY, obstacles, new1);

        newNode1->solution[agent1] = lowLevelSearchObj1.search(starts[agent1], goals[agent1]);
        Cost cost1 = lowLevelSearchObj1.getFinalCost();
        newNode1->cost -= newNode1->costs[agent1];
        newNode1->cost += cost1;
        newNode1->costs[agent1] = cost1;
        // add children to node
        node->leftChild = std::move(newNode1);
    }
    if (std::find(new2.begin(), new2.end(), c2) == new2.end()) {

        new2.push_back(c2);
        auto newNode2 = std::make_shared<CTNode>(node->cost, new2, node->solution, node->costs);
        AStar lowLevelSearchObj2(dimX, dimY, obstacles, newNode2->constraints);
        newNode2->solution[agent2] = lowLevelSearchObj2.search(starts[agent2], goals[agent2]);
        Cost cost2 = lowLevelSearchObj2.getFinalCost();
        newNode2->cost -= newNode2->costs[agent2];
        newNode2->cost += cost2;
        newNode2->costs[agent2] = cost2;
        node->rightChild = std::move(newNode2);
    }
}

void CBS::search() {
    root = std::make_shared<CTNode>();
    root->solution.resize(numberOfAgents);
    root->constraints.resize(numberOfAgents);

    std::vector<Constraint> cc;
    std::vector<GridPoint> pp;

    root->costs.resize(numberOfAgents);
    for (int i = 0; i < numberOfAgents; i++) {
        AStar lowLevelSearchObj(dimX, dimY, obstacles, cc);
        pp = lowLevelSearchObj.search(starts[i], goals[i]);
        if (pp.empty()) {
            std::cout << "No solution returned from A*.\n";
            return;
        }
        root->cost += lowLevelSearchObj.getFinalCost();
        root->costs[i] = lowLevelSearchObj.getFinalCost();
        root->solution[i] = pp;
    }

    std::priority_queue<std::shared_ptr<CTNode>, std::vector<std::shared_ptr<CTNode>>, CompareCTNode> openSet;
    openSet.push(root);

    Conflict conflict(-1, -1, GridPoint(-1, -1,Direction::Up), -1);
    while (!openSet.empty()) {
        std::shared_ptr<CTNode> currentCTNode = openSet.top();
        openSet.pop();
        onNewNode();

        conflict = currentCTNode->getFirstConflict();

        if (conflict.agent1 == -1 || conflict.agent2 == -1) {
            solutionNode = currentCTNode;
            return;
        }

        splitOnConflict(conflict, currentCTNode);

        if (currentCTNode->leftChild != nullptr)
            openSet.push(currentCTNode->leftChild);
        if (currentCTNode->rightChild != nullptr)
            openSet.push(currentCTNode->rightChild);
    }
}