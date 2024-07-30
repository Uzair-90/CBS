#include "CBS.hpp"

void test1() {

    std::vector<GridPoint> obstacles;
    obstacles.emplace_back(2, 1);
    obstacles.emplace_back(2, 2);
    obstacles.emplace_back(2, 3);
    obstacles.emplace_back(2, 4);
    obstacles.emplace_back(2, 5);
    obstacles.emplace_back(2, 6);
    obstacles.emplace_back(3, 3);
    obstacles.emplace_back(3, 4);
    obstacles.emplace_back(3, 5);
    obstacles.emplace_back(3, 6);
    obstacles.emplace_back(4, 3);
    obstacles.emplace_back(4, 4);
    obstacles.emplace_back(4, 5);
    obstacles.emplace_back(4, 6);
    obstacles.emplace_back(4, 7);
    obstacles.emplace_back(5, 7);
    obstacles.emplace_back(6, 7);
    obstacles.emplace_back(6, 8);
    obstacles.emplace_back(6, 2);
    obstacles.emplace_back(7, 2);
    obstacles.emplace_back(8, 2);
    obstacles.emplace_back(9, 2);

    std::vector<GridPoint> starts;
    std::vector<GridPoint> goals;
    starts.emplace_back(3, 2);
    starts.emplace_back(9, 9);
    starts.emplace_back(9, 1);
    starts.emplace_back(9, 3);
    starts.emplace_back(0, 0);
    goals.emplace_back(9, 9);
    goals.emplace_back(1, 1);
    goals.emplace_back(1, 9);
    goals.emplace_back(2, 9);
    goals.emplace_back(5, 5); 

    CBS cbsObj(5, 10, 10, obstacles, starts, goals);
    cbsObj.search();
    const std::shared_ptr<CTNode>& solutionNode = cbsObj.getSolutionNode();
    for (const auto& path : solutionNode->solution) {
        for (GridPoint point : path) {
            std::cout << point << "->";
        }
        std::cout << "finish" << std::endl;
    }
}

int main() {
    test1();
    return 0;
}
