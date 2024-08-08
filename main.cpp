#include <iostream>
#include "CBS.hpp"

int main() {
  // Define problem parameters
  int num_agents = 3; // Number of agents
  int dim_x = 3; // Grid dimension (X)
  int dim_y = 3; // Grid dimension (Y)

  std::vector<GridPoint> obstacles;
  obstacles.push_back(GridPoint(-1, -1,Direction::Up));

  std::vector<GridPoint> starts;
  //starts.push_back(GridPoint(0, 1, Direction::Right));
  starts.push_back(GridPoint(1, 0, Direction::Down));
  starts.push_back(GridPoint(2, 0, Direction::Down));
  starts.push_back(GridPoint(1, 1, Direction::Left));
  // starts.push_back(GridPoint(0, 0));

  std::vector<GridPoint> goals;
  //goals.push_back(GridPoint(1, 1, Direction::Right));
  goals.push_back(GridPoint(1, 2, Direction::Down));
  goals.push_back(GridPoint(0, 2, Direction::Left));
  goals.push_back(GridPoint(2, 2, Direction::Down));
  // goals.push_back(GridPoint(1, 2));

  // Create a CBS instance
  CBS cbs(num_agents, dim_x, dim_y, obstacles, starts, goals);

  // Run the search algorithm
  cbs.search();

  // Check if a solution was found
  if (cbs.getSolutionNode() != nullptr) {
    std::cout << "Solution found!" << std::endl;

    // Access the solution paths and print them for each agent
    const std::shared_ptr<CTNode>& solutionNode = cbs.getSolutionNode();
    for (int agent = 0; agent < num_agents; ++agent) {
      std::cout << "Agent " << agent + 1 << " solution: ";
      for (const GridPoint& point : solutionNode->solution[agent]) {
        std::cout << "(" << point.x << ", " << point.y<<", "<<point.direction <<") -> ";
      }
      std::cout << "Goal\n";
    }
  } else {
    std::cout << "No solution found." << std::endl;
  }

  return 0;
}
