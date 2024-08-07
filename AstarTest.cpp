#include "AStar.hpp"

int main() {
    // Define grid dimensions
    int gridWidth = 3;
    int gridHeight = 3;

    // Define obstacles (for example, obstacles at (3, 3) and (4, 4))
    std::vector<GridPoint> obstacles = {
        GridPoint(-1, -1, Direction::Down),
        GridPoint(-1, -1,Direction::Down)
    };

    // Define constraints (example: some constraints at specific time steps)
    std::vector<Constraint> constraints = {
        Constraint(GridPoint(-1, -1,Direction::Left), -1) // Constraint at (5, 5) at time step 5
    };

    // Create an instance of AStar
    AStar aStar(gridWidth, gridHeight, obstacles, constraints);

    // Define start and goal points
    GridPoint start(0, 0, Direction::Left); // Starting point with direction
    GridPoint goal(2, 0, Direction::Left);   // Goal point with direction

    // Perform the search
    std::vector<GridPoint> path = aStar.search(start, goal);

    // Output the path
    if (path.empty()) {
        std::cout << "No path found!" << std::endl;
    } else {
        std::cout << "Path found:" << std::endl;
        for (const GridPoint& point : path) {
            std::cout << point << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
