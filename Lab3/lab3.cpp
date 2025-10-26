#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>

#include "utils.h"
#include "render.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++Modify my_robot class here+++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class my_robot : public Object {
    
public:
    int tol = 20;
    int v = 0, h = 0;
    std::vector<std::vector<int>> grid;
    int lidar_range;

    my_robot(int width, int height, int env_width, int env_height, int lidar_range)
        : Object(width, height, env_width, env_height), lidar_range(lidar_range) {
        grid = std::vector<std::vector<int>>(800, std::vector<int>(800, -1));
    }

    void sense(grid_util& g) {
        int xc = this->x + this->width / 2;
        int yc = this->y + this->height / 2;

        for (int i = xc - lidar_range; i <= xc + lidar_range; ++i) {
            for (int j = yc - lidar_range; j <= yc + lidar_range; ++j) {
                if (i >= 0 && i < 800 && j >= 0 && j < 800) {
                    int dx = i - xc;
                    int dy = j - yc;
                    if (dx * dx + dy * dy <= lidar_range * lidar_range) {
                        grid[i][j] = Object::grid_value(g, this, i, j, lidar_range);
                    }
                }
            }
        }
    }

    void save_grid_csv() {
        std::string filename = "grid_pred.csv";
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }
        for (size_t row = 0; row < grid[0].size(); ++row) {
            for (size_t col = 0; col < grid.size(); ++col) {
                file << grid[col][row];
                if (col < grid.size() - 1) file << ",";
            }
            file << "\n";
        }
        file.close();
        std::cout << "Robot's grid written to " << filename << std::endl;
    }

    void detection(int radius){
        int x = this->x + radius;
        int y = this->y + radius;

        // Update member variables v and h
        if (y - tol >= 0 && this->grid[x][y - tol] == 1) v = 1;
        else if (y + tol < 800 && this->grid[x][y + tol] == 1) v = -1;
        else v = 0;

        if (x + tol < 800 && this->grid[x + tol][y] == 1) h = 1;
        else if (x - tol >= 0 && this->grid[x - tol][y] == 1) h = -1;
        else h = 0;
    }

    void moveRobot(int h, int v){
        // Use parameters h and v, and update this robot's position
        if (h == 1 && v == 1) this->y += 1;
        else if (h == 1 && v == 0) this->y += 1;
        else if (h == 1 && v == -1) this->x -= 1;
        else if (h == 0 && v == 1) this->x += 1;
        else if (h == 0 && v == 0) this->x -= 1;
        else if (h == 0 && v == -1) this->x -= 1;
        else if (h == -1 && v == 1) this->x += 1;
        else if (h == -1 && v == 0) this->y -= 1;
        else if (h == -1 && v == -1) this->y -= 1;
    }
};

//===== Main parameters =====
const int env_width {800}, env_height {800};
const int radius {10};
const int min_obj_size {50};
const int max_obj_size {100};
int lidar_range{50};

// Grid utility class
grid_util grid(env_width, env_height, min_obj_size, max_obj_size);

// Random generator
random_generator rand_gen;

// Vector of velocity commands
std::vector<std::vector<int>> robot_pos;

int main(int argc, char const *argv[])
{
    //==========CREATE ROBOT AND WALLS==========
    std::pair<std::string, bool> config = read_csv();
    std::vector<Object*> walls;

    if (config.first == "environment1.csv") {
        walls = grid.create_walls(config.first);
    } else {
        walls = grid.create_angled_walls(config.first);
    }

    my_robot robot(2 * radius, 2 * radius, env_width, env_height, lidar_range);
    my_robot robot_init = robot;

    // push the initial position onto robot_pos
    robot_pos.push_back({robot.x, robot.y});
    int limit_count = 0;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //++++++++++++++++++++++ WALL FOLLOW LOGIC +++++++++++++++++++++++++
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    while (true) {
        limit_count++;
        robot.sense(grid);

        // Call detection to update robot.h and robot.v
        robot.detection(radius);
        // Pass the member variables to moveRobot
        robot.moveRobot(robot.h, robot.v);

        robot_pos.push_back({robot.x, robot.y});

        if (limit_count >= 3600) {
            std::cout << "==== wall follow done after 3600 ====" << std::endl;
            break;
        }
    }

    float accuracy = grid.grid_accuracy(robot.grid);
    std::cout << "Percent of walls correctly mapped: " << accuracy * 100.0 << "%" << std::endl;

    if (config.second) {
        render_window(robot_pos, walls, robot_init, env_width, env_height);
    }

    render_grid(robot_init, robot_pos, robot.grid, env_width, env_height, radius, lidar_range);
    return 0;
}