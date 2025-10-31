#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>
#include <iomanip>
#include <queue>
#include "utils.h"
#include "render.h"

#define kPI 3.14159265358979323846

class my_robot : public Object {
private:
    int lidar_range{50};
    int radius{10};
    grid_util* true_grid_ptr = nullptr;
    std::vector<std::vector<int>> grid{800, std::vector<int>(800, -1)};
    std::queue<std::vector<int>> paths_queue;
    const float angle = std::cos(45 * kPI / 180);
    bool clockwise{true};

public:
    my_robot(int w, int h, int ew, int min_y, int min_x, int tol)
        : Object(w, h, ew, min_y, min_x, tol) {}

    void setGrid(grid_util& grid) {
        true_grid_ptr = &grid;
    }

    std::vector<std::vector<int>> getGrid() {
        return this->grid;
    }

    grid_util& true_grid() {
        return *true_grid_ptr;
    }

    void switch_dir() {
        clockwise = !clockwise;
    }

    bool get_dir() {
        return clockwise;
    }

    void sensor() {
        int x_c = x + radius;
        int y_c = y + radius;

        for (int i = std::max(0, x_c - lidar_range); i < std::min(800, x_c + lidar_range); i++) {
            for (int j = std::max(0, y_c - lidar_range); j < std::min(800, y_c + lidar_range); j++) {
                if ((i - x_c) * (i - x_c) + (j - y_c) * (j - y_c) <= lidar_range * lidar_range) {
                    grid[i][j] = grid_value(true_grid(), this, i, j, lidar_range);
                }
            }
        }
    }

    std::vector<int> detect_walls() {
        std::vector<int> direction(4, 0);
        int c_x = x + radius;
        int c_y = y + radius;
        int tol = 20;
        const int diag_tol = tol * angle;

        if (grid[c_x][c_y - tol] == 1) direction[0] = 1;
        else if (grid[c_x][c_y + tol] == 1) direction[0] = -1;

        if (grid[c_x + tol][c_y] == 1) direction[1] = 1;
        else if (grid[c_x - tol][c_y] == 1) direction[1] = -1;

        if (grid[c_x - diag_tol][c_y - diag_tol] == 1) direction[2] = 1;
        else if (grid[c_x + diag_tol][c_y + diag_tol] == 1) direction[2] = -1;

        if (grid[c_x + diag_tol][c_y - diag_tol] == 1) direction[3] = 1;
        else if (grid[c_x - diag_tol][c_y + diag_tol] == 1) direction[3] = -1;

        return direction;
    }

    void find_dir(const std::vector<int>& direction) {
        std::vector<int> robot_to_wall(2, 0);

        if (direction[0] == 0 && direction[1] == 0 && direction[2] == 0 && direction[3] == 0) {
            robot_to_wall = {-1, 0};
        } else {
            if (clockwise) {
                if (direction[2] == 1) robot_to_wall = {1, -1};
                if (direction[0] == 1 && direction[2] == 0 && direction[3] == 0) robot_to_wall = {1, 0};
                if (direction[3] == 1) robot_to_wall = {1, 1};
                if (direction[1] == 1 && direction[2] == 0 && direction[3] == 0) robot_to_wall = {0, 1};
                if (direction[2] == -1) robot_to_wall = {-1, 1};
                if (direction[0] == -1 && direction[2] == 0 && direction[3] == 0) robot_to_wall = {-1, 0};
                if (direction[3] == -1 && direction[2] != 1) robot_to_wall = {-1, -1};
                if (direction[1] == -1 && direction[2] == 0 && direction[3] == 0) robot_to_wall = {0, -1};
            } else {
                if (direction[2] == 1) robot_to_wall = {-1, 1};
                if (direction[1] == -1 && direction[2] == 0 && direction[3] == 0) robot_to_wall = {0, 1};
                if (direction[3] == -1) robot_to_wall = {1, 1};
                if (direction[0] == -1 && direction[2] == 0 && direction[3] == 0) robot_to_wall = {1, 0};
                if (direction[2] == -1) robot_to_wall = {1, -1};
                if (direction[1] == 1 && direction[2] == 0 && direction[3] == 0) robot_to_wall = {0, -1};
                if (direction[3] == 1 && direction[2] != 1) robot_to_wall = {-1, -1};
                if (direction[0] == 1 && direction[2] == 0 && direction[3] == 0) robot_to_wall = {-1, 0};
            }
        }

        paths_queue.push(robot_to_wall);
    }

    void move() {
        if (!paths_queue.empty()) {
            std::vector<int> movement = paths_queue.front();
            paths_queue.pop();
            x += movement[0];
            y += movement[1];
        }
    }

    void save_grid_csv() {
        std::string filename = "grid_pred.csv";
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }

        size_t maxRowSize = 0;
        for (const auto& col : grid) {
            if (col.size() > maxRowSize) maxRowSize = col.size();
        }

        for (size_t row = 0; row < maxRowSize; ++row) {
            for (size_t col = 0; col < grid.size(); ++col) {
                if (row < grid[col].size()) file << grid[col][row];
                if (col < grid.size() - 1) file << ",";
            }
            file << "\n";
        }

        file.close();
        std::cout << "Robot's grid written to " << filename << std::endl;
    }
};

const int env_width{800}, env_height{800};
const int radius{10};
const int min_obj_size{50};
const int max_obj_size{100};
int lidar_range{50};

grid_util grid(env_width, env_height, min_obj_size, max_obj_size);
random_generator rand_gen;
std::vector<std::vector<int>> robot_pos;

int main(int argc, char const* argv[]) {
    std::tuple<std::string, bool, int, int> config = read_csv();
    std::vector<Object*> walls;

    if (std::get<3>(config) == 4)
        walls = grid.create_walls(std::get<0>(config));
    else
        walls = grid.create_angled_walls(std::get<0>(config));

    int min_y_spawn = grid.get_min_y();
    int max_y_spawn = grid.get_max_y();

    std::vector<int> prev_dir(4, 0);
    int y_ref;
    bool sweep{false};
    const std::vector<int> free_dir(4, 0);
    int max_y{0}, min_y{800};

    my_robot robot(2 * radius, 2 * radius, env_width, min_y_spawn, max_y_spawn, radius + 5);
    robot.setGrid(grid);
    my_robot robot_init = robot;

    robot_pos.push_back({robot.x, robot.y});

    int limit_count = 0;
    while (true) {
        limit_count++;
        robot.sensor();

        if (!sweep) {
            robot.find_dir(robot.detect_walls());
            robot.move();

            if (robot.y > max_y) max_y = robot.y;
            if (robot.y < min_y) min_y = robot.y;

            if (grid.wall_accuracy(robot.getGrid()) >= .97 && robot.y == min_y) {
                y_ref = robot.y;
                sweep = true;
            }
        } else {
            robot.find_dir(robot.detect_walls());
            if (prev_dir == free_dir && robot.detect_walls() != free_dir) robot.switch_dir();
            robot.move();

            if (robot.y == y_ref + 50) {
                if (robot.get_dir()) {
                    while (robot.detect_walls() != free_dir) {
                        robot.sensor();
                        robot.x--;
                        robot_pos.push_back({robot.x, robot.y});
                        limit_count++;
                    }
                    while (robot.detect_walls() == free_dir) {
                        robot.sensor();
                        robot.x--;
                        robot_pos.push_back({robot.x, robot.y});
                        limit_count++;
                    }
                    y_ref += 50;
                    robot.switch_dir();
                } else if (!robot.get_dir()) {
                    while (robot.detect_walls() != free_dir) {
                        robot.sensor();
                        robot.x++;
                        robot_pos.push_back({robot.x, robot.y});
                        limit_count++;
                    }
                    while (robot.detect_walls() == free_dir) {
                        robot.sensor();
                        robot.x++;
                        robot_pos.push_back({robot.x, robot.y});
                        limit_count++;
                    }
                    y_ref += 50;
                    robot.switch_dir();
                }
            }

            if (robot.y >= max_y) break;
            prev_dir = robot.detect_walls();
        }

        robot_pos.push_back({robot.x, robot.y});

        if (limit_count >= 7200) {
            std::cout << "====Program terminated after 3600 iterations====" << std::endl;
            break;
        }
    }

    std::cout << std::fixed << std::setprecision(2);
    float wall_accuracy = grid.wall_accuracy(robot.getGrid());
    float accuracy = grid.grid_accuracy(robot.getGrid());

    std::cout << "Percent of walls correctly mapped: " << wall_accuracy * 100.0 << "%" << std::endl;
    std::cout << "Percent of environment correctly mapped: " << accuracy * 100.0 << "%" << std::endl;

    if (std::get<1>(config)) {
        render_window(robot_pos, walls, robot_init, env_width, env_height, std::get<2>(config));
    }

    render_grid(robot_init, robot_pos, robot.getGrid(), env_width, env_height, radius, lidar_range, std::get<2>(config));
    return 0;
}
