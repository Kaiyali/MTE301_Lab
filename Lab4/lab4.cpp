//ignonore this file.

#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>
#include <iomanip>
#include <string>
#include <vector>

#include "utils.h"
#include "render.h"

using namespace std;

class my_robot : public Object {
private:
    int phase = 0;
    float move_speed = 1.0;
    const float sin45 = 0.7071f;
    int rotation = 0;
    int dir_check[4] = {0};
    int cur_mode[4] = {0};
    int prev_mode[4] = {0};
    int robot_to_wall[2] = {0};

public:
    int tol = 20;
    int v = 0, h = 0;
    std::vector<std::vector<int>> grid;
    int lidar_range;



    my_robot(int w, int h, int env_w, int env_h, int lidar)
        : Object(w, h, env_w, env_h), lidar_range(lidar) {
        grid.resize(env_w, std::vector<int>(env_h, -1));
    }

    void detect_walls(grid_util& true_grid, int tol) {
        int x_c = x + width / 2;
        int y_c = y + height / 2;
        float tol_45 = sin45 * tol;

        dir_check[0] = dir_check[1] = dir_check[2] = dir_check[3] = 0;

        if (grid_value(true_grid, this, x_c - tol, y_c, lidar_range) == 1) dir_check[0] = -1;
        else if (grid_value(true_grid, this, x_c + tol, y_c, lidar_range) == 1) dir_check[0] = 1;

        if (grid_value(true_grid, this, x_c, y_c - tol, lidar_range) == 1) dir_check[1] = -1;
        else if (grid_value(true_grid, this, x_c, y_c + tol, lidar_range) == 1) dir_check[1] = 1;

        if (grid_value(true_grid, this, x_c + tol_45, y_c + tol_45, lidar_range) == 1) dir_check[2] = 1;
        else if (grid_value(true_grid, this, x_c - tol_45, y_c - tol_45, lidar_range) == 1) dir_check[2] = -1;

        if (grid_value(true_grid, this, x_c + tol_45, y_c - tol_45, lidar_range) == 1) dir_check[3] = 1;
        else if (grid_value(true_grid, this, x_c - tol_45, y_c + tol_45, lidar_range) == 1) dir_check[3] = -1;
    }

    void find_dir(grid_util& true_grid, int tol) {
        detect_walls(true_grid, tol);
        for (int i = 0; i < 4; i++) cur_mode[i] = dir_check[i];

        bool same = true;
        for (int i = 0; i < 4; i++) {
            if (cur_mode[i] != prev_mode[i]) {
                same = false;
                break;
            }
        }

        if (!same) {
            int zero_count = 0;
            for (int i = 0; i < 4; i++) if (prev_mode[i] == 0) zero_count++;
            if (zero_count >= 3) {
                for (int i = 0; i < 4; i++) cur_mode[i] -= prev_mode[i];
            }
        }

        if (cur_mode[2] == 1) {
            robot_to_wall[0] = 1; robot_to_wall[1] = 1;
        } else if (cur_mode[2] == -1) {
            robot_to_wall[0] = -1; robot_to_wall[1] = -1;
        } else if (cur_mode[3] == 1) {
            robot_to_wall[0] = 1; robot_to_wall[1] = -1;
        } else if (cur_mode[3] == -1) {
            robot_to_wall[0] = -1; robot_to_wall[1] = 1;
        } else {
            robot_to_wall[0] = cur_mode[0];
            robot_to_wall[1] = cur_mode[1];
        }

        for (int i = 0; i < 4; i++) prev_mode[i] = cur_mode[i];
    }

    void follow_wall(grid_util& true_grid, int tol) {
        find_dir(true_grid, tol);

        if (robot_to_wall[0] == 0 && robot_to_wall[1] == 0) {
            x--; return;
        }

        if (rotation == 0) {
            x += -robot_to_wall[1];
            y += robot_to_wall[0];
        } else {
            x += robot_to_wall[1];
            y += -robot_to_wall[0];
        }

        x = std::clamp(x, 0, 799);
        y = std::clamp(y, 0, 799);
    }

    void sense(grid_util& true_grid) {
        int x_c = x + width / 2;
        int y_c = y + height / 2;

        for (int i = x_c - lidar_range; i <= x_c + lidar_range; i++) {
            for (int j = y_c - lidar_range; j <= y_c + lidar_range; j++) {
                if (i >= 0 && j >= 0 && i < (int)grid.size() && j < (int)grid[0].size()) {
                    int dx = i - x_c;
                    int dy = j - y_c;
                    if (dx * dx + dy * dy <= lidar_range * lidar_range) {
                        grid[i][j] = grid_value(true_grid, this, i, j, lidar_range);
                    }
                }
            }
        }
    }

     void moveRobot(int h, int v){
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

int main() {
    const int env_width = 800, env_height = 800;
    const int radius = 10;
    const int lidar_range = 50;

    auto config = read_csv();
    std::string env_file = std::get<0>(config);
    bool show_truth = std::get<1>(config);
    int speed = std::get<2>(config);
    int env_type = std::get<3>(config);

    grid_util grid(env_width, env_height, 50, 100);
    std::vector<Object*> walls = (env_type == 4)
        ? grid.create_walls(env_file)
        : grid.create_angled_walls(env_file);

    my_robot robot(2 * radius, 2 * radius, env_width, env_height, lidar_range);
    my_robot robot_init = robot;

    std::vector<std::vector<int>> robot_path;
    robot_path.push_back({robot.x, robot.y});

    int task_choice;
    std::cout << "Enter task number (1 = Right Wall Follow, 2 = Sweep Mapping): ";
    std::cin >> task_choice;

    int count = 0;

    if (task_choice == 1) {
        while (count++ < 10000) {
            robot.sense(grid);
            robot.follow_wall(grid, 20);
            robot_path.push_back({robot.x, robot.y});
        }
    } else if (task_choice == 2) {
        bool sweep_right = true;
        while (count++ < 10000) {
            robot.sense(grid);

            int x_c = robot.x + radius;
            int y_c = robot.y + radius;

            robot.follow_wall(grid, 20);

            if (y_c - robot.tol >= 0 && robot.grid[x_c][y_c - robot.tol] == 1) robot.v = 1;
            else if (y_c + robot.tol < 800 && robot.grid[x_c][y_c + robot.tol] == 1) robot.v = -1;

            if (x_c + robot.tol < 800 && robot.grid[x_c + robot.tol][y_c] == 1) robot.h = 1;
            else if (x_c - robot.tol >= 0 && robot.grid[x_c - robot.tol][y_c] == 1) robot.h = -1;

            robot.moveRobot(robot.h, robot.v);


            robot_path.push_back({robot.x, robot.y});
        }
    } else {
        std::cout << "Invalid task number.\n";
        return 1;
    }

    std::cout << "Wall accuracy: " << grid.wall_accuracy(robot.grid) * 100 << "%\n";
    std::cout << "Environment accuracy: " << grid.grid_accuracy(robot.grid) * 100 << "%\n";

    if (show_truth) {
        render_window(robot_path, walls, robot_init, env_width, env_height, speed);
    }
    render_grid(robot_init, robot_path, robot.grid, env_width, env_height, radius, lidar_range, speed);
    return 0;
}

