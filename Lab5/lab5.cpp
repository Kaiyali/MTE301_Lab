#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <utility>
#include <vector>
#include <queue>

#include "utils.h"
#include "render.h"

using namespace std;

class my_robot : public Object {
private:
    int lidar_range{40};
    int radius{10};
    queue<vector<int>> movement_queue;
    int move_direction = 1;
    bool clockwise{true};

    grid_util* trueGrid = nullptr;
    vector<vector<int>> grid;

public:
    my_robot(int width, int height, int env_width, int env_height,
             int range, int tol, int min_y_spawn, int max_y_spawn)
        : Object(width, height, env_width, min_y_spawn, max_y_spawn, tol)
    {
        grid = vector<vector<int>>(800, vector<int>(800, -1));
        lidar_range = range;
        radius = width / 2;
    }

    void setGrid(grid_util& g) {
        trueGrid = &g;
    }

    vector<vector<int>> getGrid() {
        return grid;
    }

    grid_util& true_grid() {
        return *trueGrid;
    }

    void lidarCircle(grid_util& true_grid, int env_width, int env_height);

    vector<int> detect_walls() {
        vector<int> walls(4, 0);

        int cx = x + radius;
        int cy = y + radius;
        int tol = radius + 10;
        const int tol45 = tol * cos(45 * M_PI / 180);

        if (grid[cx][cy - tol] == 1) walls[0] = 1;
        else if (grid[cx][cy + tol] == 1) walls[0] = -1;

        if (grid[cx + tol][cy] == 1) walls[1] = 1;
        else if (grid[cx - tol][cy] == 1) walls[1] = -1;

        if (grid[cx - tol45][cy - tol45] == 1) walls[2] = 1;
        else if (grid[cx + tol45][cy + tol45] == 1) walls[2] = -1;

        if (grid[cx + tol45][cy - tol45] == 1) walls[3] = 1;
        else if (grid[cx - tol45][cy + tol45] == 1) walls[3] = -1;

        return walls;
    }

    bool detect_obstacle() {
        int cx = x + radius;
        int cy = y + radius;
        int check_radius = radius + 8;

        for (int dx = -check_radius; dx <= check_radius; dx++) {
            for (int dy = -check_radius; dy <= check_radius; dy++) {
                int sx = cx + dx;
                int sy = cy + dy;

                if (sx >= 0 && sx < (int)grid.size() &&
                    sy >= 0 && sy < (int)grid[0].size())
                {
                    if (grid[sx][sy] == 2)
                        return true;
                }
            }
        }
        return false;
    }

    void find_dir(const vector<int>& walls) {
        vector<int> step(2, 0);

        if (walls[0] == 0 && walls[1] == 0 &&
            walls[2] == 0 && walls[3] == 0)
        {
            step = {-1, 0};
        }
        else {
            if (clockwise) {
                if (walls[2] == 1) step = {1, -1};
                if (walls[0] == 1 && !walls[2] && !walls[3]) step = {1, 0};
                if (walls[3] == 1) step = {1, 1};
                if (walls[1] == 1 && !walls[2] && !walls[3]) step = {0, 1};
                if (walls[2] == -1) step = {-1, 1};
                if (walls[0] == -1 && !walls[2] && !walls[3]) step = {-1, 0};
                if (walls[3] == -1 && walls[2] != 1) step = {-1, -1};
                if (walls[1] == -1 && !walls[2] && !walls[3]) step = {0, -1};
            }
            else {
                if (walls[2] == 1) step = {-1, 1};
                if (walls[1] == -1 && !walls[2] && !walls[3]) step = {0, 1};
                if (walls[3] == -1) step = {1, 1};
                if (walls[0] == -1 && !walls[2] && !walls[3]) step = {1, 0};
                if (walls[2] == -1) step = {1, -1};
                if (walls[1] == 1 && !walls[2] && !walls[3]) step = {0, -1};
                if (walls[3] == 1 && walls[2] != 1) step = {-1, -1};
                if (walls[0] == 1 && !walls[2] && !walls[3]) step = {-1, 0};
            }
        }

        movement_queue.push(step);
    }

    void move() {
        if (!movement_queue.empty()) {
            auto step = movement_queue.front();
            movement_queue.pop();
            x += step[0];
            y += step[1];
        }
    }

    void switch_dir() {
        clockwise = !clockwise;
    }

    bool get_dir() {
        return clockwise;
    }

    void save_grid_csv() {
        string filename = "grid_pred.csv";
        ofstream file(filename);
        if (!file.is_open()) return;

        size_t maxRows = 0;
        for (const auto& col : grid)
            maxRows = max(maxRows, col.size());

        for (size_t r = 0; r < maxRows; ++r) {
            for (size_t c = 0; c < grid.size(); ++c) {
                if (r < grid[c].size()) file << grid[c][r];
                if (c < grid.size() - 1) file << ",";
            }
            file << "\n";
        }

        file.close();
    }
};
