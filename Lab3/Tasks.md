### Tasks 1
1. Initialize Robot
2. Calculate Robot Center Position
3. Loop through sensor range
4. Apple circle formula to filter points
5. Query and update grid


```C++
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
```

#### Main Loop

```C++
  robot.sense(grid); 
```

### Tasks 2

```C++
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
```

#### Main Loop

```C++
    robot.detection(radius);
    robot.moveRobot(robot.h, robot.v);
```


### Part 2
### Task 1
```C++
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
```

#### Main Loop

```C++
    robot.sense(grid);
    robot.follow_wall(grid, 13);
```

### Task 2





