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
    void sensor(){
            int x_c = x + radius;
            int y_c = y + radius;
            for (int i = std::max(0, x_c - lidar_range); i < std::min(800, x_c + lidar_range); i++) {
                for (int j = std::max(0, y_c - lidar_range); j < std::min(800, y_c + lidar_range); j++) {
                    if ((i - x_c) * (i - x_c) + (j - y_c) * (j - y_c) <= lidar_range * lidar_range) {
                        grid[i][j] = grid_value(true_grid(), this, i, j, lidar_range);
                    }
                }
            }
            std::cout << "Sensor updated grid around robot's position: (" << x_c << ", " << y_c << ")" << std::endl;
        }

        std::vector<int> detect_walls() {
            std::vector<int> direction(4, 0);  
            int c_x = x + radius, c_y = y + radius, tol = 20;  
            const int diag_tol = tol * angle;  
            if (this->grid[c_x][c_y - tol] == 1) direction[0] = 1; 
            else if (this->grid[c_x][c_y + tol] == 1) direction[0] = -1;  

            if (this->grid[c_x + tol][c_y] == 1) direction[1] = 1; 
            else if (this->grid[c_x - tol][c_y] == 1) direction[1] = -1; 
            if (this->grid[c_x - diag_tol][c_y - diag_tol] == 1) direction[2] = 1;  
            else if (this->grid[c_x + diag_tol][c_y + diag_tol] == 1) direction[2] = -1;  
            if (this->grid[c_x + diag_tol][c_y - diag_tol] == 1) direction[3] = 1;  
            else if (this->grid[c_x - diag_tol][c_y + diag_tol] == 1) direction[3] = -1;  
            std::cout << "Direction Vector in is [" << direction[0] << ", " << direction[1] << ", " << direction[2] << ", " << direction[3] << "]" << std::endl;
            return direction;  
        }
        void find_dir(const std::vector<int>& direction) {
            std::vector<int> robot_to_wall(2, 0); 
            if(direction[0] == 0 && direction[1] == 0 && direction[2] == 0 && direction[3] == 0){ 
                robot_to_wall = {-1, 0};           
            } else{
                if (clockwise){
                    if (direction[2] == 1) {              
                        robot_to_wall = {1, -1};            
                    } if (direction[0] == 1 && direction[2] == 0 && direction[3] == 0) {                
                        robot_to_wall = {1, 0};               
                    } if (direction[3] == 1) {              
                        robot_to_wall = {1, 1};             
                    } if (direction[1] == 1 && direction[2] == 0 && direction[3] == 0) {              
                        robot_to_wall = {0, 1};               
                    } if (direction[2] == -1) {             
                        robot_to_wall = {-1, 1};           
                    } if (direction[0] == -1 && direction[2] == 0 && direction[3] == 0) {               
                        robot_to_wall = {-1, 0};              
                    } if (direction[3] == -1 && direction[2] != 1) {             
                        robot_to_wall = {-1, -1};          
                    } if (direction[1] == -1 && direction[2] == 0 && direction[3] == 0) {             
                        robot_to_wall = {0, -1};              
                    }
                } else {
                    if (direction[2] == 1) {              
                        robot_to_wall = {-1, 1};            
                    } if (direction[1] == -1 && direction[2] == 0 && direction[3] == 0) {             
                        robot_to_wall = {0, 1};              
                    } if (direction[3] == -1) {             
                        robot_to_wall = {1, 1};          
                    } if (direction[0] == -1 && direction[2] == 0 && direction[3] == 0) {               
                        robot_to_wall = {1, 0};              
                    } if (direction[2] == -1) {             
                        robot_to_wall = {1, -1};           
                    } if (direction[1] == 1 && direction[2] == 0 && direction[3] == 0) {              
                        robot_to_wall = {0, -1};               
                    } if (direction[3] == 1 && direction[2] != 1) {              
                        robot_to_wall = {-1, -1};             
                    } if (direction[0] == 1 && direction[2] == 0 && direction[3] == 0) {                
                        robot_to_wall = {-1, 0};               
                    }
                }
            }
            
            paths_queue.push(robot_to_wall);
            std::cout << "Move added to queue: [" << robot_to_wall[0] << ", " << robot_to_wall[1] << "]" << std::endl;
        }
        void move() {
            if (!paths_queue.empty()) { 
                std::vector<int> movement = paths_queue.front();
                paths_queue.pop();
                this->x += movement[0];
                this->y += movement[1];
                std::cout << "Robot moved to position: (" << this->x << ", " << this->y << ")" << std::endl;

            }

        }
```

#### Main Loop

```C++
        robot.sensor();
        robot.find_dir(robot.detect_walls());
        robot.move();
```

### Task 2
```C++
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
```

#### Main Loop

```C++
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
```




