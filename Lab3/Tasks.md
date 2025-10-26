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

