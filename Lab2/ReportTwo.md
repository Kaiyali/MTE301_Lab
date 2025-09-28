# Lab 2 - MTE301

## Tasks 1

1. Represent the robot as a 20×20 square (bounding box).

2. At each step, check the four corners of the robot against the grid.

3. If any corner’s grid value equals 2, a collision with an obstacle is detected.

4. On collision → set succeed = false, print error, and stop the loop.
```C++
bool obstacleDect(const Object& robot, const grid_util& grid){
    if(grid.grid[robot.x][robot.y] == 2) return true;
    if(grid.grid[robot.x + robot.width][robot.y] == 2) return true;
    if(grid.grid[robot.x + robot.width][robot.y + robot.height] == 2) return true;
    if(grid.grid[robot.x][robot.y+ robot.height] == 2) return true;
    return false;
}
```

#### In Main Loop:

```C++
        if (obstacleDect(robot, grid)) {
        succeed = false;
        break; 
        }
```
