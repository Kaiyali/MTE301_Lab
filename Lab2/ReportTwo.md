# Lab 1 - MTE301

## Tasks 1

1. Define a function `checkWalls(int x, int y)` that checks whether the robot’s coordinates exceed the environment boundaries (0, width) or (0, height).

2. If the robot’s x or y goes outside, the function returns `true` → mission fails.

3. Inside the main loop, if `checkWalls` is true, set `succeed` = `false` and `break`.
```C++
bool checkWalls(int x, int y){
    if(x >= width || x <= 0){
        return true;
    } 
    if(y >= height || y <= 0){
        return true;
    } 
    return false;
} 
```

#### In Main Loop:

```c++
    if(checkWalls(robot.x, robot.y)){
        succeed = false;
        break;
    }
```
### Tasks 2

1. Define `touchGoal(const Object& goal, const Object& robot, float radius)` to check if the robot overlaps with the goal’s rectangle.

2. Condition: the robot’s bounding circle (± radius) intersects with the goal’s bounding box.

3. If intersection occurs, set `succeed = true` and `break` the loop.
```c++
bool touchGoal(const Object& goal, const Object& robot, float radius) {
    return (robot.x + radius >= goal.x &&
            robot.x - radius <= goal.x + goal.width &&
            robot.y + radius >= goal.y &&
            robot.y - radius <= goal.y + goal.height);
}
```

#### In Game Loop:

```c++
   if(touchGoal(goal, robot, radius)){
        succeed = true;
        break;
    }
```
### Tasks 3
1. Start from the robot’s position.

2. Move horizontally `(x-axis)` one pixel per iteration until the robot’s `x` matches the goal’s `x`.

3. After reaching correct `x`, move vertically (y-axis) one pixel per iteration until the robot’s `y` matches the goal’s `y`.

4. Uses simple incremental motion (`robot.x += 1` or `robot.y -= 1`, etc.).

#### In Game Loop:

```c++
    if (robot.x < goal.x) {
        robot.x += 1;  
    } else if (robot.x > goal.x) {
        robot.x -= 1;   
    } else {
        if (robot.y < goal.y) {
            robot.y += 1;   
        } else if (robot.y > goal.y) {
            robot.y -= 1;   
        }
    }
```
### Tasks 4
1. Similar to Task 3, but the order of movement is swapped.

2. Move vertically (y-axis) first, until the robot’s `y` matches the goal’s `y`.

3. Then move horizontally (x-axis) until the robot’s `x` matches the goal’s `x`.

4. Ensures the robot reaches the top-left of the goal by an L-shaped path.

#### In Game Loop:

```c++
    if (robot.y < goal.y) {
        robot.y += 1;  
    } else if (robot.y > goal.y) {
        robot.y -= 1;   
    } else {
        if (robot.x < goal.x) {
            robot.x += 1;   
        } else if (robot.x > goal.x) {
            robot.x -= 1;   
        }
    }
```
### Tasks 5

1. Calculate the target as the top-left corner of the goal (`goal.x, goal.y`).

2. Use `moveTowards(robot, targetCorner, 1.0f)` to compute normalized step in both x and y directions.

3. Each iteration moves the robot 1 pixel in the straight-line direction toward the goal.

4. This ensures the robot follows the shortest possible path to the goal’s corner.
#### In Game Loop:
```C++
    Point targetCorner {static_cast<float>(goal.x), static_cast<float(goal.y)};
    moveTowards(robot, targetCorner, 1.0f);
```
### Tasks 6

1. Compute the center point of the goal
2. Define a target point as `(centerX, centerY)`.

3. Use the same `moveTowards(robot, target, 1.0f)` function to move in a straight line.

4. Each iteration moves the robot 1 pixel in the direction of the goal’s center.

5. Robot follows a straight diagonal path from its initial position directly to the goal’s center.

```c++
    float centerX = goal.x + goal_width / 2.0f;
    float centerY = goal.y + goal_height / 2.0f;
```
    

#### In Game Loop:

```c++

    if (robot.y < centerY) {
        robot.y += 1;
    } else if (robot.y > centerY) {
        robot.y -= 1;
    }

    if (robot.x < centerX) {
        robot.x += 1;
    } else if (robot.x > centerX) {
        robot.x -= 1;
    }
```
### Tasks 7

1. Identify all 4 corners of the goal rectangle:
`(goal.x, goal.y), (goal.x+width, goal.y), (goal.x, goal.y+height), (goal.x+width, goal.y+height)`.

2. Compute the Euclidean distance between the robot’s starting position and each corner.

3. Select the closest corner (minimum distance).

3. Define that corner as the target point.

4. Use `moveTowards(robot, closestCorner, 1.0f)` to move in a straight line.

5. Robot follows the shortest path to whichever corner is nearest, instead of always the top-left.

```C++
    struct Point{
        float x,y;
    };

    Point findClosestCorner(const Object& goal, const Object& robot){
        Point corners[4] = {
            {goal.x, goal.y},
            {goal.x + goal.width, goal.y},
            {goal.x, goal.y + goal.height},
            {goal.x + goal.width, goal.y + goal.height}
        };

        Point closest = corners[0];
        float minDist = std::numeric_limits<float>::max();

        for (int i = 0; i < 4; i++) {
            float dx = robot.x - corners[i].x;
            float dy = robot.y - corners[i].y;
            float dist = dx * dx + dy * dy; 

            if (dist < minDist) {
                minDist = dist;
                closest = corners[i];
            }
        }
        return closest;
    }

    void moveTowards(Object& robot, const Point& target, float speed) {
        float dx = target.x - robot.x;
        float dy = target.y - robot.y;

        float distance = std::sqrt(dx * dx + dy * dy);
        if (distance > 0.0f) {
            robot.x += (dx / distance) * speed;
            robot.y += (dy / distance) * speed;
        }
    }
```

#### In Game Loop:
```C++
    Point targetCorner = findClosestCorner(goal, robot);    
    moveTowards(robot, targetCorner, 1.0f);  
```

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




## Tasks 2

1. Detect collision by checking the robot’s four corners against the grid.

2. If collision is detected, slide the robot perpendicular to its current movement direction.

    * Moving in y → slide in x (left or right toward goal).

    * Moving in x → slide in y (up or down toward goal).

3. Keep sliding step-by-step until the robot clears the obstacle.

4. Resume normal movement toward the goal.

```C++
void obstacle_avoidance(Object& robot, char direction,
                        const Object& goal, 
                        std::vector<std::vector<int>>& robot_pos) 
{
    if(moving_direction == 'y'){
        if (robot.x < goal.x){
            robot.x += 1;
        } else if (robot.x > goal.x){
            robot.x -= 1;
        }
            robot_pos.push_back({robot.x, robot.y});  
        } else if(moving_direction == 'x'){
       
        if (robot.y < goal.y){
            robot.y += 1;
        } else if (robot.y > goal.y){
            robot.y -= 1;
        }
        robot_pos.push_back({robot.x, robot.y});  
        }
}
```

#### In Main Loop:


```C++
    if (obstacleDect(robot, grid))
    {
        obstacle_avoidance(robot, moving_direction, goal, robot_pos);
    }


    if (robot.y < goal.y && obstacleDect(robot,grid) == false) { 
        robot.y += 1; 
        moving_direction = 'y';
    } else if (robot.y > goal.y && obstacleDect(robot,grid) == false) { 
        robot.y -= 1; 
        moving_direction = 'y';
    } else { 
        if (robot.x < goal.x && obstacleDect(robot,grid) == false) { 
            robot.x += 1; 
            moving_direction = 'x';
        } else if (robot.x > goal.x && obstacleDect(robot,grid) == false) { 
            robot.x -= 1; 
            moving_direction = 'x';
        }
    } 
    if(robot.y == goal.y && obstacleDect(robot,grid) == true){
        while(obstacleDect(robot, grid)){
            for (int i = 0; i < 50; i++) {   
            robot.y += 1;  
            robot_pos.push_back({robot.x, robot.y}); 
            }
        }

        while (robot.x != goal.x){
            if (goal.x > robot.x) robot.x++;
            else if (goal.x < robot.x) robot.x--;
            robot_pos.push_back({robot.x, robot.y}); 
        }
    }
    if(robot.x == goal.x && obstacleDect(robot,grid) == true){
        while(obstacleDect(robot, grid)){
            for (int i = 0; i < 50; i++) {   
            robot.x += 1;  
            robot_pos.push_back({robot.x, robot.y}); 
            }
        }

        while (robot.y != goal.y){
            if (goal.y > robot.y) robot.x++;
            else if (goal.y < robot.y) robot.y--;
            robot_pos.push_back({robot.x, robot.y}); 
        }
    }
```

## Tasks 3

1. Detect collision by checking the robot’s four corners against the grid.

2. When blocked, test both perpendicular directions (left vs right, or up vs down).

3. Calculate which option leaves the robot closer to the goal.

4. Slide step-by-step in that chosen direction until the robot clears the obstacle.

5. Resume normal movement toward the goal.

```C++
void obstacle_avoidance(Object& robot, char direction,
                        const Object& goal, 
                        std::vector<std::vector<int>>& robot_pos) 
{
    // While robot is still colliding
    while (obstacleDect(robot, grid)) {
        Object optionA = robot;
        Object optionB = robot;

        if (direction == 'x') {
            optionA.y += 1;
            optionB.y -= 1;
        } else {
            optionA.x += 1;
            optionB.x -= 1;
        }

        double distA = distanceToGoal(optionA, goal);
        double distB = distanceToGoal(optionB, goal);

        if (distA <= distB) {
            robot = optionA;
        } else {
            robot = optionB;
        }

        robot_pos.push_back({robot.x, robot.y});
    }
}
```

#### In Main Loop:


```C++
    if (obstacleDect(robot, grid))
    {
        obstacle_avoidance(robot, moving_direction, goal, robot_pos);
    }
```