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




* Cover page
* Executive summary
* Table of contents, figures, and tables
* Introduction: Lab description and objectives
* Methodology: Description of the proposed solutions provided in pseudocode or
flowchart format.
* Experiments & Discussions: Summary of the experiments performed, and
observations made during the lab. Answer the questions asked in the lab
manual, here.
* Conclusions
* References
* Appendices: codes, figures, tables, graphs.