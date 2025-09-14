# Lab 1 - MTE301

## Tasks 1
### Methodology
1. Place the robot at a random starting position on the screen
2. Increase the robot's position by 1 on the x-axis
3. After moving the robot checks it checks
4. If a robot collied the software closes
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

#### In Game Loop:

```c++
    if(checkWalls(robot.x, robot.y)){
        succeed = false;
        break;
    }
```
### Tasks 2
1. Place the robot in a random position.
2. On each step, increase the x-position by 1 and decrease the y-position by 2.
3. After moving, check for wall collision.
4. If a collision is detected, stop and mark as failed.
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
1. Compare the robot’s x-position to the goal’s x-position.
2. Move right or left until both x-values are equal.
3. Then compare the robot’s y-position to the goal’s y-position.
4. Move up or down until the y-values are equal.
5. When both positions match the goal, stop and mark as successful.

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
1. Compare the robot’s y-position to the goal’s y-position.
2. Move right or left until both y-values are equal.
3. Then compare the robot’s x-position to the goal’s x-position.
4. Move up or down until the x-values are equal.
5. When both positions match the goal, stop and mark as successful.

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
### Tasks 6

1. Check the robot’s x and y positions in every loop.
2. If x doesn’t match the goal, move left or right.
3. If y doesn’t match the goal, move up or down.
4. Adjust both x and y positions in the same step.
5. Repeat until the robot overlaps with the goal.

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