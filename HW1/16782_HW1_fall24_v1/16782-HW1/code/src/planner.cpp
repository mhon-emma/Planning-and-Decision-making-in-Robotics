/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define MAX(A, B)    ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B)    ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    // 8-connected grid movements
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // Define the goal position (last point in the target trajectory)
    int goalposeX = target_traj[target_steps - 1];
    int goalposeY = target_traj[target_steps - 1 + target_steps];

    int start = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
    int goal = GETMAPINDEX(goalposeX, goalposeY, x_size, y_size);

    // A* setup 
    typedef std::pair<double, int> PriorityNode;
    std::priority_queue<PriorityNode, std::vector<PriorityNode>, std::greater<PriorityNode>> frontier;
    std::unordered_map<int, int> from;
    std::unordered_map<int, double> cost;
    frontier.emplace(0.0, start);
    from[start] = 0; 
    cost[start] = 0.0;

    while (!frontier.empty()) {
        int current = frontier.top().second;
        frontier.pop();

        // Get current node's (x, y) coordinates
        int x = (current % x_size) + 1;
        int y = (current / x_size) + 1;

        // Explore neighbors
        for (int dir = 0; dir < NUMOFDIRS; dir++) 
        {
            int newx = x + dX[dir+1]; //speed up for map 2
            int newy = y + dY[dir+1];

            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) //check boundary
            {
                int neighbor = GETMAPINDEX(newx, newy, x_size, y_size);

                if (map[neighbor] >= 0 && map[neighbor] < collision_thresh) 
                {
                    double move_cost = (std::abs(dX[dir]) == 1 && std::abs(dY[dir]) == 1) ? std::sqrt(2) : 1.0;
                    double new_cost = cost[current] + move_cost;

                    // unvisit or cheaper path
                    if (cost.find(neighbor) == cost.end() || new_cost < cost[neighbor]) 
                    {
                        cost[neighbor] = new_cost;
                        int goalx = (goal % x_size) + 1;
                        int goaly = (goal / x_size) + 1;
                        double heuristic_cost = std::sqrt((newx - goalx)*(newx - goalx) + (newy - goaly)*(newy - goaly));

                        double priority = new_cost + heuristic_cost;
                        frontier.emplace(priority, neighbor);
                        from[neighbor] = current;
                    }
                }
            }
        }
        if (current == goal) {
            break;
        }
    }

    // reverse A* list
    std::vector<int> path;
    int current = goal;
    if (from.find(goal) == from.end()) 
    {
        // no path
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }

    while (current != start) 
    {
        path.push_back(current);
        current = from[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    // keep going 
    if (path.size() > 1) 
    {
        int next_node = path[1];
        int next_x = (next_node % x_size) + 1;
        int next_y = (next_node / x_size) + 1;
        action_ptr[0] = next_x;
        action_ptr[1] = next_y;
    } else 
    {
        // at goal
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }

    return;
}
