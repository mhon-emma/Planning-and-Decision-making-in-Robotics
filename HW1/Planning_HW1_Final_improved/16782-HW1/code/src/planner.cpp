/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <cmath> 
#include <queue> 
#include <unordered_map> //get element w/unique key
#include <iostream> //print to check
#include <algorithm> 
#include <vector> 

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
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    int goalposeX = target_traj[target_steps-1];
    int goalposeY = target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    int start = GETMAPINDEX(robotposeX,robotposeY,x_size,y_size);
    int goal = GETMAPINDEX(goalposeX,goalposeY,x_size,y_size);

    // A* setup
    typedef std::pair<double, int> Priority;
    std::priority_queue<Priority,std::vector<Priority>,std::greater<Priority>>frontier;
    std::unordered_map<int, int> from;
    std::unordered_map<int, double> cost;
    frontier.emplace(0.0, start); 
    from[start] = 0; 
    cost[start] = 0.0;

    //boundary & neighbor check
    while (!frontier.empty()) 
    {  
        int current = frontier.top().second; 
        frontier.pop(); 
        int x = current%x_size;
        int y = current/x_size;
        for (int dir = 0; dir < NUMOFDIRS; dir++) 
        {
            int newx = x+dX[dir]+1;   
            int newy = y+dY[dir]+1;

            //go to goal and sit 
            if (newx >= 1 && newx<=x_size && newy>=1 && newy<=y_size)
            {
                int neighbor = GETMAPINDEX(newx, newy, x_size, y_size);
                if (map[neighbor]>= 0 && map[neighbor]<collision_thresh) //if free
                {
                    double move_cost;
                    if(dX[dir] != 0 && dY[dir] !=0) 
                    {
                        move_cost = std::sqrt(2); //diagonal
                    }
                    else
                    {
                        move_cost = 1.0; //straight
                    }
                    double new_cost =cost[current]+move_cost*100;
                
                    if (cost.find(neighbor) == cost.end() || new_cost<cost[neighbor]) //unexplored or cheaper
                    {
                        cost[neighbor] = new_cost;
                        int goalx = goal%x_size;
                        int goaly = goal/x_size;
                        
                        double heur_cost = std::sqrt((newx-goalx)*(newx-goalx)+(newy-goaly)*(newy-goaly));
                        double priority = new_cost+heur_cost;
                        frontier.emplace(priority, neighbor);
                        from[neighbor] = current;
                        //std::cout << "cost " << new_cost << std::endl;
                    }
                }
            }
        }

        if (current==goal) 
        {
            break;
        }
    }

    // reverse A* list
    std::vector<int> path;
    int current = goal;
    //flip path
    while (current !=start) 
    {
        path.push_back(current);
        current = from[current];
    }
    path.push_back(start); //from back
    std::reverse(path.begin(), path.end()); //flip! 

    // use flipped path, keep going until goal
    if (path.size()> 1) 
    {
        int next = path[1];
        int nextx = (next%x_size)+1; 
        int nexty = (next/x_size)+1; 
        action_ptr[0] = nextx;
        action_ptr[1] = nexty;
    } else 
    {
        // at goal
        action_ptr[0] =robotposeX;
        action_ptr[1] =robotposeY;
    }
    //no path, map6
    if (from.find(goal) == from.end()) 
    {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }

        //greedy planner from default
        // int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
        // double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
        // double disttotarget;
        // for(int dir = 0; dir < NUMOFDIRS; dir++)
        // {
        //     int newx = robotposeX + dX[dir];
        //     int newy = robotposeY + dY[dir];

        //     if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        //     {
        //         if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
        //         {
        //             disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
        //             if(disttotarget < olddisttotarget)
        //             {
        //                 olddisttotarget = disttotarget;
        //                 bestX = dX[dir];
        //                 bestY = dY[dir];
        //             }
        //         }
        //     }
        // }
        // robotposeX = robotposeX + bestX;
        // robotposeY = robotposeY + bestY;
        // action_ptr[0] = robotposeX;
        // action_ptr[1] = robotposeY;


    return;
}
