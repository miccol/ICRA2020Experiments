#ifndef PLANNER_H
#define PLANNER_H

#include<gridworld_triangles.h>
#include<node.h>
#include<vector>

bool plan (GridWorldTriangles *model, bstate start_state,
           std::vector<bstate>* path_to_return, std::vector<Action>* actions_to_return, std::vector<Node>* closed_list, const bool has_heuristic=true, const bool has_watchdog=false);


#endif // PLANNER_H
