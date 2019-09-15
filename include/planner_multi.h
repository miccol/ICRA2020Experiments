#ifndef PLANNERMULTI_H
#define PLANNERMULTI_H

#include<gridworld_triangles_multi.h>
#include<node.h>
#include<vector>

bool plan (GridWorldTrianglesMulti* model, bstate start_state, std::vector<bstate>* path_to_return, std::vector<Action>* actions_to_return, std::vector<Node>* closed_list, const bool has_heuristic=true);


#endif // PLANNER_H
