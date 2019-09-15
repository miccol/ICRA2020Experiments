#ifndef GRIDWORLD_H
#define GRIDWORLD_H

#include <definitions.h>
#include <vector>
#include <map>
#include <model.h>


class GridWorld : public Model
{


private:


    const int x_max_, y_max_;
//    int physical_initial_state_, current_physical_state_;

    std::vector<int> border_up_, border_down_, border_left_, border_right_;
    std::vector<int> states_with_window_, states_with_rack_;
public:
 GridWorld(const int x_max, const int y_max, int physical_initial_state, std::vector<Action> actuations, std::vector<Action> sensing,
           std::vector<Observation> observations,std::map<Action, std::vector<Observation>> observation_map,  std::vector<int> states_with_window, std::vector<int> states_with_rack);

 double get_observation_probability(Action action, int state, Observation observation);
 double get_transition_probability(Action action, int state_from, int state_to);

};
#endif // GRIDWORLD_H
