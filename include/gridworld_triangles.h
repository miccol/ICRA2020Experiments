#ifndef GRIDWORLDTRIANGLES_H
#define GRIDWORLDTRIANGLES_H

#include <definitions.h>
#include <vector>
#include <map>
#include <model.h>




class GridWorldTriangles : public Model
{


private:
    const int x_max_, y_max_, n_of_slices_, n_of_physical_states_;
    std::vector<int> border_up_, border_down_, border_left_, border_right_;
    std::vector<int> states_with_A_, states_with_B_, states_with_C_, states_with_D_;
public:
    GridWorldTriangles(int physical_initial_state, std::vector<Action> actuations, std::vector<Action> sensing,
               std::vector<std::vector<Observation>> observations,  std::map<Action, std::vector<std::vector<Observation>>> observation_map,
               std::vector<int> states_with_A, std::vector<int> states_with_B, std::vector<int> states_with_C, std::vector<int> states_with_D, int x_size, int y_size, int n_of_slices);

    double get_observation_probability(Action action, int state, std::vector<Observation> observation) override;
    double get_single_observation_probability(Action action, int state, Observation observation);
    double get_transition_probability(Action action, int state_from, int state_to) override;
//    void get_next_belief_states(bstate belief_state, std::vector<bstate> *next_belief_states,
//                                std::vector<Action> *actions_list, std::vector<Observation> *observations_list,
//                                std::vector<float> *entropies_list, std::vector<float> *prob_list) override;



    const double get_cost(Action action);


};
#endif // GRIDWORLDTRIANGLES_H
