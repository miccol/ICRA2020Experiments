#ifndef R1MODEL_H
#define R1MODEL_H

#include <definitions.h>
#include <vector>
#include <map>
#include <model.h>




class R1Model : public Model
{


private:
    const int x_max_, y_max_, n_of_slices_;
    std::vector<int> border_up_, border_down_, border_left_, border_right_;
    double get_single_observation_probability(Action action, int state, Observation observation);
public:
    R1Model(int physical_initial_state, std::vector<Action> actuations, std::vector<Action> sensing,
               std::vector<Observation> observations, std::map<Action, std::vector<Observation>> observation_map, int x_max, int y_max, int n_of_slices);

    double get_observation_probability(Action action, int state, std::vector<Observation> observation) override;
    double get_transition_probability(Action action, int state_from, int state_to) override;

    const double get_cost(Action action);

};
#endif // R1MODEL_H
