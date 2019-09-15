#ifndef MODEL_H
#define MODEL_H

#include <definitions.h>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <workspace.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
class Model
{


protected:


    int physical_initial_state_, current_physical_state_;

    std::vector<int> states_;
    std::vector<Action> actuations_, sensing_; //actuation and sensing actions
    std::vector<std::vector<Observation>> observations_;

    std::map<Action, std::vector<std::vector<Observation>>> observation_map_; // maps sensing actions with possible observations         s
    std::map<Action, MatrixXd> Transitions_matrices_map_;
//    std::map<Action, std::map<Observation, VectorXd>> Observations_vector_map_;

public:
 Model(int physical_initial_state, std::vector<Action> actuations, std::vector<Action> sensing,
          std::vector<std::vector<Observation>> observations, std::map<Action, std::vector<std::vector<Observation> > > observation_map);

 std::vector<Observation> take_action(Action action, Workspace *ws);

 void get_next_belief_states(bstate belief_state, std::vector<bstate> *next_belief_states,
                             std::vector<Action> *actions_list, std::vector<std::vector<Observation>> *observations_list,
                                     std::vector<float> *entropies_list, std::vector<float> *prob_list, const bool has_watchdog=false);
 virtual double get_observation_probability(Action action, int state, std::vector<Observation> observation) = 0;
 virtual double get_transition_probability(Action action, int state_from, int state_to) = 0;
 bstate updateBelief(bstate current_belief, Action action, std::vector<Observation> observation, bool *flag);
 int current_physical_state();
 virtual const double get_cost(Action action) = 0;

 void set_current_physical_state(int current_physical_state);
};
#endif // MODEL_H
