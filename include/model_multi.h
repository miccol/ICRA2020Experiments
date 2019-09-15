#ifndef MODELMULTI_H
#define MODELMULTI_H

#include <definitions.h>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <workspace_multi.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
class ModelMulti
{


protected:


    int physical_initial_state_, current_physical_state_;

    std::vector<int> states_;
    std::vector<Action> actuations_, sensing_; //actuation and sensing actions
    std::vector<std::vector<bool>> observations_;

    std::map<Action, std::vector<std::vector<bool>>> observation_map_; // maps sensing actions with possible observations         s
    std::map<Action, MatrixXd> Transitions_matrices_map_;
//    std::map<Action, std::map<Observation, VectorXd>> Observations_vector_map_;

public:
 ModelMulti(int physical_initial_state, std::vector<Action> actuations, std::vector<Action> sensing,
          std::vector<std::vector<bool> > observations, std::map<Action, std::vector<std::vector<bool> > > observation_map);

 std::vector<bool> take_action(Action action, WorkspaceMulti *ws);

 void get_next_belief_states(bstate belief_state, std::vector<bstate> *next_belief_states,
                             std::vector<Action> *actions_list, std::vector<std::vector<bool> > *observations_list,
                                     std::vector<float> *entropies_list, std::vector<float> *prob_list);
 virtual double get_observation_probability(Action action, int state, std::vector<bool> observation) = 0;
 virtual double get_transition_probability(Action action, int state_from, int state_to) = 0;
 bstate updateBelief(bstate current_belief, Action action, std::vector<bool> observation, bool *flag);
 int current_physical_state();
 virtual const double get_cost(Action action) = 0;

 void set_current_physical_state(int current_physical_state);
};
#endif // MODEL_H
