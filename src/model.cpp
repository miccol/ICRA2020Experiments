#include "model.h"
#include <functional>

#include <algorithm>
#include <utils.h>
#include <thread>
#include <Eigen/Dense>
using Eigen::Matrix2d;

void Model::set_current_physical_state(int current_physical_state)
{
    current_physical_state_ = current_physical_state;
}

Model::Model(int physical_initial_state, std::vector<Action> actuations, std::vector<Action> sensing,
             std::vector<std::vector<Observation>> observations,  std::map<Action, std::vector<std::vector<Observation>>> observation_map) :
    physical_initial_state_(physical_initial_state), current_physical_state_(physical_initial_state),
    actuations_(actuations), sensing_(sensing),
    observations_(observations), observation_map_(observation_map)
{


}


std::vector<Observation> Model::take_action(Action action, Workspace *ws)
{

    if (std::find(actuations_.begin(), actuations_.end(), action) != actuations_.end())
    {
        std::vector<float> next_physical_states_distribution;

        for ( int i = 0; i < ws->n_of_physical_states(); i++)
        {
            next_physical_states_distribution.push_back(get_transition_probability(action, current_physical_state_, i));
        }
        current_physical_state_ = draw(next_physical_states_distribution);
        return {Observation::NOTHING};
    }
    else if (std::find(sensing_.begin(), sensing_.end(), action) != sensing_.end())
    {
        //state does not change, only observation.
        std::vector<float> observations_distribution;

        //           std::vector<std::vector<Observation>> ob = observation_map_[action];



        for (auto it = observations_.begin(); it != observations_.end(); it++)

        {
           // for (auto it2 = (*it).begin(); it2 != (*it).end(); it2++)
            {
                observations_distribution.push_back(get_observation_probability(action, current_physical_state_, *it));
            }
        }
        return observations_.at(draw(observations_distribution));

    }
    else
    {
        cout << "Take action does not recognize action" << endl;
    }

    return {Observation::NOTHING};

}

bstate Model::updateBelief(bstate current_belief, Action action, std::vector<Observation> observation, bool *flag)
{
    bstate new_belief(current_belief.size());

    if (std::find(sensing_.begin(), sensing_.end(), action) != sensing_.end())
    {
        float total = 0.0;

        for (int i = 0; i < current_belief.size(); i ++)
        {
            new_belief[i] = current_belief[i]*get_observation_probability(action, i, observation);
            total = total + new_belief[i];
        }

        for (int i = 0; i < current_belief.size(); i ++)
        {
            new_belief[i] = new_belief[i] / total;
        }

        if (total == 0)
        {
            *flag = false;
        }
    }
    else
    {
        new_belief = Transitions_matrices_map_[action] * current_belief;
    }
    return new_belief;

}

int Model::current_physical_state()
{
    return current_physical_state_;
}

void Model::get_next_belief_states(bstate belief_state, std::vector<bstate> *next_belief_states,
                                   std::vector<Action> *actions_list, std::vector<std::vector<Observation>> *observations_list,
                                   std::vector<float> *entropies_list, std::vector<float> *prob_list, const bool has_watchdog )
{
    auto start_time = chrono::steady_clock::now();
    auto time_now = chrono::steady_clock::now();

    int n_of_physical_states = belief_state.size();

    for (auto it = actuations_.begin(); it != actuations_.end(); it++)
    {

        bstate next_belief_state = Transitions_matrices_map_[*it] * belief_state;
        observations_list->push_back({Observation::NOTHING});
        actions_list->push_back(*it);
        prob_list->push_back(1.0);

        bstate norm_next_belief_state = next_belief_state/next_belief_state.sum();
        if (abs(norm_next_belief_state.sum() - 1.0) > 0.0001)
        {
            cout << "WTF "<< norm_next_belief_state.sum()<<endl;
            printState(norm_next_belief_state);
        }

        next_belief_states->push_back(norm_next_belief_state);
    }


    //     if (entropy(belief_state) > 0.0)
    {
        for (auto it_s = sensing_.begin(); it_s != sensing_.end(); it_s++)
        {
            std::vector<double> p_vec;
            double total_p = 0.0;

            std::vector<std::vector<Observation>> obs = observation_map_[*it_s];

            for (auto it_ol = obs.begin(); it_ol != obs.end(); it_ol++)
            {


                for (auto it_o = (*it_ol).begin(); it_o != (*it_ol).end(); it_o++)
                {
                    bstate next_belief_state = VectorXd::Zero(n_of_physical_states);// = bstate {0.0}; //initialized all zeros
                    if(has_watchdog)
                    {
                        time_now = chrono::steady_clock::now();

                        float elapsed_time = pow(10,-9)*chrono::duration_cast<chrono::nanoseconds>(time_now - start_time).count();
                        cout << "time" << elapsed_time<< endl;

                        if (elapsed_time > 60.0) // more that 60 seconds to plan
                        {
                            cout << "The uniform approach is taking too much time!" << endl;
                            cout << "Stopping here" << endl;
                            return;

                        }

                    }

                    double total = 0.0;
                    double probability = 0.0;
                    double p = 0.0;


                    //                 next_belief_state = Observations_vector_map_[*it_s][*it_o].cwiseProduct(belief_state);

                    for (int p_state_from = 0; p_state_from < n_of_physical_states; p_state_from++)
                    {
                        if (belief_state[p_state_from] < 0.0000001) // skipping
                            continue;
                        p = get_observation_probability(*it_s, p_state_from, *it_ol);
                        probability +=p;
                        double T = belief_state[p_state_from] * p;
                        total = total + T;
                        next_belief_state[p_state_from] = next_belief_state[p_state_from] + T;
                    }

                    if (total == 0.0)
                    {
                        continue;
                    }


                    //                    double myconstant{1.0/total};
                    //                    std::transform(next_belief_state.begin(), next_belief_state.end(), next_belief_state.begin(), [myconstant](auto& c){return c*myconstant;});

                    bstate norm_next_belief_state = next_belief_state/next_belief_state.sum();



                    if (abs(norm_next_belief_state.sum() - 1.0) > 0.0001)
                    {
                        cout << "WTF TOTAL " << norm_next_belief_state.sum()<< endl;
                        printState(norm_next_belief_state);
                    }


                    if (std::find(next_belief_states->begin(), next_belief_states->end(), norm_next_belief_state) == next_belief_states->end())
                    {
                        observations_list->push_back(*it_ol);
                        actions_list->push_back(*it_s);
                        next_belief_states->push_back(norm_next_belief_state);
                        p_vec.push_back(probability);
                        total_p +=probability;
                    }
                }
            }

            for (auto p : p_vec)
            {
                prob_list->push_back(p/total_p);
            }
        }
    }
}

