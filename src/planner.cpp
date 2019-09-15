#include "planner.h"
#include <model.h>
#include <node.h>
#include <vector>
#include <utils.h>

using namespace std;

bool plan (GridWorldTriangles* model, bstate start_state, std::vector<bstate>* path_to_return, std::vector<Action>* actions_to_return,
           std::vector<Node>* closed_list, const bool has_heuristic, const bool has_watchdog)
{
    auto start_time = chrono::steady_clock::now();
    auto time_now = chrono::steady_clock::now();

    std::vector<Node> open_list;
    std::vector<bstate> closed_list_b;


    std::vector<bstate> next_states;
    std::vector<Action> actions;
    std::vector<std::vector<Observation>> observations;
    std::vector<float> entropies, probabilities;

    Node start_node = Node(start_state, nullptr, Action::NONE);
    open_list.push_back(start_node);


    while (!open_list.empty())
    {
        if (has_watchdog)
        {
            time_now = chrono::steady_clock::now();

            float elapsed_time = pow(10,-9)*chrono::duration_cast<chrono::nanoseconds>(time_now - start_time).count();

            if (elapsed_time > 60.0) // more that 60 seconds to plan
            {
                cout << "The uniform approach is taking too much time!" << endl;
                cout << "Stopping here" << endl;
                return true;

            }

        }

        Node current_node = open_list.at(0);
        vector<Node>::iterator it_to_remove = open_list.begin();

        // find the node at the frontier to expand
        for (auto it = open_list.begin(); it !=open_list.end(); it++)
        {
            if ((*it).f < current_node.f)
            {
                current_node = *it;
                it_to_remove = it;
            }
        }

        open_list.erase(it_to_remove);

        if ( current_node.max >= PROB)
        {
            // search is over

            Node* current = &current_node;


            int x = 0;

            while(current->belief_state_ != start_node.belief_state_)
            {
                path_to_return->insert(path_to_return->begin(), current->belief_state_);
                actions_to_return->insert(actions_to_return->begin(), current->action_from_parent_);
                current = current->parent_prt_;
            }
            return true;
        }

        // computes the next possible states

        next_states.clear();
        actions.clear();
        observations.clear();
        entropies.clear();
        probabilities.clear();

        model->get_next_belief_states(current_node.belief_state_, &next_states, &actions, &observations, &entropies, &probabilities);
        closed_list->push_back(current_node);
        closed_list_b.push_back(current_node.belief_state_);

        // check same size
        for (int i = 0; i < next_states.size(); i++)
        {
            Node* parent_node = new Node(current_node.belief_state_, current_node.parent_prt_, current_node.action_from_parent_);

            Node new_node = Node(next_states.at(i), parent_node, actions.at(i));

            double cost =model->get_cost(actions.at(i));



//            if (actions.at(i) == Action::LOOK || actions.at(i) == Action::LOOK_A || actions.at(i) == Action::LOOK_B ||actions.at(i) == Action::LOOK_C || actions.at(i) == Action::LOOK_D)
//            {
//                if (actions.at(i) == Action::LOOK_A || actions.at(i) == Action::LOOK_B ||actions.at(i) == Action::LOOK_C || actions.at(i) == Action::LOOK_D)
//                    cout << "WTF************************************************** "<< endl;

//                cost = 1.0; // some actions have lower cost (just made up)
//            }
            new_node.g = current_node.g + cost;
            if(has_heuristic)
            {
                new_node.h = new_node.entropy_/(new_node.max * (probabilities.at(i)));
            }
            else
            {
                new_node.h = 0.0;
            }
            new_node.f = (new_node.g + new_node.h);

//            cout << "action " << int(actions.at(i)) << endl;
//            cout << "heuristic " << new_node.h << endl;
//            cout << "f " << new_node.f << endl;


       //     if (!std::find(closed_list_b.begin(), closed_list_b.end(), next_states.at(i)))
                if (std::find(open_list.begin(), open_list.end(), new_node) == open_list.end()
                        && std::find(closed_list->begin(), closed_list->end(), new_node) == closed_list->end()
                        )
            {
                open_list.push_back(new_node);
            }
        }
    }

      cout << "No solution found "<< endl;
      return false;

}
