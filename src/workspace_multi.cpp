#include <workspace_multi.h>
#include <definitions.h>
#include <iostream>
#include <utils.h>
#include <thread>

using namespace std;
WorkspaceMulti::WorkspaceMulti(const int size_x, const int size_y, const int num_of_objects)
{

    n_of_physical_states_ = size_x*size_y*4;


    cout << "Creating WorkspaceMulti..." << endl;


    int n_of_i = generateRandomNumber(n_of_physical_states_);
int s;

    while (states_with_objects_.size() < num_of_objects)
    {
    std::vector<int> states_with_i;
    while (states_with_i.size() < n_of_i)
    {
        s = generateRandomNumber(n_of_physical_states_);
        if (std::find(states_with_i.begin(), states_with_i.end(), s) == states_with_i.end())
        {
            states_with_i.push_back(s);
        }
    }
    states_with_objects_.push_back(states_with_i);
    }


    start_b_state_ = bstate(n_of_physical_states_);

    for( int i = 0; i < n_of_physical_states_; i++)
    {
        start_b_state_[i] = 1.0/n_of_physical_states_;
    }

    observations_ = get_all_observations(num_of_objects);
    observation_list_[Action::LOOK] = get_all_observations(num_of_objects);
}

std::vector<std::vector<bool> > WorkspaceMulti::observations() const
{
    return observations_;
}

std::map<Action, std::vector<std::vector<bool> > > WorkspaceMulti::observation_list() const
{
    return observation_list_;
}

bstate WorkspaceMulti::start_b_state() const
{
    return start_b_state_;
}


std::vector<Action> WorkspaceMulti::sensing() const
{
    return sensing_;
}


std::vector<Action> WorkspaceMulti::actuations() const
{
    return actuations_;
}

int WorkspaceMulti::n_of_physical_states() const
{
    return n_of_physical_states_;
}


