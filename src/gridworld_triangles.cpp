#include <gridworld_triangles.h>
#include <definitions.h>
#include <algorithm>
#include <utils.h>
#include <Eigen/Dense>
using Eigen::MatrixXd;
GridWorldTriangles::GridWorldTriangles (int physical_initial_state, std::vector<Action> actuations, std::vector<Action> sensing,
                                        std::vector<std::vector<Observation> > observations, std::map<Action, std::vector<std::vector<Observation>>> observation_map, std::vector<int> states_with_A, std::vector<int> states_with_B,  std::vector<int> states_with_C, std::vector<int> states_with_D,
                                        int x_size, int y_size, int n_of_slices) :
    Model (physical_initial_state, actuations, sensing, observations,  observation_map),
    states_with_A_(states_with_A), states_with_B_(states_with_B), states_with_C_(states_with_C), states_with_D_(states_with_D),x_max_(x_size-1), y_max_(y_size-1), n_of_slices_(n_of_slices), n_of_physical_states_(x_size*y_size*n_of_slices)
{
    cout <<"Setting up Model..." << endl;

    int i = 0;
    for (int y = 0; y <= y_max_; y++ )
    {
        for (int x = 0; x <= x_max_; x++ )
        {
            for(int orientation = 0; orientation < n_of_slices_; orientation++)
            {
                states_.push_back(i);

                if(x == 0)
                {
                    // on the left border
                    switch (orientation)
                    {
                    case 0:
                        border_down_.push_back(i);
                        break;
                    case 1:
                        border_left_.push_back(i);
                        break;
                    case 2:
                        border_up_.push_back(i);
                        break;
                    case 3:
                        border_right_.push_back(i);
                        break;
                    default:
                        break;
                    }

                }
                if (x == x_max_)
                {
                    switch (orientation)
                    {
                    case 0:
                        border_up_.push_back(i);
                        break;
                    case 1:
                        border_right_.push_back(i);
                        break;
                    case 2:
                        border_down_.push_back(i);
                        break;
                    case 3:
                        border_left_.push_back(i);
                        break;
                    default:
                        break;
                    }
                }

                if(y == 0)
                {
                    // on the left border
                    switch (orientation)
                    {
                    case 0:
                        border_right_.push_back(i);
                        break;
                    case 1:
                        border_down_.push_back(i);
                        break;
                    case 2:
                        border_left_.push_back(i);
                        break;
                    case 3:
                        border_up_.push_back(i);
                        break;
                    default:
                        break;
                    }

                }
                 if (y == y_max_)
                {
                    switch (orientation)
                    {
                    case 0:
                        border_left_.push_back(i);
                        break;
                    case 1:
                        border_up_.push_back(i);
                        break;
                    case 2:
                        border_right_.push_back(i);
                        break;
                    case 3:
                        border_down_.push_back(i);
                        break;
                    default:
                        break;
                    }
                }

                i++;
            }

        }
    }


    for (auto it = actuations_.begin(); it != actuations_.end(); it++)
    {
        MatrixXd T(n_of_physical_states_, n_of_physical_states_);

        for (int p_state_from = 0; p_state_from < n_of_physical_states_; p_state_from ++)
        {
            for (int p_state_to = 0; p_state_to < n_of_physical_states_; p_state_to ++)
            {
                T(p_state_to, p_state_from) = get_transition_probability(*it, p_state_from, p_state_to);
            }
        }
        Transitions_matrices_map_.insert(std::make_pair(*it, T));
    }
}

double GridWorldTriangles::get_observation_probability(Action action, int state, std::vector<Observation> observation)
{
    double prob = 1.0;

    switch (action)
    {

    case Action::LOOK:
    {
        if ( std::find(observation.begin(), observation.end(), Observation::A_SEEN) != observation.end())
        {
            prob *= get_single_observation_probability(Action::LOOK_A, state, Observation::A_SEEN);
        }
        else
        {
            prob *= get_single_observation_probability(Action::LOOK_A, state, Observation::A_NOT_SEEN);

        }

        if ( std::find(observation.begin(), observation.end(), Observation::B_SEEN) != observation.end())
        {
            prob *= get_single_observation_probability(Action::LOOK_B, state, Observation::B_SEEN);
        }
        else
        {
            prob *= get_single_observation_probability(Action::LOOK_B, state, Observation::B_NOT_SEEN);

        }

        if ( std::find(observation.begin(), observation.end(), Observation::C_SEEN) != observation.end())
        {
            prob *= get_single_observation_probability(Action::LOOK_C, state, Observation::C_SEEN);
        }
        else
        {
            prob *= get_single_observation_probability(Action::LOOK_C, state, Observation::C_NOT_SEEN);

        }

        if ( std::find(observation.begin(), observation.end(), Observation::D_SEEN) != observation.end())
        {
            prob *= get_single_observation_probability(Action::LOOK_D, state, Observation::D_SEEN);
        }
        else
        {
            prob *= get_single_observation_probability(Action::LOOK_D, state, Observation::D_NOT_SEEN);

        }

        return prob;

    }
    default:
    {
        // is an actuation action
        if(observation.at(0) == Observation::NOTHING)
        {
            return 1.0;
        }
        cout << "Warning! action and observation do not match "<< endl;
        return -1.0;
    }


    }
}


double GridWorldTriangles::get_single_observation_probability(Action action, int state, Observation observation)
{
    double prob_false_positive = 0.01;
    double prob_false_negative = 0.01;

    switch (action)
    {

    case Action::LOOK_A:
    {
        switch(observation)
        {
        case Observation::A_SEEN:
            if ( std::find(states_with_A_.begin(), states_with_A_.end(), state) !=states_with_A_.end() )
            {
                return 1.0 - prob_false_negative;
            }
            else
            {
                return prob_false_positive;
            }
            break;
        case Observation::A_NOT_SEEN:
            if ( std::find(states_with_A_.begin(), states_with_A_.end(), state) != states_with_A_.end())
            {
                return prob_false_negative;
            }
            else
            {
                return 1.0 - prob_false_positive;
            }
            break;
        default:
            return 0.0;
        }
        break;
    }
    case Action::LOOK_B:
    {
        switch(observation)
        {
        case Observation::B_SEEN:
            if ( std::find(states_with_B_.begin(), states_with_B_.end(), state) !=states_with_B_.end() )
            {
                return 1.0 - prob_false_negative;
            }
            else
            {
                return prob_false_positive;
            }
            break;
        case Observation::B_NOT_SEEN:
            if ( std::find(states_with_B_.begin(), states_with_B_.end(), state) != states_with_B_.end())
            {
                return prob_false_negative;
            }
            else
            {
                return 1.0 - prob_false_positive;
            }
            break;
        default:
            return 0.0;
        }
        break;
    }
    case Action::LOOK_C:
    {
        switch(observation)
        {
        case Observation::C_SEEN:
            if ( std::find(states_with_C_.begin(), states_with_C_.end(), state) !=states_with_C_.end() )
            {
                return 1.0 - prob_false_negative;
            }
            else
            {
                return prob_false_positive;
            }
            break;
        case Observation::C_NOT_SEEN:
            if ( std::find(states_with_C_.begin(), states_with_C_.end(), state) != states_with_C_.end())
            {
                return prob_false_negative;
            }
            else
            {
                return 1.0 - prob_false_positive;
            }
            break;
        default:
            return 0.0;
        }
        break;
    }
    case Action::LOOK_D:
    {
        switch(observation)
        {
        case Observation::D_SEEN:
            if ( std::find(states_with_D_.begin(), states_with_D_.end(), state) !=states_with_D_.end() )
            {
                return 1.0 - prob_false_negative;
            }
            else
            {
                return prob_false_positive;
            }
            break;
        case Observation::D_NOT_SEEN:
            if ( std::find(states_with_D_.begin(), states_with_D_.end(), state) != states_with_D_.end())
            {
                return prob_false_negative;
            }
            else
            {
                return 1.0 - prob_false_positive;
            }
            break;
        default:
            return 0.0;
        }
        break;
    }


    default:
    {
        // is an actuation action
        if(observation == Observation::NOTHING)
        {
            return 1.0;
        }
        cout << "Warning! action and observation do not match "<< endl;
        return -1.0;
        break;
    }
    }

}



double GridWorldTriangles::get_transition_probability(Action action, int state_from, int state_to)
{
    const double prob_success = 0.98;
    const int max_state = n_of_physical_states_ - 1;

    const int n_of_squares = (1 + x_max_)* (1 + y_max_);



    int state_counter_clockwise = -1;
    int state_clockwise = -1;

    int orientation = state_from%n_of_slices_;

    if(state_from%n_of_slices_ == 0)
    {
        // "first" slice in the cell
        state_clockwise = state_from + n_of_slices_ - 1;
    }
    else
    {
        state_clockwise = state_from - 1;
    }

    if(orientation == (n_of_slices_-1))
    {
        // "last" slice in the cell
        state_counter_clockwise = state_from - (n_of_slices_ - 1);
    }
    else
    {
        state_counter_clockwise = state_from + 1;
    }




    int state_forwards = -1;


    switch (orientation) {
    case 0:
        state_forwards = state_from + n_of_slices_;
        break;
    case 1:
        state_forwards = state_from + (x_max_ + 1)*n_of_slices_;
        break;
    case 2:
        state_forwards = state_from - n_of_slices_;
        break;
    case 3:
        state_forwards = state_from - (x_max_ + 1)*n_of_slices_;
        break;
    default:
        cout << "WARNING orientation not recongnized" << endl;
        break;
    }


    int state_backwards = -1;


    switch (orientation) {
    case 0:
        state_backwards = state_from - n_of_slices_;
        break;
    case 1:
        state_backwards = state_from - (x_max_ + 1)*n_of_slices_;
        break;
    case 2:
        state_backwards = state_from + n_of_slices_;
        break;
    case 3:
        state_backwards = state_from + (x_max_ + 1)*n_of_slices_;
        break;

    default:
        cout << "WARNING orientation not recongnized" << endl;
        break;
    }


//    int state_left = -1;

//    switch (orientation) {
//    case 0:
//        state_left = state_from + (x_max_ + 1)*n_of_slices_;
//        break;
//    case 1:
//        state_left = state_from - n_of_slices_;
//        break;
//    case 2:
//        state_left = state_from - (x_max_ + 1)*n_of_slices_;
//        break;
//    case 3:
//        state_left = state_from + n_of_slices_;
//        break;
//    default:
//        cout << "WARNING orientation not recongnized" << endl;
//        break;
//    }


//    int state_right = -1;

//    switch (orientation) {
//    case 0:
//        state_right = state_from - (x_max_ + 1)*n_of_slices_;
//        break;
//    case 1:
//        state_right = state_from + n_of_slices_;
//        break;
//    case 2:
//        state_right = state_from + (x_max_ + 1)*n_of_slices_;
//        break;
//    case 3:
//        state_right = state_from - n_of_slices_;
//        break;
//    default:
//        cout << "WARNING orientation not recongnized" << endl;
//        break;
//    }


    int state_up = state_forwards;
    int state_down  = state_backwards;


    if (action == Action::ROTATE_CLOCKWISE)
    {
        if(state_to == state_clockwise)
        {
            return prob_success;
        }
        else
        {
            return 0.0;
        }

    }
    else if (action == Action::ROTATE_COUNTERCLOCKWISE)
    {
        if(state_to == state_counter_clockwise)
        {
            return prob_success;
        }
        else
        {
            return 0.0;
        }

    }

    else
    {

        switch (orientation) {
        case 0:
            if (state_from == state_to)
            {
                if (
                        (action == Action::FORWARD && std::find(border_up_.begin(), border_up_.end(), state_from) != border_up_.end()) ||
                        (action == Action::BACKWARD && std::find(border_down_.begin(), border_down_.end(), state_from) !=border_down_.end())
                        )

                {
                    return prob_success;
                }
                else
                {
                    return 1.0 - prob_success; // it does not move
                }
            }
            else
            {

                if (
                        (action == Action::FORWARD && std::find(border_up_.begin(), border_up_.end(), state_from) == border_up_.end() && state_to == state_up) ||
                        (action == Action::BACKWARD && std::find(border_down_.begin(), border_down_.end(), state_from) == border_down_.end() && state_to == state_down)
                        )
                {
                    return prob_success;
                }
                else
                {
                    return 0.0;
                }

            }


            break;

        case 1:
            if (state_from == state_to)
            {
                if (
                        (action == Action::FORWARD && std::find(border_up_.begin(), border_up_.end(), state_from) != border_up_.end()) ||
                        (action == Action::BACKWARD && std::find(border_down_.begin(), border_down_.end(), state_from) !=border_down_.end())
                        )


                {
                    return prob_success;
                }
                else
                {
                    return 1.0 - prob_success; // it does not move
                }
            }
            else
            {

                if (
                        (action == Action::FORWARD && std::find(border_up_.begin(), border_up_.end(), state_from) == border_up_.end() && state_to == state_up) ||
                        (action == Action::BACKWARD && std::find(border_down_.begin(), border_down_.end(), state_from) == border_down_.end() && state_to == state_down)
                        )
                {
                    return prob_success;
                }
                else
                {
                    return 0.0;
                }

            }

            break;
        case 2:
            if (state_from == state_to)
            {
                if (
                        (action == Action::FORWARD && std::find(border_up_.begin(), border_up_.end(), state_from) != border_up_.end()) ||
                        (action == Action::BACKWARD && std::find(border_down_.begin(), border_down_.end(), state_from) !=border_down_.end())
                        )


                {
                    return prob_success;
                }
                else
                {
                    return 1.0 - prob_success; // it does not move
                }
            }
            else
            {

                if (
                        (action == Action::FORWARD && std::find(border_up_.begin(), border_up_.end(), state_from) == border_up_.end() && state_to == state_up) ||
                        (action == Action::BACKWARD && std::find(border_down_.begin(), border_down_.end(), state_from) == border_down_.end() && state_to == state_down)
                        )
                {
                    return prob_success;
                }
                else
                {
                    return 0.0;
                }

            }
            break;
        case 3:
            if (state_from == state_to)
            {
                if (
                        (action == Action::FORWARD && std::find(border_up_.begin(), border_up_.end(), state_from) != border_up_.end()) ||
                        (action == Action::BACKWARD && std::find(border_down_.begin(), border_down_.end(), state_from) !=border_down_.end())
                        )

                {
                    return prob_success;
                }
                else
                {
                    return 1.0 - prob_success; // it does not move
                }
            }
            else
            {

                if (
                        (action == Action::FORWARD && std::find(border_up_.begin(), border_up_.end(), state_from) == border_up_.end() && state_to == state_up) ||
                        (action == Action::BACKWARD && std::find(border_down_.begin(), border_down_.end(), state_from) == border_down_.end() && state_to == state_down)
                        )
                {
                    return prob_success;
                }
                else
                {
                    return 0.0;
                }

            }
            break;
        default:
            cout << "WARNING orientation not recongnized" << endl;
            break;
        }
    }

    cout << "WARNING! something wrong in get_transition_probability" << endl;
    return -1.0;
}

const double GridWorldTriangles::get_cost(Action action)
{
    switch (action) {
    case Action::LOOK:
        return 1.0;
    default:
        return 10.0;
    }
}



