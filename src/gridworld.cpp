#include <gridworld.h>
#include <algorithm>
#include <utils.h>

    GridWorld::GridWorld (const int x_max, const int y_max, int physical_initial_state, std::vector<Action> actuations, std::vector<Action> sensing,
           std::vector<Observation> observations,std::map<Action, std::vector<Observation>> observation_map,  std::vector<int> states_with_window, std::vector<int> states_with_rack) :
        Model (physical_initial_state, actuations, sensing,
                observations,  observation_map), x_max_(x_max), y_max_(y_max), states_with_window_(states_with_window), states_with_rack_(states_with_rack)
    {
        int i = 0;
        for (int y = 0; y <= y_max_; y++ )
        {
            for (int x = 0; x <= x_max_; x++ )
            {
                states_.push_back(i);
                if (y == 0)
                {
                    border_down_.push_back(i);
                }
                else if (y == y_max_)
                {
                    border_up_.push_back(i);
                }
                if(x == 0)
                {
                    border_left_.push_back(i);
                }
                else if (x == x_max_)
                {
                    border_right_.push_back(i);
                }
                i++;
            }
        }
    }

 double GridWorld::get_observation_probability(Action action, int state, Observation observation)
 {
     double prob_false_positive = 0.0;
     double prob_false_negative = 0.0;

     switch (action)
     {
     case Action::LOOKWINDOW:
     {
         switch(observation)
         {
         case Observation::WINDOW_SEEN:
            if ( std::find(states_with_window_.begin(), states_with_window_.end(), state) !=states_with_window_.end() )
            {
                return 1.0 - prob_false_negative;
            }
            else
            {
                return prob_false_positive;
            }
             break;
         case Observation::WINDOW_NOT_SEEN:
            if ( std::find(states_with_window_.begin(), states_with_window_.end(), state) != states_with_window_.end())
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
     case Action::LOOKRACK :
     {
         switch(observation)
         {
         case Observation::RACK_SEEN:
            if ( std::find(states_with_rack_.begin(), states_with_rack_.end(), state) != states_with_rack_.end())
            {
                return 1.0 - prob_false_negative;
            }
            else
            {
                return prob_false_positive;
            }
             break;
         case Observation::RACK_NOT_SEEN:
            if ( std::find(states_with_rack_.begin(), states_with_rack_.end(), state) != states_with_rack_.end())
            {
                return prob_false_negative;
            }
            else
            {
                return 1.0 - prob_false_positive;
            }
             break;
         default:
//             cout << "Warning! sensing action  and observation dont match"<< endl;
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

 double GridWorld::get_transition_probability(Action action, int state_from, int state_to)
 {
     double prob_success = 1.0;
     double prob_slide_left = 0.0;
     double prob_slide_right = 0.0;




     int state_left = state_from - 1;
     int state_right = state_from + 1;

     int state_up = state_from + x_max_ + 1;
     int state_down = state_from - (x_max_ + 1);

     if (state_from == state_to)
     {
         if (
            (action == Action::LEFT && std::find(border_left_.begin(), border_left_.end(), state_from) != border_left_.end()) ||
            (action == Action::RIGHT && std::find(border_right_.begin(), border_right_.end(), state_from) !=  border_right_.end()) ||
            (action == Action::UP && std::find(border_up_.begin(), border_up_.end(), state_from) != border_up_.end()) ||
            (action == Action::DOWN && std::find(border_down_.begin(), border_down_.end(), state_from) !=border_down_.end())
             )

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

         if (
            (action == Action::LEFT && std::find(border_left_.begin(), border_left_.end(), state_from) == border_left_.end() && state_to == state_left) ||
            (action == Action::RIGHT && std::find(border_right_.begin(), border_right_.end(), state_from) == border_right_.end() && state_to == state_right) ||
            (action == Action::UP && std::find(border_up_.begin(), border_up_.end(), state_from) == border_up_.end() && state_to == state_up) ||
            (action == Action::DOWN && std::find(border_down_.begin(), border_down_.end(), state_from) == border_down_.end() && state_to == state_down)
             )
         {
             return prob_success;
         }
         else
         {
             return 0.0;
         }

     }

 }



