#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <chrono>
#include <thread>

#include <utils.h>
#include <definitions.h>
#include <node.h>
#include <gridworld_triangles.h>
#include <planner.h>
#include <workspace.h>
#include <fstream>

void play(Workspace* ws, int X_MAX, int Y_MAX, int start_physical_state,
          std::vector<bstate> executed_b_state,
          std::vector<Action> executed_actions,
          std::vector<int> executed_physical_states)

{
    std::array<std::string, 13> actions_names = {"NONE", "PLAN", "ROTATE_CLOCKWISE", "ROTATE_COUNTERCLOCKWISE", "LOOK_A", "LOOK_B", "LOOK_C", "LOOK_D",  "FORWARD", "BACKWARD", "LEFT", "RIGHT", "LOOK"};

    std::vector<int> states_with_A = ws->states_with_A();
    std::vector<int> states_with_B = ws->states_with_B();

    std::vector<int> states_with_C =  ws->states_with_C();
    std::vector<int> states_with_D =  ws->states_with_D();

    ofstream myfile_setup;

    myfile_setup.open ("setup.txt");
    myfile_setup << start_physical_state << endl;
    myfile_setup << X_MAX << endl;
    myfile_setup << Y_MAX << endl;

    for (int s : states_with_A)
    {
        myfile_setup << s << ",";
    }

    myfile_setup << endl;


    for (int s : states_with_B)
    {
        myfile_setup << s << ",";
    }

    myfile_setup << endl;
    for (int s : states_with_C)
    {
        myfile_setup << s << ",";
    }

    myfile_setup << endl;
    for (int s : states_with_D)
    {
        myfile_setup << s << ",";
    }

    myfile_setup << endl;

    myfile_setup.close();

    ofstream myfile;

    myfile.open ("example.txt");

    for (int i = 0; i < executed_actions.size(); i++)
    {
        myfile << actions_names[int(executed_actions.at(i))] << ","
                                                             << bstate_to_string(executed_b_state.at(i)) << ","
                                                             << executed_physical_states.at(i) << endl;
    }

    myfile.close();

    cout<<"Showing execution on screen" << endl;
    system("python play.py"); // I will change this eventually
}



bool  run_our_approach(Workspace* ws, GridWorldTriangles* model, std::vector<float> *elapsed_times,
                       double *plan_cost, int *n_of_replans, std::vector<bstate>* executed_b_state,
                       std::vector<Action>* executed_actions, std::vector<int>* executed_physical_states, const bool has_heuristic=true)
{
    bool has_solution = false;
    std::this_thread::sleep_for(std::chrono::nanoseconds(2000));

    std::vector<bstate> path;
    std::vector<Action> actions;

    std::vector<Node> closed_list;

    auto start = chrono::steady_clock::now();
    has_solution = plan(model, ws->start_b_state(), &path, &actions, &closed_list, has_heuristic);
    auto end = chrono::steady_clock::now();

    if (!has_solution)
    {
        cout<<"No solution found" << endl;
        return false;
    }

    elapsed_times->push_back(pow(10,-9)*chrono::duration_cast<chrono::nanoseconds>(end - start).count());

    bstate current_b_state = ws->start_b_state();
    bstate expected_next_b_state = path.at(0);
    Action current_action = actions.at(0);
    std::vector<Observation> current_observation = {Observation::NOTHING};

    int i = -1;
    while (true)
    {
        i++;

        if(i == actions.size() || i == path.size() || current_b_state.maxCoeff() >= PROB )
        {
            break;
        }

        current_action = actions.at(i);
        executed_b_state->push_back(current_b_state);
        executed_actions->push_back(current_action);
        executed_physical_states->push_back(model->current_physical_state());
        (*plan_cost)+= model->get_cost(current_action);
        current_observation = model->take_action(current_action, ws);
        expected_next_b_state = path.at(i);

        vector<Action> se = ws->sensing();

        bool is_sensing = std::find(se.begin(), se.end(), current_action) != se.end();

        if(is_sensing)
        {

            bool is_ok;
            current_b_state = model->updateBelief(current_b_state, current_action, current_observation, &is_ok);

            if (!bstateCompare(current_b_state, expected_next_b_state))
            {
                if(current_b_state.maxCoeff() >= PROB )
                {
                    break;
                }

                executed_b_state->push_back(current_b_state);
                executed_actions->push_back(Action::REPLAN);
                executed_physical_states->push_back(model->current_physical_state());
                (*n_of_replans)++;

                i = -1;
                path.clear();
                actions.clear();
                start = chrono::steady_clock::now();

                has_solution = plan(model, current_b_state, &path, &actions, &closed_list, has_heuristic);
                end = chrono::steady_clock::now();
                if(has_solution)
                {
                    elapsed_times->push_back(pow(10,-9)*chrono::duration_cast<chrono::nanoseconds>(end - start).count());
                }
                else
                {
                    cout<<"No solution found" << endl;
                    return false;
                }

            }
        }
        else
        {
            if(current_b_state.maxCoeff() >= PROB )
            {
                break;
            }
            current_b_state = expected_next_b_state;
        }
    }
    executed_b_state->push_back(current_b_state);
    executed_actions->push_back(Action::NONE);
    executed_physical_states->push_back(model->current_physical_state());

    return true;
}

void run_single_random(const int size_x, const int size_y)
{

    std::vector<bstate> executed_b_state;
    std::vector<Action> executed_actions;
    std::vector<int> executed_physical_states;

    double our_costs = 0.0;
    int our_replans = 0;

    std::vector<float> our_elapsed_times, uniform_elapsed_times;

    Workspace* ws = new Workspace(size_x, size_y);

    int start_physical_state = generateRandomNumber(ws->n_of_physical_states());

    GridWorldTriangles* model = new GridWorldTriangles(start_physical_state, ws->actuations(), ws->sensing(), ws->observations(), ws->observation_list(),
                                                       ws->states_with_A(), ws->states_with_B(), ws->states_with_C(), ws->states_with_D(), size_x, size_y, 4);
    our_costs = 0.0;
    our_replans = 0.0;
    our_elapsed_times.clear();
    cout << " Running our approach "  << endl;

    bool has_solution = run_our_approach(ws, model, &our_elapsed_times, &our_costs, &our_replans, &executed_b_state, &executed_actions, &executed_physical_states);

    if(!has_solution)
    {
        cout << " Solution not found " << endl;
    }
    else
    {
        float sum_of_times= 0.0;
        for (auto& t : our_elapsed_times)
            sum_of_times += t;

        cout << " Solution found! " << endl;
        cout << " Mean planning times: " <<sum_of_times/our_elapsed_times.size() << endl;
        cout << " N of replans: " << our_replans<<endl;
        cout << " Plan cost: " << our_costs << endl;

        play(ws, size_x - 1,size_y - 1, start_physical_state,
             executed_b_state,
             executed_actions,
             executed_physical_states);
    }
}

void run_single(const int size_x, const int size_y,
                int start_physical_state,
                std::vector<int> states_with_A,
                std::vector<int> states_with_B,
                std::vector<int> states_with_C,
                std::vector<int> states_with_D)
{


    std::vector<bstate> executed_b_state;
    std::vector<Action> executed_actions;
    std::vector<int> executed_physical_states;

    double our_costs = 0.0;
    int our_replans = 0;
    std::vector<float> our_elapsed_times;

    Workspace* ws = new Workspace(size_x, size_y, states_with_A, states_with_B, states_with_C, states_with_D);

    GridWorldTriangles* model = new GridWorldTriangles(start_physical_state, ws->actuations(), ws->sensing(), ws->observations(), ws->observation_list(),
                                                       ws->states_with_A(), ws->states_with_B(), ws->states_with_C(), ws->states_with_D(), size_x, size_y, 4);
    our_costs = 0.0;
    our_replans = 0.0;
    our_elapsed_times.clear();
    cout << " Running our approach "  << endl;

    bool has_solution = run_our_approach(ws, model, &our_elapsed_times, &our_costs, &our_replans,
                                         &executed_b_state, &executed_actions, &executed_physical_states);

    if(!has_solution)
    {
        cout << " Solution not found!" << endl;
    }
    else
    {
        float sum_of_times= 0.0;
        for (auto& t : our_elapsed_times)
            sum_of_times += t;

        cout << " Solution found! " << endl;
        cout << " Mean planning times: " <<sum_of_times/our_elapsed_times.size() << endl;
        cout << " N of replans: " << our_replans<<endl;
        cout << " Plan cost: " << our_costs << endl;

        play(ws, size_x - 1,size_y - 1, start_physical_state,
             executed_b_state,
             executed_actions,
             executed_physical_states);
    }
}


int main()
{
    // Read the readme to understand how to edit these
    const int size_x = 5;
    const int size_y = 5;

    const int initial_physical_state = 0;

    const std::vector<int> states_with_A = {0, 1, 3, 10, 15,16};
    const std::vector<int> states_with_B = {4, 5, 7, 11, 23};
    const std::vector<int> states_with_C = {8, 9, 11, 32, };
    const std::vector<int> states_with_D = {12, 13, 14, 15};


    run_single(size_x, size_y, initial_physical_state, states_with_A, states_with_B,states_with_C, states_with_D);
    //run_single_random(size_x, size_y); //use this to run a random workspace
    return 0;
}
