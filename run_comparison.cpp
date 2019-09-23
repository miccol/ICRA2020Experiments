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

bool  run_random_approach(Workspace* ws, GridWorldTriangles* model, double *plan_cost)
{

    // random

    bstate current_b_state_random = ws->start_b_state();

    std::vector<Action> possibile_actions = {Action::NONE, Action::REPLAN, Action::ROTATE_CLOCKWISE, Action::ROTATE_COUNTERCLOCKWISE, Action::LOOK_A, Action::LOOK_B, Action::LOOK_C, Action::LOOK_D, Action::FORWARD, Action::BACKWARD, Action::LEFT, Action::RIGHT, Action::LOOK};
    std::vector<float> actuation_action_probability = {0.0, 0.0, 1.0/4, 1.0/4, 0.0, 0.0, 0.0, 0.0, 1.0/4, 1.0/4, 0.0, 0.0, 0.0};

    Action current_action_random;
    std::vector<Observation> current_observation_random = {Observation::NOTHING};


    bstate initial_b_random(ws->n_of_physical_states());

    for( int i = 0; i < ws->n_of_physical_states(); i++)
    {
        initial_b_random[i] = 0.0;
    }

    int n_random = 0;
    float cost_random = 0.0;
    bstate old_b_state = ws->start_b_state();
    int current_p_state = model->current_physical_state();
    bool is_ob_ok = true;
    while(current_b_state_random.maxCoeff() < PROB)
    {current_observation_random.clear();
        n_random++;

        current_p_state = model->current_physical_state();
        if (n_random%2==0)
        {
            int x = draw(actuation_action_probability);
            current_action_random = possibile_actions.at(x);
            cost_random +=  model->get_cost(current_action_random);
            current_observation_random = model->take_action(current_action_random, ws);
            current_b_state_random = model->updateBelief(current_b_state_random, current_action_random, current_observation_random,  &is_ob_ok);
        }
        else
        {
            current_action_random = Action::LOOK;
            current_observation_random = model->take_action(Action::LOOK, ws);
            cost_random +=  model->get_cost(current_action_random);
            old_b_state = current_b_state_random;
            current_b_state_random = model->updateBelief(current_b_state_random, current_action_random, current_observation_random, &is_ob_ok);
        }
        if(!is_ob_ok)
        {
            cout << "Wrong observation "<< endl;
            return false;
        }

    }

    *plan_cost = cost_random;

    return true;

}

bool  run_our_approach(Workspace* ws, GridWorldTriangles* model, std::vector<float> *elapsed_times,
                       double *plan_cost, int *n_of_replans, const bool has_heuristic=true,  const bool has_watchdog=false)
{
    bool has_solution = false;
    std::this_thread::sleep_for(std::chrono::nanoseconds(2000));

    std::vector<bstate> path, executed_b_state;
    std::vector<Action> actions, executed_actions;
    std::vector<int> executed_physical_states;

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

        executed_b_state.push_back(current_b_state);
        executed_actions.push_back(current_action);
        executed_physical_states.push_back(model->current_physical_state());
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

                (*n_of_replans)++;

                i = -1;
                path.clear();
                actions.clear();
                start = chrono::steady_clock::now();

                has_solution = plan(model, current_b_state, &path, &actions, &closed_list, has_heuristic, has_watchdog);
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
    executed_b_state.push_back(current_b_state);
    executed_actions.push_back(Action::NONE);
    executed_physical_states.push_back(model->current_physical_state());

    return true;
}

bool  run_uniform_approach(Workspace* ws, GridWorldTriangles* model, std::vector<float> *elapsed_times, double *plan_cost, int *n_of_replans)
{
    return run_our_approach(ws, model, elapsed_times, plan_cost, n_of_replans, false, true);
}

void run_comparison(const int size_x, const int size_y, const int N_OF_EXPERIMENS, const bool compare_uniform, const bool write_on_file)
{


    std::vector<float> our_mean_times;
    std::vector<float> our_n_replans, our_mean_cost;
    std::vector<float> uniform_mean_times;
    std::vector<float> uniform_n_of_replans, uniform_mean_costs;
    std::vector<float> random_mean_costs;

    double our_costs = 0.0;
    int our_replans = 0;

    double uniform_costs = 0.0;
    int uniform_replans = 0;

    double random_costs = 0.0;

    std::vector<float> our_elapsed_times, uniform_elapsed_times;

    for(int i=0; i< N_OF_EXPERIMENS; i++)
    {

        Workspace* ws = new Workspace(size_x, size_y);


        int start_physical_state = generateRandomNumber(ws->n_of_physical_states());

        GridWorldTriangles* model = new GridWorldTriangles(start_physical_state, ws->actuations(), ws->sensing(), ws->observations(), ws->observation_list(),
                                                           ws->states_with_A(), ws->states_with_B(), ws->states_with_C(), ws->states_with_D(), size_x, size_y, 4);
        our_costs = 0.0;
        our_replans = 0.0;
        our_elapsed_times.clear();
        cout << " Episode  " << i << endl;
        cout << " Running our approach " << i << endl;

        bool has_solution = run_our_approach(ws, model, &our_elapsed_times, &our_costs, &our_replans);

        if(!has_solution)
        {
            continue;
        }

        double total = 0.0;
        for (int i = 0; i < our_elapsed_times.size(); i++)
        {
            total +=our_elapsed_times.at(i);
        }


        our_mean_times.push_back(total/our_elapsed_times.size());
        our_n_replans.push_back(our_replans);
        our_mean_cost.push_back(our_costs);

        uniform_costs = 0.0;
        uniform_replans = 0.0;
        uniform_elapsed_times.clear();


        if(compare_uniform)
        {
            cout << " Running uniform cost search approach" << i << endl;

            run_uniform_approach(ws, model, &uniform_elapsed_times, &uniform_costs, &uniform_replans);
        }
        else
        {
            cout << " Uniform  approach cannot handle lage state spaces" << i << endl;

        }
        total = 0.0;
        for (int i = 0; i < uniform_elapsed_times.size(); i++)
        {
            total +=uniform_elapsed_times.at(i);
        }

        uniform_mean_times.push_back(total/uniform_elapsed_times.size());
        uniform_n_of_replans.push_back(uniform_replans);
        uniform_mean_costs.push_back(uniform_costs);


        random_costs = 0.0;


        cout << " Running pseudo random approach " << i << endl;

        run_random_approach(ws, model, &random_costs);


        random_mean_costs.push_back(random_costs);

    }

    double our_total_mean_times = 0.0;
    double our_total_mean_costs = 0.0;
    double our_total_mean_replans = 0.0;


    for (int i = 0; i < our_mean_times.size(); i++)
    {
        our_total_mean_times +=our_mean_times.at(i);
        our_total_mean_costs +=our_mean_cost.at(i);
        our_total_mean_replans +=our_n_replans.at(i);
    }


    cout << "Our mean elapsed plan time "<< our_total_mean_times/our_mean_times.size() << endl;
    cout << "Our mean plan cost "<< our_total_mean_costs/our_mean_cost.size() << endl;
    cout << "Our mean replan "<< our_total_mean_replans/our_n_replans.size() << endl;


    double uniform_total_mean_times = 0.0;
    double uniform_total_mean_costs = 0.0;
    double uniform_total_mean_replans = 0.0;


    for (int i = 0; i < our_mean_times.size(); i++)
    {
        uniform_total_mean_times +=uniform_mean_times.at(i);
        uniform_total_mean_costs +=uniform_mean_costs.at(i);
        uniform_total_mean_replans +=uniform_n_of_replans.at(i);
    }
    cout << "Uniform mean elapsed plan time "<< uniform_total_mean_times/uniform_mean_times.size() << endl;
    cout << "Uniform mean plan cost "<< uniform_total_mean_costs/uniform_mean_costs.size() << endl;
    cout << "Uniform mean replan "<< uniform_total_mean_replans/uniform_n_of_replans.size() << endl;


    double random_total_mean_costs = 0.0;


    for (int i = 0; i < our_mean_times.size(); i++)
    {
        random_total_mean_costs +=random_mean_costs.at(i);
    }

    cout << "------ Number of physical states:  " << size_x*size_y*4 << "------ "<< endl;

    cout << "Uniform mean elapsed plan time : "<< uniform_total_mean_times/uniform_mean_times.size() << endl;
    cout << "Our mean elapsed plan time    : "<< our_total_mean_times/our_mean_times.size() << endl;

    cout << "Random mean plan cost : "<< random_total_mean_costs/random_mean_costs.size() << endl;
    cout << "Uniform mean plan cost: "<< uniform_total_mean_costs/uniform_mean_costs.size() << endl;
    cout << "Our mean plan cost    : "<< our_total_mean_costs/our_mean_cost.size() << endl;

    cout << "Uniform mean replan   : "<< uniform_total_mean_replans/uniform_n_of_replans.size() << endl;
    cout << "Our mean replan       : "<< our_total_mean_replans/our_n_replans.size() << endl;

    cout << "Experiments without a solution not counted."  << endl;




    if(write_on_file)

    {

        ofstream myfile_state;

        myfile_state.open ("results"+ std::to_string(size_x) + "x" + std::to_string(size_y)+".txt");
        double our_total_mean_times = 0.0;
        double our_total_mean_costs = 0.0;
        double our_total_mean_replans = 0.0;


        for (int i = 0; i < our_mean_times.size(); i++)
        {
            our_total_mean_times +=our_mean_times.at(i);
            our_total_mean_costs +=our_mean_cost.at(i);
            our_total_mean_replans +=our_n_replans.at(i);
        }

        double uniform_total_mean_times = 0.0;
        double uniform_total_mean_costs = 0.0;
        double uniform_total_mean_replans = 0.0;


        for (int i = 0; i < our_mean_times.size(); i++)
        {
            uniform_total_mean_times +=uniform_mean_times.at(i);
            uniform_total_mean_costs +=uniform_mean_costs.at(i);
            uniform_total_mean_replans +=uniform_n_of_replans.at(i);
        }
        double random_total_mean_costs = 0.0;


        for (int i = 0; i < our_mean_times.size(); i++)
        {
            random_total_mean_costs +=random_mean_costs.at(i);
        }

        myfile_state << "------ Number of physical states:  " << size_x*size_y*4 << "------ "<< endl;

        myfile_state << "Uniform mean elapsed plan time : "<< uniform_total_mean_times/uniform_mean_times.size() << endl;
        myfile_state << "Our mean elapsed plan time    : "<< our_total_mean_times/our_mean_times.size() << endl;

        myfile_state << "Random mean plan cost : "<< random_total_mean_costs/random_mean_costs.size() << endl;
        myfile_state << "Uniform mean plan cost: "<< uniform_total_mean_costs/uniform_mean_costs.size() << endl;
        myfile_state << "Our mean plan cost    : "<< our_total_mean_costs/our_mean_cost.size() << endl;

        myfile_state << "Uniform mean replan   : "<< uniform_total_mean_replans/uniform_n_of_replans.size() << endl;
        myfile_state << "Our mean replan       : "<< our_total_mean_replans/our_n_replans.size() << endl;

        myfile_state << "Experiments without a solution not counted"  << endl;

        myfile_state.close();

    }


}






int main()
{

    const int N_OF_EXPERIMENS = 100;
    int size_x = 5;
    int size_y = 5;
    bool compare_uniform = true;
    bool save_results_on_file = true;

    run_comparison(size_x, size_y, N_OF_EXPERIMENS, compare_uniform, save_results_on_file);

    size_x = 16;
    size_y = 16;

    run_comparison(size_x, size_y, N_OF_EXPERIMENS, compare_uniform, save_results_on_file);

    size_x = 50;
    size_y = 50;

    compare_uniform = false; // uniform approach does not handle such large state space.
    run_comparison(size_x, size_y, N_OF_EXPERIMENS, compare_uniform, save_results_on_file);

    return 0;
}
