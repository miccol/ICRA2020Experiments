#include <workspace.h>
#include <definitions.h>
#include <iostream>
#include <utils.h>
#include <thread>

using namespace std;

Workspace::Workspace(const int size_x, const int size_y,
                     std::vector<int> states_with_A,
                     std::vector<int> states_with_B,
                     std::vector<int> states_with_C,
                     std::vector<int> states_with_D)
    : states_with_A_(states_with_A),states_with_B_(states_with_B),
      states_with_C_(states_with_C),states_with_D_(states_with_D)
{

    n_of_physical_states_ = (size_x)*(size_y)*4;


    start_b_state_ = bstate(n_of_physical_states_);

    for( int i = 0; i < n_of_physical_states_; i++)
    {
        start_b_state_[i] = 1.0/n_of_physical_states_;
    }

    for (Observation a: {Observation::A_SEEN, Observation::A_NOT_SEEN})
    {
        for (Observation b: {Observation::B_SEEN, Observation::B_NOT_SEEN})
        {
            for (Observation c: {Observation::C_SEEN, Observation::C_NOT_SEEN})
            {
                for (Observation d: {Observation::D_SEEN, Observation::D_NOT_SEEN})
                {
                    std::vector<Observation> l = {a,b,c,d};
                    observation_list_[Action::LOOK].push_back(l);
                    observations_.push_back(l);
                }

            }

        }

    }
}



Workspace::Workspace(const int size_x, const int size_y)
{

    n_of_physical_states_ = (size_x)*(size_y)*4;


    cout << "Creating Random Simulated Environment with " << n_of_physical_states_ << " states." << endl;
    cout <<"This may take a while to ensure randomity..." << endl;
    int n_of_A =  generateRandomNumber(n_of_physical_states_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    int n_of_B = generateRandomNumber(n_of_physical_states_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    int n_of_C = generateRandomNumber(n_of_physical_states_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    int n_of_D = generateRandomNumber(n_of_physical_states_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    int s = 0;

    cout << "N of objects of class A :" << n_of_A << endl;
    cout << "N of objects of class B :" << n_of_B << endl;
    cout << "N of objects of class C :" << n_of_C << endl;
    cout << "N of objects of class D :" << n_of_D << endl;

    cout <<"Scattering the objects at random places." << endl;
    cout <<"This may take a while to ensure randomity..." << endl;





    while (states_with_A_.size() < n_of_A)
    {
        s = generateRandomNumber(n_of_physical_states_);
        if (std::find(states_with_A_.begin(), states_with_A_.end(), s) == states_with_A_.end())
        {
            states_with_A_.push_back(s);
        }
    }


    while (states_with_B_.size() < n_of_B)
    {
        s = generateRandomNumber(n_of_physical_states_);
        if (std::find(states_with_B_.begin(), states_with_B_.end(), s) == states_with_B_.end())
        {
            states_with_B_.push_back(s);
        }
    }

    while (states_with_C_.size() < n_of_C)
    {
        s = generateRandomNumber(n_of_physical_states_);
        if (std::find(states_with_C_.begin(), states_with_C_.end(), s) == states_with_C_.end())
        {
            states_with_C_.push_back(s);
        }
    }

    while (states_with_D_.size() < n_of_D)
    {
        s = generateRandomNumber(n_of_physical_states_);
        if (std::find(states_with_D_.begin(), states_with_D_.end(), s) == states_with_D_.end())
        {
            states_with_D_.push_back(s);
        }
    }

    start_b_state_ = bstate(n_of_physical_states_);

    for( int i = 0; i < n_of_physical_states_; i++)
    {
        start_b_state_[i] = 1.0/n_of_physical_states_;
    }

    for (Observation a: {Observation::A_SEEN, Observation::A_NOT_SEEN})
    {
        for (Observation b: {Observation::B_SEEN, Observation::B_NOT_SEEN})
        {
            for (Observation c: {Observation::C_SEEN, Observation::C_NOT_SEEN})
            {
                for (Observation d: {Observation::D_SEEN, Observation::D_NOT_SEEN})
                {
                    std::vector<Observation> l = {a,b,c,d};
                    observation_list_[Action::LOOK].push_back(l);
                    observations_.push_back(l);
                }

            }

        }

    }
}

std::vector<std::vector<Observation> > Workspace::observations() const
{
    return observations_;
}

std::map<Action, std::vector<std::vector<Observation> > > Workspace::observation_list() const
{
    return observation_list_;
}

bstate Workspace::start_b_state() const
{
    return start_b_state_;
}

std::vector<int> Workspace::states_with_D() const
{
    return states_with_D_;
}

std::vector<int> Workspace::states_with_C() const
{
    return states_with_C_;
}

std::vector<int> Workspace::states_with_B() const
{
    return states_with_B_;
}

std::vector<int> Workspace::states_with_A() const
{
    return states_with_A_;
}


std::vector<Action> Workspace::sensing() const
{
    return sensing_;
}


std::vector<Action> Workspace::actuations() const
{
    return actuations_;
}

int Workspace::n_of_physical_states() const
{
    return n_of_physical_states_;
}


