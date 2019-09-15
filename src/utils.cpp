#include <utils.h>

#include <iostream>
#include <definitions.h>
#include <math.h>       /* log2 */
#include <chrono>
#include <random>
#include <thread>
#include <sstream>
#include <fstream>
using namespace std;
void printState(bstate state)
{

if (state.rows() > 0)
{
    for (int i = 0; i < state.size(); i++ )
    {
            cout << state(i) << " ";
    }
}

         cout << endl;

}

string bstate_to_string(bstate state)
{
    std::ostringstream strs;
    if (state.rows() > 0)
    {
    for (int i = 0; i < state.size(); i++ )
    {
            strs << state[i] << " ";
    }

    }

    return strs.str();

}


float entropy(bstate state)
{
    float entropy = 0.0;

    for (int i = 0; i < state.size(); i++ )
    {
        if(state[i] > 0) entropy -= state[i]*log(state[i]);
    }
    return entropy;
}

int draw(std::vector<float> probabilities)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    std::discrete_distribution<int> distribution(probabilities.begin(), probabilities.end());

    return distribution(gen);

}

int     generateRandomNumber(int max) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> dis(0, max - 1);
    return dis(generator);
}

bool bstateCompare(bstate bstate_i, bstate bstate_j,  double epsilon)
{

    // bstate have fixed lengh
    for(int i = 0; i< bstate_i.size(); i++)
    {
        if(std::abs(bstate_i[i] - bstate_j[i]) > epsilon)
        {
            return false;
        }
    }
        return true;
}



vector < vector < int >> genperms (int n, int m)
{

vector < vector < int >>perms;

vector < int >curr (n, 1);

perms.push_back (curr);

while (true)
    {

int change = 0;

for (int i = n - 1; i >= 0; i--)
    {

if (curr[i] < m)
        {

curr[i]++;

for (int j = i + 1; j < n; j++)
        {

curr[j] = 1;

}
change = 1;

perms.push_back (curr);

break;		//try to change again
        }

}

if (change == 0)
    break;			//if we couldnt make a change we are done
    }

return perms;

}



std::vector<std::vector<bool>> get_all_observations(int n_of_objects)
{
    std::vector<std::vector<bool>> to_return;
    vector < vector < int >>res = genperms (n_of_objects, 2);


    for (auto r:res)
    {
        std::vector<bool> single_obs;

        for (auto i:r)
        {
            single_obs.push_back(i==1);
        }
        to_return.push_back(single_obs);
    }

    return to_return;

}


