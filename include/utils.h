#ifndef UTILS
#define UTILS

#include <iostream>
#include <definitions.h>
//#include <algorithm>
#include <math.h>       /* log2 */
#include <chrono>
#include <random>

using namespace std;

void printState(bstate state);     // prints on console a belief state
float entropy(bstate state); // computes entropy
int draw(std::vector<float> probabilities); // given a vector of probability distribution draws an index from such distribution
int generateRandomNumber(int max); // generates a randon number
bool bstateCompare(bstate bstate_i, bstate bstate_j, double epsilon = 0.001); // compare belief states with a threshold
string bstate_to_string(bstate state);
void compute_max_entropy();
std::vector<std::vector<bool>> get_all_observations(int n_of_objects);
#endif // UTILS
