#ifndef R1DEFINITIONS_H
#define R1DEFINITIONS_H
#include <array>
#include <Eigen/Dense>
using Eigen::VectorXd;

/*
#define N_OF_PHYSICAL_STATES 100
#define X_MAX 9
#define Y_MAX 9

enum class Action {UP, DOWN, LEFT, RIGHT, LOOKWINDOW, LOOKRACK, NONE, REPLAN, ROTATE_CLOCKWISE, ROTATE_COUNTERCLOCKWISE, LOOK_A, LOOK_B, LOOK_C, LOOK_D};
enum class Observation {NOTHING, WINDOW_SEEN, WINDOW_NOT_SEEN, RACK_SEEN, RACK_NOT_SEEN,
                        A_SEEN, A_NOT_SEEN, B_SEEN, B_NOT_SEEN, C_SEEN, C_NOT_SEEN, D_SEEN, D_NOT_SEEN};

using bstate = std::array<double, N_OF_PHYSICAL_STATES>;
*/

#define N_OF_SLICES  4
//#define N_OF_SQUARES  4
#define X_MAX 2
#define Y_MAX 1
#define N_OF_PHYSICAL_STATES 24
///#define EPSILON 0.9
#define PROB 0.9

//struct physical_state
//{
//    int position;
//    int orientation;
//};

//using physical_state = std::pair<int, int>;

enum class Action {NONE, REPLAN, ROTATE_CLOCKWISE, ROTATE_COUNTERCLOCKWISE, LOOK_A, LOOK_B, LOOK_C, LOOK_D, FORWARD, BACKWARD, LEFT, RIGHT};
//enum class Observation {NOTHING, A_SEEN, A_NOT_SEEN, B_SEEN, B_NOT_SEEN, C_SEEN, C_NOT_SEEN, D_SEEN, D_NOT_SEEN};
enum class Observation {NOTHING, B00TO15, B15TO30, B30TO45, B45TO60, B60TO75, B75TO90};

//using bstate = std::array<double, N_OF_PHYSICAL_STATES>;

using bstate = VectorXd;





#endif // R1DEFINITIONS_H
