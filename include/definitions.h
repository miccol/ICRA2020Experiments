#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#include <array>
#include <Eigen/Dense>
#define PROB 0.9

enum class Action {NONE, REPLAN, ROTATE_CLOCKWISE, ROTATE_COUNTERCLOCKWISE, LOOK_A, LOOK_B, LOOK_C, LOOK_D, FORWARD, BACKWARD, LEFT, RIGHT, LOOK};
enum class Observation {NOTHING, A_SEEN, A_NOT_SEEN, B_SEEN, B_NOT_SEEN, C_SEEN, C_NOT_SEEN, D_SEEN, D_NOT_SEEN};
using bstate = Eigen::VectorXd;





#endif // DEFINITIONS_H
