#ifndef INCLUDEWORKSPACE_H
#define INCLUDEWORKSPACE_H
#include <vector>
#include <definitions.h>
#include <map>
class Workspace
{
public:
    Workspace(const int size_x, const int size_y,
                         std::vector<int> states_with_A,
                         std::vector<int> states_with_B,
                         std::vector<int> states_with_C,
                         std::vector<int> states_with_D); // manual configuration
    Workspace(const int size_x, const int size_y); // random
    std::vector<Action> actuations_ = {Action::ROTATE_CLOCKWISE, Action::ROTATE_COUNTERCLOCKWISE, Action::FORWARD, Action::BACKWARD};//, Action::LEFT, Action::RIGHT};
    std::vector<Action> sensing_ = {Action::LOOK};
    std::vector<int> states_with_A_, states_with_B_, states_with_C_, states_with_D_ ;
    bstate start_b_state_;
    std::map<Action, std::vector<std::vector<Observation>>> observation_list_;
    std::vector<std::vector<Observation>> observations_;

    std::vector<Action> actuations() const;
    std::vector<Action> sensing() const;
    std::vector<int> states_with_A() const;
    std::vector<int> states_with_B() const;
    std::vector<int> states_with_C() const;
    std::vector<int> states_with_D() const;
    bstate start_b_state() const;
    std::map<Action, std::vector<std::vector<Observation> > > observation_list() const;
    std::vector<std::vector<Observation> > observations() const;
    int n_of_physical_states() const;
private:
    int n_of_physical_states_;

};

#endif // INCLUDEWORKSPACE_H
