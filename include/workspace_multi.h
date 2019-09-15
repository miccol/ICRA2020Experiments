#ifndef INCLUDEWORKSPACEMULTI_H
#define INCLUDEWORKSPACEMULTI_H
#include <vector>
#include <definitions.h>
#include <map>
class WorkspaceMulti
{
public:
    WorkspaceMulti(const int size_x, const int size_y, const int num_of_objects);
    std::vector<Action> actuations_ = {Action::ROTATE_CLOCKWISE, Action::ROTATE_COUNTERCLOCKWISE, Action::FORWARD, Action::BACKWARD};//, Action::LEFT, Action::RIGHT};
    std::vector<Action> sensing_ = {Action::LOOK};
    std::vector<std::vector<int>> states_with_objects_ ;
    bstate start_b_state_;
    std::map<Action, std::vector<std::vector<bool>>> observation_list_;
    std::vector<std::vector<bool>> observations_;

    std::vector<Action> actuations() const;
    std::vector<Action> sensing() const;
    std::vector<int> states_with_A() const;
    std::vector<int> states_with_B() const;
    std::vector<int> states_with_C() const;
    std::vector<int> states_with_D() const;
    bstate start_b_state() const;
    std::map<Action, std::vector<std::vector<bool> > > observation_list() const;
    std::vector<std::vector<bool> > observations() const;
    int n_of_physical_states() const;
private:
    int n_of_physical_states_;

};

#endif // INCLUDEWORKSPACE_H
