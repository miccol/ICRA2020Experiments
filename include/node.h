#ifndef NODE_H
#define NODE_H

#include <definitions.h>

class Node
{
    public:
    Node(bstate belief_state,
         Node* parent_prt,
         Action action_from_parent);
    ~Node();
    bool operator ==(const Node other) const;
    bstate belief_state_ ;
    Node* parent_prt_;
    Action action_from_parent_;
    float entropy_;
    double max;
    double g, h, f; // cost, heuristic and sum of them, needed for the search


//    Node(Node &&other) noexcept;
//    Node &operator =(Node &&other) noexcept;
private:
    int num_of_constructor;
    int num_of_destructor;

};
#endif // NODE_H
