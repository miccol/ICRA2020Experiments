#include "node.h"
#include <utils.h>
#include <array>
#include <algorithm>

Node::Node(bstate belief_state,
           Node* parent_prt,
           Action action_from_parent) : belief_state_(belief_state),
    parent_prt_(parent_prt), action_from_parent_(action_from_parent),
    g(0.0), h(0.0), f(0.0)
{
    //        cout << "Constructor Called!" << endl;
    num_of_constructor++;
    max = belief_state.maxCoeff();
    entropy_=entropy(belief_state);

}
Node::~Node()
{
    //        cout << "Destructor Called!" << endl;
    num_of_destructor++;
}
bool Node::operator ==(const Node other) const
    {
        //if (belief_state_ == other.belief_state_)// && action_from_parent_ == other.action_from_parent_)
        if(bstateCompare(belief_state_, other.belief_state_, 0.001)
                && action_from_parent_ == other.action_from_parent_
                && this->g == other.g)
        {
            return true;
        }

        return false;
    }

//        Node::Node(Node&& other) noexcept:
//          belief_state_(std::move(other.belief_state_)),
//          parent_prt_(other.parent_prt_),
//          action_from_parent_(other.action_from_parent_),
//          g(other.g), h(other.h), f(other.h), max(other.max)
//        {
//         other.parent_prt_ = nullptr;
//         //other.belief_state_.fill(1.0/N_OF_PHYSICAL_STATES);
//        }


//        // move assignment
//        Node &Node::operator =(Node &&other) noexcept
//        {
//          if(this != &other)
//          {

//              Node moved(std::move(other));
//              std::swap(moved.parent_prt_, parent_prt_);

//          }
//          return *this;
//        }
