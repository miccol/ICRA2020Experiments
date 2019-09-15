# belief-task-planning

Repository containing the ideas and program prototypes for the project of task planning in belief space.
Each sub-project lies in a separate folder


The problem is to carry out a task considering two types of uncertanety:
- Current state uncertanety: we have a probability distribution over the current stat  of the system
- Next state uncertanety: we have a probability distribution the next state, given the current state and the action performed.

The uncertanety over a state is represented as a belief state.
For example, if s is the state of the robot the state space is {A, B} (the systam can be either in state A or B) the belief state is b = [pa, pb] with pa = P(a=A) and pb = P(s=B).

# Current directions considered
We are exploring the follwing directions. **The last one is the most promising one**

## Belief state planning using Hierarchical Task Nework (HTN)
The idea is to take advantages of the recursion in HTN to recursively improve the estimation on the current state (i.e. less entropy in the belief)
This direction is **ON HOLD**


Check [HERE](https://github.com/hsp-iit/belief-task-planning/tree/master/BeSHOP) for more information about the ongoing work.

## Belief state planning using Markov Decision Processes (MDP)
The idea is to consider particles (like in particle filter) as states in a MDP.

Check [HERE](https://github.com/hsp-iit/belief-task-planning/tree/master/MDP) for more information about the ongoing work
This direction is **Replaced with the Search Solution**


## Belief state planning using Partially Observable Markov Decision Processes (POMDP)
The idea is to consider classical POMDP solutions to solve the task as ground base. (POMDP solutions are computationally demanding)

Check [HERE](https://github.com/hsp-iit/belief-task-planning/tree/master/POMDP) for more information about the ongoing work
This direction is **DONE**



## MOST PROMISING! Framing the problem as a search problem (Belief state planning using Transition Likelihood Stochastic shortest path problem. )

It is basically an MDP in which all rewards are negative, there is a set of goal states, and the objective is to minimize the total expected cost (negative reward) incurred before reaching a goal state and terminating. 

Check [HERE](https://github.com/hsp-iit/belief-task-planning/tree/master/Search) for more information about the ongoing work

This direction is **ONGOING**

