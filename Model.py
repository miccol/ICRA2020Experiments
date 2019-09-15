import numpy as np


def draw_arg(probs):
    # sample one element from a probability distribution
    if abs(sum(probs) - 1.0) > 0.00000001:
        print(probs)
    assert(abs(sum(probs) - 1.0) < 0.00000001)
    probs = np.array(probs)
    # Do a second normalisation to avoid the problem described here: https://stackoverflow.com/questions/46539431/np-random-choice-probabilities-do-not-sum-to-1
    return np.random.choice(list(range(len(probs))), p=probs/probs.sum())

class Model():
    def __init__(self, x_max, y_max, real_initial_state, actuation_actions_list = [], sensing_actions_list = [],
                 observations_list = {}, states_with_window = [], states_with_rack = [], states_with_plant = [],
                 states_with_computer = []):

        self.x_min = 0
        self.y_min = 0
        self.x_max = x_max
        self.y_max = y_max
        self.curr_state = real_initial_state
        self.states = []

        self.border_down = []
        self.border_up = []
        self.border_left = []
        self.border_right = []
        i = 0
        for y in range(self.y_min, self.y_max + 1):
            for x in range(self.x_min, self.x_max + 1):
                if y == self.y_min:
                    self.border_down.append(i)
                if y == self.y_max:
                    self.border_up.append(i)
                if x == self.x_min:
                    self.border_left.append(i)
                if x == self.x_max:
                    self.border_right.append(i)
                self.states.append(i)
                i += 1

        self.actuation_actions_list = actuation_actions_list
        self.sensing_actions_list = sensing_actions_list

        self.observations_list = observations_list
        self.states_with_window = states_with_window
        self.states_with_rack = states_with_rack
        self.states_with_plant = states_with_plant
        self.states_with_computer = states_with_computer

    def get_next_b_states(self, belief_state):
        next_belief_states_list = []
        for action in self.actuation_actions_list:
            next_belief_state = [0] * len(belief_state)
            for state_from, physical_state_prob_from in enumerate(belief_state):
                if physical_state_prob_from == 0.0:
                    continue
                for state_to, physical_state_prob_to in enumerate(belief_state):
                    T = physical_state_prob_from * self.transition_function(action, state_from, state_to)
                    next_belief_state[state_to] += T
                    # actuations are deterministic in the belief space

            normalized_next_belief_state = [x / sum(next_belief_state) for x in next_belief_state]

            next_belief_states_list.append((tuple(normalized_next_belief_state), action))

        for action in self.sensing_actions_list:
            for observation in self.observations_list[action]:
                next_belief_state = [0] * len(belief_state)
                for state_to, physical_state_prob_to in enumerate(belief_state):

                    T = self.observation_function(action, state_to, observation)*physical_state_prob_to
                    next_belief_state[state_to] += T
                    # sensing are non deterministic in the belief space

                if sum(next_belief_state) == 0:
                    continue

                normalized_next_belief_state = [x / sum(next_belief_state) for x in next_belief_state]

                next_belief_states_list.append((tuple(normalized_next_belief_state), action))

        return next_belief_states_list  # belief state is a tuple

    def observation_function(self, action, end_state, obs):
        if action == 'lookwindow':
            if obs == 'window_seen':
                if end_state in self.states_with_window:
                    return 1.0
                else:
                    return 0.0
            elif obs == 'window_not_seen':
                if end_state not in self.states_with_window:
                    return 1.0
                else:
                    return 0.0
            else:
                return 0.0
        elif action == 'lookrack':
            if obs == 'rack_seen':
                if end_state in self.states_with_rack:
                    return 1.0
                else:
                    return 0.0
            elif obs == 'rack_not_seen':
                if end_state not in self.states_with_rack:
                    return 1.0
                else:
                    return 0.0
            else:
                return 0.0
        elif action == 'lookplant':
            if obs == 'plant_seen':
                if end_state in self.states_with_plant:
                    return 1.0
                else:
                    return 0.0
            elif obs == 'plant_not_seen':
                if end_state not in self.states_with_plant:
                    return 1.0
                else:
                    return 0.0
            else:
                return 0.0

        elif action == 'lookcomputer':
            if obs == 'computer_seen':
                if end_state in self.states_with_computer:
                    return 1.0
                else:
                    return 0.0
            elif obs == 'computer_not_seen':
                if end_state not in self.states_with_computer:
                    return 1.0
                else:
                    return 0.0
            else:
                return 0.0
        else:
            print("ERROR! Action ", action, " not recognized")
            return -1.0

    def transition_function(self, action, state_from, state_to):
        prob_down = 0.0
        prob_left = 0.0
        prob_right = 0.0
        prob_up = 1.0

        state_left = state_from - 1
        state_right = state_from + 1

        state_up = state_from + self.x_max + 1
        state_down = state_from - (self.x_max + 1)

        if state_from == state_to:
            if (action == 'left' and state_from in self.border_left) \
                    or (action == 'right' and state_from in self.border_right) \
                    or (action == 'down' and state_from in self.border_down) \
                    or (action == 'up' and state_from in self.border_up):
                return prob_up
            else:
                return 0.0

        elif (action == 'left' and state_from not in self.border_left and state_to == state_left) \
                or (action == 'right' and state_from not in self.border_right and state_to == state_right) \
                or (action == 'up' and state_from not in self.border_up and state_to == state_up) \
                or (action == 'down' and state_from not in self.border_down and state_to == state_down):
            return prob_up


        else:
            return 0.0

    def take_action(self, action):
        """
        Accepts an action and changes the underlying environment state

        action: action to take
        return: next state, observation and reward
        """
        state, observation = self.simulate_action(self.curr_state, action)
        # state, observation, reward, cost = self.simulate_action(self.curr_state, action)
        self.curr_state = state
        return state, observation#, reward, cost

    def simulate_action(self, si, ai, debug=False):
        """
        Query the resultant new state, observation and rewards, if action ai is taken from state si

        si: current state
        ai: action taken at the current state
        return: next state, observation and reward
        """
        # get new state
        if ai in self.actuation_actions_list:
            s_probs = [self.transition_function(ai, si, sj) for sj in self.states]
            state = self.states[draw_arg(s_probs)]
        elif ai in self.sensing_actions_list:
            state = si
        else:
            print("simulate_action, cannot recognize action", si)

        # get new observation
        if ai in self.sensing_actions_list:
            o_probs = [self.observation_function(ai, state, oj) for oj in self.observations_list[ai]]
            possible_observations = self.observations_list[ai]
            observation = possible_observations[draw_arg(o_probs)]
        else:
            observation = None

        if debug:
            print('taking action {} at state {}'.format(ai, si))
            print('transition probs: {}'.format(s_probs))
            print('obs probs: {}'.format(o_probs))

        return state, observation


    def update_belief(self, belief, action, obs):


        b_new = belief
        b_new_old =b_new

        for sj in self.states:
            p_o_prime = self.observation_function(action, sj, obs)
            summation = 1.0
            x = sj
            l_bnew = list(b_new)
            l_bnew[x] = l_bnew[x] * p_o_prime * summation
            b_new = tuple(l_bnew)

            b_new_old = b_new

        # normalize
        total = sum(b_new)
        if abs(total - 1.0) < 0.00001:
            total = 1.0
        b_new = [x / total for x in b_new]
        return tuple(b_new)