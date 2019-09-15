from GridWorldDirections import GridWorld
import pygame
import time
# On screen execution
pygame.init()


executed_actions = []

executed_belief_states = []

executed_real_states = []
states_with_A = []
states_with_B = []
states_with_C = []
states_with_D = []


with open('setup.txt', 'r') as myfile:
    data = myfile.readlines()
    initial_state = int(data[0])
    x_max = int(data[1])
    y_max = int(data[2])
    states_with_A_str = data[3].split(",")
    states_with_B_str = data[4].split(",")
    states_with_C_str = data[5].split(",")
    states_with_D_str = data[6].split(",")

    # states_with_A = list(map(int,states_with_A_str))
    # states_with_B = list(map(int,states_with_B_str))
    # states_with_C = list(map(int,states_with_C_str))
    # states_with_D = list(map(int,states_with_D_str))
    #



    for i in states_with_A_str:
        if i != '\n': states_with_A.append(int(i))

    for i in states_with_B_str:
        if i != '\n': states_with_B.append(int(i))

    for i in states_with_C_str:
        if i != '\n': states_with_C.append(int(i))

    for i in states_with_D_str:
        if i != '\n' and i !='': states_with_D.append(int(i))
    #
    #





with open('example.txt', 'r') as myfile:
    data = myfile.readlines()
    for lines in data:
        line = lines.split(',')
        executed_actions.append(line[0])
        lst = line[1].split()
        executed_belief_states.append(list(map(float, lst)))
        executed_real_states.append(int(line[2]))





# Set window size and title, and frame delay
surfaceSize = (100 * (x_max + 1), 100 * (y_max + 1))

windowTitle = 'Grid_World'

surface = pygame.display.set_mode(surfaceSize, 0, 0)
pygame.display.set_caption(windowTitle)




board = GridWorld(surface, (x_max + 1, y_max + 1),
                   robot_pos=executed_real_states[0], belief=executed_belief_states[0], states_with_A= states_with_A, states_with_B=states_with_B, states_with_C=states_with_C, states_with_D=states_with_D)
snapshot_index = 0
for action, belief_state, real_state in zip (executed_actions, executed_belief_states, executed_real_states):
    board.set_current_action(action)
    board.set_belief(list(belief_state))
    board.set_robot_pos(real_state)
    board.draw()
    pygame.display.update()
    fname = "snapshot" + str(snapshot_index) + ".png"
    pygame.image.save(pygame.display.get_surface(), fname)
    snapshot_index += 1
    time.sleep(1)



