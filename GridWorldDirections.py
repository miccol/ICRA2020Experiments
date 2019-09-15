import pygame, sys, time, random, math
from pygame.locals import *
import numpy as np
from pygame import gfxdraw
action_dict = {'0': "Forward",
               '1': "Backward",
               '2': "Right",
               '3': "Left",
               }


# User-defined classes

class Triangle:

    borderColor = pygame.Color('black')
    borderWidth = 4  # the pixel width of the tile border

    def __init__(self, x, y, wall, surface, orientation , vector_value, objects = [], tile_size=(100, 100)):
        # Initialize a tile to contain an image
        # - x is the int x coord of the upper left corner
        # - y is the int y coord of the upper left corner
        # - image is the pygame.Surface to display as the
        # exposed image
        # - surface is the window's pygame.Surface object
        self.vector_value = vector_value
        self.wall = wall
        self.origin = (x, y)
        self.tile_coord = [x // 100, (y) // 100]

        self.objects = objects

        self.surface = surface
        # self.vector_value = x//100 + ((100*(board_width-2.0)-y)//100) * (board_width + 1.0)
        self.tile_size = tile_size
        self.probabiliy = 0.0
        self.orientation = orientation

        self.tile_text_center = (
            int(self.tile_coord[0] + self.tile_size[0] / 2.0), int(self.tile_coord[1] + self.tile_size[1] / 2.0))


        self.tile_center = (
            int(self.tile_coord[0] + self.tile_size[0] / 2.0), int(self.tile_coord[1] + self.tile_size[1] / 2.0))

        self.tile_coord_p0 = [self.origin[0] + tile_size[0] / 2, self.origin[1] + tile_size[0] / 2]


        text_offset = 25
        if self.orientation is 0:
            self.tile_coord_p1 = [self.tile_coord_p0[0] + tile_size[0] / 2.0, self.tile_coord_p0[1] + tile_size[1] / 2.0]
            self.tile_coord_p2 = [self.tile_coord_p0[0] + tile_size[0] / 2.0, self.tile_coord_p0[1] - tile_size[1] / 2.0]
            self.tile_text_center = (int(self.tile_coord_p0[0] + ((tile_size[0] / 2.0) - text_offset)), int(self.tile_coord_p0[1]))
            self.A_text_center = [int(self.tile_coord_p0[0] + (tile_size[0]/2.0) - 10 ), int(self.tile_coord_p0[1])]
        elif self.orientation is 3:
            self.tile_coord_p1 = [(self.tile_coord_p0[0]  - tile_size[0]/2.0), (self.tile_coord_p0[1]  +  tile_size[1]/2.0)]
            self.tile_coord_p2 = [(self.tile_coord_p0[0]  + tile_size[0]/2.0), (self.tile_coord_p0[1] +  tile_size[1]/2.0)]
            self.tile_text_center = (int(self.tile_coord_p0[0]), int(self.tile_coord_p0[1] + (tile_size[1]/2.0 - text_offset) ))
            self.A_text_center = [int(self.tile_coord_p0[0]  ), int(self.tile_coord_p0[1] + (tile_size[0]/2.0) - 14)]


        elif self.orientation is 2:
            self.tile_coord_p1 = [(self.tile_coord_p0[0]  - tile_size[0]/2), (self.tile_coord_p0[1] +  tile_size[1]/2)]
            self.tile_coord_p2 = [(self.tile_coord_p0[0]  - tile_size[0]/2), (self.tile_coord_p0[1] -  tile_size[1]/2)]
            self.tile_text_center = (int(self.tile_coord_p0[0] - (tile_size[0] / 2.0 - text_offset/2.0)), int(self.tile_coord_p0[1]))
            self.A_text_center = [self.tile_text_center[0] + text_offset/2.0, self.tile_text_center[1]]
            self.A_text_center = [int(self.tile_coord_p0[0] - (tile_size[0]/2.0 - 8) ), int(self.tile_coord_p0[1])]

        else:
            self.tile_coord_p1 = [(self.tile_coord_p0[0]  - tile_size[0]/2), (self.tile_coord_p0[1] -  tile_size[1]/2)]
            self.tile_coord_p2 = [(self.tile_coord_p0[0]  + tile_size[0]/2), (self.tile_coord_p0[1] -  tile_size[1]/2)]
            self.tile_text_center = (int(self.tile_coord_p0[0]), int(self.tile_coord_p0[1] - (tile_size[1]/2.0 - text_offset/2.0) ))
            self.A_text_center = [int(self.tile_coord_p0[0]  ), int(self.tile_coord_p0[1] - (tile_size[0]/2.0 - 2))]



        # #



    def arrow(screen, lcolor, tricolor, start, end, trirad):
        pygame.draw.line(screen, lcolor, start, end, 2)
        rotation = math.degrees(math.atan2(start[1] - end[1], end[0] - start[0])) + 90
        pygame.draw.polygon(screen, tricolor, (
            (end[0] + trirad * math.sin(math.radians(rotation)), end[1] + trirad * math.cos(math.radians(rotation))), (
                end[0] + trirad * math.sin(math.radians(rotation - 120)),
                end[1] + trirad * math.cos(math.radians(rotation - 120))), (
                end[0] + trirad * math.sin(math.radians(rotation + 120)),
                end[1] + trirad * math.cos(math.radians(rotation + 120)))))

    def draw(self, pos, color='white', action='none', probability=0.0, has_window=False, has_rack=False,
             has_plant=False, has_computer=False, has_robot=False):

        # Draw the tile.
        rectangle = pygame.Rect((self.origin[0], self.origin[1]), self.tile_size)
        # if self.orientation == 0:
            #first triangle
            # pygame.draw.rect(self.surface, pygame.Color('white'), rectangle, 0)
            # pygame.draw.rect(self.surface, Triangle.borderColor, rectangle, Triangle.borderWidth)
        # pygame.display.update()

        # pygame.draw.polygon(self.surface, pygame.Color(color),[self.tile_coord, self.tile_coord_p1, self.tile_coord_p2], 5)


        prob_color = (255 * (1 - probability), 255 * (1 - probability), 255 * (1 - probability))
        pygame.draw.polygon(self.surface, prob_color, [self.tile_coord_p0, self.tile_coord_p1, self.tile_coord_p2])
        pygame.draw.polygon(self.surface, pygame.Color('black'),[self.tile_coord_p0, self.tile_coord_p1, self.tile_coord_p2], 2)

        pygame.display.update()


        x = int(self.origin[0] + self.tile_size[0] / 2.0)
        y = int(self.origin[1] + self.tile_size[1] / 2.0)






        for index, object in enumerate(self.objects):

            font = pygame.font.SysFont(pygame.font.get_fonts()[1], 10)
            text = font.render(object, True, pygame.Color('black'))
            if self.orientation == 1 or self.orientation == 3:
                self.surface.blit(text, (self.A_text_center[0] -  len(self.objects)*15/2.0 + index*15, self.A_text_center[1]))
            else:
                self.surface.blit(text, (self.A_text_center[0], self.A_text_center[1] -  len(self.objects)*15/2.0 + index*15))




        if  has_robot:
            pygame.draw.circle(self.surface, pygame.Color('blue'), self.tile_text_center, 5)
            font = pygame.font.SysFont(pygame.font.get_fonts()[1], 10)
            text = font.render(action, True, pygame.Color('blue'))
            text_width, text_height = font.size(action)

            self.surface.blit(text, (x - text_width / 2.0, y - self.tile_size[1]/ 3.0))

        return
        

class GridWorld():
    # An object in this class represents a Grid_World game.
    tile_width = 100
    tile_height = 100

    def __init__(self, surface, board_size, wall_coords=[], start_coord=0, goal_coords=[], windows_coords=[],
                 rack_coords=[], plant_coords=[], computer_coords=[], pitfalls_coords=[],
                 actions={}, current_action='none', robot_pos=0, belief=[], states_with_A= [], states_with_B=[], states_with_C=[], states_with_D=[]):
        # Intialize a Grid_World game.
        # - surface is the pygame.Surface of the window

        self.surface = surface
        self.bgColor = pygame.Color('black')
        self.board_size = list(board_size)
        self.pitfalls_coords = pitfalls_coords
        self.windows_coords = windows_coords
        self.rack_coords = rack_coords
        self.plant_coords = plant_coords
        self.computer_coords = computer_coords
        self.actions = actions

        if not wall_coords:
            self.wall_coords = [[2, i] for i in range(board_size[1] - 1)]
        else:
            self.wall_coords = wall_coords

        self.start_coord = start_coord
        self.goal_coords = goal_coords
        self.position = start_coord
        self.robot_pos = robot_pos
        self.belief = belief
        self.reward = 0
        self.current_action = current_action

        self.calc_wall_coords()
        self.createTiles(states_with_A, states_with_B, states_with_C, states_with_D)

    def calc_wall_coords(self):
        self.board_wall_coords = [[self.board_size[0] - x - 1, y] for x, y in self.wall_coords]

    def find_board_coords(self, pos):
        # print("pos ", pos)
        return pos
        # x = pos[1]
        # y = self.board_size[0] - pos[0] -1
        # return [x,y]

    def set_belief(self, new_belief):
        self.belief = new_belief

    def set_robot_pos(self, new_pos):
        self.robot_pos = new_pos

    def set_current_action(self, new_action):
        self.current_action = new_action

    def createTiles(self, states_with_A= [], states_with_B=[], states_with_C=[], states_with_D=[]):
        # Create the Tiles
        # - self is the Grid_World game
        self.board = []
        vector_value = 0
        for rowIndex in range(0, self.board_size[1]):
            column = []
            for columnIndex in range(0, self.board_size[0]):
                triangles = []
                x = columnIndex * GridWorld.tile_width
                y = (self.board_size[1] - 1 - rowIndex) * GridWorld.tile_height
                if [rowIndex, columnIndex] in self.board_wall_coords:
                    wall = True
                else:
                    wall = False
                # for orientation in [0, 1, 2, 3]:
                #     print('create triangle', x, y, orientation)

                    # tile = Triangle(x, y, wall, self.surface, orientation, vector_value)

                objects = []
                if vector_value in states_with_A:
                    objects.append('A')
                if vector_value in states_with_B:
                    objects.append('B')
                if vector_value in states_with_C:
                    objects.append('C')
                if vector_value in states_with_D:
                    objects.append('D')

                triangles.append(Triangle(x, y, wall, self.surface, 0, vector_value, objects))
                vector_value += 1

                objects = []
                if vector_value in states_with_A:
                    objects.append('A')
                if vector_value in states_with_B:
                    objects.append('B')
                if vector_value in states_with_C:
                    objects.append('C')
                if vector_value in states_with_D:
                    objects.append('D')

                triangles.append(Triangle(x, y, wall, self.surface, 1, vector_value, objects))
                vector_value += 1

                objects = []
                if vector_value in states_with_A:
                    objects.append('A')
                if vector_value in states_with_B:
                    objects.append('B')
                if vector_value in states_with_C:
                    objects.append('C')
                if vector_value in states_with_D:
                    objects.append('D')

                triangles.append(Triangle(x, y, wall, self.surface, 2, vector_value, objects))
                vector_value += 1

                objects = []
                if vector_value in states_with_A:
                    objects.append('A')
                if vector_value in states_with_B:
                    objects.append('B')
                if vector_value in states_with_C:
                    objects.append('C')
                if vector_value in states_with_D:
                    objects.append('D')

                triangles.append(Triangle(x, y, wall, self.surface, 3, vector_value, objects))
                vector_value += 1
                column.append(triangles)
            self.board.append(column)
        print('n of tiles', vector_value)

    def draw(self):
        # Draw the tiles.
        # - self is the Grid_World game
        pos = self.find_board_coords(self.position)
        # goal = self.find_board_coords(self.goal_coords[0])
        self.surface.fill(self.bgColor)
        tile_with_robot = []
        for row in self.board:
            for q in row:
                for tile in q:
                    transformed_coords = tile.vector_value

                    # if transformed_coords < 0.0:
                    #     ##is the parked state
                    #     continue

                    # transformed_coords[1] = (self.board_size[1] -1) - transformed_coords[1]  # pygame considers the (0, 0) the upper left corner
                    probability = self.belief[transformed_coords]
                    if (transformed_coords == self.robot_pos):
                        tile_with_robot = tile

                    if transformed_coords in self.pitfalls_coords:
                        tile.draw(list(tile.tile_coord), 'red', self.current_action, probability,
                                  transformed_coords in self.windows_coords, transformed_coords in self.rack_coords,
                                  transformed_coords in self.plant_coords, transformed_coords in self.computer_coords,
                                  transformed_coords == self.robot_pos)
                    elif transformed_coords in self.goal_coords:
                        tile.draw(list(tile.tile_coord), 'green', self.current_action, probability,
                                  transformed_coords in self.windows_coords, transformed_coords in self.rack_coords,
                                  transformed_coords in self.plant_coords, transformed_coords in self.computer_coords,
                                  transformed_coords == self.robot_pos)
                    else:
                        tile.draw(list(tile.tile_coord), 'white', self.current_action, probability,
                                  transformed_coords in self.windows_coords, transformed_coords in self.rack_coords,
                                  transformed_coords in self.plant_coords, transformed_coords in self.computer_coords,
                                  transformed_coords == self.robot_pos)

        probability = self.belief[tile_with_robot.vector_value]
        tile_with_robot.draw(list(tile_with_robot.tile_coord), 'white', self.current_action, probability,
                             tile_with_robot.vector_value in self.windows_coords, tile_with_robot.vector_value in self.rack_coords,
                             tile_with_robot.vector_value in self.plant_coords, tile_with_robot.vector_value in self.computer_coords,
                             tile_with_robot.vector_value == self.robot_pos)
