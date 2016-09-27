import numpy as np
import math
from maze import Maze
import turtle

delta = [[ 0,  1], # up
         [ 1,  0], # right
         [ 0, -1], # down
         [-1,  0]] # left

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        self.maze_dim = maze_dim
        self.map = np.full([maze_dim, maze_dim], 15, dtype=np.int)
        print 'dimensions: ', maze_dim
        self.reset()
        self.first_run = True

        # for drawing
        self.bot = turtle.Turtle()
        self.wally = turtle.Turtle()

    def draw_map(self, x, y):
        # Intialize the window and drawing turtle.
        self.wally.speed(0)
        self.wally.hideturtle()
        self.wally.penup()
        self.wally.color('black')

        # maze centered on (0,0), squares are 20 units in length.
        sq_size = 20
        origin = self.maze_dim * sq_size / -2

        if not self.map[x][y] & 1 > 0:
            self.wally.goto(origin + sq_size * x, origin + sq_size * (y+1))
            self.wally.setheading(0)
            self.wally.pendown()
            self.wally.forward(sq_size)
            self.wally.penup()

        if not self.map[x][y] & 2 > 0:
            self.wally.goto(origin + sq_size * (x+1), origin + sq_size * y)
            self.wally.setheading(90)
            self.wally.pendown()
            self.wally.forward(sq_size)
            self.wally.penup()

        if not self.map[x][y] & 4 > 0:
            self.wally.goto(origin + sq_size * x, origin + sq_size * y)
            self.wally.setheading(0)
            self.wally.pendown()
            self.wally.forward(sq_size)
            self.wally.penup()

        if not self.map[x][y] & 8 > 0:
            self.wally.goto(origin + sq_size * x, origin + sq_size * y)
            self.wally.setheading(90)
            self.wally.pendown()
            self.wally.forward(sq_size)
            self.wally.penup()

    def draw_bot(self):
        # maze centered on (0,0), squares are 20 units in length.
        sq_size = 20
        origin = self.maze_dim * sq_size / -2

        # draw turtle bot
        self.bot.hideturtle()
        self.bot.penup()
        self.bot.shape('turtle')
        self.bot.color('green')
        self.bot.shapesize(0.3, 0.3, 0.3)
        self.bot.goto(origin + sq_size * self.location[0] + sq_size/2, origin + sq_size * self.location[1] + sq_size/2)
        self.bot.showturtle()
        self.bot.pendown()

    def reset(self):
        self.location = [0, 0]
        self.heading = 0
        self.goal = [(x, y) for x in [self.maze_dim/2, self.maze_dim/2-1] for y in [self.maze_dim/2, self.maze_dim/2-1]]
        print 'goal: ', self.goal

    def set_goal(self, x, y):
        self.goal = [(x, y)]

    def update_map(self, sensors):
        from collections import deque
        sense = deque(sensors)
        sense.append(0)
        sense.rotate(self.heading - 1)
        # print sensors
        # print sense
        if not self.heading == 2:
            self.map[self.location[0]][self.location[1]+sense[0]] &= ~1
            self.draw_map(self.location[0], self.location[1]+sense[0])
        if not self.heading == 3:
            self.map[self.location[0]+sense[1]][self.location[1]] &= ~2
            self.draw_map(self.location[0]+sense[1], self.location[1])
        if not self.heading == 0:
            self.map[self.location[0]][self.location[1]-sense[2]] &= ~4
            self.draw_map(self.location[0], self.location[1]-sense[2])
        if not self.heading == 1:
            self.map[self.location[0]-sense[3]][self.location[1]] &= ~8
            self.draw_map(self.location[0]-sense[3], self.location[1])

    def heuristic(self, x, y):
        '''
        Heuristic function using euclidean distance to the goal
        '''
        return math.sqrt(((self.maze_dim-1)/2-x)**2+((self.maze_dim-1)/2-y)**2)

    def update_plan(self):
        cost = 1
        opened = [(self.heuristic(self.location[0], self.location[1]), 0, None, self.location[0], self.location[1])]
        closed = np.full_like(self.map, -1)
        closed[self.location[0]][self.location[1]] = 0
        route = []

        while True:
            if len(opened) == 0:
                return "Fail"
            else:
                opened.sort()
                f, g, h, x, y  = opened.pop(0)
                # route.append(h)
                if (x, y) in self.goal:
                    step = closed[x][y]
                    while step != 0:
                        for i in range(len(delta)):
                            x2 = x - delta[i][0]
                            y2 = y - delta[i][1]
                            if x2 >= 0 and x2 < len(self.map) and y2 >=0 and y2 < len(self.map[0]):
                                if  closed[x2][y2] > -1 and closed[x2][y2] == step-1 and 1 << i & self.map[x2][y2] > 0:
                                    route.append(i)
                                    step -= 1
                                    x = x2
                                    y = y2
                                    break
                    route.reverse()
                    break
                else:
                    for i in range(len(delta)):
                        if 1 << i & self.map[x][y] > 0: # can go in that direction
                            x2 = x + delta[i][0]
                            y2 = y + delta[i][1]
                            if x2 >= 0 and x2 < len(self.map) and y2 >=0 and y2 < len(self.map[0]): # make sure we're not exceeding bounds
                                if closed[x2][y2] < 0: # and we haven't covered this location before
                                    g2 = g + cost
                                    f2 = g2 + self.heuristic(x2, y2)
                                    opened.append([f2, g2, i, x2, y2])
                                    closed[x2][y2] = g2
        # print closed
        return route

    def move(self, rotation, movement):
        self.heading = (self.heading + rotation/90) % 4
        self.location[0] += delta[self.heading][0]*movement
        self.location[1] += delta[self.heading][1]*movement

    def visualize(self):
        # Intialize the window and drawing turtle.
        self.wally.speed(0)
        self.wally.hideturtle()
        self.wally.penup()
        self.wally.color('black')

        # maze centered on (0,0), squares are 20 units in length.
        sq_size = 20
        origin = self.maze_dim * sq_size / -2

        # iterate through squares one by one to decide where to draw walls
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                if not self.map[x][y] & 1 > 0:
                    self.wally.goto(origin + sq_size * x, origin + sq_size * (y+1))
                    self.wally.setheading(0)
                    self.wally.pendown()
                    self.wally.forward(sq_size)
                    self.wally.penup()

                if not self.map[x][y] & 2 > 0:
                    self.wally.goto(origin + sq_size * (x+1), origin + sq_size * y)
                    self.wally.setheading(90)
                    self.wally.pendown()
                    self.wally.forward(sq_size)
                    self.wally.penup()

                if not self.map[x][y] & 4 > 0:
                    self.wally.goto(origin + sq_size * x, origin + sq_size * y)
                    self.wally.setheading(0)
                    self.wally.pendown()
                    self.wally.forward(sq_size)
                    self.wally.penup()

                if not self.map[x][y] & 8 > 0:
                    self.wally.goto(origin + sq_size * x, origin + sq_size * y)
                    self.wally.setheading(90)
                    self.wally.pendown()
                    self.wally.forward(sq_size)
                    self.wally.penup()

        self.bot.hideturtle()
        self.bot.penup()
        self.bot.shape('turtle')
        self.bot.color('green')
        self.bot.shapesize(0.3, 0.3, 0.3)
        self.bot.goto(origin + sq_size * self.location[0] + sq_size/2, origin + sq_size * self.location[1] + sq_size/2)
        self.bot.showturtle()
        self.bot.pendown()

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        if tuple(self.location) in self.goal:
            # if self.first_run:
            #     self.first_run = False
            #     self.set_goal(0, 0)
            # else:
            self.reset()
            return 'Reset', 'Reset'

        self.update_map(sensors)
        self.draw_bot()
        route = self.update_plan()
        # print 'location: ', self.location
        # print 'heading: ', self.heading
        # print route
        rotation = 90
        movement = 0
        for r in [0, 1, -1]:
            if (self.heading + r + 4) % 4 == route[0]:
                rotation = r * 90
                movement = 1
                break
        # print rotation, movement
        # raw_input("Press Enter to continue...")
        self.move(rotation, movement)
        return rotation, movement
