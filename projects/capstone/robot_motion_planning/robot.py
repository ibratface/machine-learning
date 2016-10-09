import numpy as np
import math
import turtle
import random
import sys

delta = [[ 0,  1], # up
         [ 1,  0], # right
         [ 0, -1], # down
         [-1,  0]] # left

class Visualizer(object):
    def __init__(self, dim):
        # maze centered on (0,0), squares are 20 units in length.
        self.sq_size = 20
        self.origin = dim * self.sq_size / -2

        # for drawing the bot
        self.bot = turtle.Turtle()
        self.bot.shape('turtle')
        self.bot.color('red')
        self.bot.shapesize(0.4, 0.4, 0.4)
        self.bot.penup()
        self.bot.hideturtle()

        # for drawing the path
        self.path = turtle.Turtle()
        self.path.speed(0)
        self.path.shape('circle')
        self.path.color('green')
        self.path.shapesize(0.2, 0.2, 0.2)

        # for drawing walls
        self.wally = turtle.Turtle()
        self.wally.speed(0)
        self.wally.hideturtle()
        self.wally.penup()

    def draw_bot(self, x, y, h):
        # draw bot
        self.bot.showturtle()
        self.bot.goto(self.origin + self.sq_size * x + self.sq_size/2, self.origin + self.sq_size * y + self.sq_size/2)

    def draw_path(self, x, y, path):
        self.path.clear()
        px = self.origin + self.sq_size * x + self.sq_size/2
        py = self.origin + self.sq_size * y + self.sq_size/2
        self.path.penup()
        self.path.goto(px, py)
        self.path.stamp()
        self.path.pendown()
        for p in path:
            px += delta[p][0]*self.sq_size
            py += delta[p][1]*self.sq_size
            self.path.goto(px, py)
            self.path.stamp()

    def draw_cell(self, x, y, v):
        if not v & 1 > 0:
            self.wally.goto(self.origin + self.sq_size * x, self.origin + self.sq_size * (y+1))
            self.wally.setheading(0)
            self.wally.pendown()
            self.wally.forward(self.sq_size)
            self.wally.penup()

        if not v & 2 > 0:
            self.wally.goto(self.origin + self.sq_size * (x+1), self.origin + self.sq_size * y)
            self.wally.setheading(90)
            self.wally.pendown()
            self.wally.forward(self.sq_size)
            self.wally.penup()

        if not v & 4 > 0:
            self.wally.goto(self.origin + self.sq_size * x, self.origin + self.sq_size * y)
            self.wally.setheading(0)
            self.wally.pendown()
            self.wally.forward(self.sq_size)
            self.wally.penup()

        if not v & 8 > 0:
            self.wally.goto(self.origin + self.sq_size * x, self.origin + self.sq_size * y)
            self.wally.setheading(90)
            self.wally.pendown()
            self.wally.forward(self.sq_size)
            self.wally.penup()

    def draw_maze(self, maze):
        for x in range(len(maze.map)):
            for y in range(len(maze.map[0])):
                self.draw_cell(x, y, maze.map[x][y])

class Maze(object):
    def __init__(self, dim):
        self.dim = dim
        self.map = np.full([dim, dim], 15, dtype=np.int)

    @staticmethod
    def from_file(filename):
        with open(filename, 'rb') as f_in:
            # First line should be an integer with the maze dimensions
            dim = int(f_in.next())
            maze = Maze(dim)
            # Subsequent lines describe the permissability of walls
            walls = []
            for line in f_in:
                walls.append(map(int,line.split(',')))
            maze.map = np.array(walls)
            return maze

    def shape(self):
        return self.map.shape

    def bounds(self, x, y):
        return x >= 0 and x < len(self.map) and y >=0 and y < len(self.map[0])

    def passable(self, x, y, h):
        return self.bounds(x, y) and self.bounds(x+delta[h][0], y+delta[h][1]) and self.map[x][y] & (1 << h) > 0

    def block_cell(self, x, y, h):
        '''
        return True if a change was made
        '''
        passable = self.passable(x, y, h)
        if passable: self.map[x][y] &= ~(1 << h)
        return passable

class AStarPlanner(object):
    def __init__(self, maze, start=(0, 0), goal=None, cost=1):
        self.map = maze
        self.start = start
        self.goal = goal
        self.cost = 1
        if self.goal == None:
            center = self.map.dim/2
            self.goal = [(x, y) for x in [center, center-1] for y in [center, center-1]]

    def heuristic(self, x, y):
        '''
        Heuristic function using euclidean distance to the goal
        '''
        gx, gy = np.mean(self.goal, axis=0) # goal is usually a room so we just get the center of that room
        return math.sqrt((gx-x)**2 + (gy-y)**2)

    def plan(self):
        '''
        Do A* search. Return an optimal route.
        '''
        x, y = self.start
        opened = [(self.heuristic(x, y), 0, x, y)]
        closed = np.full(self.map.shape(), -1, dtype=np.int)
        closed[x][y] = 0
        route = None
        while len(opened) > 0:
            opened.sort()
            f, g, x, y  = opened.pop(0)
            if (x, y) in self.goal:
                # We've found the goal. Backtrace a route to the starting location
                route = []
                step = closed[x][y]
                i = 0
                while step != 0:
                    while True:
                        x2 = x - delta[i][0]
                        y2 = y - delta[i][1]
                        if self.map.passable(x2, y2, i) and closed[x2][y2] == step-1:
                            # check if our planner went through this cell and if it was the previous step
                            route.append(i)
                            step -= 1
                            x = x2
                            y = y2
                            break
                        else:
                            i = (i + 1) % 4
                route.reverse()
                break
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if self.map.passable(x, y, i) and closed[x2][y2] < 0:
                        # passable and we haven't covered this location before
                        g2 = g + self.cost
                        f2 = g2 + self.heuristic(x2, y2)
                        opened.append([f2, g2, x2, y2])
                        closed[x2][y2] = g2
        # print closed
        self.closed = closed
        return route

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        self.maze = Maze(maze_dim)
        self.viz = Visualizer(maze_dim)
        self.reset()
        self.route = []
        self.route_i = 0

        # meta plan
        self.flipflop = True
        self.practice = True

        # stats
        self.trips = 1
        self.steps = 0
        self.steps_expected = 0
        self.turns = 0
        self.train_time = 0
        self.test_time = 0

    def reset(self):
        self.loc = [0, 0]
        self.heading = 0
        self.planner = AStarPlanner(self.maze, self.loc)

    def update_map(self, sensors):
        changed = False
        for i, d in enumerate(sensors):
            h = (self.heading + i - 1 + 4) % 4
            # block off detected walls
            changed = self.maze.block_cell(
                self.loc[0]+delta[h][0]*sensors[i],
                self.loc[1]+delta[h][1]*sensors[i],
                h) or changed
            # also block off opposite cell
            changed = self.maze.block_cell(
                self.loc[0]+delta[h][0]*sensors[i]+delta[h][0],
                self.loc[1]+delta[h][1]*sensors[i]+delta[h][1],
                (h+2)%4) or changed
            if changed:
                self.viz.draw_cell(
                    self.loc[0]+delta[h][0]*sensors[i],
                    self.loc[1]+delta[h][1]*sensors[i],
                    self.maze.map
                        [self.loc[0]+delta[h][0]*sensors[i]]
                        [self.loc[1]+delta[h][1]*sensors[i]])
        return changed

    def update_plan(self):
        self.route = self.planner.plan()
        self.route_i = 0
        self.viz.draw_path(self.loc[0], self.loc[1], self.route)

    def move(self, rotation, movement):
        self.heading = (self.heading + rotation + 4) % 4
        self.loc[0] += delta[self.heading][0]*movement
        self.loc[1] += delta[self.heading][1]*movement

    def print_stats(self):
        print "------------------------------------------------------------"
        print "Reached goal! {}".format(self.loc)
        print "Trip No.: ", self.trips
        print "Steps Expected: ", self.steps_expected
        print "Steps Used: ", self.steps
        print "Turns Used: ", self.turns
        print "------------------------------------------------------------"
        print "Total Train Time: ", self.train_time
        print "Total Test Time: ", self.test_time
        raw_input("Press ENTER to continue...")

    def next_move(self, sensors):
        # update training and test times
        if self.practice: self.train_time += 1
        else: self.test_time += 1

        # update turns used
        self.turns += 1

        reset = False
        # update map based on sensors
        # update plan if necessary
        if self.update_map(sensors): # returns true if map changed
            self.update_plan()

        if self.route == None:
            # No route to goal
            raise Exception('No route to goal!')
        elif self.route_i < len(self.route):
            # We are still en route
            pass
        else:
            # We've reached the goal
            if self.steps <= self.steps_expected:
                # Bot is ready for the actual run
                self.reset()
                reset = True
                self.practice = False
                self.flipflop = True
            # Flip flop between the start location and goal until bot is already
            # taking the optimal path from start -> goal
            elif self.flipflop:
                # make it run back to the start location
                self.planner = AStarPlanner(self.maze, self.loc, [(0, 0)])
                self.flipflop = not self.flipflop
            else:
                # make it do one more trip to the goal
                self.planner = AStarPlanner(self.maze, self.loc)
                self.flipflop = not self.flipflop
            self.update_plan()
            self.steps_expected = len(self.route)
            self.steps = 0
            self.turns = 0
            self.trips += 1

        # print 'location: ', self.loc
        # print 'heading: ', self.heading
        # print 'route: ', self.route
        # print 'route_i: ', self.route_i

        if reset:
            rotation = 'Reset'
            movement = 'Reset'
        else:
            # turn right by default when a u-turn is needed
            rotations = [0, 1, -1, -1]
            movements = [1, 1, 0,  1]
            h = (self.route[self.route_i] - self.heading + 4) % 4
            rotation = rotations[h]
            movement = movements[h]
            if movement > 0:
                self.route_i += 1
                # maximize movement. use up to 3 steps
                while self.route_i < len(self.route) and \
                    self.route[self.route_i] == self.route[self.route_i-1] and \
                    movement < 3:
                        self.route_i += 1
                        movement += 1
            self.move(rotation, movement)
            rotation *= 90
            self.steps += movement

        self.viz.draw_bot(self.loc[0], self.loc[1], self.heading)

        if self.route_i == len(self.route): self.print_stats()

        # print 'rotation: ', rotation, ' movement: ', movement
        return rotation, movement

if __name__ == '__main__':
    maze = Maze.from_file(str(sys.argv[1]))
    print maze.map
    planner = AStarPlanner(maze)
    route = planner.plan()
    print "Value Map: "
    print planner.closed
    print "Shortest Route: "
    print route
    print "Steps Required: ", len(route)
    viz = Visualizer(maze.dim)
    viz.draw_maze(maze)
    viz.draw_path(0, 0, route)
    raw_input("Press ENTER to exit...")
