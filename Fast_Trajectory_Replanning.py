#!/usr/bin/python3


# Assignment 1: Fast Trajectory Re-planning
# CS 440: Intro to Artificial Intelligence
# Due date: 26th February 2018
# authors: Ayush Joshi, Nikhil Mathur, and Dan Snyder

import pygame
from pygame.locals import *
import threading
from random import *
import sys
import argparse
from functools import total_ordering
from heapq import *
from pympler import tracker

# Convention for the Environment
# Initialize Grid
# 0 is blocked and 1 is unblocked for track_maze
# 0 is unvisited and 1 is visited for track_maze
# gn = get neighbour
# nn = new neighbour
# p = parent

# Initialize Variables

block_width = 8  # Drawing dimensions of block
GridCols = 101  # No of columns
GridRows = 101  # No of rows


def heuristic(start, goal):
    # Manhattan distance
    return abs(start.x - goal.x) + abs(start.y - goal.y)


@total_ordering
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self._search = 0
        self._visited = False
        self._blocked = False
        self._g = sys.maxsize
        self._h = sys.maxsize
        self._start = False
        self._end = False
        self._color = None

    def start(self):
        return self._start

    def end(self):
        return self._end

    def mark_start(self):
        self._start = True

    def mark_end(self):
        self._end = True

    def color(self, newc=None):
        if newc is not None:
            self._color = newc
        else:
            return self._color

    def search(self, news=None):
        if news is not None:
            self._search = news
        else:
            return self._search

    def g(self, newg=None):
        if newg is not None:
            self._g = newg
        else:
            return self._g

    def h(self, goal=None):
        if goal is not None:
            self._h = heuristic(self, goal)
        else:
            return self._h

    def f(self):
        return self.g() + self.h()

    def reset(self):
        self._visited = False
        self._g = sys.maxsize

    def visit(self):
        self._visited = True

    def block(self):
        self._blocked = True

    def unblock(self):
        self._blocked = False

    def visited(self):
        return self._visited

    def blocked(self):
        return self._blocked

    def walkable(self):
        # this is to be used for the path finding algorithm to account for fog of war
        return not self.visited() or (self.visited() and not self.blocked())

    def __eq__(self, other):
        if isinstance(other, Node):
            return self.x == other.x and self.y == other.y
        return False

    def __ne__(self, other):
        return not self == other

    def __lt__(self, other):
        if isinstance(other, Node):
            return self.f() < other.f()
        raise ValueError("Object must be of type Node not '%s'" % str(type(other)))

    def __repr__(self):
        return "Node<%d, %d>" % (self.x, self.y)


class Grid:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.maze = [[Node(x, y) for x in range(cols)] for y in range(rows)]

    def node(self, x, y):
        if x < 0 or x >= self.rows:
            return None
        if y < 0 or y >= self.cols:
            return None
        return self.maze[y][x]

    def neighbors(self, node):
        neighbors = [
                self.node(node.x, node.y-1),
                self.node(node.x, node.y+1),
                self.node(node.x-1, node.y),
                self.node(node.x+1, node.y)]
        return tuple(filter(lambda n: n, neighbors))

    def down(self, node):
        return self.node(node.x, node.y+1)

    def up(self, node):
        return self.node(node.x, node.y-1)

    def left(self, node):
        return self.node(node.x-1, node.y)

    def right(self, node):
        return self.node(node.x+1, node.y)


class MazeBuilder:
    def __init__(self, rows, cols, maze_seed=None, limit=False):
        if maze_seed:
            seed(maze_seed)
        self.rows = rows
        self.cols = cols
        self.grid = Grid(rows, cols)
        self.limit = limit

    @staticmethod
    def init_node(node):
        node.visit()
        if random() < 0.3:
            node.block()
            return False
        return True

    def random_neighbor(self, node):
        options = list(filter(lambda x: not x.visited(), self.grid.neighbors(node)))
        if len(options) == 0:
            return None

        return options[randrange(0, len(options))]

    def build(self):
        backtrace = []
        start_x = randrange(0, self.cols)
        start_y = randrange(0, self.rows)
        finished = False
        start_node = self.grid.node(start_x, start_y)
        current = start_node
        current.visit()

        clock = pygame.time.Clock()

        while not finished:
            if self.limit:
              clock.tick(100)
            next_node = self.random_neighbor(current)
            if not next_node and current == start_node:
                # we're back at the start with nowhere else to explore
                finished = True
            elif not next_node:
                # reached a dead end -- begin back tracking
                current = backtrace.pop()
            elif next_node and self.init_node(next_node):
                # there is an unblocked node to move to -- move to it
                backtrace.append(current)
                current = next_node
            else:
                # otherwise, there was a valid neighbor, but it is now blocked
                continue

        for row in self.grid.maze:
            # mark all unvisited nodes as blocked
            for node in row:
                if not node.visited():
                    node.block()
                node.reset()
        print("finished generating")
        return self.grid

    def build2(self):
        for row in self.grid.maze:
            for node in row:
                node.block()

        backtrace = []
        start_x = randrange(0, self.cols)
        start_y = randrange(0, self.rows)
        finished = False
        start_node = self.grid.node(start_x, start_y)
        current = start_node
        current.unblock()

        clock = pygame.time.Clock()

        while not finished:
            if self.limit:
                clock.tick(100)
            extended_neighbors = [
                self.grid.node(current.x, current.y - 2),
                self.grid.node(current.x, current.y + 2),
                self.grid.node(current.x - 2, current.y),
                self.grid.node(current.x + 2, current.y)
            ]

            possible_bridge = list(filter(lambda x: x and not x.blocked(), extended_neighbors))
            # chance we create a bridge between two tunnels
            # this prevents there from being precisely 1 path between any two points
            if len(possible_bridge) > 0 and random() < 0.1:
                node = possible_bridge[0]
                middle = self.grid.node(int((current.x + node.x) / 2), int((current.y + node.y) / 2))
                middle.unblock()

            # get only extended neighbors that exist and are blocked (meaning we can unblock them)
            extended_neighbors = list(filter(lambda x: x and x.blocked(), extended_neighbors))

            if len(extended_neighbors) == 0 and len(backtrace) == 0:
                finished = True
                continue
            elif len(extended_neighbors) == 0:
                # if we have no extended neighbors to move to, backtrack
                current = backtrace.pop()
                continue

            node = extended_neighbors[randrange(0, len(extended_neighbors))]
            # we can move to this node
            node.unblock()
            middle = self.grid.node(int((current.x+node.x)/2), int((current.y+node.y)/2))
            middle.unblock()
            backtrace.append(current)
            current = node
        return self.grid


class AgentAlgorithm:
    def __init__(self, limit):
        self.counter = 0
        self.limit = limit

    def compute_path(self, grid, goal, open_list):
        raise NotImplemented()

    @staticmethod
    def cost(node, next_b):
        if next_b.blocked():
            return sys.maxsize
        else:
            return 1

    def run(self, grid, start, end):
        self.counter = 0
        clock = pygame.time.Clock()
        agent = start
        agent.visit()

        for neighbor in grid.neighbors(agent):
            # visit all neighbors
            neighbor.visit()

        while agent != end:
            self.counter += 1
            agent.g(0)
            agent.search(self.counter)
            end.g(sys.maxsize)
            end.search(self.counter)

            # path = self.compute_path2(self.grid, start, end, [start], self.counter)
            path = self.compute_path(grid, agent, end)
            # path should be a list of nodes leading from agent to goal
            if len(path) == 0:
                return False

            for node in path:
                if self.limit:
                    clock.tick(60)
                # for each node in the path
                if node.walkable():
                    agent = node
                    agent.color((150, 0, 0))
                    # if we can move to it, move to it and visit neighbors
                    for neighbor in grid.neighbors(agent):
                        neighbor.visit()
                else:
                    break

        if agent == end:
            print("Got to the goal in %d iteration%s" % (self.counter, 's' if self.counter > 1 else ''))
        else:
            print("Did not make it")

    def compute_path2(self, grid, start, goal, open_list, counter):
        reached = False
        current = open_list[0]
        neighbors = grid.neighbors(current)

        while goal.g() > current.f():
            heappop(open_list)

            if len(neighbors) == 0:
                return []

            for node in neighbors:
                if node.search() < counter:
                    node.search(counter)
                    node.g(sys.maxsize)

                if node.blocked():
                    continue

                if node.g() > current.g() + 1:
                    node.g(start.g() + 1)
                    node.parent = current
                    if node == goal:
                        goal = node
                        heappush(open_list, node)
                        reached = True
                        break
                    heappush(open_list, node)
            if reached:
                break

            if len(open_list) != 0:
                current = open_list[0]
                neighbors = grid.neighbors(current)
            else:
                break

        path = []
        current = goal
        # backtrack from goal to agent
        while current != start:
            path.append(current)
            current = current.parent

        path.reverse()
        return path


class RandomAlgorithm(AgentAlgorithm):
    def compute_path(self, grid, start, goal):
        neighbors = start.neighbors(start)
        return [neighbors[randrange(0, len(neighbors))]]


class DFSAlgorithm(AgentAlgorithm):

    def compute_path(self, grid, start, goal):
        for row in grid.maze:
            for node in row:
                node.g(sys.maxsize)

        current = start
        current.g(0)
        options = []
        path = []

        while current != goal:
            for neighbor in grid.neighbors(current):
                if neighbor.g() > current.g() + 1 and neighbor.walkable():
                    options.append(neighbor)
                    neighbor.g(current.g() + 1)

            current = min(options, key=lambda x: x.g())
            options.remove(current)

        path.append(current)

        current = goal
        # backtrack from goal to agent
        while current != start:
            neighbors = grid.neighbors(current)
            options = sorted(neighbors, key=lambda x: x.g())
            if len(options) == 0:
                # no path can be found
                return []
            if options[0].g() == sys.maxsize:
                # no path was traced all the way to here
                return []

            path.append(options[0])
            current = options[0]

        path.reverse()
        return path


class AStarAlgorithm(AgentAlgorithm):

    def compute_path(self, grid, start, goal):
        start.h(goal)
        open_list = [start]
        path = []

        i = 0
        while goal.g() > open_list[0].f():
            # i += 1
            # print(i)
            current = heappop(open_list)
            current.color((0, min(255, current.f()), 0))
            print(current.f())
            for neighbor in grid.neighbors(current):
                if neighbor.search() < self.counter:
                    neighbor.g(sys.maxsize)
                    neighbor.search(self.counter)
                if current.g() + self.cost(current, neighbor) < neighbor.g():
                    neighbor.g(current.g() + self.cost(current, neighbor))
                    neighbor.parent = current

                    if neighbor in open_list:
                        open_list.remove(neighbor)
                        heapify(open_list)

                    neighbor.h(goal)
                    heappush(open_list, neighbor)

        if len(open_list) == 0:
            return []

        current = goal
        # backtrack from goal to agent
        while current != start:
            path.append(current)
            current = current.parent

        path.reverse()
        return path

# This code is currently unused -- probably won't use it.
class Renderable:
    def __init__(self):
        self._alive = True

    def is_alive(self):
        return self._alive

    def kill(self):
        self._alive = False

    def render(self, screen):
        raise NotImplemented()


class Renderer:
    def __init__(self):
        self._objs = set()

    def add(self, obj):
        if not isinstance(obj, Renderable):
            raise ValueError()

        self._objs.add(obj)

    def render(self, screen):
        dead = set()
        for obj in self._objs:
            if not obj.is_alive():
                dead.add(obj)

        self._objs = self._objs.difference(dead)


class PygameHandler:
    """class handles all things Pygame. It has a main loop that both renders the screen and
    processes user input. If the user chooses not to visualize the process then no user input
    should be processed by Pygame. If they want to stop, the use ctrl+C. This thread gets all
    visual information from the global game state variables. If we discover that visuals are
    messed up because of the algorithms modifying data in the middle of rendering, then we
    can add semaphores or timed leases for accessing global data that will have minimal overhead.
    """

    FRAME_RATE = 30  # in frames per second

    def __init__(self, grid):
        super().__init__()
        # For visual
        pygame.init()
        self.GameScreen = pygame.display.set_mode((GridCols * block_width + 200, GridRows * block_width + 34))
        self.GridSurface = pygame.Surface(self.GameScreen.get_size())
        self.myfont = pygame.font.SysFont("monospace", 15)
        self.clock = pygame.time.Clock()
        self.hide_unvisited = False
        self.renderer = Renderer()
        self.running = False
        self.grid = grid

    def run(self):
        self.running = True
        while self.running:
            self.handleInputs()
            self.update()
            self.clock.tick(self.FRAME_RATE)  # delay appropriate time frame

        pygame.quit()
        quit()

    def stop(self):
        self.running = False

    def handleInputs(self):
        # Get Input
        for event in pygame.event.get():
            if event.type == QUIT:
                self.stop()
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    self.stop()
                elif event.key == K_s:
                    self.hide_unvisited = not self.hide_unvisited

    def update(self):
        # render surface onto screen and update screen
        #self.GridSurface.fill((255, 255, 255))
        #self.renderer.render(self.GridSurface)
        self.drawMyMaze()
        self.GameScreen.blit(self.GridSurface, (0, 0))
        pygame.display.update()

    def drawMyMaze(self):
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # we should do something better for drawing. This is messy and ugly
        self.GridSurface.fill((255, 255, 255))
        for y in range(self.grid.rows):
            for x in range(self.grid.cols):
                node = self.grid.node(x, y)
                color1 = (255, 255, 255)
                color2 = (100, 100, 100)

                if node.color():
                    color1 = node.color()
                if node.start():
                    color1 = (0, 0, 255)
                    color2 = (100, 100, 100)
                elif node.end():
                    color1 = (70, 255, 70)
                    color2 = (100, 100, 100)
                elif not node.walkable():
                    color1 = (0, 0, 0)
                    color2 = (0, 0, 0)
                elif node.blocked() and not self.hide_unvisited:
                    color1 = (0, 0, 0)
                    color2 = (0, 0, 0)
                elif not node.visited() and self.hide_unvisited:
                    color1 = (40, 40, 40)
                    color2 = (40, 40, 40)
                # more color options go here

                pygame.draw.rect(self.GridSurface, color1,
                                 (node.x * block_width + 10, node.y * block_width + 10, block_width, block_width), 0)
                pygame.draw.rect(self.GridSurface, color2, (
                    node.x * block_width + 10, node.y * block_width + 10, block_width + 1, block_width + 1), 1)


class ProcessingThread(threading.Thread):
    def __init__(self, builder, start, end, algorithm):
        super().__init__()
        self.builder = builder
        self.start_node = start
        self.end_node = end
        self.algorithm = algorithm

    def run(self):
        print("Entering")
        maze = self.builder.build2()

        if self.start_node is None:
            for y in range(maze.cols):
                for x in range(maze.rows):
                    if not maze.node(x, y).blocked() and random() < 0.1:
                        self.start_node = maze.node(x, y)
                    break
                if self.start is not None:
                    break

        if self.end_node is None:
            for y in reversed(range(maze.cols)):
                for x in reversed(range(maze.rows)):
                    if not maze.node(x, y).blocked() and random() < 0.1:
                        self.end_node = maze.node(x, y)
                    break
                if self.end_node is not None:
                    break

        print(self.start_node, self.end_node)

        self.algorithm.run(maze, self.start_node, self.end_node)


"""
For running the program:
    Stick with Fast_Trajectory_Replanning.py -la -v 1 4 3 0 97 100
    
    The -la tells the code to limit the speed of the algorithm. You can also use -lg to limit the speed of the
    map generation. -v tells the code to visualize it with Pygame. The 1 is algorithm selection, 4 is the map number
    (and for map number, I just multiply by 10 to get a seed) and the remaining are starting and ending coordinates. 
"""
def main():
    parser = argparse.ArgumentParser(description="CS440 Project 1 -- Fast_Trajectory_Replanning")
    parser.add_argument("Algorithm", type=int, choices=[1, 2, 3], help="1. A* forward, 2. A* backward, 3. Adaptive A*")
    parser.add_argument("T", type=int, help="Tree number between 0-50")
    parser.add_argument("s_x", type=int, help="Start x")
    parser.add_argument("s_y", type=int, help="Start_y")
    parser.add_argument("g_x", type=int, help="Goal_x")
    parser.add_argument("g_y", type=int, help="Goal_y")
    parser.add_argument("-r", "--random", help="Fully Random", action='store_true')
    parser.add_argument("-v", "--visual", help="Visualize", action='store_true')
    parser.add_argument("-la", "--limit_algorithm", help="Whether or not to limit algorithm speed for visualization",
                        action='store_true')
    parser.add_argument("-lg", "--limit_generation", help="Whether or not to limit generation speed for visualization",
                        action='store_true')

    args = vars(parser.parse_args())

    # variables to be set by arguments
    visual = False
    maze_builder = None
    start = None
    goal = None

    visual = args['visual']
    limit_a = args['limit_algorithm']
    limit_g = args['limit_generation']

    tr = tracker.SummaryTracker()

    algorithms = [AStarAlgorithm(limit_a)]   # populate this array with algorithms corresponding to the argument options
    algorithm = algorithms[args['Algorithm'] - 1]

    if args['random']:
        maze_builder = MazeBuilder(GridRows, GridCols, limit=limit_g)
    else:
        maze_builder = MazeBuilder(GridRows, GridCols, maze_seed=args["T"]*10, limit=limit_g)
        start = maze_builder.grid.node(args['s_x'], args['s_y'])
        goal = maze_builder.grid.node(args['g_x'], args['g_y'])
        start.mark_start()
        goal.mark_end()

    # if we're visualizing, run the processing in another thread, and run the pygame handler here
    if visual:
        processing_t = ProcessingThread(maze_builder, start, goal, algorithm)
        processing_t.daemon = True    # set as daemon so it dies with the main thread
        print(processing_t)
        processing_t.start()
        pygame_handler = PygameHandler(maze_builder.grid)
        pygame_handler.run()
    else:
        # otherwise, we run the processing in the main thread
        processing_t = ProcessingThread(maze_builder, start, goal, algorithm)
        processing_t.run()

    tr.print_diff()


if __name__ == "__main__":
    main()
