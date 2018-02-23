#!/usr/bin/python3


# Assginment 1: Fast Trajectory Replanning
# CS 440: Intro to Artificial Intelligence
# Due date: 26th Feburary 2018
# authors: Ayush Joshi, Nikhil Mathur, and Dan Snyder

import pygame
from pygame.locals import *
import os
import signal
import threading
from random import *
import sys
import argparse
from functools import total_ordering
from heapq import *

# Convention for the Environment
# Initialize Grid
# 0 is blocked and 1 is unblocked for track_maze
# 0 is unvisited and 1 is visited for track_maze
# gn = get neighbour
# nn = new neighbour
# p = parent

# Initialize Variables

blockwidth = 8  # Drawing dimensions of block
GridCols = 101   # No of columns
GridRows = 101   # No of rows


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

    def h(self):
        global end
        return heuristic(self, end)

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
            return self.x == other.x and self.y == other.y and self.f() == other.f()
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
    def __init__(self, rows, cols, maze_seed=None):
        if maze_seed:
            seed(maze_seed)
        self.rows = rows
        self.cols = cols
        self.grid = Grid(rows, cols)
        self.current = None
        self.clock = pygame.time.Clock()

    def init_node(self, node):
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
        self.current = start_node
        self.current.visit()

        while not finished:
            #self.clock.tick(500)
            next_node = self.random_neighbor(self.current)
            if not next_node and self.current == start_node:
                # we're back at the start with nowhere else to explore
                finished = True
            elif not next_node:
                # reached a dead end -- begin back tracking
                self.current = backtrace.pop()
            elif next_node and self.init_node(next_node):
                # there is an unblocked node to move to -- move to it
                backtrace.append(self.current)
                self.current = next_node
            else:
                # otherwise, there was a valid neighbor, but it is now blocked
                continue

        for row in self.grid.maze:
            # mark all unvisited nodes as blocked
            for node in row:
                if not node.visited():
                    node.block()
                node.reset()
        print("finished")
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
        self.current = start_node
        self.current.unblock()

        while not finished:
            #self.clock.tick(60)
            extended_neighbors = [
                self.grid.node(self.current.x, self.current.y - 2),
                self.grid.node(self.current.x, self.current.y + 2),
                self.grid.node(self.current.x - 2, self.current.y),
                self.grid.node(self.current.x + 2, self.current.y)
            ]
            # get only extended neighbors that exist and are blocked (meaning we can unblock them)
            extended_neighbors = list(filter(lambda x: x and x.blocked(), extended_neighbors))

            if len(extended_neighbors) == 0 and len(backtrace) == 0:
                finished = True
                continue
            elif len(extended_neighbors) == 0:
                # if we have no extended neighbors to move to, backtrack
                self.current = backtrace.pop()
                continue

            node = extended_neighbors[randrange(0, len(extended_neighbors))]
            # we can move to this node
            node.unblock()
            middle = self.grid.node(int((self.current.x+node.x)/2), int((self.current.y+node.y)/2))
            middle.unblock()
            backtrace.append(self.current)
            self.current = node


class LoopingThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.running = False

    def run(self):
        self.running = True
        while self.running:
            self.execute()
        self.shutdown()

    def stop(self):
        self.running = False

    def shutdown(self):
        # code to execute once the thread has stopped
        pass

    def execute(self):
        # code to execute each time the thread loops -- MUST be implemented
        raise NotImplemented()


class DummyThread(LoopingThread):
    def execute(self):
        self.stop()


class PygameThread(LoopingThread):
    """class handles all things Pygame. It has a main loop that both renders the screen and
    processes user input. If the user chooses not to visualize the process then no user input
    should be processed by Pygame. If they want to stop, the use ctrl+C. This thread gets all
    visual information from the global game state variables. If we discover that visuals are
    messed up because of the algorithms modifying data in the middle of rendering, then we
    can add semaphores or timed leases for accessing global data that will have minimal overhead.
    """

    FRAME_RATE = 60 # in frames per second

    def __init__(self):
        super().__init__()
        ## For visual
        pygame.init()
        self.GameScreen = pygame.display.set_mode((GridCols*blockwidth+200,GridRows*blockwidth+34))
        self.GridSurface = pygame.Surface(self.GameScreen.get_size())
        self.myfont = pygame.font.SysFont("monospace", 15)
        self.clock = pygame.time.Clock()
        self.hide_unvisited = True

    def execute(self):
        self.handleInputs()
        self.drawMyMaze()
        #self.drawGrid()
        self.update()
        self.clock.tick(self.FRAME_RATE) # delay appropriate time frame

    def shutdown(self):
        pygame.quit()
        os.kill(os.getpid(), signal.SIGINT)

    def handleInputs(self):
        # Get Input
        for event in pygame.event.get():
            if event.type == QUIT:
                self.stop()
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    self.stop()
                elif event.key == K_m:
                    self.drawMaze()
                    self.update()
                    self.FRAME_RATE = 0
                elif event.key == K_s:
                    self.hide_unvisited = not self.hide_unvisited

    def update(self):
        # render surface onto screen and update screen
        if self.FRAME_RATE == 0:
            return
        self.GameScreen.blit(self.GridSurface, (0,0))
        pygame.display.update()

    def drawMyMaze(self):
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # we should do something better for drawing. This is messy and ugly
        self.GridSurface.fill((255, 255, 255))
        global mazeBuilder
        for y in range(mazeBuilder.grid.rows):
            for x in range(mazeBuilder.grid.cols):
                node = mazeBuilder.grid.node(x, y)
                color1 = (255, 255, 255)
                color2 = (100, 100, 100)

                if not node.walkable():
                    color1 = (0, 0, 0)
                    color2 = (0, 0, 0)
                elif node.blocked() and not self.hide_unvisited:
                    color1 = (0, 0, 0)
                    color2 = (0, 0, 0)
                elif not node.visited() and self.hide_unvisited:
                    color1 = (40, 40, 40)
                    color2 = (40, 40, 40)

                pygame.draw.rect(self.GridSurface, color1, (node.x*blockwidth+10,node.y*blockwidth+10,blockwidth,blockwidth), 0)
                pygame.draw.rect(self.GridSurface, color2, (node.x*blockwidth+10,node.y*blockwidth+10,blockwidth+1,blockwidth+1), 1)
        cur = mazeBuilder.current
        pygame.draw.rect(self.GridSurface, (255, 0, 0), (cur.x*blockwidth+10, cur.y*blockwidth+10, blockwidth, blockwidth), 0)
        pygame.draw.rect(self.GridSurface, (255, 0, 0), (cur.x*blockwidth+10, cur.y*blockwidth+10, blockwidth+1, blockwidth+1), 1)
        global start
        global end
        pygame.draw.rect(self.GridSurface, (0, 0, 255), (start.x*blockwidth+10, start.y*blockwidth+10, blockwidth, blockwidth), 0)
        pygame.draw.rect(self.GridSurface, (0, 0, 255), (start.x*blockwidth+10, start.y*blockwidth+10, blockwidth+1, blockwidth+1), 1)
        pygame.draw.rect(self.GridSurface, (0, 255, 0), (end.x*blockwidth+10, end.y*blockwidth+10, blockwidth, blockwidth), 0)
        pygame.draw.rect(self.GridSurface, (0, 255, 0), (end.x*blockwidth+10, end.y*blockwidth+10, blockwidth+1, blockwidth+1), 1)
        global algorithm
        if algorithm.agent:
            pygame.draw.rect(self.GridSurface, (0, 255, 255), (algorithm.agent.x*blockwidth+10, algorithm.agent.y*blockwidth+10, blockwidth, blockwidth), 0)
            pygame.draw.rect(self.GridSurface, (0, 255, 255), (algorithm.agent.x*blockwidth+10, algorithm.agent.y*blockwidth+10, blockwidth+1, blockwidth+1), 1)


class AgentAlgorithm:
    def __init__(self, grid):
        self.grid = grid
        self.counter = 0
        self.agent = None

    def compute_path(self, goal, open_list):
        raise NotImplemented()

    def cost(self, node, next):
        if not next.walkable():
            return sys.maxsize
        else:
            return 1

    def run(self, start, end):
        self.agent = start
        self.counter = 0
        clock = pygame.time.Clock()

        for neighbor in self.grid.neighbors(self.agent):
            # visit all neighbors
            neighbor.visit()

        while self.agent != end:
            self.counter += 1
            self.agent.g(0)
            self.agent.search(self.counter)
            end.g(sys.maxsize)
            end.search(self.counter)

            path = self.compute_path(start, end)
            # path should be a list of nodes leading from agent to goal
            if len(path) == 0:
                return False

            for node in path:
                clock.tick(60)
                # for each node in the path
                if node.walkable():
                    self.agent = node
                    # if we can move to it, move to it and visit neighbors
                    for neighbor in self.grid.neighbors(self.agent):
                        neighbor.visit()
                else:
                    break

        if self.agent == end:
            print("Got to the end in %d iterations" % self.counter)
        else:
            print("Did not make it")


class RandomAlgorithm(AgentAlgorithm):
    def compute_path(self, start, goal):
        neighbors = self.grid.neighbors(self.agent)
        return [neighbors[randrange(0, len(neighbors))]]


class DFSAlgorithm(AgentAlgorithm):

    def compute_path(self, start, goal):
        for row in self.grid.maze:
            for node in row:
                node.g(sys.maxsize)

        current = self.agent
        current.g(0)
        options = []
        path = []

        while current != goal:
            for neighbor in self.grid.neighbors(current):
                if neighbor.g() > current.g() + 1 and neighbor.walkable():
                    options.append(neighbor)
                    neighbor.g(current.g() + 1)

            current = min(options, key=lambda x: x.g())
            options.remove(current)

        path.append(current)

        current = goal
        # backtrack from goal to agent
        while current != self.agent:
            neighbors = self.grid.neighbors(current)
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

    def compute_path(self, start, goal):
        open_list = [start]
        path = []

        while goal.g() > open_list[0].f():
            current = heappop(open_list)
            for neighbor in self.grid.neighbors(current):
                if neighbor.search() < self.counter:
                    neighbor.g(sys.maxsize)
                    neighbor.search(self.counter)
                if current.g() + self.cost(current, neighbor) < neighbor.g():
                    neighbor.g(current.g() + self.cost(current, neighbor))
                    neighbor.parent = current

                    if neighbor in open_list:
                        open_list.remove(neighbor)
                        heapify(open_list)
                    heappush(open_list, neighbor)

        if len(open_list) == 0:
            return []

        current = goal
        # backtrack from goal to agent
        while current != self.agent:
            path.append(current.parent)
            current = current.parent

        path.reverse()
        return path


# having this as a global is kinda weird and makes for weird code
# I tried to make it not global and couldn't get it quite right so I gave up
# feel free to try if it becomes inconvenient to you
mazeBuilder = MazeBuilder(GridRows, GridCols, maze_seed=40)
algorithm = AStarAlgorithm(mazeBuilder.grid)
start = mazeBuilder.grid.node(3, 0)
end = mazeBuilder.grid.node(97, 100)


class ProcessingThread(LoopingThread):
    def execute(self):
        global mazeBuilder
        global algorithm
        global start
        global end

        maze = mazeBuilder.build()
        algorithm.run(start, end)
        self.stop()


def main():

    parser = argparse.ArgumentParser(description="CS440 Project 1 -- Fast_Trajectory_Replanning")
    parser.add_argument("Algorithm", type= int,choices=[1,2,3], help= "1. A* forward, 2. A* backward, 3. Adaptive A*")
    parser.add_argument("T", type=int, help="Tree number between 0-50")
    parser.add_argument("s_x", type = int, help="Start x")
    parser.add_argument("s_y", type = int, help="Start_y")
    parser.add_argument("g_X", type = int, help="Goal_x")
    parser.add_argument("g_y", type = int, help="Goal_y")
    parser.add_argument("-R", help="Fully Random",action="store_true")

    args = vars(parser.parse_args())

    #if len(args) != 6:
    #   parser.print_help()
    #    sys.exit(1)
    args=parser.parse_args()
    #print(args)
    #num_registers = args["k"]
    ##algorithm = args["algorithm"]
    #filename = args["file"]
    # toggles whether or not screen gets drawn
    visual = True
    pyThread = DummyThread()
    processing = ProcessingThread()

    if visual:
        # define signal handler so pygame can tell main thread to stop
        def pygame_exit_signal_handler(signal, frame):
            quit()
        # set that signal handler to accept SIGUSR1
        signal.signal(signal.SIGINT, pygame_exit_signal_handler)
        pyThread = PygameThread()
        processing.daemon = True

    processing.start()
    pyThread.run()


if __name__ == "__main__":
    main()
