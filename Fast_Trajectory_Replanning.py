#!/usr/bin/python3

# Assignment 1: Fast Trajectory Re-planning
# CS 440: Intro to Artificial Intelligence
# Due date: 26th February 2018
# authors: Ayush Joshi, Dan Snyder, and Nikhil Mathur

import pygame
from pygame.locals import *
import threading
from random import *
import sys
import argparse
from time import sleep, monotonic


# Initialize Grid Variables
block_width = 8  # Drawing dimensions of block
GridCols = 101  # No of columns
GridRows = 101  # No of rows


def sift(ls, node):
    left = node * 2 + 1
    right = node * 2 + 2
    has_right = right < len(ls)

    min_index = left

    if has_right and ls[right] < ls[left]:
        min_index = right

    if ls[min_index] < ls[node]:
        ls[min_index], ls[node] = ls[node], ls[min_index]
        node = min_index

    return node  # return where the original parent now resides


def heapify(ls):
    if len(ls) < 2:
        return

    current = int((len(ls) - 2) / 2)

    while current >= 0:
        sift(ls, current)
        current -= 1


def heappush(ls, value):
    index = len(ls)
    ls.append(value)
    parent = int((index - 1) / 2)

    while ls[index] < ls[parent]:
        ls[index], ls[parent] = ls[parent], ls[index]
        index = parent
        parent = int((index - 1) / 2)


def heappop(ls):
    if len(ls) == 1:
        return ls.pop()

    item = ls[0]
    ls[0] = ls.pop()

    index = -1
    next_i = 0

    while next_i != index and int(next_i * 2 + 1) < len(ls):
        index = next_i
        next_i = sift(ls, index)
    return item


def heuristic2(start, goal):
    # Euclidean distance
    return (abs(goal.x - start.x)**2 + abs(goal.y - start.y)**2)**0.5


def heuristic(start, goal):
    # Manhattan distance
    return abs(goal.x - start.x) + abs(goal.y - start.y)


def heuristic_adaptive(start, goal):
    if goal.g() != sys.maxsize:
        print(abs(goal.x - start.x) + abs(goal.y), " => ", goal.g() - start.g())
        return goal.g() - start.g()
    return abs(goal.x - start.x) + abs(goal.y - start.y)


class Node:
    VISITED = 0b00000001
    BLOCKED = 0b00000010
    START   = 0b00000100
    GOAL    = 0b00001000
    CLOSED  = 0b00010000
    COLOR   = 0b11100000

    colors = (None, (150, 0, 0), (80, 80, 120))

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self._search = 0
        self._g = sys.maxsize
        self._h = sys.maxsize
        self.flags = 0

    def closed(self):
        return bool(self.flags & Node.CLOSED)

    def close(self):
        self.flags |= Node.CLOSED

    def start(self):
        return bool(self.flags & Node.START)

    def goal(self):
        return bool(self.flags & Node.GOAL)

    def mark_start(self):
        self.flags |= Node.START

    def mark_goal(self):
        self.flags |= Node.GOAL

    def color(self, newc=None):
        if newc is not None:
            self.flags &= ~Node.COLOR
            self.flags |= newc << 5
        else:
            col = (self.flags & Node.COLOR) >> 5
            if col == 7:
                return h_to_color(self.h(), roof=GridCols + GridRows)
            return Node.colors[col]

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

    def h(self, newh=None):
        if newh is not None:
            self._h = newh
        else:
            return self._h

    def f(self):
        return self.g() + self.h()

    def reset(self):
        self.flags &= ~Node.VISITED
        self._g = sys.maxsize
        self._h = sys.maxsize
        self._search = 0
        self.parent = None

    def visit(self):
        self.flags |= Node.VISITED

    def block(self):
        self.flags |= Node.BLOCKED

    def unblock(self):
        self.flags &= ~Node.BLOCKED

    def visited(self):
        return bool(self.flags & Node.VISITED)

    def blocked(self):
        return bool(self.flags & Node.BLOCKED)

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
            if self.f() == other.f():
                return self.g() > other.g()
            return self.f() < other.f()
        raise ValueError("Object must be of type Node not '%s'" % str(type(other)))

    def __repr__(self):
        return "Node<%d, %d>" % (self.x, self.y)

    def __hash__(self):
        return hash(str(self))


class Grid:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.maze = tuple(tuple((Node(x, y) for x in range(cols))) for y in range(rows))

    def node(self, x, y):
        if x < 0 or x >= self.rows:
            return None
        if y < 0 or y >= self.cols:
            return None
        return self.maze[y][x]

    def neighbors(self, node):
        neighbors = (
                self.node(node.x, node.y-1),
                self.node(node.x, node.y+1),
                self.node(node.x-1, node.y),
                self.node(node.x+1, node.y))
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
        if maze_seed is None:
            maze_seed = randrange(0, 10000)
        print("Using maze seed:    %d" % maze_seed)
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
            extended_neighbors = (
                self.grid.node(current.x, current.y - 2),
                self.grid.node(current.x, current.y + 2),
                self.grid.node(current.x - 2, current.y),
                self.grid.node(current.x + 2, current.y)
        )

            possible_bridge = tuple(filter(lambda x: x and not x.blocked(), extended_neighbors))
            # chance we create a bridge between two tunnels
            # this prevents there from being precisely 1 path between any two points
            if len(possible_bridge) > 0 and random() < 0.4:
                node = possible_bridge[0]
                middle = self.grid.node(int((current.x + node.x) / 2), int((current.y + node.y) / 2))
                middle.unblock()

            # get only extended neighbors that exist and are blocked (meaning we can unblock them)
            extended_neighbors = tuple(filter(lambda x: x and x.blocked(), extended_neighbors))

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
    def __init__(self, limit_m, limit_a):
        self.counter = 0
        self.limit_a = limit_a
        self.limit_m = limit_m
        self.total = 0

    def compute_path(self, grid, goal, open_list):
        raise NotImplemented()

    @staticmethod
    def cost(node, next_b):
        if not next_b.walkable():
            return sys.maxsize
        else:
            return 1

    def run(self, grid, start, end):
        self.total = 0
        self.counter = 0
        clock = pygame.time.Clock()
        agent = start
        agent.visit()
        final_path = set()
        start_time = monotonic()

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

            for node in final_path:
                node.color(1)
            # path should be a list of nodes leading from agent to goal
            if len(path) == 0:
                return False

            for node in path:
                node.color(2)

            for node in path:
                if not node.walkable():
                    break
                final_path.add(node)
                agent = node
                agent.color(1)
                # if we can move to it, move to it and visit neighbors
                for neighbor in grid.neighbors(agent):
                    neighbor.visit()

                if self.limit_m:
                    clock.tick(30)

            for node in path:
                node.color(7)

        end_time = monotonic()

        for node in final_path:
            node.color(1)

        print("Algorithm expanded: %d nodes" % self.total)
        print("Iterations:         %d" % self.counter)
        print("Agent visited:      %d nodes" % len(final_path))
        print("Visits/iteration:   %d nodes" % (self.total / self.counter))
        print("Time in seconds:    %0.6f seconds" % (end_time - start_time))


class Optimal(AgentAlgorithm):
    @staticmethod
    def cost(node, next_b):
        if next_b.blocked():
            return sys.maxsize
        else:
            return 1

    def compute_path(self, grid, start, goal):
        start.g(0)
        start.search(self.counter)
        start.h(heuristic(start, goal))
        open_list = [start]
        path = []

        while goal.g() > open_list[0].f():
            current = heappop(open_list)
            self.total += 1
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

                    neighbor.h(heuristic(neighbor, goal))
                    heappush(open_list, neighbor)

        if len(open_list) == 0:
            return ()

        current = goal
        # backtrack from goal to agent
        while current != start:
            path.append(current)
            current = current.parent

        path.reverse()
        return tuple(path)

class AStarAlgorithm(AgentAlgorithm):

    def compute_path(self, grid, start, goal):
        start.h(heuristic(start, goal))
        open_list = [start]
        path = []

        while goal.g() > open_list[0].f():
            self.total += 1
            current = heappop(open_list)
            # print(current, current.g(), current.f())

            current.color(7)
            if self.limit_a:
                sleep(0.005)

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

                    neighbor.h(heuristic(neighbor, goal))
                    heappush(open_list, neighbor)

        #print("A* visited %d nodes" % i)

        if len(open_list) == 0:
            return ()

        current = goal
        # backtrack from goal to agent
        while current != start:
            path.append(current)
            current = current.parent

        path.reverse()
        return tuple(path)


class ReverseAStar(AStarAlgorithm):
    def run(self, grid, start, goal):
        super().run(grid, goal, start)


class AdaptiveAStarAlgorithm(AgentAlgorithm):

    def compute_path(self, grid, start, goal):
        open_list = [start]
        closed = set()
        path = []

        start.h(heuristic(start, goal))

        while goal.g() > open_list[0].f():
            self.total += 1
            current = heappop(open_list)
            closed.add(current)
            # print(current, current.g(), current.f())

            current.color(7)
            if self.limit_a:
                sleep(0.005)

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

                    if neighbor.h() == sys.maxsize:
                        neighbor.h(heuristic(neighbor, goal))
                    heappush(open_list, neighbor)

        for node in closed:
            #if node.h() < goal.g() - node.g():
            #    print("%d => %d" % (node.h(), goal.g() - node.g()))
            node.h(goal.g() - node.g())

        if len(open_list) == 0:
            return ()

        current = goal
        # backtrack from goal to agent
        while current != start:
            path.append(current)
            current = current.parent

        path.reverse()
        return tuple(path)


def h_to_color(value, roof=200):
    alpha = min(value, roof) / roof
    inv = 1 - alpha
    return int(255*alpha), int(255*inv), 0


class PygameHandler:
    """class handles all things Pygame. It has a main loop that both renders the screen and
    processes user input. If the user chooses not to visualize the process then no user input
    should be processed by Pygame. If they want to stop, the use ctrl+C. This thread gets all
    visual information from the global game state variables. If we discover that visuals are
    messed up because of the algorithms modifying data in the middle of rendering, then we
    can add semaphores or timed leases for accessing global data that will have minimal overhead.
    """

    FRAME_RATE = 30  # in frames per second

    def __init__(self):
        super().__init__()
        # For visual
        pygame.init()
        self.GameScreen = pygame.display.set_mode((GridCols * block_width + 200, GridRows * block_width + 34))
        self.GridSurface = pygame.Surface(self.GameScreen.get_size())
        self.myfont = pygame.font.Font("roboto.ttf", 14)
        self.clock = pygame.time.Clock()
        self.hide_unvisited = False
        self.running = False
        self.grid = None
        self.info = None

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
            elif event.type == MOUSEMOTION:
                if not self.grid:
                    continue
                mouse_pos = pygame.mouse.get_pos()
                x, y = int((mouse_pos[0] - 10) / block_width), int((mouse_pos[1] - 10) / block_width)
                node = self.grid.node(x, y)

                if node:
                    info = "X: %d Y: %d\nG: %d\nH: %d\nF: %d\nUpdate Iteration: %d\n" % (node.x, node.y, node.g(), node.h(), node.f(), node.search())
                    info = info.split("\n")
                    self.info = [self.myfont.render(msg, True, (0, 0, 0)) for msg in info]
                else:
                    self.info = None

    def update(self):
        # render surface onto screen and update screen
        #self.GridSurface.fill((255, 255, 255))
        #self.renderer.render(self.GridSurface)
        self.drawMyMaze()
        if self.info:
            y_offset = 100
            for msg in self.info:
                self.GridSurface.blit(msg, (830, y_offset))
                y_offset += 20
        self.GameScreen.blit(self.GridSurface, (0, 0))
        pygame.display.update()

    def drawMyMaze(self):
        if not self.grid:
            return

        self.GridSurface.fill((255, 255, 255))
        for y in range(self.grid.rows):
            for x in range(self.grid.cols):
                node = self.grid.node(x, y)
                color1 = (255, 255, 255)
                color2 = (100, 100, 100)

                if node.blocked() and not self.hide_unvisited:
                    color1 = (0, 0, 0)
                    color2 = (0, 0, 0)
                elif not node.walkable():
                    color1 = (0, 0, 0)
                    color2 = (0, 0, 0)
                elif node.start():
                    color1 = (0, 0, 255)
                    color2 = (100, 100, 100)
                elif node.goal():
                    color1 = (255, 0, 0)
                    color2 = (100, 100, 100)
                elif not node.visited() and self.hide_unvisited:
                    color1 = (40, 40, 40)
                    color2 = (0, 0, 0)
                elif node.color():
                    color1 = node.color()
                # more color options go here

                pygame.draw.rect(self.GridSurface, color1,
                                 (node.x * block_width + 10, node.y * block_width + 10, block_width, block_width), 0)
                pygame.draw.rect(self.GridSurface, color2, (
                    node.x * block_width + 10, node.y * block_width + 10, block_width + 1, block_width + 1), 1)


def get_size(obj, seen=None):
    """Recursively finds size of objects in bytes"""
    size = sys.getsizeof(obj)
    if seen is None:
        seen = set()
    obj_id = id(obj)
    if obj_id in seen:
        return 0
    # Important mark as seen *before* entering recursion to gracefully handle
    # self-referential objects
    seen.add(obj_id)
    if hasattr(obj, '__dict__'):
        for member in obj.__dict__.values():
            size += get_size(member, seen)
    if isinstance(obj, dict):
        size += sum((get_size(v, seen) for v in obj.values()))
        size += sum((get_size(k, seen) for k in obj.keys()))
    elif hasattr(obj, '__iter__') and not isinstance(obj, (str, bytes, bytearray)):
        size += sum((get_size(i, seen) for i in obj))
    return size


class ProcessingThread(threading.Thread):
    def __init__(self, start, end, algorithm, seed_=None, full_sim=False, pyhandler=None, limit_g=False):
        super().__init__()
        self.start_node = start
        self.end_node = end
        self.algorithm = algorithm
        self.seed = seed_
        self.full_sim = full_sim
        self.pyhandler = pyhandler
        self.limit_g = limit_g

    def run(self):
        if threading.current_thread().getName() != "__main__":
            sleep(1)  # give time for pygame to initialize

        runs = 50 if self.full_sim else 1
        print("Entering execution portion")
        maze = None
        start_time = monotonic()

        for i in range(runs):
            seed_ = i if self.full_sim else self.seed
            seed_ = int((2**seed_) - 1) if seed_ is not None else seed_
            builder = MazeBuilder(GridRows, GridCols, seed_, self.limit_g)
            if self.pyhandler:
                self.pyhandler.grid = builder.grid

            maze = builder.build()
            half_h = int(maze.rows / 2)
            start_half = randrange(0, 2)  # randomly pick either top half or bottom half
            end_half = 1 - start_half
            if self.start_node is not None and self.end_node is not None:
                self.start_node = maze.node(self.start_node[0],self.start_node[1])
                self.start_node.unblock()
                self.start_node.visited()
                self.end_node = maze.node(self.end_node[0],self.end_node[1])
                self.end_node.unblock()
                self.end_node.visited()
            while self.start_node is None:
                # pick a starting point in the leftmost quarter of the board
                node = maze.node(randrange(0, int(maze.cols/4)), randrange(start_half*half_h, (start_half+1)*half_h))
                if not node.blocked():
                    self.start_node = node

            while self.end_node is None:
                # pick an ending point in the rightmost quarter of the board
                node = maze.node(randrange(int(3*maze.cols/4), maze.cols), randrange(end_half*half_h, (end_half+1)*half_h))
                if not node.blocked():
                    self.end_node = node

            print(self.start_node, self.end_node)
            self.start_node.mark_start()
            self.end_node.mark_goal()
            self.algorithm.run(maze, self.start_node, self.end_node)
            # reset for optimal path finding
            for row in maze.maze:
                for node in row:
                    node.h(sys.maxsize)
                    node.g(sys.maxsize)
                    node.search(0)

            optimal_alg = Optimal(False, False)
            optimal = optimal_alg.compute_path(maze, self.start_node, self.end_node)
            optimal_len = len(optimal)
            optimal_steps = optimal_alg.total
            print("Optimal route:      %d nodes" % optimal_len)
            print("Optimal expanded:   %d nodes" % optimal_steps)

            self.start_node = None
            self.end_node = None

        end_time = monotonic()
        print("Total time:        ", end_time - start_time, "seconds")
        print("Maze size:         ", get_size(Node(0, 0))*GridRows*GridCols, "bytes")


def main():
    parser = argparse.ArgumentParser(description="CS440 Project 1 -- Fast_Trajectory_Replanning")
    parser.add_argument("algorithm", type=int, choices=[1, 2, 3], help="1. A* forward, 2. A* backward, 3. Adaptive A*")
    parser.add_argument("m", type=int, help="map number between 1-50", nargs='?', default=None)
    parser.add_argument("s_x", type=int, help="Start x", nargs='?', default=None)
    parser.add_argument("s_y", type=int, help="Start y", nargs='?', default=None)
    parser.add_argument("g_x", type=int, help="Goal x", nargs='?', default=None)
    parser.add_argument("g_y", type=int, help="Goal y", nargs='?', default=None)
    parser.add_argument("-r", "--random", help="Fully Random", action='store_true')
    parser.add_argument("-v", "--visual", help="Visualize", action='store_true')
    parser.add_argument("-la", "--limit_algorithm", help="Whether or not to limit algorithm speed for visualization",
                        action='store_true')
    parser.add_argument("-lm", "--limit_movement", help="Whether or not to limit movement speed for visualization",
                        action='store_true')
    parser.add_argument("-lg", "--limit_generation", help="Whether or not to limit generation speed for visualization",
                        action='store_true')
    parser.add_argument("-f", "--full_sim", help="Run the set of 50 simulations", action='store_true')

    args = vars(parser.parse_args())

    # variables to be set by arguments
    visual = args['visual']
    limit_a = args['limit_algorithm']
    limit_g = args['limit_generation']
    limit_m = args['limit_movement']
    full_sim = args['full_sim']
    algorithm = args['algorithm']
    start_coords = args['s_x'], args['s_y']
    goal_coords = args['g_x'], args['g_y']
    run_random = args['random']
    map_number = args['m']

    # ensure a valid set of arguments was passed
    if full_sim and run_random:
        print(parser.format_help())
        print("Only one of -f and -r can be specified at once.")
        quit()
    elif not (full_sim or run_random) and goal_coords[1] is None:
        print(parser.format_help())
        print("Starting and ending coordinates must be fully specified when not running full sim or randomly.")
        quit()

    # populate this array with algorithms corresponding to the argument options
    algorithms = [AStarAlgorithm(limit_m, limit_a), ReverseAStar(limit_m, limit_a), AdaptiveAStarAlgorithm(limit_m, limit_a)]
    algorithm = algorithms[algorithm - 1]
    
    if None in start_coords or None in goal_coords:
        start = None
        goal = None
    else:
        start = start_coords
        goal = goal_coords
        

    maze_builder = None
    
    num_runs = 50 if full_sim else 1
    map_number = 0 if full_sim else map_number

    # set thread name so that processing thread and identify whether or not it is the main thread
    # if it is not the main thread, is sleeps to allow pygame to initialize
    threading.current_thread().setName("__main__")

    if run_random:
        map_number = None
        start = None
        goal = None

    # if we're visualizing, run the processing in another thread, and run the pygame handler here
    if visual:
        pygame_handler = PygameHandler()
        processing_t = ProcessingThread(start, goal, algorithm, map_number, full_sim, pygame_handler, limit_g)
        processing_t.daemon = True    # set as daemon so it dies with the main thread
        processing_t.start()
        pygame_handler.run()
    else:
        # otherwise, we run the processing in the main thread
        processing_t = ProcessingThread(start, goal, algorithm, map_number, full_sim, limit_g=limit_g)
        processing_t.run()


if __name__ == "__main__":
    main()
