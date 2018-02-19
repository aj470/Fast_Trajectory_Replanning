#!/usr/bin/python3

#Assginment 1: Fast Trajectory Replanning
#CS 440: Intro to Artificial Intelligence
#Due date: 26th Feburary 2018
#authors: Ayush Joshi, Nikhil Mathur, and Dan Snyder

import heapq
import pygame
import os
import math
import time
import signal
import threading
from random import *
from random import random, uniform
from ast import literal_eval as make_tuple

#Convention for the Environment
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

# should these be arrays of rows instead of arrays of columns?
# does it matter?
maze = [[0 for y in range(GridRows)] for x in range(GridCols)]
track_maze = [[0 for y in range(GridRows)] for x in range(GridCols)]
first_parent_x = 0
first_parent_y = 0
last_parent_x = 0
last_parent_y = 0

#main tree
tree = {}

#get the string for the cell location
def get_String(x,y):
    return str(x)+","+str(y)

#get the coordinates for the cell.
def get_coord(cell):
    li = cell.split(",")
    return int(li[0]),int(li[1])

#check if the cell is visited or not.
def is_visited(x,y):
    if x is None and y is None:
        return False
    return track_maze[x][y] == 1

def is_not_visited(x,y):
    if x is None and y is None:
        return False
    return track_maze[x][y] == 0

def is_first(x,y):
    return x == first_parent_x and y == first_parent_y

# get the key for the value from the tree
def get_key(value):
    key_value_dict = {}
    for a,b in tree.items():
        for item in b:
            key_value_dict[str(item)] = str(a)
    return key_value_dict[value]

def valid_coordinates(x,y):
    if x is None and y is None:
        return False
    return 0<= x <GridCols and 0<= y <GridRows

#get the neighbours for a cell.
def get_neighbour(x,y,point):
    if point == 0 and valid_coordinates(x-1,y):
        return x-1,y
    if point == 1 and valid_coordinates(x+1,y):
        return x+1,y
    if point == 2 and valid_coordinates(x,y-1):
        return x,y-1
    if point == 3 and valid_coordinates(x,y+1):
        return x,y+1
    return None,None

#check if the cell has neighbours that are not visited.
def has_validate_neighbour(x,y):
    tf_value = False
    if valid_coordinates(x-1,y) and is_not_visited(x-1,y):
        tf_value = True
    if valid_coordinates(x+1,y) and is_not_visited(x+1,y):
        tf_value = True
    if valid_coordinates(x,y-1) and is_not_visited(x,y-1):
        tf_value = True
    if valid_coordinates(x,y+1) and is_not_visited(x,y+1):
        tf_value = True
    return tf_value

#get the neighbour for a cell
def get_validate_neighbour(x,y):
    if valid_coordinates(x-1,y) and is_not_visited(x-1,y):
        return x-1, y
    if valid_coordinates(x+1,y) and is_not_visited(x+1,y):
        return x+1, y
    if valid_coordinates(x,y-1) and is_not_visited(x,y-1):
        return x, y-1
    if valid_coordinates(x,y+1) and is_not_visited(x,y+1):
        return x, y+1
    return tf_value

def label_maze(x,y):
    if uniform(0,1) < 0.3:
        maze[x][y] = 0
    else:
        maze[x][y] = 1

#build_tree returns last child which was visited
def build_tree(p_x,p_y):
    last_p_x = p_x
    last_p_y = p_y

    while has_validate_neighbour(p_x,p_y):
        nn_x,nn_y = get_neighbour(p_x,p_y,randint(0,3))
        if nn_x is None or nn_y is None or is_visited(nn_x,nn_y):
            while has_validate_neighbour(p_x,p_y):
                nn_x,nn_y = get_neighbour(p_x,p_y,randint(0,3))
                if valid_coordinates(nn_x,nn_y) and not is_visited(nn_x,nn_y):
                    break
        if valid_coordinates(nn_x,nn_y) and not is_visited(nn_x,nn_y):
            tree[get_String(p_x,p_y)] = [get_String(nn_x,nn_y)]
            track_maze[int(nn_x)][int(nn_y)] = 1
            last_p_x = nn_x
            last_p_y = nn_y
            label_maze(nn_x,nn_y)
            p_x,p_y = nn_x,nn_y

    return last_p_x, last_p_y

def back_track(last_x,last_y):
    child_x = last_x
    child_y = last_y
    while not is_first(child_x,child_y):
        parent = get_key(get_String(child_x,child_y))
        parent_x,parent_y = get_coord(parent)
        if has_validate_neighbour(parent_x,parent_y):
            nnn_x,nnn_y = get_validate_neighbour(parent_x,parent_y)
            if nnn_x is not None and nnn_y is not None and not is_visited(nnn_x,nnn_y):
                try:
                    newlist = [item for item in tree[get_String(parent_x,parent_y)]]
                    newlist.append(get_String(int(nnn_x),int(nnn_y)))
                    #update the list
                    tree[get_String(parent_x,parent_y)] = newlist
                    track_maze[int(nnn_x)][int(nnn_y)] = 1
                    label_maze(nnn_x,nnn_y)
                except:
                    tree[get_String(parent_x,parent_y)] = [get_String(int(nnn_x),int(nnn_y))]
                    track_maze[int(nnn_x)][int(nnn_y)] = 1
                    label_maze(nnn_x,nnn_y)
                return get_String(int(nnn_x),int(nnn_y))
        child_x,child_y = parent_x,parent_y
    return None
#generateStartFinish()

# Make Random Grid Visual
def makeGrid():
    i = 0
    gn_x = None
    gn_y = None
    p_x = None
    p_y = None
    while True:
        if gn_x != None and gn_y != None:
            print("first")
            break
        p_x = randint(0,GridCols-1)
        p_y = randint(0,GridRows-1)
        global first_parent_x
        first_parent_x = p_x
        global first_parent_y
        first_parent_y = p_y
        ra = randint(0,3)
        gn_x,gn_y = get_neighbour(p_x,p_y,ra) #get neighbor

    track_maze[p_x][p_y] = 1
    track_maze[gn_x][gn_y] = 1
    label_maze(p_x,p_y)
    label_maze(gn_x,gn_y)
    tree[str(p_x)+","+str(p_y)] = [get_String(gn_x,gn_y)]
    p_x,p_y = gn_x,gn_y

    while True:
        new_x,new_y = build_tree(p_x,p_y)
        print("build returns",new_x,new_y)
        global last_parent_x
        global last_parent_y
        if last_parent_x == new_x and last_parent_y == new_y:
            break

        last_parent_x = new_x
        last_parent_y = new_y

        if new_x == first_parent_x and new_y == first_parent_y:
            break

        if valid_coordinates(int(new_x),int(new_y)):
            new_parent = back_track(new_x,new_y)
            print("Back track found", new_parent)
            if new_parent:
                new_px,new_py = get_coord(new_parent)
                p_x,p_y = new_px,new_py
                if p_x == first_parent_x and p_y == first_parent_y:
                    break
            if new_parent is None:
                break


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

    FRAME_RATE = 30 # in frames per second

    def __init__(self):
        super().__init__()
        ## For visual
        pygame.init()
        self.GameScreen = pygame.display.set_mode((GridCols*blockwidth+200,GridRows*blockwidth+34))
        self.GridSurface = pygame.Surface(self.GameScreen.get_size())
        self.myfont = pygame.font.SysFont("monospace", 15)
        self.clock = pygame.time.Clock()

    def execute(self):
        self.drawGrid()
        self.update()
        self.handleInputs()
        self.clock.tick(self.FRAME_RATE) # delay appropriate time frame

    def shutdown(self):
        pygame.quit()
        os.kill(os.getpid(), signal.SIGINT)

    def handleInputs(self):
        # Get Input
    	for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.stop()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.stop()

    def update(self):
        # render surface onto screen and update screen
        self.GameScreen.blit(self.GridSurface, (0,0))
        pygame.display.update()

    def drawGrid(self):
        # draw grid on surface
        self.GridSurface.fill((255,255,255))
        for x in range(len(track_maze)):
            for y in range(len(track_maze[x])):
                if track_maze[x][y] == 0:
                    pygame.draw.rect(self.GridSurface, (40,40,40), (x*blockwidth+10,y*blockwidth+10,blockwidth,blockwidth), 0)
                    pygame.draw.rect(self.GridSurface, (40,40,40), (x*blockwidth+10,y*blockwidth+10,blockwidth+1,blockwidth+1), 1)
                elif track_maze[x][y] == 1:
                    pygame.draw.rect(self.GridSurface, (255,255,255), (x*blockwidth+10,y*blockwidth+10,blockwidth,blockwidth), 0)
                    pygame.draw.rect(self.GridSurface, (100,100,100), (x*blockwidth+10,y*blockwidth+10,blockwidth+1,blockwidth+1), 1)

    def drawMaze(self):
        """!!!!!! I have not fixed this method yet !!!!!!!"""
        myGridSurface.fill((255,255,255))
        for x in range(len(maze)):
            for y in range(len(maze[x])):
                if maze[x][y] == 0:
                    pygame.draw.rect(myGridSurface, (40,40,40), (x*blockwidth+10,y*blockwidth+10,blockwidth,blockwidth), 0)
                    pygame.draw.rect(myGridSurface, (40,40,40), (x*blockwidth+10,y*blockwidth+10,blockwidth+1,blockwidth+1), 1)
                elif maze[x][y] == 1:
                    pygame.draw.rect(myGridSurface, (255,255,255), (x*blockwidth+10,y*blockwidth+10,blockwidth,blockwidth), 0)
                    pygame.draw.rect(myGridSurface, (100,100,100), (x*blockwidth+10,y*blockwidth+10,blockwidth+1,blockwidth+1), 1)
        myGridSurface = myGridSurface.convert()
        return myGridSurface


def main():
    # toggles whether or not screen gets drawn
    visual = True
    pyThread = DummyThread()

    if visual:
        # define signal handler so pygame can tell main thread to stop
        def pygame_exit_signal_handler(signal, frame):
            quit()
        # set that signal handler to accept SIGUSR1
        signal.signal(signal.SIGINT, pygame_exit_signal_handler)

        pyThread = PygameThread()

    pyThread.start()
    makeGrid()


if __name__ == "__main__":
    main()
"""
start_x,start_y,goal_x,goal_y = generateStartFinish()
final_path = []
closed_list = []
cell_costs = []
priority_list = []
heuristic_list = []
path_cost = 0
elapsed_time = 0
drawmode = 0
nodes_expanded = 0
makeGrid(True)
start_x,start_y,goal_x,goal_y = 0,0,0,0
MySearch = AStarSearch() # Initialize Object

while(running):
	# Get Input
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			running = False
		elif event.type == pygame.KEYDOWN:
			if event.key == pygame.K_ESCAPE:
				running = False
			elif event.key == pygame.K_g:
				track_maze = [['1' for y in range(GridRows)] for x in range(GridCols)]
				areacoordinates = makeGrid()
				GridSurface = drawGrid(GridSurface)
				start_x,start_y,goal_x,goal_y = generateStartFinish()
				final_path = []
				closed_list = []
				priority_list = []
				heuristic_list = []
				cell_costs = []
				path_cost = 0
				elapsed_time = 0
				nodes_expanded = 0
				print("Generated new map")
			elif event.key == pygame.K_e:
				start_x,start_y,goal_x,goal_y = generateStartFinish()
				final_path = []
				closed_list = []
				priority_list = []
				heuristic_list = []
				cell_costs = []
				path_cost = 0
				elapsed_time = 0
				nodes_expanded = 0
				print("Generated new start and finish points")
			elif event.key == pygame.K_s:
				# Save map: get filename
				filename = raw_input("Save map to: ")
				with open(os.path.join("./gen",filename),"w") as mapfile:
					mapfile.write(str((start_x,start_y)))		# Write start
					mapfile.write("\n")
					mapfile.write(str((goal_x,goal_y)))			# Write goal
					mapfile.write("\n")

					for area in areacoordinates:				# Write hard to traverse area centers
						mapfile.write(str((area[0],area[1])))
						mapfile.write("\n")

					for y in range(len(track_maze[x])):				# Write each cell
						for x in range(len(track_maze)):
							mapfile.write(str(track_maze[x][y]))
						mapfile.write("\n")

					mapfile.close()
				print("Map saved!")
			elif event.key == pygame.K_l:
				# Load map: get filename
				filename = raw_input("Load map from: ")
				with open(os.path.join("./gen",filename),"r") as mapfile: #changed to allow using /maps folder
					new_start = make_tuple(mapfile.readline())
					start_x = new_start[0]
					start_y = new_start[1]
					new_goal = make_tuple(mapfile.readline())
					goal_x = new_goal[0]
					goal_y = new_goal[1]

					areacoordinates = []

					for i in range(8):
						new_area = make_tuple(mapfile.readline())
						areacoordinates.append((new_area[0],new_area[1]))

					for y in range(len(track_maze[x])):				# Read each cell
						for x in range(len(track_maze)):
							track_maze[x][y] = mapfile.read(1)
						mapfile.read(1)

					mapfile.close()
				final_path = []
				closed_list = []
				cell_costs = []
				priority_list = []
				heuristic_list = []
				path_cost = 0
				elapsed_time = 0
				nodes_expanded = 0
				GridSurface = drawGrid(GridSurface)
				print("Map loaded!")
			elif event.key == pygame.K_UP:
				if cursor_y-1 >= 0:
					cursor_y -= 1
			elif event.key == pygame.K_LEFT:
				if cursor_x-1 >= 0:
					cursor_x -= 1
			elif event.key == pygame.K_RIGHT:
				if cursor_x+1 < GridCols:
					cursor_x += 1
			elif event.key == pygame.K_DOWN:
				if cursor_y+1 < GridRows:
					cursor_y += 1
			elif event.key == pygame.K_v:
				# draw closed list
				if drawmode == 0:
					drawmode = 1
				else:
					drawmode = 0
			elif event.key == pygame.K_a:		# -------- A* Search --------
				choice = -1
				while int(choice) < 1 or int(choice) > 6:
					choice = raw_input ("Enter (1) for Manhattan distance, (2) for Euclidean distance, (3) for Octile distance, (4) for Chebyshev distance, (5) for Straight-Diagonal Distance, or (6) Best/Minimum of all: ")
				MySearch = AStarSearch()
				start_time = time.time()
				closed_list, cell_costs, final_path, path_cost, priority_list, heuristic_list = MySearch.Search(start_x, start_y, goal_x, goal_y,choice)
				elapsed_time = time.time() - start_time
				nodes_expanded = len(closed_list)
			elif event.key == pygame.K_u:		# -------- Uniform Cost Search --------
				MySearch = UniformCostSearch()
				start_time = time.time()
				closed_list, cell_costs, final_path, path_cost, priority_list, heuristic_list = MySearch.Search(start_x, start_y, goal_x, goal_y,1)
				elapsed_time = time.time() - start_time
				nodes_expanded = len(closed_list)
			elif event.key == pygame.K_w:		# -------- Weighted A* Search --------
				choice = -1 #heuristic choice
				weight = 0 #weight of heuristic

				while (int(choice) < 1 or int(choice) > 6):
					choice = raw_input("Enter (1) for Manhattan distance, (2) for Euclidean Distance, (3) for Octile Distance, (4) for Chebyshev Distance, (5) for Straight-Diagonal Distance, or (6) Best/Minimum of all: ")
				while float(weight) < 1:
					weight = raw_input("Enter the selected weight for Weighted A* - must be >= 1: ")

				MySearch = WeightedAStarSearch(weight)
				start_time = time.time()
				closed_list, cell_costs, final_path, path_cost, priority_list, heuristic_list = MySearch.Search(start_x, start_y, goal_x, goal_y, choice)
				elapsed_time = time.time() - start_time

				nodes_expanded = len(closed_list)
			elif event.key == pygame.K_q:		# -------- Sequential A* Search --------
				MySearch = AStarSearch()


		#print(GridSurface,closed_list,final_path,path_cost,nodes_expanded,drawmode,elapsed_time,cell_costs, priority_list, heuristic_list)
		drawScreen(GridSurface,closed_list,final_path,path_cost,nodes_expanded,drawmode,elapsed_time,cell_costs, priority_list, heuristic_list)

pygame.quit()
"""
