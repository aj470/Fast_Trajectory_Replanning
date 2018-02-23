import heapq as h
import os
import pygame
import math
import numpy 



def a_star(start,goal,grid):
	closedset = set()
	came_from[start] = None
	g_cost = {start:0}
	f_cost = {start:heuristic(start,goal)}
	heap = []
	
	while heap:
		current = heappop(heap)[1]
		if current == goal:
			data = []
			while current in came_from:
				data.append(current)
				current = came_from[current]
			return data
		close_set.add(current)
		for x,y in get_validate_neighbor:
			neighbor = current[0] + x, current[1], y
			temp_gcost = g_cost[current] + heuristic(current,neighbor)
			if 0 <= neighbor[0] < array.shape[0]:
				if 0 <= neighbor[1] < array.shape[1]:
					if array [neighbor[0]][neighbor[1]] == 1:
						continue
				else:
						continue
			else:
				continue
			
			if neighbor in closedset and temp_gcost >= g_cost.get(neighbor,0):
				continue
			if temp_gcost < g_cost.get(neighbor,0) or neighbor not in [x[1] for x in heap]:
				came_from[neighbor] = current
				gcost[neighbor] = temp_gcost
				fscore[neighbor] = temp_gcost + heuristic(neighbor, goal)
				heappush(heap, (fscore[neighbor], neighbor))
	return false

def heuristic(start,goal):
#Manhattan distance
return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

def a_star(start,goal):
	closedset = set()
	came_from = {}
	g_cost = {start:0}
	f_cost = {start:heuristic(start,goal)}
	heap = []
	
	while heap:
		current = heappop(heap)[1]
		if current == goal:
			data = []
			while current in came_from:
				data.append(current)
				current = came_from[current]
			return data
		close_set.add(current)
		for x,y in get_validate_neighbor:
			neighbor = current[0] + x, current[1], y
			temp_gcost = g_cost[current] + heuristic(current,neighbor)
			if 0 <= neighbor[0] < array.shape[0]:
				if 0 <= neighbor[1] < array.shape[1]:
					if array [neighbor[0]][neighbor[1]] == 1:
						continue
				else:
						continue
			else:
				continue
			
			if neighbor in closedset and temp_gcost >= g_cost.get(neighbor,0):
				continue
			if temp_gcost < g_cost.get(neighbor,0) or neighbor not in [x[1] for x in heap]:
				came_from[neighbor] = current
				gcost[neighbor] = temp_gcost
				fscore[neighbor] = temp_gcost + heuristic(neighbor, goal)
				heappush(heap, (fscore[neighbor], neighbor))
	return false

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
"""	
	tree = PriorityQueue() 
	tree.put(start,0)
	cost_this_far[start] = 0
	while not tree.empty():
		current = tree.get()
		if current == goal:
			break;
			
			for next in tree.get_neighbours(current):
				new_cost = cost_so_far[current] + tree.cost(current,next)
				if next not in cost_so_far or new_cost < cost_so_far[next]:
					cost_so_far[next] = new_cost
					priority = new_cost + heuristic(goal,next) 
					tree.put(next,priority)
					came_from[next] = current
"""
