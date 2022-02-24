'''
ECE 172A, Homework 2 Maze Pathfinding
Author: regreer@ucsd.edu
Maze generator adapted from code by Łukasz Nojek
For use by UCSD ECE 172A students only.
'''

import matplotlib.pyplot as plt 
import pickle
import numpy as np
from sklearn import neighbors

def draw_path(final_path_points, other_path_points):
	'''
	final_path_points: the list of points (as tuples or lists) comprising your final maze path. 
	other_path_points: the list of points (as tuples or lists) comprising all other explored maze points. 
	(0,0) is the start, and (49,49) is the goal.
	Note: the maze template must be in the same folder as this script.
	'''
	im = plt.imread('172maze2021.png')
	x_interval = (686-133)/49
	y_interval = (671-122)/49
	plt.imshow(im)
	fig = plt.gcf()
	ax = fig.gca()
	circle_start = plt.Circle((133,800-122), radius=4, color='lime')
	circle_end = plt.Circle((686, 800-671), radius=4, color='red')
	ax.add_patch(circle_start)
	ax.add_patch(circle_end)
	for point in other_path_points:
		if not (point[0]==0 and point[1]==0) and not (point[0]==49 and point[1]==49):
			circle_temp = plt.Circle((133+point[0]*x_interval, 800-(122+point[1]*y_interval)), radius=4, color='blue')
			ax.add_patch(circle_temp)
	for point in final_path_points:
		if not (point[0]==0 and point[1]==0) and not (point[0]==49 and point[1]==49):
			circle_temp = plt.Circle((133+point[0]*x_interval, 800-(122+point[1]*y_interval)), radius=4, color='yellow')
			ax.add_patch(circle_temp)
	plt.show()

### Your Work Below: 
maze = pickle.load( open( "172maze2021.p", "rb" ) )
#print(maze[1,1])
#print(maze.keys()) #[0,0] [0,1] ...
#print(maze.values()) #[t f f f ], [t t f f] ... [N, E, S, W]
# [1,1] center, N = [1,2] E = [2,1] S = [1,0] W = [0,1]
# turn pickle to adjacency list
adj_list = {}
keys_list = list(maze)
value_list = list(maze.values())
#print(len(keys_list),len(value_list))
#print(value_list[0][3])
for i in range(len(keys_list)):
	adjacent = []
	if value_list[i][0] == True:
		adjacent.append(tuple([keys_list[i][0],keys_list[i][1]+1]))
	if value_list[i][1] == True:
		adjacent.append(tuple([keys_list[i][0]+1,keys_list[i][1]]))
	if value_list[i][2] == True:
		adjacent.append(tuple([keys_list[i][0],keys_list[i][1]-1]))
	if value_list[i][3] == True:
		adjacent.append(tuple([keys_list[i][0]-1,keys_list[i][1]]))
	
	adj_list[keys_list[i]] = adjacent

# get the index of current location in range 0 - 2499
def getIndex(location):
	index = 0
	for i in range(len(keys_list)):
		if location == keys_list[i]:
			index = i
	return index

def DFS(adj_list):
	all_path = []
	final_path = []
	#current index
	i = 0
	keys_list = list(adj_list)
	# neightbors
	value_list = list(adj_list.values())
	stack = []
	# visited = 1, not visited = 0
	visited = np.zeros(len(adj_list))
	parents = np.zeros(len(adj_list))
	# append start location
	stack.append(keys_list[i])

	while stack and i != 2499:
		removed = stack.pop()
		i = getIndex(removed)
		all_path.append(removed)
		if(visited[i] == 0):
			visited[i] = 1
		neighbors = value_list[i]
		for j in range(len(neighbors)):
			index = getIndex(neighbors[j])
			if visited[index] == 0:
				stack.append(neighbors[j])
				parents[index] = i
	k = 2499
	parents.astype(np.int64)
	while k != 0:
		final_path.append(keys_list[k])
		k = int(parents[k])

	all_path = []
	return all_path, final_path

all_path, final_path = DFS(adj_list)
draw_path(all_path, final_path)