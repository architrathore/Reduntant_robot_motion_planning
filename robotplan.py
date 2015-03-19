import pygame, math, numpy, networkx as nx
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
import fileinput
import pygraph

path_graph = nx.Graph()
coord = []
running = 1
numConfig = 100
numNearNgbr = 10
epsilon = 50
linecolor = 0, 200, 150
robotcolor = 255,255,255
bgcolor = 40, 40, 40
HEIGHT = 1000
WIDTH = 1000
# screen = pygame.display.set_mode((HEIGHT,WIDTH))

#--------------------------------------------------------------------------------------


# def intersect (p1,p2,p3,p4):
# 	par = (p1[0] - p2[0])*(p3[1] - p4[1]) - (p1[1]-p2[1])*(p3[0]-p4[0])
# 	if(par!=0):
# 		Px = ((p1[0]*p2[1] - p1[1]*p2[0])*(p3[0]-p4[0]) - (p1[0]-p2[0])*(p3[0]*p4[1]-p4[0]*p3[1]))/par
# 		Py = ((p1[0]*p2[1] - p1[1]*p2[0])*(p3[1]-p4[1]) - (p1[1]-p2[1])*(p3[0]*p4[1]-p4[0]*p3[1]))/par
# 		if(p1[0] <= Px and Px <= p2[0] and p3[0] <= Px and Px <= p4[0]) :
# 			return True
# 	else :
# 		return False
#--------------------------------------------------------------------------------------

def ccw(p1,p2,p3):
	return (p3[1]-p1[1]) * (p2[0]-p1[0]) > (p2[1]-p1[1]) * (p3[0]-p1[0])

# Return true if line segments AB and CD intersect
def intersect(p1,p2,p3,p4):
	return ccw(p1,p3,p4) != ccw(p2,p3,p4) and ccw(p1,p2,p3) != ccw(p1,p2,p4)

#--------------------------------------------------------------------------------------

def intersect_obst(coord,obst_vertex):
	for i in range(len(coord)-1):
		x1y1 = coord[i]
		x2y2 = coord[i+1]
		for j in range(0,len(obst_vertex),2):
			obst_x1y1 = obst_vertex[j]
			obst_x2y2 = obst_vertex[j+1]
			if(intersect(x1y1,x2y2,obst_x1y1,obst_x2y2)):
				return True

#--------------------------------------------------------------------------------------

def theta_to_coord(angle_conf,numDOF,link_len):
	theta = 0
	xy_vector = []
	xy_vector.append([0,0])
	for i in range(numDOF):
		theta = theta + angle_conf[i]
		tempx = xy_vector[i][0] + link_len[i]*numpy.cos(numpy.radians(theta))
		tempy = xy_vector[i][1] + link_len[i]*numpy.sin(numpy.radians(theta))
		xy_vector.append([tempx,tempy])
	return xy_vector

#--------------------------------------------------------------------------------------

def generate_valid_configs(numDOF,numConfig,obst_vertex,link_len):
	valid_conf = []
	i = 0
	while(i < numConfig):
		angle_conf = numpy.random.rand(numDOF)*360
		coord = theta_to_coord(angle_conf,numDOF,link_len)
		if(intersect_obst(coord,obst_vertex)):
			pass
		else:
			valid_conf.append(angle_conf)
			path_graph.add_node(i)
			i+=1
	return valid_conf

#--------------------------------------------------------------------------------------

def gen_knn_graph(theta_array):
	ngbr = NearestNeighbors(n_neighbors = numNearNgbr, algorithm = 'auto').fit(theta_array)
	return ngbr.kneighbors(theta_array)

#--------------------------------------------------------------------------------------

def check_collision(Qstart, Qfinal):
	# print numpy.linalg.norm(Qstart-Qfinal)
	if(numpy.linalg.norm(Qstart-Qfinal) < epsilon):
		return
	Qmid = (Qstart + Qfinal)/2
	if(intersect_obst(theta_to_coord(Qmid,numDOF,link_len),obst_vertex)):
		return True
	else:
		check_collision(Qstart,Qmid)
		check_collision(Qmid,Qfinal)

def create_path_graph(dist,indices, theta_array):
	for config in range(numConfig):
		Qstart = theta_array[config]
		for near_ngbr in range(1,numNearNgbr):
			Qfinal = theta_array[indices[config, near_ngbr]]
			# print near_ngbr
			if(not check_collision(Qstart,Qfinal)):
				path_graph.add_edge(config,indices[config,near_ngbr])

#--------------------------------------------------------------------------------------

def robot_info(filename):
	robot = open(filename,"r+").read().split("\n")
	numDOF = int(robot[0])
	link_len = map(int, robot[1].split(" "))
	numObst = int(robot[2])
	for i in range(3,numObst*2+3):
		coord.append(map(int, robot[i].split(" ")))
	return numDOF,link_len,coord

#--------------------------------------------------------------------------------------

def goal_info(filename):
	config = open(filename,"r+").read().split("\n")
	angles = []
	for i in config:
		angles.append(map(int,i.split(" ")))
	return angles

#--------------------------------------------------------------------------------------

def draw_obst(obst_vertex):
	for i in range(0,len(obst_vertex),2):
		x1,y1 = obst_vertex[i]
		x2,y2 = obst_vertex[i+1]
		pygame.draw.aaline(screen,linecolor,(WIDTH/2+x1,HEIGHT/2-y1),(WIDTH/2+x2,HEIGHT/2-y2))
		# print "pygame.draw.aaline(screen,linecolor,(",x1,y1,"),(",x2,y2,"))"

#--------------------------------------------------------------------------------------
def draw_robot(angle_conf,link_len):
	assert	len(angle_conf) == len(link_len)
	angle_conf = list(angle_conf)
	x1,y1 = (WIDTH/2,HEIGHT/2)
	for i in range(len(link_len)):
		if(i>0):
			angle_conf[i] += angle_conf[i-1]
		x2 = x1 + math.cos(math.radians(angle_conf[i]))*link_len[i]
		y2 = y1 - math.sin(math.radians(angle_conf[i]))*link_len[i]
		pygame.draw.aaline(screen,robotcolor,(x1,y1),(x2,y2))
		x1,y1 = x2,y2

#--------------------------------------------------------------------------------------
numDOF,link_len,obst_vertex = robot_info("robot.dat")
# draw_obst(obst_vertex)

goal_angles = goal_info("goals.dat")
theta_array = generate_valid_configs(numDOF,numConfig,obst_vertex,link_len)
dist,indices = gen_knn_graph(theta_array)
create_path_graph(dist,indices, theta_array)
print path_graph.edges()
nx.draw(path_graph)
plt.draw()
plt.show()
# while running:
#     event = pygame.event.poll()
#     if event.type == pygame.QUIT:
#         running = 0

#     screen.fill(bgcolor)

#     for i in range(len(theta_array)):
#     	draw_robot(theta_array[i],link_len)
#     draw_obst(obst_vertex)
    
#     pygame.display.flip()
#--------------------------------------------------------------------------------------
