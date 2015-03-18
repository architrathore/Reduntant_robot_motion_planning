import pygame, math, numpy
import fileinput

coord = []
running = 1
numConfig = 1000
linecolor = 0, 200, 150
robotcolor = 255,255,255
bgcolor = 40, 40, 40
HEIGHT = 1000
WIDTH = 1000
screen = pygame.display.set_mode((HEIGHT,WIDTH))

def ccw(p1,p2,p3):
	return (p3[1]-p1[1]) * (p2[0]-p1[0]) > (p2[1]-p1[1]) * (p3[0]-p1[0])

# Return true if line segments AB and CD intersect
def intersect(p1,p2,p3,p4):
	return ccw(p1,p3,p4) != ccw(p2,p3,p4) and ccw(p1,p2,p3) != ccw(p1,p2,p4)

def theta_to_coordinate(theta_array,numDOF,link_len):
	assert theta_array.shape[1] == numDOF
	coord_matrix = numpy.zeros((theta_array.shape[0],2*(numDOF+1)))
	coord_matrix[:,0] = 500
	coord_matrix[:,1] = 500
	for i in range(1,numDOF):
		theta_array[:,i] += theta_array[:,i-1]
	for i in range(1,numDOF+1):
		coord_matrix[:,2*i] = coord_matrix[:,2*i-2] + numpy.cos(numpy.radians(theta_array[:,i-1]))*link_len[i-1]
		coord_matrix[:,2*i+1] = coord_matrix[:,2*i-1] - numpy.sin(numpy.radians(theta_array[:,i-1]))*link_len[i-1]
		# print x2.shape
	return coord_matrix

def generate_configs(numDOF,numConfig):
	return numpy.random.rand(numConfig,numDOF)*360


def valid_conf(robot_arm_coord, obst_vertex, numDOF, numObst):
	intersect_idx = [False]*robot_arm_coord.shape[0]
	flag = 0
	for h in range(robot_arm_coord.shape[0]):
		flag = 0
		for i in range(0,2*numDOF,2):
			robo_x1y1 = robot_arm_coord[h,i],robot_arm_coord[h,i+1]
			robo_x2y2 = robot_arm_coord[h,i+2],robot_arm_coord[h,i+3]
			for j in (0,2*numObst-2,2):
				obst_x1y1 = obst_vertex[j]
				obst_x2y2 = obst_vertex[j+1]
				print j
				if(intersect(robo_x1y1,robo_x2y2,obst_x1y1,obst_x2y2)):
					flag = 1
					break
			if(flag==1):
				break
		if(flag==0):
			intersect_idx[h]=True
	return intersect_idx

def robot_info(filename):
	robot = open(filename,"r+").read().split("\n")
	numDOF = int(robot[0])
	link_len = map(int, robot[1].split(" "))
	numObst = int(robot[2])
	for i in range(3,numObst*2+3):
		coord.append(map(int, robot[i].split(" ")))
	return numDOF,link_len,coord

def goal_info(filename):
	config = open(filename,"r+").read().split("\n")
	angles = []
	for i in config:
		angles.append(map(int,i.split(" ")))
	return angles


def draw_obst(obst_vertex):
	# print obst_vertex
	for i in range(0,len(obst_vertex),2):
		x1,y1 = obst_vertex[i]
		x2,y2 = obst_vertex[i+1]
		pygame.draw.aaline(screen,linecolor,(WIDTH/2+x1,HEIGHT/2-y1),(WIDTH/2+x2,HEIGHT/2-y2))
		# print "pygame.draw.aaline(screen,linecolor,(",x1,y1,"),(",x2,y2,"))"

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

numDOF,link_len,obst_vertex = robot_info("robot.dat")
draw_obst(obst_vertex)

goal_angles = goal_info("goals.dat")
theta_array = generate_configs(numDOF,numConfig)
robot_arm_coord= numpy.array(theta_to_coordinate(theta_array,numDOF,link_len))
intersect_idx = valid_conf(robot_arm_coord, numpy.array(obst_vertex), numDOF, len(obst_vertex)/2)
print intersect_idx
count = 0
for i in range(len(intersect_idx)):
	if(intersect_idx[i]):
		count+=1
print count
while running:
    event = pygame.event.poll()
    if event.type == pygame.QUIT:
        running = 0

    screen.fill(bgcolor)
    # pygame.draw.line(screen, linecolor, (0, y), (WIDTH - 1,y))
    # pygame.draw.aaline(screen, (0, 0, 255), (639, 0), (0, 479))
    # draw_obst(obst_vertex)
    for i in range(robot_arm_coord.shape[0]):
    	if(intersect_idx[i]== False):
    		draw_robot(theta_array[i],link_len)
    draw_obst(obst_vertex)

    
    pygame.display.flip()