# from matplotlib.pyplot import grid
import matplotlib.pyplot as plt
import pygame
import time
import numpy as np
import heapq
import copy
# Define Graph parameters.
HEIGHT, WIDTH = 250, 400
GRAY = (220, 220, 220)
BLUE = (0, 20, 108)
CYAN = (136, 255, 196)
BLACK = (0, 0, 0)

##########################################################################################
############################### Define Obstacles #########################################
##########################################################################################

def defineObstacles():
    """ Modify this function to define the Obstacles in the map 
    """
    # 200, HEIGHT - 100, 70
    # WIDTH-100, HEIGHT - 65, 40
    # (36, HEIGHT-185), (105, HEIGHT-100), (105-25, HEIGHT-180), (115, 40)
    hexObstacle = Hexagon(200, 100, 70)      
    circObstacle = Circle(WIDTH-100, 185, 40) 
    p1, p2, p3, p4 = (36, 185), (105, 100), (105-25, 180), (115, 210)
    polygObstacle = RandomPolygon(p1, p2, p3, p4)
    return [circObstacle, hexObstacle, polygObstacle]

class Hexagon:
    def __init__(self, x, y, Dx):
        self.type = 'polygon'
        self.cx, self.cy = x, y
        a = Dx/np.sqrt(2) # side length
        DY_2 =  np.sqrt(a**2 - (Dx/2)**2)

        self.p1, self.p4 =(x, y + DY_2), (x, y - DY_2)
        self.p2, self.p3 = (x - Dx/2, y+DY_2/2), (x - Dx/2, y-DY_2/2) 
        self.p5, self.p6 = (x + Dx/2, y-DY_2/2), (x + Dx/2, y+DY_2/2)
        self.points = np.array([self.p1, self.p2, self.p3, self.p4, self.p5, self.p6])
        print(f"Hexagon with corners at {self.points}")

    def isInside(self, x,y):
        # Observed empirically using DESMOS
        m12, m34, m45,  m61 = 0.5,  -0.5, 0.5, -0.5   
        b12, b34, b45,  b61 = 36, 165, -35, 235, 
        b56, b23 = 235, 165        
        
        f1 = (y-m12*x - b12 ) < 0
        f2 = x - b23 > 0  
        f3 = (y-m34*x - b34 ) > 0
        f4 = (y-m45*x - b45 ) > 0
        f5 = x - b56 < 0
        f6 = (y-m61*x - b61 ) <0
        return  f1 and f2 and f3 and f4 and f5 and f6 

class Circle:
    def __init__(self, x, y, radius):
        self.type = 'circle'
        self.x, self.y = x,y
        self.radius = radius
        print(f"Circle at {x, y} with radius {radius}")

    def isInside(self, x,y):
        return  (x - self.x) **2 + (y- self.y)**2 - self.radius**2 < 0 

class RandomPolygon:
    def __init__(self, *args):
        self.type = 'polygon'
        self.points = np.array(args)
        print(f"Polygon with corners at {self.points}")

    def isInside(self, x,y):
        # Observed empirically using DESMOS
        m12, m23, m34, m41 = -1.24, -3.2, 0.85, 0.32
        b12, b23, b34, b41 =  230,439, 112, 173
        f1 = (y - m12* x - b12) > 0   
        f2 = (y - m23* x - b23) < 0 
        fmidleft = (y - (-0.1)* x - 189) < 0

        fmidright = (y - (-0.1)* x - 189) >= 0
        f3 = (y - m34* x - b34) > 0  
        f4 = (y - m41* x - b41) < 0 

        return f1 and f2 and fmidleft or fmidright and f4 and f3
        

##########################################################################################
###############################     Define Node     ######################################
##########################################################################################
class Node:
    """
    Node class : This class is built to store the node information.
    A node is a location in the map, and it has attributes like neighbors, 
    parent nodes, distance taken from start point 
    """
    def __init__(self, x ,y):
        self.x = x
        self.y = y
        self.distanceToReach = float('inf')
        self.neighbours = {}
        self.parent = None # useful to back track
        
    def __lt__(self, other):
        return self.distanceToReach < other.distanceToReach


##########################################################################################
###############################     Define Map class     #################################
##########################################################################################
def flip_yaxis(pts):
    points = copy.deepcopy(pts)
    for i in range(len(points)):    
        points[i][1] = HEIGHT - points[i][1]
    return points  

class Map:
    """
    Inputs: 
    obstacles : List of Objects of various Obstacles 
    gridDisplay:  object for pygam grid display

    Map class : This class defines all methods to generate a graph.
    Note: Modify the graph `defineObstacles`  
    """
    def __init__(self, obstacles, gridDisplay):
        self.obstacles = obstacles
        self.gridDisplay = gridDisplay
        self.generate_map()

    def generate_map(self):
        """
        Map generation method. Note: flip the y-axis before drawing.
        """
        self.gridDisplay.fill(GRAY)
        for obstacle in self.obstacles:
            if obstacle.type == 'circle':
                pygame.draw.circle(self.gridDisplay, BLUE, [obstacle.x, HEIGHT - obstacle.y], obstacle.radius)
            if obstacle.type == 'polygon': ## Hexagon and polygon
                pygame.draw.polygon(self.gridDisplay, BLUE, flip_yaxis(obstacle.points))
    
    def isinObstacle(self, x,y):
        """
        Description: Checks if a point is inside any of the obstacles. 
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """
        states = []
        for obstacle in self.obstacles:
            states.append(obstacle.isInside(x, y)) 
        return sum(states) > 0
    
    def plotObstacles(self):
        """
        Plots a binary image of where the obstacle regions are present.
        by performing Obstacle Check across entire Grid 
        """
        grid = np.array([[0 for j in range(HEIGHT)] for i in range(WIDTH)])
        grid = np.array(grid)    
        print(f"\n\n GRID SHAPE: {grid.shape}")   
        for row in range(WIDTH):
            for col in range(HEIGHT):
                grid[row][col] = int(self.isinObstacle(row, col))*255
        plt.imshow(np.flipud(grid.T))    
        plt.show()