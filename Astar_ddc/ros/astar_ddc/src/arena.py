import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


HEIGHT, WIDTH = 100, 100

WHITE = (220, 220, 220)
GREEN = (0, 255, 0)
CYAN = (136, 255, 196)
BLACK = (0, 0, 0)
BLUE = (0, 20, 108)

class Circle:
    def __init__(self, x, y, radius, clearance=0):
        self.type = 'circle'
        self.x, self.y = x,y
        self.radius = radius
        self.clearance = clearance
        # print(f"Circle at {x, y} with radius {radius}")

    def isInside(self, x,y):     
        return  (x - self.x) **2 + (y- self.y)**2  < (self.radius+self.clearance)**2

    def plot(self, ax):
        full_radius = self.clearance + self.radius
        c_outer = patches.Circle((self.x,self.y), radius=full_radius, linewidth=0.5, alpha=0.5, edgecolor='k', facecolor='k')
        c_inner = patches.Circle((self.x,self.y), radius=self.radius, edgecolor='k', facecolor='k')
        ax.add_patch(c_outer)
        ax.add_patch(c_inner)

class Rectangle:
    def __init__(self, x, y, w, h, clearance=0):
        self.type = 'rectangle'
        self.x, self.y = x,y
        self.w, self.h = w,h
        self.clearance  = clearance
        self.xlim = (x-w/2, x+w/2)
        self.ylim = (y- h/2, y+h/2)

        # print(f"Rectangle origin at {x, y} with width {w} and height {h}")

    def isInside(self, x,y):     
        return (self.xlim[0]-self.clearance  < x < self.xlim[1]+self.clearance ) \
            and (self.ylim[0]-self.clearance  < y < self.ylim[1]+self.clearance )

    def plot(self, ax):
        anchor_inner = (self.xlim[0], self.ylim[0])
        anchor_outer = (self.xlim[0]-self.clearance, self.ylim[0]-self.clearance)

        rect_outer = patches.Rectangle(anchor_outer, self.w+2*self.clearance, self.h+2*self.clearance, linewidth=0.5, alpha=0.5, edgecolor='k', facecolor='k')
        rect_inner = patches.Rectangle(anchor_inner, self.w, self.h, edgecolor='k', facecolor='k')
        ax.add_patch(rect_outer)
        ax.add_patch(rect_inner)


class Graph:
    def __init__(self, clearence):
        self.clearance = clearence
        self.obstacleList = self.createObstacles()

    def createObstacles(self):
        circObstacle1 = Circle(20, 20, 10, self.clearance) 
        circObstacle2 = Circle(20, HEIGHT-20, 10, self.clearance) 
        rectObstacle1 = Rectangle(50, 50, 25, 15, self.clearance) 
        rectObstacle2 = Rectangle(80, 30, 15, 20, self.clearance) 

        obstacleList=[circObstacle1, circObstacle2, rectObstacle1, rectObstacle2]
        return obstacleList

    def insideObstacle(self, x, y):
        """
        Description: Checks if the point (x,y) is Inside an obstacle or not.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """
        state = False
        for obstacle in self.obstacleList:
            state = state or obstacle.isInside(x,y)
        return state

    def Cspace(self):
        grid = np.ones((HEIGHT, WIDTH),np.float32)
        for i in range(HEIGHT):
            for j in range(WIDTH):
                grid[i][j] = 1.0 if self.insideObstacle(i, j) else 0
        return grid

    def isInsideArena(self, x, y):
        """
        Description: Checks if the point (x,y) is outside the areana or not.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """
        return (self.clearance <x< WIDTH-self.clearance and  self.clearance < y < HEIGHT-self.clearance)

    def validityCheck(self, start, end):        
        # Checking is start and end are in obstancle.
        isvalid = True
        if not self.isInsideArena(start.i, start.j):
            isvalid=False
            print("Starting point is outside the arena!")

        if not self.isInsideArena(end.i, end.j):
            isvalid=False
            print("Ending point is outside the arena!")

        if self.insideObstacle(start.i, start.j):
            isvalid=False
            print("Starting point is inside the obstacle!")

        if self.insideObstacle(end.i, end.j):
            isvalid=False
            print("Ending point is inside the obstacle!")
                    
        return isvalid
    ###############################
    #### visualization methods ####
    ###############################
    def plot_Cfree(self):  
        grid = self.Cspace()
        plt.figure(figsize=(5,5))
        plt.axis('off')
        plt.imshow(np.flipud(grid.T))
        return 

    def plotObstacles(self, ax):
        for obstacle in self.obstacleList:
            obstacle.plot(ax)
        return ax            
    
class Node:
    """
    A node is simply a location on a map. 
    class members: 
        - the 2D pose of the node: (i, j, theta), 
        - costs: (costToCome, costToGo) 
        - other nodes: neighbours, parent nodes
        - action set: valid_actions set
    class methods:
        lesser than operator: check if the cost of node is lesser than the other 
    """

    def __init__(self, i, j, goal_i, goal_j, theta):
        self.i, self.j  = i, j
        self.theta = theta
        self.costToCome = 0.0
        self.costToGo = 2.5*(math.sqrt((i - goal_i) ** 2 + (j - goal_j) ** 2))
        self.cost = None
        self.neighbours = {}
        self.valid_actions = {}
        self.parent = None

    def __lt__(self, other):
        return self.cost < other.cost

class Robot():
    """
    Defining the robot's parameters
    """
    def __init__(self, rpm1, rpm2, clearance):
        self.RPM1, self.RPM2 = rpm1, rpm2
        self.clearence =  clearance

        self.radius, self.wheelDistance, self.dt = 0.033*10, 0.354*10, 0.1

