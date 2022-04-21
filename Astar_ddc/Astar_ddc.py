import math, time, heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse

import utils
from arena import Graph, Node, Robot



HEIGHT, WIDTH = 100, 100

WHITE = (220, 220, 220)
GREEN = (0, 255, 0)
CYAN = (136, 255, 196)
BLACK = (0, 0, 0)
BLUE = (0, 20, 108)


class Astar_DDC(Graph): 
    def __init__(self, robot, start, goal, clearance):
        # super(Graph, self).__init__()
        Graph.__init__(self, clearance)
        
        self.start, self.goal = start, goal
        self.robot = robot
        # defining set of actions that can be done by the constraint.
        self.actions = [[0, robot.RPM1], [robot.RPM1, 0], [robot.RPM1, robot.RPM1], \
                   [0, robot.RPM2], [robot.RPM2, 0], [robot.RPM2, robot.RPM2], \
                   [robot.RPM1, robot.RPM2], [robot.RPM2, robot.RPM1]]

        self.visited = self.Cspace() 
        self.visualize = False
        self.ax= None
        self.savepath = utils.foldercheck('./results')
        self.counter=0
        
    def reset(self):
        self.visualize=False
        self.visited = self.Cspace()
        self.ax= None
        self.counter=0

    def search(self):
        """
        Description: Defining initial constants - Visited array, Rows, Cols, Target String.
        Input: Starting and ending node for the robot to browse.
        Output: Returns True or False to define if an optimal path can be found or not.
        """
        path= []
        if not self.validityCheck(self.start, self.goal): 
            print("[ERROR] Cannot search for this configuration")
            return False, path

        print("Finding path...")
        priorityQueue = []
        
        heapq.heappush(priorityQueue, (self.start.cost, self.start))

        if self.visualize:
            print("GOAL and START is being plotted...")            
            self.ax.add_patch(patches.Circle( (self.start.i,self.start.j), radius=2.0, linewidth=1, alpha=0.5, edgecolor='r', facecolor='r'))
            self.ax.add_patch(patches.Circle( (self.goal.i,self.goal.j), radius=2.0, linewidth=1, alpha=0.5, edgecolor='g', facecolor='g'))
            plt.savefig(f"{self.savepath}/{self.counter}.png")
            self.counter+=1
            time.sleep(0.1)
            print("Plotting curves .....")  

        
        while len(priorityQueue):
            currentNode = heapq.heappop(priorityQueue)[1]
            currentDistance = currentNode.costToCome
            # check if reached goal and backtrack
            if self.reachedTarget(currentNode.i, currentNode.j):
                print("Found a path!")
                
                path= self.backTrack(currentNode, path)
                return True, path

            # check and mark visited                
            if self.visited[currentNode.i][currentNode.j]: continue
            self.visited[currentNode.i][currentNode.j] = True

            neighbours, valid_actions = self.getNeighbours(currentNode)
            currentNode.neighbours = neighbours
            currentNode.valid_actions = valid_actions

            for neighbourNode, newDistance in neighbours.items():
                
                if self.visited[neighbourNode.i][neighbourNode.j]: continue

                neighbourNode.costToCome = currentDistance + newDistance
                neighbourNode.cost = neighbourNode.costToCome + neighbourNode.costToGo
                neighbourNode.parent = currentNode
                
                heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNode))
                # print('Neighbor node', (neighbourNode.i, neighbourNode.j))

        print("Cannot find a path :(")
        return False, path

    def backTrack(self, child, path):
        """
        Backtracking from the finishing node to the start node.
        Input: Node after reaching target
        """
        while child != None:
            path.append(child)
            # print(child.i, child.j, "Path")
            child = child.parent
        return path

    def getNeighbours(self, currentNode):
        """
        Description: Returns neighbours for the currentNode.
        """
        i, j, theta = currentNode.i, currentNode.j, currentNode.theta
        neighbours = {}
        valid_actions = {}
        for UL, UR in self.actions:
            x, y, newTheta, distance = self.dd_constrained_pose(i, j, theta, UL, UR)
            
            if (self.isInsideArena(x, y)) and (not self.insideObstacle(x, y)):    
                if self.visited[x][y] : continue

                newNode = Node(x, y, self.goal.i, self.goal.j, newTheta)
                neighbours[newNode] = distance
                valid_actions[newNode] = [UL, UR]
                
        return neighbours, valid_actions

    def dd_constrained_pose(self, x, y, theta, UL, UR, path_mode=False):
        # same as in cost.py that was provided
        r, L, dt = self.robot.radius, self.robot.wheelDistance, self.robot.dt
        t = 0
        newx, newy = x,y
        theta_new = utils.deg2rad(theta)
        D = 0
        while t < 1:
            t = t + dt
            oldx, oldy = newx,newy
            d_x = 0.5 * r * (UL + UR) * math.cos(theta_new) * dt
            d_y = 0.5 * r * (UL + UR) * math.sin(theta_new) * dt
            newx += d_x
            newy += d_y
            theta_new += (r / L) * (UR - UL) * dt
            D = D + math.sqrt(math.pow(d_x, 2) + math.pow(d_y, 2))
            if self.visualize:
                if path_mode:
                    self.ax.plot([oldx, newx], [oldy, newy], color='r', linewidth = 2)
                else:
                    self.ax.plot([oldx, newx], [oldy, newy], color='b', alpha=0.5,)

        if self.visualize:
            plt.savefig(f"{self.savepath}/{self.counter}.png")
            self.counter+=1
            time.sleep(0.05)

        theta_new = utils.rad2deg(theta_new)
        return int(newx), int(newy), theta_new, D

    def reachedTarget(self, x, y, thresh=2.0):
        """
        Checks if the currentnode is in target area to terminal the program
        Input: Current Node co-ordinates
        Output: Boolean
        """
        return (x - self.goal.i) ** 2 + (y - self.goal.j) ** 2 - thresh <= 0

    def _setup_world(self):
        fig, self.ax = plt.subplots(figsize = (10, 10))
        plt.tight_layout()
        self.ax.set(xlim=(0, 100), ylim = (0,100))
        self.ax = self.plotObstacles(self.ax)
        
        self.ax.set_aspect("equal")
        self.ax.grid()
        # self.ax.xlim(0,100);self.ax.ylim(0,1)
        self.ax.set_title('A star differential constraints',fontsize=10)
    
    def visualizeSearch(self):

        ret, _ = self.search()
        if not ret: 
            print("[ERROR] Cannot visualize without a correct path")
            return 
        
        self.reset()
        
        #### Perform search to obtain paths ####
        self._setup_world()
        self.visualize = True
        ret, path = self.search()
        path.reverse()
        time.sleep(0.1)
        
        ######## Use the paths to plot the smooth direction curves ######
        self._setup_world()
        for idx in range(len(path)-1):
            node = path[idx]
            action = node.valid_actions[path[idx+1]]
            UL, UR = action[0], action[1]
            self.dd_constrained_pose(node.i, node.j, node.theta, UL, UR, path_mode=True)
            time.sleep(0.05)
        


if __name__ == "__main__":
    Parser = argparse.ArgumentParser()
    Parser.add_argument('--init_state', default="10, 10, 90", help='initial state of the puzzle')
    Parser.add_argument('--goal_state', default="90, 90, 0", help='goal state to be reached')
    Parser.add_argument('--rpms', default="25, 15", help='robot wheel rpms')
    Parser.add_argument('--clr', default="2", help='clearence')
    args = Parser.parse_args()

    # SETUP
    START = utils.string_to_int(args.init_state)
    GOAL = utils.string_to_int(args.goal_state)
    rpm1, rpm2 = utils.string_to_int(args.rpms)
    clearance = int(args.clr)

    print(START, GOAL, rpm1, rpm2, clearance)

    x1,y1,theta_start = START
    x2,y2, _ = GOAL

    start = Node(x1, y1, x2, y2, theta_start)
    start.costToCome = 0
    goal = Node(x2, y2, x2, y2, 0)
    
    robot = Robot(rpm1, rpm2, clearance)

    planner = Astar_DDC(robot, start, goal, clearance)
    planner.visualizeSearch()
        
    utils.createMovie("./results/")
    utils.deleteFolder("./results")