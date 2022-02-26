
import matplotlib.pyplot as plt
import pygame
import time
import cv2
from glob import glob
from tqdm import tqdm
import heapq
import os
from playground import *


##########################################################################################
###############################     Search Engine      ###################################
##########################################################################################
class SearchEngine:

    def __init__(self, map, start, end):
        self.map = map
        self.start = start
        self.end = end
        self.visited = {}        
        self.directions = [ [-1, -1],[-1, 0], [-1, 1],
                            [0, -1], [0, 0], [0, 1],
                            [1, -1], [1, 0], [1, 1]]
        if self.map.isinObstacle(self.start.x, self.start.y) and self.map.isinObstacle(self.end.x, self.end.y):
            print("[ERROR] Starting and ending point are inside the obstacle!")
            return
        if self.map.isinObstacle(self.start.x, self.start.y):
            print("[ERROR] Starting point is inside the obstacle!")
            return 
        if self.map.isinObstacle(self.end.x, self.end.y):
            print("[ERROR] Ending point is inside the obstacle!")
            return 
        if (0<=start.x<WIDTH) and (0<=start.y<HEIGHT) and (0<=end.x<WIDTH) and (0<=end.y<HEIGHT):
           print("Valid intial and goal positions")
            
        self.frame_counter = 0

    def getNeighbours(self, currNode):
        """
        Description: Returns neighbours for the currentNode.
        """
        x, y = currNode.x, currNode.y
        neighbours ={}
        for direction in self.directions:
            x_new = x+direction[0]
            y_new = y+direction[1]
            
            if (0<=x_new<WIDTH) and (0<=y_new<HEIGHT) and (not self.map.isinObstacle(x_new, y_new)):
                neighbours[Node(x_new, y_new)] =1

        return neighbours     

    def isPath(self):
        """
        Description: 
        Input: uses starting and ending node for the robot to browse.
        Output: Returns True or False to define if an optimal path can be found or not.
        """
        visited = {}

        priorityQueue = []
        heapq.heappush(priorityQueue, (self.start.distanceToReach, self.start))
        while len(priorityQueue):            
            node = heapq.heappop(priorityQueue)[1]

            if node.x == self.end.x and node.y == self.end.y:
                print("[INFO] Found a path!")
                return True
            
            if tuple([node.x, node.y]) in visited:
                continue
            # mark as visited 
            visited[tuple([node.x, node.y])] = True

            node.neighbours = self.getNeighbours(node)
            
            for neighbourNode, newDistance in node.neighbours.items():

                neighbourNode.distanceToReach = node.distanceToReach + newDistance
                neighbourNode.parent = node
                heapq.heappush(priorityQueue, (neighbourNode.distanceToReach, neighbourNode))

        return False

    def save(self, i, freq=500):
        if i%freq == 0:
            file_name = "./results/image" + str(i).zfill(6) + ".png"
            pygame.image.save(self.map.gridDisplay, file_name)

    def visualizeDijkstra(self):
        """
        Description: Visualization of the algorithm.
        Input: Starting and ending node for the robot to browse.
        Output: A binary grid of points where the path corresponds to path.
        """       
        
        grid = [[0 for _ in range(HEIGHT+1)] for _ in range(WIDTH+1)]
        priorityQueue = []
        heapq.heappush(priorityQueue, (self.start.distanceToReach, self.start))

        end_game = False
        while len(priorityQueue):
            for event in pygame.event.get():  
                if event.type == pygame.QUIT:
                    end_game = True  
            if end_game:
                print(" GAME ENDED before compeletion")
                break

            node = heapq.heappop(priorityQueue)[1]

            if node.x == self.end.x and node.y == self.end.y:
                grid = self.backTrack(node, grid)
                print("Distance Required to reach from start to end is:", node.distanceToReach)
                return True, grid 
            
            if tuple([node.x, node.y]) in self.visited: continue

            self.visited[tuple([node.x, node.y])] = True
            for neighbourNode, newDistance in node.neighbours.items():
                nx, ny = neighbourNode.x, neighbourNode.y


                # draw a big black pixel at start and end
                if ((nx-5 < self.start.x < nx+5) and (ny-5 < self.start.y < ny+5)) or ((nx-5 < self.end.x < nx+5) and (ny-5 < self.end.y < ny+5)):
                    pygame.draw.rect(self.map.gridDisplay, BLACK, [nx, HEIGHT - ny, 2,2])
                    pygame.display.update()         
                else:    
                    # color in cyan elsewhere
                    pygame.draw.rect(self.map.gridDisplay, CYAN, [nx, HEIGHT - ny, 2,2]) 
                    pygame.display.update()
                self.save(self.frame_counter, 2500); self.frame_counter+=1

                neighbourNode.distanceToReach = node.distanceToReach + newDistance
                neighbourNode.parent = node
                heapq.heappush(priorityQueue, (neighbourNode.distanceToReach, neighbourNode))

        return False, grid

    def backTrack(self, child, grid):
        """
        Description: Backtracking from the finishing node to the start node.
        Input: Ending Node
        Output: A animation of the path generated.
        """
        clock = pygame.time.Clock()
        while child != None :
            
            grid[child.x][child.y] = 1
            pygame.draw.rect(self.map.gridDisplay, BLACK,[child.x, HEIGHT - child.y, 2,2])
            pygame.display.update()
            self.save(self.frame_counter, freq=1); self.frame_counter+=1
            clock.tick(500)
            
            child = child.parent

        return grid

####################################################################################################################
############################################### File Utils  ########################################################
####################################################################################################################
def foldercheck(save_path):
    if not os.path.exists(save_path):
        os.makedirs(save_path)
def remove_file(file):
    try:
        os.remove(file)
    except OSError as e:
        print("Error: %s : %s" % (file, e.strerror))




####################################################################################################################
############################################### Main Function ######################################################
####################################################################################################################
if __name__ == "__main__":

    x1 = int(input("Start point in x axis: "))
    y1 = int(input("Start point in y axis: "))

    x2 = int(input("End point in x axis: "))
    y2 = int(input("End point in y axis: "))

    foldercheck("results")

    start = Node(x1,y1)
    start.distanceToReach = 0
    end = Node(x2,y2)

    pygame.init() #Setup Pygame
    gridDisplay = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Dijkstra's Algorithm - Point Robot")
    
    obstacles = defineObstacles()
    map = Map(obstacles, gridDisplay)

    # [UNCOMMENT TO CHECK OBSTACLES]
    # map.plotObstacles()
    # exit()

    searchEngine = SearchEngine(map, start, end)
    if not searchEngine.isPath():
        print(f"[ERROR] Cannot find path :( recheck the start {(x1,y1)} and end locations end {(x2,y2)}")
        exit()
    flag, path_grid = searchEngine.visualizeDijkstra()
    if not flag:
        print("[ERROR] Couldnt Visualize ")
    else:
        print("Generating video from results folder")
        images = sorted(glob("results/*.png"))
        for i in tqdm(range(len(images))):
            image = cv2.imread(images[i])
            if i==0:
                h, w, _ = image.shape
                videowriter = cv2.VideoWriter('video_result.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 30, (w,h))
            remove_file(images[i])
            videowriter.write(image)
        videowriter.release()

    print(" Closing in 5 seconds ")
    remove_file('results')
    time.sleep(5)
    pygame.quit()
    exit()
