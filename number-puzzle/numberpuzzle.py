import numpy as np
from collections import deque
import os
import time
import argparse
from plot_path import visualize 

###############################################################
#################### helper functions #########################
###############################################################

def to_grid(state,size=3):
    """
    Returns state in grid form with row-wise vector input 
    """
    return state.reshape(size,size)
def to_str(state): 
    """
    Returns state in string form for gridinput 
    """
    return str(state.T.reshape(-1))[1:-1]

def string_to_int(input_str):
    return np.array([int(e) if e.isdigit() else e for e in input_str.split(' ')])

def foldercheck(save_path):
    if not os.path.exists(save_path):
        os.makedirs(save_path)

def logResults(savefolder, moves, nodePaths, nodesInfo):
    foldercheck(savefolder)
    movesfile = open(f"{savefolder}/moves.txt", "w")
    for move in moves:     
        movesfile.write(move +'\n' )
    movesfile.close()

    nodePathfile = open(f"{savefolder}/nodePath.txt", "w")
    for node in nodePaths:  
        nodePathfile.write(to_str(node.state) +'\n' )
    nodePathfile.close()

    Nodesfile = open(f"{savefolder}/Nodes.txt", "w")
    NodesInfo = open(f"{savefolder}/NodesInfo.txt", "w")
    NodesInfo.write('Node_index   Parent_Node_index   Cost' +'\n')   
    for (node, node_idx, parent_idx, cost) in nodesInfo:
        NodesInfo.write( f'{node_idx}   {parent_idx} {cost} \n')   
        Nodesfile.write(to_str(node.state) +'\n' )
    NodesInfo.close()
    Nodesfile.close()

###############################################################
################## puzzle tile object #########################
###############################################################
class Node():
    def __init__(self, state, parent, move, depth, cost): 
        self._state = state
        self._parent = parent
        self._move = move
        self._cost = cost
        self._depth = depth
        
    @property    
    def depth(self): return self._depth
    @depth.setter		
    def depth(self, val): self._depth = val

    @property    
    def state(self): return self._state
    @state.setter		
    def state(self, val): self._state = val
 
    @property    
    def parent(self): return self._parent
    @parent.setter		
    def parent(self, val): self._parent = val

    @property    
    def move(self): return self._move
    @move.setter		
    def move(self, val): self._move = val
	
    @property    
    def cost(self): return self._cost
    @cost.setter		
    def cost(self, val): self._cost = val


    def backtrace(self):        
        moves = []
        nodes = []
        current_node = self
        while(current_node.move):

            moves.append(current_node.move)
            nodes.append(current_node)
            current_node = current_node.parent

        nodes.append(current_node)
        moves.reverse()
        nodes.reverse()
        
        return moves, nodes

###############################################################
################## puzzle solver class ########################
###############################################################

class NumberPuzzleSolver:
    def __init__(self, goal_state, size):
        self.goal_state = goal_state
        self.size = size

    def up(self, M):
        state = M.copy()
        x,y = np.squeeze(np.where(state==0))
        if (x-1)>-1 : # up boundary
            state[x,y] = state[x-1][y]
            state[x-1][y] = 0
        else:
            state = None    

        return state

    def down(self, M):
        state = M.copy()
        x,y = np.squeeze(np.where(state==0))
        if (x+1) < len(M) : # down  boundary
            state[x,y] = state[x+1][y]
            state[x+1][y] = 0
        else:
            state = None
        return state

    def left(self, M):
        state = M.copy()
        x,y = np.squeeze(np.where(state==0))
        if (y-1)>-1 : #left  boundary
            state[x,y] = state[x][y-1]
            state[x][y-1] = 0
        else:
            state = None    
        return state

    def right(self, M):
        state = M.copy()
        x,y = np.squeeze(np.where(state==0))
        if (y+1) < len(M) : # right boundary
            state[x,y] = state[x][y+1]
            state[x][y+1] = 0
        else:
            state = None
        return state

    def grow(self, parent):

        children = []
        for direction in ['up', 'down', 'right', 'left']:            
            if direction == 'up': childstate = self.up(parent.state)
            elif direction == 'down': childstate = self.down(parent.state)
            elif direction == 'left': childstate = self.left(parent.state)
            elif direction == 'right': childstate = self.right(parent.state)
            if childstate is not None:
                childnode = Node( state=childstate, parent=parent, move=direction, depth= parent.depth+1, cost=0)
                children.append(childnode)
        return children

    def isSolvable(self, init_state):
        size=self.size
        ## init_state is a n^2 vector.   
        if size ** 2 != len(init_state):
            print('bad input'); return
        count =0
        for i in range(1, size*size):
            for j in range(i, size*size):
                if (init_state[i] > init_state[j]) and (init_state[j] != 0):
                    count += 1
        # print("count = ", count)

        #for odd size grids
        if (size % 2) == 1: return count % 2 == 0 # even count means solvable

        #for even size grids
        elif (size % 2) == 0: 
            # find position of zero tile
            position = np.where(init_state == 0)[0][0]

            # if 0 tile is in even index
            if position%2 == 0 : return count % 2 == 1 # odd count means solvable 
            # if 0 tile is in odd index
            else: return count % 2 ==0 # even count means solvable    
        else: return False


    def bfs_search(self, init_state):

        goal = to_grid(self.goal_state)

        nodes = deque()
        visited = set()

        init_node = Node(to_grid(init_state), None, None, 0, 0)
        nodes.append(init_node)
        
        curr_idx, parent_idx = 0,0
        nodesInfo =[]
        nodesInfo.append((init_node, curr_idx, parent_idx, 0))
        while(nodes):
            current_node = nodes.popleft()
            visited.add(to_str(current_node.state))
            
            if to_str(current_node.state) == to_str(goal):
                print("Goal Reached!")
                print("Total number of nodes explored:", len(visited))
                moves, nodePaths = current_node.backtrace()
                return moves, nodePaths, nodesInfo

            children = self.grow(current_node)
            parent_idx = curr_idx
            for child in children:
                if to_str(child.state) not in visited:
                    curr_idx+=1
                    nodesInfo.append((child, curr_idx, parent_idx, child.cost))
                    nodes.append(child)
        print("Could not find solution...")
        return None, None, None
  

if __name__ == "__main__":
    Parser = argparse.ArgumentParser()
    Parser.add_argument('--init_state', default="1 4 7 0 2 8 3 5 6", help='initial state of the puzzle')
    Parser.add_argument('--goal_state', default="1 4 7 2 5 8 3 6 0", help='goal state to be reached')
    Parser.add_argument('--savefolder', default= "./results/test", type = str, help='save files here')
    Parser.add_argument('--colwise_order', default= True, type = lambda x: bool(int(x)), help='to tell if input is a matrix flattened column wise')

    args = Parser.parse_args()

    # set input to correct format 
    init_state = string_to_int(args.init_state)
    goal_state = string_to_int(args.goal_state)

    size = int(np.sqrt(len(init_state)))

    # convert to row wise flat vector 
    if args.colwise_order:
        init_state = to_grid(init_state).T.reshape(-1) 
        goal_state = to_grid(goal_state).T.reshape(-1) 

    start_time = time.time()


    solver = NumberPuzzleSolver(goal_state, size)
    print("Checking solvability ..", end='\t') 
    if not solver.isSolvable(init_state):
        print("puzzle is not solvable"); exit()

    print("DoneChecking \n Solving the puzzle....... ", end='\t')

    moves, nodePaths, nodesInfo = solver.bfs_search(init_state)
    if moves is  None or nodePaths is None:
        exit()
    
    print("Solved in %s seconds" % (time.time() - start_time))

    print(" Logging results ....", '\t')        
    logResults(args.savefolder, moves, nodePaths, nodesInfo)

    visualize(args.savefolder)

    print("Done Logging \n  \n")

    print("#"*20)
    print("#"*20)
