#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist

import numpy as np
import heapq
from Astar_ddc import Astar_DDC
from arena import Node, Robot, Graph
from utils import velocities, gazebo2map, save_data, load_data

import sys
from open_loop import open_loop_publisher         
from closed_loop import closed_loop_publisher         
import os
if __name__ == '__main__':
    try:

        START = [gazebo2map(float(sys.argv[1])), gazebo2map(float(sys.argv[2])), gazebo2map(float(sys.argv[3]))]
        GOAL = [gazebo2map(float(sys.argv[4])), gazebo2map(float(sys.argv[5])), 0]
        clearance = int(sys.argv[6])
        rpm1, rpm2 = int(sys.argv[7]), int(sys.argv[8])        
        # START = [10, 10, 90];  GOAL = [90, 90, 0];  rpm1, rpm2 = [10, 15]; clearance = 1;    
        print(START, GOAL, rpm1, rpm2, clearance)

        x1, y1, theta_start = START
        x2,y2, _ = GOAL
        start = Node(x1, y1, x2, y2, theta_start)
        start.costToCome = 0
        goal = Node(x2, y2, x2, y2, 0)
        
        robot = Robot(rpm1, rpm2, clearance)
        planner = Astar_DDC(robot, start, goal, clearance)
        ret, path = planner.search()
        
        if not ret: 
            print("Exiting... nothing to run here") 
            exit()
        rospy.sleep(3.)

        path.reverse()     

        print(len(path))
        # open_loop_publisher(robot, path)
        closed_loop_publisher(robot, path)            

    except rospy.ROSInterruptException:
        pass

