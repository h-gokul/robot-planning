import os
import cv2
import shutil
from glob import glob

# import imageio
import numpy as np

def string_to_int(input_str):
    return np.array([int(e) for e in input_str.split(',')])

def foldercheck(path):
    if not os.path.exists(path):
        os.makedirs(path)
    return path

def deleteFolder(path):
    if os.path.exists(path):
        shutil.rmtree(path)


def remove_file(file):
    try:
        os.remove(file)
    except OSError as e:
        print("Error: %s : %s" % (file, e.strerror))

# def createMovie(path):
#     image_folder = path
#     video_name = 'simulation_video.mp4'

#     images = sorted(glob("results/*.png"))
#     frame = cv2.imread(os.path.join(images[0]))
#     height, width, channels = frame.shape
#     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#     video = cv2.VideoWriter(video_name, fourcc, 30, (width,height))
#     imgs = []
#     for i in tqdm(range(len(images))):
#         imgs.append(imageio.imread(images[i]))
#         image = cv2.imread(images[i])
#         if i==0:
#             h, w, _ = image.shape
#         video.write(image)

#     for i in range(20):
#         imgs.append(imageio.imread(images[len(images)-1]))
#     imageio.mimsave('simulation_video.gif', imgs, fps=30)
#     cv2.destroyAllWindows()
#     video.release()


def deg2rad(rot): 
    return 3.14*rot/180

def rad2deg(theta_new):
    theta_new = 180*theta_new/3.14    
    theta_new = theta_new % 360 if theta_new > 0 else (theta_new + 360) % 360
    return theta_new

def velocities(robot, UL, UR):
    r, L = robot.radius, robot.wheelDistance
    ang_vel = (r / L) * (UR - UL)
    lin_vel = 0.5 * r * (UL + UR)
    return lin_vel, ang_vel

def gazebo2map(x, rev=False):
    if rev:
        return (x-50)/10
    return int(10*x +50)  
