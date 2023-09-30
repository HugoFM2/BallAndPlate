import numpy as np
from numpy import sin,cos,deg2rad

def rotate3d(coords, angle_x,angle_y,angle_z):
    # coords is a 3x1 matrix with x,y,z coordinates
    Rz = np.matrix([
          [+cos(angle_z),  -sin(angle_z),  0],
          [+sin(angle_z),  +cos(angle_z),  0],
          [ 0           ,   0           ,  1]])
    
    Ry = np.matrix([
          [+cos(angle_y),   0           , sin(angle_y)],
          [ 0           ,   1           ,  0],
          [-sin(angle_y),  0            , +cos(angle_y)]])

    
    Rx = np.matrix([
          [1           ,   0           ,  0],
          [0,   +cos(angle_x)           , -sin(angle_x)],
          [0,   +sin(angle_x)           , +cos(angle_x)]])
    
    res = Rz@Ry@Rx@coords
    
 
    return res
    
def translate3d(coords,translated_coords):
    return coords+translated_coords