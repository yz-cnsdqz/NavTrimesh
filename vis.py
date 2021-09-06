import os, sys, glob
import pdb

import numpy as np
import trimesh
import matplotlib.pyplot as plt
from navigation import navigation



def visualize(mesh, loc_s, loc_t, traj):

    loc_s_vis = trimesh.primitives.Sphere(radius=0.1, center=loc_s)
    loc_s_vis.visual.face_colors = [255,0,0,255]
    loc_t_vis = trimesh.primitives.Sphere(radius=0.1, center=loc_t)
    loc_t_vis.visual.face_colors = [0,0,0,255]
    traj_vis = trimesh.load_path(traj)
    scene = trimesh.Scene([mesh, loc_s_vis, loc_t_vis, traj_vis])
    scene.show()



if __name__=='__main__':
    filename = 'data/navi_maze.obj'
    mesh = trimesh.load(filename, force='mesh')

    '''sample two points on the mesh'''
    pts = trimesh.sample.sample_surface(mesh, 2, face_weight=None)[0]
    loc_s = pts[0]
    loc_t = pts[1]

    '''perform search'''
    traj = navigation(mesh, loc_s, loc_t, loc_mode='on_edge_random')
    if traj is None:
        print('search failed')
    else:
        visualize(mesh, loc_s, loc_t, traj)








