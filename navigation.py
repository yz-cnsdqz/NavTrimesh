'''
Assuming the input is the navigation mesh (triangle mesh!), this script produces a path for navigation.
Note that this algorithm is not deterministic. The neighbour is dependent on the adjacent triangles, but we sample points in each triangle.
Input:
|-- triangle mesh (maybe a file)
|-- starting location
|-- target location
Output:
|-- a plausible path
'''

import os, sys, glob
import argparse
import heapq, copy
import pdb
import numpy as np
import trimesh

sys.setrecursionlimit(10000)

class MinHeap(object):
    def __init__(self):
        self.data = []

    def push(self, node):
        heapq.heappush(self.data, node)

    def pop(self):
        try:
            node = heapq.heappop(self.data)
        except IndexError as e:
            node=None
        return node

    def clear(self):
        self.data.clear()

    def is_empty(self):
        return True if len(self.data)==0 else False

    def deepcopy(self):
        return copy.deepcopy(self)

    def len(self):
        return len(self.data)



class Node(object):
    def __init__(self, loc, tri_id, g_val, h_val):
        self.loc = loc
        self.tri_id = tri_id
        self.g_val = g_val # path length from the start
        self.h_val = h_val # heuristic length to the target
        self.parent=None

    def __lt__(self, other):
        return (self.g_val+self.h_val) < (other.g_val+other.h_val)

    def __eq__(self, other):
        return self.tri_id == other.tri_id

    def set_parent(self, parent):
        self.parent = parent



def sample_in_triangle(triangle):
    '''
    source:
    - https://math.stackexchange.com/questions/18686/uniform-random-point-in-triangle-in-3d
    - https://www.cs.princeton.edu/~funk/tog02.pdf
    '''
    r1 = np.random.rand()
    r2 = np.random.rand()
    a = triangle[0]
    b = triangle[1]
    c = triangle[2]
    return (1-r1**0.5)*a + (r1**0.5*(1-r2))*b + (r2*(r1**0.5))*c



def reconstruct_traj(node_t):
    node = node_t
    traj = []
    while True:
        traj.append(node.loc)
        if node.parent is not None:
            node = node.parent
        else:
            break
    traj.reverse()
    return traj



def select_point_on_shared_edge(tri1,tri2, loc_in):
    ## find the shared edge
    tri1_edge_set = set([tuple(x) for x in tri1])
    tri2_edge_set = set([tuple(x) for x in tri2])
    edge = [np.array(x) for x in tri1_edge_set&tri2_edge_set]
    ## calculate the interested location
    eaxis = edge[1]-edge[0]
    enorm = np.linalg.norm(eaxis)
    etarget = loc_in-edge[0]
    proj = np.inner(etarget, eaxis/enorm)
    proj = min(max(0, proj), enorm)
    loc_out = proj*eaxis/enorm + edge[0]

    return loc_out



def search_astar(mesh, loc_s, tri_id_s, loc_t, tri_id_t, loc_mode='random'):
    '''more structures about the trianglemesh'''
    face_adj = mesh.face_adjacency.tolist()
    openset = MinHeap()
    closedset = MinHeap()
    node_s = Node(loc_s, tri_id_s, g_val=0, h_val=np.linalg.norm(loc_t-loc_s))
    node_t = Node(loc_t, tri_id_t, g_val=1e8, h_val=0)
    openset.push(node_s)
    while not openset.is_empty():
        node_curr = openset.pop()
        closedset.push(node_curr)
        tri_id_curr = node_curr.tri_id

        ## when reaching the target triangle id
        if tri_id_curr == tri_id_t:
            node_t.set_parent(node_curr)
            traj = reconstruct_traj(node_t)
            return traj

        tri_id_adj = [x for x in face_adj if tri_id_curr in list(x)]
        for pair in tri_id_adj:
            tri_id = pair[0] if tri_id_curr != pair[0] else pair[1]
            # create node with calculated values
            tri = mesh.triangles[tri_id]
            if loc_mode == 'random':
                loc = sample_in_triangle(tri)
            elif loc_mode == 'on_edge':
                loc1 = select_point_on_shared_edge(tri,
                        mesh.triangles[tri_id_curr], node_curr.loc)
                loc2 = select_point_on_shared_edge(tri,
                        mesh.triangles[tri_id_curr], node_t.loc)
                loc = (loc1+loc2)/2

            g_val = np.linalg.norm(loc-node_curr.loc)+node_curr.g_val
            h_val = np.linalg.norm(loc-loc_t)
            node_ = Node(loc, tri_id, g_val=g_val, h_val=h_val)

            if node_ not in openset.data and node_ not in closedset.data:
                node_.set_parent(node_curr)
                openset.push(node_)

    return None



def navigation(mesh, loc_s, loc_t, loc_mode='random'):
    '''
    navigation based on the mesh
    - output: the triangle ids, in terms of list
    '''
    loc_closest, dist, tri_id_init = trimesh.proximity.closest_point(mesh, np.stack([loc_s, loc_t]))
    # if any of them is not on the mesh, quit
    if np.sum(dist) > 1e-6:
        print('[ERROR] given locations are not on the mesh')
        sys.exit()
    # if they are at the same triangle, return the two given locations
    if tri_id_init[0] == tri_id_init[1]:
        return [loc_s, loc_t]

    # if they are at different triangles, then employ search algorithm
    traj = search_astar(mesh, loc_s, tri_id_init[0], loc_t, tri_id_init[1], loc_mode)
    return traj



if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--filename', default=None)
    parser.add_argument('--algo', default='astar')
    args = parser.parse_args()

    '''load the mesh'''
    verts = np.stack([[0,0,0], [0,1,0], [1,0,0], [1,1,0]])
    faces = np.stack([[0,1,2], [1,2,3]])
    # mesh = trimesh.load(args.filename)
    mesh = trimesh.Trimesh(vertices=verts, faces=faces)

    '''load the locations'''
    loc_s = np.zeros(3)
    loc_t = np.array([1,1,0])

    '''perform search'''
    traj = navigation(mesh, loc_s, loc_t)
    if traj is None:
        print('search failed')
    else:
        print(traj)









































