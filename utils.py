"""
@author: Dhruv Parikh
@file: utils.py
@brief: All Utility functions for the rest of the code
"""
import numpy as np
import matplotlib.pyplot as plt

def readG2O(file):
    vertices = []
    edges = []
    with open(file, 'r') as f:
        for line in f:
            
            if "VERTEX" in line:
                line = line.split()
                # VERTEX_SE2 i x y theta
                vertices.append(line[1:])
            elif "EDGE" in line:
                line = line.split()
                # EDGE_SE2 i j x y theta info(x, y, theta)
                edges.append(line[1:])
    return np.array(vertices, dtype=np.float64), np.array(edges, dtype=np.float64)

def fullCovMatrix():
    pass

def plotPoses(pose1, pose2, dim = 2):
    if dim == 2:
        plt.plot(pose1[:,0], pose1[:,1], ".", label='Optimized')
        plt.plot(pose2[:,0], pose2[:,1], "--", label='Raw')
        plt.show()
    else:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(pose1[:,0], pose1[:,1], pose1[:,2], label='Optimized')
        ax.plot(pose2[:,0], pose2[:,1], pose2[:,2], label='Raw')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.legend()

        # Show the plot
        plt.show()


def construct_info_mat(info_v:list)->np.ndarray:
    """
    Construct a information matrix from a list of its upper triangular entries.
    Only 2D and 3D spaces are supported.
    """
    info_m = None
    if len(info_v) == 6:
        info_m = np.zeros((3, 3), dtype=np.float64)
        count = 0
        for i in range(3):
            for j in range(i, 3):
                info_m[i, j] = info_v[count]
                count += 1
  
    elif len(info_v) == 21:
        info_m = np.zeros((6, 6), dtype=np.float64)
        count = 0
        for i in range(6):
            for j in range(i, 6):
                info_m[i, j] = info_v[count]
                count += 1
    else:
        raise NotImplementedError()
    
    info_m = info_m + info_m.T - np.diag(info_m.diagonal())
    return info_m



