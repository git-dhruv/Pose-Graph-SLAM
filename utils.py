"""
@author: Dhruv Parikh
@file: utils.py
@brief: All Utility functions for the rest of the code
"""
import numpy as np

def readG2O(file):
    vertices = []
    edges = []
    with open(file, 'r') as f:
        for line in f:
            line = line.split()
            if "VERTEX_SE2" in line:
                # VERTEX_SE2 i x y theta
                vertices.append(line[1:])
            elif "EDGE_SE2" in line:
                # EDGE_SE2 i j x y theta info(x, y, theta)
                edges.append(line[1:])
    return np.array(vertices, dtype=np.float64), np.array(edges, dtype=np.float64)

def fullCovMatrix():
    pass

def parseSE2Poses():
    pass

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


if __name__ == '__main__':
    readG2O(r"data/input_INTEL_g2o.g2o")    

