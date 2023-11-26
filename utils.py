"""
@author: Dhruv Parikh
@file: utils.py
@brief: All Utility functions for the rest of the code
"""

def readG2O(file):
    vertices = []
    edges = []
    with open(file, 'r') as f:
        for line in f:
            line = line.split()
            if "VERTEX" in line:
                # VERTEX_SE2 i x y theta
                vertices.append(line[1:])
            elif "EDGE_SE2" in line:
                # EDGE_SE2 i j x y theta info(x, y, theta)
                edges.append(line[1:])
    return vertices, edges

def fullCovMatrix():
    pass

def parseSE2Poses():
    pass

if __name__ == '__main__':
    readG2O(r"data/input_INTEL_g2o.g2o")    

