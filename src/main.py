#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
main.py: 
"""

import sys
import logging
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from tqdm import trange
import gtsam
from gtbook import driving
import gtsam.utils.plot as gtsam_plot
import helpers
import tqdm
from scipy.spatial.transform import Rotation
import teaserpp_python
import open3d as o3d

# Module constants
# CONSTANT_1 = "value"

# Module "global" variables
# global_var = None

class slam:
    """
    
    """
    
    def __init__(self, lidarpath):
        """

        """
        scans_fnames = []
        for file in sorted(os.listdir(lidarpath)):
            scans_fnames.append(os.path.join(lidarpath, file))
        self.clouds = helpers.read_ply(*scans_fnames)

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()


        solver_params = teaserpp_python   .RobustRegistrationSolver.Params()
        solver_params.cbar2 = 1
        solver_params.noise_bound = 0.01
        solver_params.estimate_scaling = True
        solver_params.rotation_estimation_algorithm = (
            teaserpp_python.RobustRegistrationSolver   .ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
        )
        solver_params.rotation_gnc_factor = 1.4
        solver_params.rotation_max_iterations = 100
        solver_params.rotation_cost_threshold = 1e-12
        self.solver_params = solver_params



    def addEdge(self, i1, i2):
        ICP_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))
        ip1_T_i = self.icp(self.clouds[i1], self.clouds[i2], self.prevPose)
        i_T_ip1 = ip1_T_i.inverse()
        self.graph.add(gtsam.BetweenFactorPose3(i1, i2, i_T_ip1, ICP_NOISE))
        self.initial_estimate.insert(i2, ip1_T_i)
        self.prevPose = ip1_T_i


    def runISAM(self):
        T = self.readGT()
        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(0.1)
        isam = gtsam.ISAM2(params)

        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))
        ICP_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))
        initial_pose = gtsam.Pose3(gtsam.Rot3(0.9982740, -0.0572837,  0.0129474, 0.0575611,  0.9980955, -0.0221840, -0.0116519,  0.0228910,  0.9996701),
                                gtsam.Point3(-263.9464864482589, 2467.3015467381383, -19.374652610889633))


        self.graph.add(gtsam.PriorFactorPose3(0, initial_pose, prior_noise))
        self.initial_estimate.insert(0, initial_pose)
        self.prevPose = gtsam.Pose3()


        for i in tqdm.tqdm(range(len(self.clouds)-1  ) ):
            self.addEdge(i, i+1)
            isam.update(self.graph, self.initial_estimate)
            self.graph.resize(0)
            self.initial_estimate.clear()
            result = isam.calculateEstimate()
            # self.pltlibViz(i, result)

            if i==50:
                result = isam.calculateEstimate()
                rotError = 0; transError = 0
                for j in range(50):
                    ErrMatrix = (result.atPose3(j)*(T[j].inverse())).matrix()
                    rotError += np.linalg.norm(Rotation.from_matrix(ErrMatrix[:3,:3]).as_rotvec()*(180/np.pi))
                    transError += np.linalg.norm(ErrMatrix[:3,-1])
                print(rotError/50, transError/50)

        result = isam.calculateEstimate()
        # self.pltlibViz(i, result)
        self.visualizeResult(result)
        # self.createMap(result, 100)

    def visualizeResult(self, result):
        poses_cloud = np.array([[], [], []])
        for i in range(len(self.clouds)):
            poses_cloud = np.hstack([poses_cloud, np.array([[result.atPose3(i).x()], [result.atPose3(i).y()], [result.atPose3(i).z()]])])
        
        init_car_pose = gtsam.Pose3(gtsam.Rot3(0.9982740, -0.0572837,  0.0129474, 0.0575611,  0.9980955, -0.0221840, -0.0116519,  0.0228910,  0.9996701),
                                    gtsam.Point3(-263.9464864482589, 2467.3015467381383, -19.374652610889633))

        driving.visualize_clouds([poses_cloud, init_car_pose.transformFrom(self.clouds[0])], show_grid_lines=True, cloud_colors=['#FFFFFF', '#ffa500'])

    def icp(self, clouda, cloudb, initial_transform=gtsam.Pose3(), max_iterations=25):

        est = initial_transform
        transform_A = est.transformFrom(clouda)
        cloudB_closest = helpers.assign_closest_pairs_KDTree(transform_A, cloudb)


        source_cloud = o3d.geometry.PointCloud()
        source_cloud.points = o3d.utility.Vector3dVector(clouda.T)
        target_cloud = o3d.geometry.PointCloud()
        target_cloud.points = o3d.utility.Vector3dVector(cloudB_closest.T)
        # Perform ICP registration
        reg_p2p = o3d.pipelines.registration.registration_icp(
        source_cloud, target_cloud, 100, est.matrix(),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000,relative_fitness=1e-6, 
                                                        relative_rmse=1e-6))
        #Got the result
        transformation_matrix = reg_p2p.transformation
        return gtsam.Pose3(gtsam.Rot3(transformation_matrix[:3,:3]), transformation_matrix[:3,-1])

    
    def createMap(self, result, idx):
        transforms = []
        for i in range(len(self.clouds)):
            transforms.append(result.atPose3(i))
        driving.visualize_clouds(self.transform_map(transforms, self.clouds), show_grid_lines=True, cloud_colors="#C6C6C6")

    def transform_map(self, transforms, clouds):
        cloud_map = []
        for i in range(1, len(clouds), 2):
            cloud_map.append(transforms[i].transformFrom(clouds[i]))
        return cloud_map
    
    def readGT(self):
        serial_str = helpers.get_partial_map_results()
        partial_result = gtsam.Values()
        partial_result.deserialize(serial_str)

        transforms = []
        for i in range((50)):
            transforms.append(partial_result.atPose3(i))
        return transforms
    
    def pltlibViz(self, N, result):
        # self.clouds -> 3xN (transformed clouds)
        for i in range(N):
            pose = result.atPose3(i)
            plt.plot( pose.transformFrom(self.clouds[i])[0,:],pose.transformFrom(self.clouds[i])[1,:], 'k.' , markersize=0.1)
            translation = pose.matrix()[:3,-1]
            plt.plot(translation[0],translation[1], 'r*')
        plt.show()


def main()->None:    
    worker = slam('lidar/')
    worker.runISAM()

if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(level=logging.INFO)
    main()