import utils
import gtsam
from gtsam.utils import plot
import matplotlib.pyplot as plt
import numpy as np

def iSAM2d(file, debug = 0):
    vertex, edges = utils.readG2O(file)

    params = gtsam.ISAM2Params()
    # Only relinearize variables whose linear delta magnitude is greater than this threshold
    params.setRelinearizeThreshold(0.1)
    isam = gtsam.ISAM2(params)

    priorModel = gtsam.noiseModel.Diagonal.Variances(gtsam.Point3(1e-6, 1e-6, 1e-8))

    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    for pose in vertex:
        
        poseNum = int(pose[0])
        #Add a prior if we are on first node
        if(poseNum==0):
            _, x,y,theta = pose
            graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(x,y,theta), priorModel))
            initial_estimate.insert(poseNum, gtsam.Pose2(x,y,theta))

        else:
            #Prev Pose is the initial estimate
            prevPose = result.atPose2(poseNum - 1)
            initial_estimate.insert(poseNum, prevPose)
            for edge in edges:
                #Add edges that connect from curr node
                if(int(edge[1]) == poseNum):
                    id_e1, id_e2, dx, dy, dtheta, *info = edge
                    id_e1 = id_e1.astype(np.int32)
                    id_e2 = id_e2.astype(np.int32)                    
                    info_m = utils.construct_info_mat(info)
                    noise_model = gtsam.noiseModel.Gaussian.Information(info_m)
                    graph.add(gtsam.BetweenFactorPose2(id_e1, id_e2 , gtsam.Pose2(dx, dy, dtheta), noise_model))

        # Perform incremental update to iSAM2's internal Bayes tree, optimizing only the affected variables.
        isam.update(graph, initial_estimate)
        result = isam.calculateEstimate()
        graph.resize(0)
        initial_estimate.clear()

    resultPoses = gtsam.utilities.extractPose2(result)
    return resultPoses, vertex[:,1:]


def plotPoses(pose1, pose2):
    plt.plot(pose1[:,0], pose1[:,1], ".")
    plt.plot(pose2[:,0], pose2[:,1], "--")
    plt.show()

pose1, pose2 = iSAM2d("data/input_INTEL_g2o.g2o", debug=0)
plotPoses(pose1, pose2)