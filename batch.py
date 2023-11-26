import utils
import gtsam
from gtsam.utils import plot
import matplotlib.pyplot as plt
import numpy as np

def batchOptimization_2d(file, debug = 0):
    # Load data/input_INTEL_g2o.g2o 

    # parses a g2o file and stores the measurements into a NonlinearFactorGraph and the initial guess in a Values structure.
    graph, initial = gtsam.readG2o(file, is3D=False)

    # Add a priori
    priorModel = gtsam.noiseModel.Diagonal.Variances(gtsam.Point3(1e-2, 1e-2, 1e-8))
    # Key, Prior Value, Noise Model
    graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), priorModel))

    
    
    params = gtsam.GaussNewtonParams()
    params.setVerbosity("TERMINATED")
    params.setAbsoluteErrorTol(-1e+10)
    params.setRelativeErrorTol(-1e+3)

    optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
    result = optimizer.optimize()
    

    print("initial error = ", graph.error(initial))
    print("final error = ", graph.error(result))

    if debug:
        marginals = gtsam.Marginals(graph, result)
        for i in range(1, 6):
            plot.plot_pose2(0, result.atPose2(i), 0.5,
                                marginals.marginalCovariance(i))
        plt.axis('equal'); plt.show()

    finalPose = gtsam.utilities.extractPose2(result)
    initialPose = utils.readG2O(file)[0]
    initialPose = np.array(initialPose, dtype=np.float64)
    return finalPose, initialPose

def plotPoses(pose1, pose2):
    plt.plot(pose1[:,0], pose1[:,1], ".")
    plt.plot(pose2[:,0], pose2[:,1], "--")
    plt.show()

pose1, pose2 = batchOptimization_2d("data/input_INTEL_g2o.g2o", debug=0)
plotPoses(pose1, pose2)