import utils
import gtsam
from gtsam.utils import plot
import matplotlib.pyplot as plt
import numpy as np

def batchOptimization_2d(file, debug = 0):
    # parses a g2o file and stores the measurements into a NonlinearFactorGraph and the initial guess in a Values structure.
    graph, initial = gtsam.readG2o(file, is3D=False)

    # Add a priori
    priorModel = gtsam.noiseModel.Diagonal.Variances(gtsam.Point3(1e-10,1e-10,1e-10))
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
    initialPose = np.array(initialPose, dtype=np.float64)[:,1:]
    return finalPose, initialPose

def batchOptimization_3d(file, debug = 0):
    # parses a g2o file and stores the measurements into a NonlinearFactorGraph and the initial guess in a Values structure.
    graph, initial = gtsam.readG2o(file, is3D=True)

    # Add a priori
    priorModel = gtsam.noiseModel.Diagonal.Variances(np.array([1e-5,1e-5,1e-5,1e-5,1e-5,1e-5]))
    # Key, Prior Value, Noise Model
    graph.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(), priorModel))

    
    
    params = gtsam.GaussNewtonParams()
    params.setVerbosity("TERMINATED")
    
    params.setAbsoluteErrorTol(-1e+10)
    params.setRelativeErrorTol(-1e+3)

    optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
    result = optimizer.optimize()
    

    print("initial error = ", graph.error(initial))
    print("final error = ", graph.error(result))

    finalPose = gtsam.utilities.extractPose3(result)[:,-3:]
    initialPose = utils.readG2O(file)[0]
    initialPose = np.array(initialPose, dtype=np.float64)[:,1:]
    return finalPose, initialPose


# pose1, pose2 = batchOptimization_2d("data/input_INTEL_g2o.g2o", debug=0)
# plotPoses(pose1, pose2)

pose1, pose2 = batchOptimization_3d("data/parking-garage.g2o", debug=0)
utils.plotPoses(pose1, pose2, dim = 3)
