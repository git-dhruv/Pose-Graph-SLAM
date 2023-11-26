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
        if debug:
            if poseNum<200 or poseNum%10==0:
                resultPoses = gtsam.utilities.extractPose2(result)
                plt.plot(resultPoses[:,0], resultPoses[:,1], "--")
                plt.xlim([-15, 15])
                plt.ylim([-25, 10])
                # plt.show()
                plt.savefig(f"{poseNum}.png")
                plt.close()

    resultPoses = gtsam.utilities.extractPose2(result)
    return resultPoses, vertex[:,1:]

def iSAM3d(file, debug = 0):
    vertex, edges = utils.readG2O(file)

    params = gtsam.ISAM2Params()
    params.setRelinearizeThreshold(0.1)
    isam = gtsam.ISAM2(params)

    ## Build the Graph ##
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    for pose in vertex:
        poseNum = int(pose[0])
        #Add a prior if we are on first node
        if(poseNum==0):
            priorModel = gtsam.noiseModel.Diagonal.Variances(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))
            _,x,y,z,q1,q2,q3,q0 = pose
            rotation = gtsam.Rot3(q0,q1,q2,q3)
            trans = gtsam.Point3(x,y,z)
            absPose = gtsam.Pose3(rotation,trans)

            graph.add( gtsam.PriorFactorPose3(0, absPose, priorModel)  )
            initial_estimate.insert(poseNum, absPose)
        #Rest of the Poses and vertices
        else:
            prev_pose = result.atPose3(poseNum - 1)
            initial_estimate.insert(poseNum, prev_pose)
            for edge in edges:
                if int(edge[1]) == poseNum:
                    id_e1, id_e2, x,y,z,q1,q2,q3,q0, *info = edge
                    id_e1 = id_e1.astype(np.int32)
                    id_e2 = id_e2.astype(np.int32)                    
                    rotation = gtsam.Rot3(q0,q1,q2,q3)
                    trans = gtsam.Point3(x,y,z)
                    absPose = gtsam.Pose3(rotation,trans)
                    info_m = utils.construct_info_mat(info)
                    noise_model = gtsam.noiseModel.Gaussian.Information(info_m)
                    graph.add(gtsam.BetweenFactorPose3(id_e1, id_e2 , absPose, noise_model))

        isam.update(graph, initial_estimate)
        result = isam.calculateEstimate()
        graph.resize(0)
        initial_estimate.clear()
    resultPoses = gtsam.utilities.extractPose3(result)[:,-3:]
    return resultPoses, vertex[:,1:]


pose1, pose2 = iSAM2d("data/input_INTEL_g2o.g2o", debug=1)
# utils.plotPoses(pose1, pose2)
# pose1, pose2 = iSAM3d("data/parking-garage.g2o", debug=0)
# utils.plotPoses(pose1, pose2, dim = 3)


import imageio
images = []
for i in range(1200):
    filename = str(i)+".png"
    try:
        images.append(imageio.imread(filename))
    except:
        pass
imageio.mimsave('movie.gif', images)
