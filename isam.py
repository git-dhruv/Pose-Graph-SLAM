import utils
import gtsam
from gtsam.utils import plot
import matplotlib.pyplot as plt
import numpy as np


def iSAM2d(file, debug = 0):
    pass


def plotPoses(pose1, pose2):
    plt.plot(pose1[:,0], pose1[:,1], ".")
    plt.plot(pose2[:,0], pose2[:,1], "--")
    plt.show()

pose1, pose2 = iSAM2d("data/input_INTEL_g2o.g2o", debug=0)
plotPoses(pose1, pose2)