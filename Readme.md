# iSAM based Pose Graph Optimization

<p align="center">
  <img src="iSAM/results/gif2dcrop.gif" alt="Pay for the internet">
</p>

This project focuses on SLAM using the GTSAM library. Specifically we do a simple Incremental Smoothing and Mapping that update the Bayes tree incrementally as new information becomes available.

## Algorithm
The algorithm is particularly simple (gtsam takes the difficult part). 
- Perform ICP on 2 consecutive point clouds to obtain Lie Algebra. 
- Add this as an odometry factor to the Non Linear Factor Graph
- Perform iSAM update. 

That's it :D


## Results
## iSAM

## Dead Reckoning
