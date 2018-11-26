HMRF ICP Demo
=============

Included is the data necessary to demonstrate the hidden Markov random field
iterative closest point algorithm. Follow the steps below to load and run the
examples. The point clouds in the first dataset have a grid topology, whereas
the clouds in the second set do not.

```
>>> setup
>>> [tf_1 data_1] = hmrf_icp(fixed_1, free_1, params);
>>> plot_icp(fixed_1, free_1, tf_1, data_1.z);
>>> params.neighbor_structure = 'unstructured';
>>> [tf_2 data_2] = hmrf_icp(fixed_2, free_2, params);
>>> plot_icp(fixed_2, free_2, tf_2, data_2.z);
```

The point clouds included are taken from the datasets below.

J. Sturm, N. Engelhard, F. Endres, W. Burgard, and D. Cremers. "A benchmark for
the evaluation of RGB-D SLAM systems." In Proc. of the International Conference
on Intelligent Robot Systems (IROS), Oct. 2012.

A. Geiger, P. Lenz, and R. Urtasun. "Are we ready for autonomous driving? The
KITTI vision benchmark suite." In Conference on Computer Vision and Pattern
Recognition (CVPR), 2012.
