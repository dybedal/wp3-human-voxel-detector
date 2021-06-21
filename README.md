# Human detector for point clouds using point cloud flattening an CNN scene classifier

This repository contains the software necessary to reprocuce the method described in "J. Dybedal and G. Hovland, CNN-based People Detection in Voxel Space using Intensity Measurements and Point Cluster Flattening, MIC, 2021, In Press.".

The point cloud data used for training and verification is published under "Dybedal, Joacim, 2021, "Replication Data for: CNN-based People Detection in Voxel Space using Intensity Measurements and Point Cluster Flattening", https://doi.org/10.18710/HMJVFM, DataverseNO".

The repository also contains the ROS node used to filter and merge multiple XYZI Point Clouds into a single time-synchronized point cloud.

## Prerequisites
Matlab and ROS Kinetic Kame or above.

## Usage for demonstration
Open the matlab project and load the CNN20210131_2.mat workspace to use the pre-trained network.
Play back the 2018-08-24-095008-4human-lowsun-merged.bag rosbag file and start the "classify.m" Matlab Script.
Use RVIZ to subscribe to the "pointcloud_merger/pointcloud_out" and the "visualization_msgs/Marker" messages.

## Licence
See LICENCE file.

## Citation
If you use or reference this repository, please cite:
"J. Dybedal and G. Hovland, CNN-based People Detection in Voxel Space using Intensity Measurements and Point Cluster Flattening, MIC, 2021, In Press.".

