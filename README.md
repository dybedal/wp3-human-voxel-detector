# Human detector for point clouds using point cloud flattening an CNN scene classifier

This repository contains the software necessary to reprocuce the method described in "J. Dybedal and G. Hovland, CNN-based People Detection in Voxel Space using Intensity Measurements and Point Cluster Flattening. Modeling, Identification and Control, Vol. 42, No.2, pp.37-46, 2021.".

The point cloud data used for training and verification is published under "Dybedal, Joacim, "Replication Data for: CNN-based People Detection in Voxel Space using Intensity Measurements and Point Cluster Flattening", https://doi.org/10.18710/HMJVFM, DataverseNO, 2021".

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
"J. Dybedal and G. Hovland, CNN-based People Detection in Voxel Space using Intensity Measurements and Point Cluster Flattening. Modeling, Identification and Control, Vol. 42, No.2, pp.37-46, 2021.".

BibTeX:
```
@article{MIC-2021-2-1,
  title={{CNN-based People Detection in Voxel Space using Intensity Measurements and Point Cluster Flattening}},
  author={Dybedal, Joacim and Hovland, Geir},
  journal={Modeling, Identification and Control},
  volume={42},
  number={2},
  pages={37--46},
  year={2021},
  doi={10.4173/mic.2021.2.1},
  publisher={Norwegian Society of Automatic Control}
}
```
