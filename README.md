# Sensor Fusion Self-Driving Car Course, Lidar

<img src="media/PointCloudClustering.gif" width="700" height="400" />

## Objects detection from scratch
In this repo, I show how to do pointcloud segmentation and clustering from scrath. Though there are functions in PCL library to do this task more efiiciently I found it pretty beneficial to also learn a working fundamental.

Implemenation includes:
- RANSAC to separate between plane and obstacles.
- KD-tree insert and search to help speed up clustering process.
- Euclidence clustering.

Other processed that get some help from PCL library:
- Downsampling using voxel grid.
- Bounding box.
- Visualization.

