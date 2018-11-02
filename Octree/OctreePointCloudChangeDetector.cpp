/*
An octree is a tree-based data structure for organizing sparse 3-D data. 
In this tutorial we will learn how to use the octree implementation
for detecting spatial changes between multiple unorganized point clouds which could vary in size, resolution, density and point ordering.
By recursively comparing the tree structures of octrees, 
spatial changes represented by differences in voxel configuration can be identified. Additionally,
we explain how to use the pcl octree “double buffering” technique allows us to efficiently process multiple point clouds over time.
*/

#include "stdafx.h"
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include "KinnectGrabber.h"

int main()
{
	pcl::io::OpenNI2Grabber grabber;
	KinnectGrabber<pcl::PointXYZ> v(grabber);
	v.run();

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = v.getLatestCloud();

	if (cloud == nullptr)
	{
		std::cout << "Get cloud failed!" << std::endl;
	}
	else
	{
		srand((unsigned int)time(NULL));

		// octree resolution -side length of octree voxels
		float resolution = 32.0f;

		// Instantiate octree-based point cloud change detection class
		pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);
		
		// Add points cloud from cloudA to octree
		octree.setInputCloud(cloud);
		octree.addPointsFromInputCloud();

		// Swtich octree buffers: This resets octree but keeps previous tree structure in memory.
		octree.switchBuffers();

		v.run();
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ = v.getLatestCloud();
		if (cloud_ != nullptr)
		{
			// Add points cloud to octree
			octree.setInputCloud(cloud_);
			octree.addPointsFromInputCloud();

			std::vector<int> newPointIdxVector;

			// Get vector of point indices from octree voxels which did not exsit in previous buffer
			octree.getPointIndicesFromNewVoxels(newPointIdxVector);

			// Output points
			std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
			for (size_t i = 0; i < newPointIdxVector.size(); ++i)
			{
				std::cout << i << "# Index:" << newPointIdxVector[i]
					<< " point:" << cloud_->points[newPointIdxVector[i]].x << " "
					<< cloud_->points[newPointIdxVector[i]].y << " "
					<< cloud_->points[newPointIdxVector[i]].z << " "
					<< std::endl;
			}
		}
	}

	return 0;

}
