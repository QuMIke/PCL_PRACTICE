/*
An octree is a tree-based data structure for managing sparse 3-D data. Each internal node has exactly eight children. 
In this tutorial we will learn how to use the octree for spatial partitioning and neighbor search within pointcloud data.
Particularly,we explain how to perform a “Neighbors within Voxel Search”, the “K Nearest Neighbor Search” 
and “Neighbors within Radius Search”.
*/

#include "stdafx.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
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
		pcl::PointXYZ searchPoint;
		searchPoint.x = 512.0f * rand() / (RAND_MAX + 1.0f); // watch out search point must be within cloud's size!
		searchPoint.y = 128.0f * rand() / (RAND_MAX + 1.0f);
		searchPoint.z = 512.0f * rand() / (RAND_MAX + 1.0f);

		float resolution = 128.0f;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
		octree.setInputCloud(cloud);
		octree.addPointsFromInputCloud();

		// Neighbors with voxel search
		std::vector<int> pointIdxVec;
		if (octree.voxelSearch(searchPoint, pointIdxVec))
		{
			std::cout << "Neiighbors within voxel search at (" << searchPoint.x
				<< " " << searchPoint.y
				<< " " << searchPoint.z << ")"
				<< std::endl;
		 }
		for (size_t i = 0; i < pointIdxVec.size(); ++i)
		{
			std::cout << "	" << cloud->points[pointIdxVec[i]].x
				<< " " << cloud->points[pointIdxVec[i]].y
				<< " " << cloud->points[pointIdxVec[i]].z
				<< std::endl;
		}

		// K nearest neighbor search
		int K = 10;
		std::vector<int> pointIdxKNNSearch;
		std::vector<float>pointKNNSearchDis;

		std::cout << "K nearest neighbor search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z
			<< ") with K=" << K
			<< std::endl;
		if (octree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSearchDis)>0)
		{
			for (size_t i = 0; i < pointIdxKNNSearch.size(); ++i)
			{
				std::cout << "	" << cloud->points[pointIdxKNNSearch[i]].x
					<< " " << cloud->points[pointIdxKNNSearch[i]].y
					<< " " << cloud->points[pointIdxKNNSearch[i]].z
					<< " (squared distance: " << pointKNNSearchDis[i] << ")"
					<< std::endl;
			}
		}

		// Neighbors with radius search
		float radius = 256.0f * rand() / (RAND_MAX + 1);
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSaquredDis;
		std::cout << "Neighbors within radiussearch at ("
			<< " " << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z
			<< ") with radius="<< radius
			<< std::endl;
		if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSaquredDis) > 0)
		{
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				std::cout << "	" << cloud->points[pointIdxRadiusSearch[i]].x
					<< " " << cloud->points[pointIdxRadiusSearch[i]].y
					<< " " << cloud->points[pointIdxRadiusSearch[i]].z
					<< " (squared distance: " << pointRadiusSaquredDis[i]
					<< std::endl;
			}
		}
	}

	return 0;

}
