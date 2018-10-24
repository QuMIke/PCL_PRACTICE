/*
the following code snippet will estimate a set of surface normals for all the points in the input dataset,
but will estimate their nearest neighbors using another dataset.
As previously mentioned, a good usecase for this is when the input is a downsampled version of the surface.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include "KinnectGrabber.h"

int main()
{
	pcl::io::OpenNI2Grabber grabber;
	KinnectGrabber<pcl::PointXYZRGBA> v(grabber);
	v.run();

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud = nullptr;
	cloud = v.getLatestCloud();
	if (cloud == nullptr)
	{
		std::cout << "Get cloud failed!" << std::endl;
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGBA>); // need initialize to avoid bug--"px!=0"

		// Create voxel filter object
		pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(0.02, 0.02, 0.02);
		sor.filter(*cloud_downsampled);

		std::cerr << "PointCloud after filtering: " << cloud_downsampled->width * cloud_downsampled->height
			<< " data points (" << pcl::getFieldsList(*cloud_downsampled) << ").";

		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
		ne.setInputCloud(cloud_downsampled);
		ne.setSearchSurface(cloud);
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
		ne.setSearchMethod(tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch(0.03);

		// Compute the features
		ne.compute(*cloud_normals);

	}

	return 0;
}

