/*
This tutorial will address the problem, 
that is, given a point cloud dataset, 
directly compute the surface normals at each point in the cloud.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
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
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
		ne.setInputCloud(cloud);

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

		boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer(new pcl::visualization::PCLVisualizer("captured cloud"));
		cloud_viewer->setCameraFieldOfView(1.02259994f);
		cloud_viewer->spinOnce();
		cloud_viewer->setPosition(0, 0);
		cloud_viewer->setSize(cloud->width, cloud->height);
		cloud_viewer->addPointCloud(cloud, "captured cloud");
		cloud_viewer->resetCameraViewpoint("captured cloud");
		cloud_viewer->setCameraPosition(
			0, 0, 0,		// Position
			0, 0, 1,		// Viewpoint
			0, -1, 0);	// Up
		cloud_viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud, cloud_normals, 10, 0.05, "captured cloud");

	}

	return 0;
}
