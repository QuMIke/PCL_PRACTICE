/*
For real-time or near real-time applications, the computation of 
Point Feature Histograms in dense point neighborhoods can represent one of the major bottlenecks.
This tutorial describes a simplification of the PFH formulation, 
called Fast Point Feature Histograms (FPFH), that reduces the computational complexity of the algorithm to O(nk), 
while still retaining most of the discriminative power of the PFH.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
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
		// create output normals
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

		// Create the FPFH estimation class, and pass the input dataset+normals to it
		pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud(cloud);
		fpfh.setInputNormals(normals);
		// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);

		fpfh.setSearchMethod(tree);

		// Output datasets
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch(0.05);

		// Compute the features
		fpfh.compute(*fpfhs);

	}

	return 0;
}
