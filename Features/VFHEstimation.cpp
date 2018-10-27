/*
This document describes the Viewpoint Feature Histogram ([VFH]) descriptor, 
a novel representation for point clusters for the problem of Cluster (e.g., Object) Recognition and 6DOF Pose Estimation.
The Viewpoint Feature Histogram (or VFH) has its roots in the FPFH descriptor (see Fast Point Feature Histograms (FPFH) descriptors).
Due to its speed and discriminative power, we decided to leverage the strong recognition results of FPFH, 
but to add in viewpoint variance while retaining invariance to scale.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include "KinnectGrabber.h"

int main()
{
	pcl::io::OpenNI2Grabber grabber;
	KinnectGrabber<pcl::PointXYZRGBA> v(grabber);
	v.run();

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud = v.getLatestCloud();

	if (cloud == nullptr)
	{
		std::cout << "Get cloud failed!" << std::endl;
	}
	else
	{
		// create normal estimator
		pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
		ne.setInputCloud(cloud);

		// create kdtree for search
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZRGBA>());
		ne.setSearchMethod(tree_n);

		// output normals
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		// using all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch(0.03);

		// compute normals
		ne.compute(*normals);

		// Create the VFH estimation class, and pass the input dataset+normals to it
		pcl::VFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::VFHSignature308> vfh_e;
		vfh_e.setInputCloud(cloud);
		vfh_e.setInputNormals(normals);
		// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_f(new pcl::search::KdTree<pcl::PointXYZRGBA>);
		vfh_e.setSearchMethod(tree_f);

		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		vfh_e.setRadiusSearch(0.02);

		// Output datasets
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
		
		// Compute the features
		vfh_e.compute(*vfhs);

	}

	return 0;
}
