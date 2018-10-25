/*
This tutorial introduces a family of 3D feature descriptors 
coined PFH (Point Feature Histograms) for simplicity, presents their theoretical advantages and 
discusses implementation details from PCL’s perspective.
As a prerequisite, please go ahead and read the Estimating Surface Normals in a PointCloud tutorial first,
as PFH signatures rely on both xyz 3D data as well as surface normals.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
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
		// create normal estimator
		pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
		ne.setInputCloud(cloud);

		// create kdtree for search
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
		ne.setSearchMethod(tree);

		// output normals
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		
		// using all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch(0.03);

		// compute normals
		ne.compute(*normals);

		// create PFH estimator , input clouds and normals
		pcl::PFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHSignature125> pfh;
		pfh.setInputCloud(cloud);
		pfh.setInputNormals(normals);

		// create kdtree for search
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr f_tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
		pfh.setSearchMethod(f_tree);

		// create out feture
		pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

		// use all neighbor in a sphere of radius 5cm
		pfh.setRadiusSearch(0.05);

		// compute pfh fetures
		pfh.compute(*pfhs);

	}

	return 0;
}
