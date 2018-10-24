/*
The following code snippet will estimate a set of surface normals for a subset of the points in the input dataset.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
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
		// Create a set of indixes to be used. For simmplicity, use the first 10% of the points in cloud
		std::vector<int> indixes(floor(cloud->points.size() / 10));
		for (size_t i = 0; i < indixes.size(); ++i) indixes[i] = i;

		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
		ne.setInputCloud(cloud);
	
		// Pass the indixes
		boost::shared_ptr<std::vector<int>> indicesPtr(new std::vector<int>(indixes));
		ne.setIndices(indicesPtr);

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
