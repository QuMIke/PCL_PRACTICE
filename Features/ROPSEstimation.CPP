/*
The idea of the feature extraction method is as follows.
Having a mesh and a set of points for which feature must be computed we perform some simple steps. 
First of all for a given point of interest the local surface is cropped. 
Local surface consists of the points and triangles that are within the given support radius. 
For the given local surface LRF (Local Reference Frame) is computed. 
LRF is simply a triplet of vectors, the comprehensive information about how these vectors are computed you can find in the article.
What is really important is that using these vectors we can provide the invariance to the rotation of the cloud.
To do that, we simply translate points of the local surface in such way that point of interest became the origin,
after that we rotate local surface so that the LRF vectors were aligned with the Ox, Oy and Oz axes.
Having this done, we then start the feature extraction.
For every axis Ox, Oy and Oz the following steps are performed, we will refer to these axes as current axis:
  local surface is rotated around the current axis by a given angle;
  points of the rotated local surface are projected onto three planes XY, XZ and YZ;
  for each projection distribution matrix is built, this matrix simply shows how much points fall onto each bin. Number of bins represents the matrix dimension and is the parameter of the algorithm, as well as the support radius;
  for each distribution matrix central moments are calculated: M11, M12, M21, M22, E. Here E is the Shannon entropy;
  calculated values are then concatenated to form the sub-feature.
  We iterate through these steps several times. Number of iterations depends on the given number of rotations. Sub-features for different axes are concatenated to form the final RoPS descriptor.
  
For this tutorial we will use the model from the Queen’s Dataset. You can choose any other point cloud, 
but in order to make the code work you will need to use the triangulation algorithm in order to obtain polygons. 
*/

#include "stdafx.h"
#include <pcl/features/normal_3d.h>
#include <pcl/features/rops_estimation.h>
#include <pcl/surface/gp3.h>
#include "KinnectGrabber.h"

int main()
{
	pcl::io::OpenNI2Grabber grabber;
	KinnectGrabber<pcl::PointXYZ> v(grabber);

	v.run();
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = nullptr;
	cloud = v.getLatestCloud();
	if (cloud == nullptr)
	{
		std::cout << "Get cloud failed!" << std::endl;
	}
	else
	{
		// estimate normals
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>);
		normal_estimator.setInputCloud(cloud);
		normal_estimator.setRadiusSearch(0.01);
		normal_estimator.setSearchMethod(tree_n);
		normal_estimator.compute(*normal);

		// concatenate cloud and normal
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normal, *cloud_normal);

		// create polygonmesh to store output
		pcl::PolygonMesh triangles;
		// create search method kdtree
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree_c_n(new pcl::search::KdTree<pcl::PointNormal>);

		// create greedy projection triangulation object and paras
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		gp3.setSearchRadius(1.f);
		gp3.setMu(2.5f);
		gp3.setMaximumNearestNeighbors(1000);
		gp3.setMaximumSurfaceAngle(M_PI / 4.0);
		gp3.setMinimumAngle(M_PI / 18);
		gp3.setMaximumAngle(2 * M_PI / 3);
		gp3.setNormalConsistency(false);
		gp3.setSearchMethod(tree_c_n);

		// input cloud with normal and reconstruct
		gp3.setInputCloud(cloud_normal);
		gp3.reconstruct(triangles);

		//// make kdtree for ros feature searching 
		pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method(new pcl::search::KdTree<pcl::PointXYZ>);
		search_method->setInputCloud(cloud);

		// make ROSEstimation obj and set paras
		float support_radius = 0.0285f;
		unsigned int number_of_partition_bins = 5;
		unsigned int number_of_rotations = 3;

		pcl::ROPSEstimation<pcl::PointXYZ, pcl::Histogram<135>> ros_estimator;
		ros_estimator.setSearchMethod(search_method);
		ros_estimator.setSearchSurface(cloud);
		ros_estimator.setInputCloud(cloud);
		ros_estimator.setTriangles(triangles.polygons);
		ros_estimator.setRadiusSearch(support_radius);
		ros_estimator.setNumberOfPartitionBins(number_of_partition_bins);
		ros_estimator.setNumberOfRotations(number_of_rotations);
		ros_estimator.setSupportRadius(support_radius);

		// declare histogram to instore output and begin estimate
		pcl::PointCloud<pcl::Histogram<135>>::Ptr histograms(new pcl::PointCloud<pcl::Histogram<135>>());
		ros_estimator.compute(*histograms);

	}

	return 0;

}
