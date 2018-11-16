#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 10000;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->resize(cloud->width * cloud->height);

	// generate planar x2 + y2 + z2 = 1 point cloud data
	size_t num = cloud->size();
	for (size_t i = 0; i < num; ++i)
	{
		cloud->points[i].x = rand() / (RAND_MAX + 1.0);
		cloud->points[i].y = rand() / (RAND_MAX + 1.0);

		// generate outliers
		if (i % 5 == 0)
		{
			cloud->points[i].z = rand() / (RAND_MAX + 1.0);
		}
		else
		{
			cloud->points[i].z = 1 - cloud->points[i].x - cloud->points[i].y;
		}

	}

	// visulize source point cloud 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer(new pcl::visualization::PCLVisualizer);
	cloud_viewer->addPointCloud(cloud, "cloud");
	while (!cloud_viewer->wasStopped())
	{
		cloud_viewer->spinOnce();
	}
	
	// to save seg output 
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// create sac segmentation object and set params
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud->makeShared());
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Cloud not estimate a planar model for the given dataset.\n");
		return(-1);
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;


	// visulize plannar point cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, inliers->indices, *plane);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> plane_viewer(new pcl::visualization::PCLVisualizer);
	plane_viewer->addPointCloud(plane, "plane");
	while (!plane_viewer->wasStopped())
	{
		plane_viewer->spinOnce();
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}

	return 0;

}
