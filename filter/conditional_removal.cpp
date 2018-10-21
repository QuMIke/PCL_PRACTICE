/*
This document demonstrates how to remove outliers from a PointCloud 
using several different methods in the filter module. 
we will look at how to use a ConditionalRemoval filter 
which removes all indices in the given input cloud that do not satisfy one or more given conditions.
*/


#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
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
		std::cerr << "PointCloud befre filtering: " << cloud->width * cloud->height
			<< " data points(" << pcl::getFieldsList(*cloud) << ")." << std::endl;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
		// Build condition
		pcl::ConditionAnd<pcl::PointXYZRGBA>::Ptr  range_cloud(new pcl::ConditionAnd<pcl::PointXYZRGBA>());
		range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGBA>("z", pcl::ComparisonOps::GT, 1.0)));
		range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGBA>("z", pcl::ComparisonOps::LT, 10.0)));
		// Build filter
		pcl::ConditionalRemoval<pcl::PointXYZRGBA> condrem;
		condrem.setCondition(range_cloud);
		condrem.setInputCloud(cloud);
		condrem.setKeepOrganized(true);
		// Apply filter
		condrem.filter(*cloud_filtered);

		boost::shared_ptr<pcl::visualization::PCLVisualizer> filtered_viewer(new pcl::visualization::PCLVisualizer("filtered cloud"));
		filtered_viewer->setCameraFieldOfView(1.02259994f);
		filtered_viewer->spinOnce();
		filtered_viewer->setPosition(400, 100);
		filtered_viewer->setSize(cloud_filtered->width, cloud_filtered->height);
		filtered_viewer->addPointCloud(cloud_filtered);
		filtered_viewer->resetCameraViewpoint("filtered cloud");
		filtered_viewer->setCameraPosition(
			0, 0, 0,		// Position
			0, 0, 1,		// Viewpoint
			0, -1, 0);	    // Up
		std::cerr << "PointCloud befre filtering: " << cloud_filtered->width * cloud_filtered->height
			<< " data points(" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
	}

	return 0;
}
