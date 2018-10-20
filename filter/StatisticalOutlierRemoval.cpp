#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
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
		boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_(new pcl::visualization::PCLVisualizer("PCL OpenNI2 cloud"));
		cloud_viewer_->setCameraFieldOfView(1.02259994f);

		cloud_viewer_->spinOnce();

		cloud_viewer_->setPosition(0, 0);
		cloud_viewer_->setSize(cloud->width, cloud->height);
		cloud_viewer_->addPointCloud(cloud, "OpenNICloud");
		cloud_viewer_->resetCameraViewpoint("OpenNICloud");
		cloud_viewer_->setCameraPosition(
			0, 0, 0,		// Position
			0, 0, 1,		// Viewpoint
			0, -1, 0);	// Up
		std::cerr << "PointCloud befre filtering: " << cloud->width * cloud->height
			<< " data points(" << pcl::getFieldsList(*cloud) << ")." << std::endl;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);

		// creaate the filtering object
		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.filter(*cloud_filtered);

		boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_Filter_viewer_(new pcl::visualization::PCLVisualizer("Filtered cloud"));
		cloud_Filter_viewer_->setCameraFieldOfView(1.02259994f);
		cloud_Filter_viewer_->spinOnce();
		cloud_Filter_viewer_->setPosition(0, 0);
		cloud_Filter_viewer_->setSize(cloud->width, cloud->height);
		cloud_Filter_viewer_->addPointCloud(cloud_filtered, "Filtered cloud");
		cloud_Filter_viewer_->resetCameraViewpoint("Filtered cloud");
		cloud_Filter_viewer_->setCameraPosition(
			0, 0, 0,		// Position
			0, 0, 1,		// Viewpoint
			0, -1, 0);	// Up
		std::cerr << "PointCloud after filtering: " << cloud_filtered->width*cloud_filtered->height
			<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
	}

	return 0;
}
