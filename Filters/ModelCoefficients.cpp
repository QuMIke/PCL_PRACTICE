/*
In this tutorial we will learn how to project points onto a parametric model (e.g., plane, sphere, etc). 
The parametric model is given through a set of coefficients –
in the case of a plane, through its equation: ax + by + cz + d = 0.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
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
		std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
			<< " data points(" << pcl::getFieldsList(*cloud) << ")." << std::endl;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGBA>);

		// Create a set of planar coefficients with X=Y=0,Z=1
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		coefficients->values.resize(4);
		coefficients->values[0] = coefficients->values[1] = 0;
		coefficients->values[2] = 1.0;
		coefficients->values[3] = -80;

		// Create the filtering object
		pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_projected);

		boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_Filter_viewer_(new pcl::visualization::PCLVisualizer("Filtered cloud"));
		cloud_Filter_viewer_->setCameraFieldOfView(1.02259994f);
		cloud_Filter_viewer_->spinOnce();
		cloud_Filter_viewer_->setPosition(0, 0);
		cloud_Filter_viewer_->setSize(cloud->width, cloud->height);
		cloud_Filter_viewer_->addPointCloud(cloud_projected, "Filtered cloud");
		cloud_Filter_viewer_->resetCameraViewpoint("Filtered cloud");
		cloud_Filter_viewer_->setCameraPosition(
			0, 0, 0,		// Position
			0, 0, 1,		// Viewpoint
			0, -1, 0);	// Up
		std::cerr << "PointCloud after filtering: " << cloud_projected->width*cloud_projected->height
			<< " data points (" << pcl::getFieldsList(*cloud_projected) << ")." << std::endl;
	}

	return 0;
}
