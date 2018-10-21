/*
we will learn how to us a RadiusOutlierRemoval filter 
which removes all indices in it’s input cloud that 
don’t have at least some number of neighbors within a certain range.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
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
    /* using PCLVisualizer will make pcl::removeNaNFromPointCloud failed 
    beacuase of cloud_viewer->spinOnce employ cloud to renew display window */
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer(new pcl::visualization::PCLVisualizer("captured cloud"));
		//cloud_viewer->setCameraFieldOfView(1.02259994f);
		//cloud_viewer->spinOnce();
		//cloud_viewer->setPosition(0, 0);
		//cloud_viewer->setSize(cloud->width, cloud->height);
		//cloud_viewer->addPointCloud(cloud, "captured cloud");
		//cloud_viewer->resetCameraViewpoint("captured cloud");
		//cloud_viewer->setCameraPosition(
		//	0, 0, 0,		// Position
		//	0, 0, 1,		// Viewpoint
		//	0, -1, 0);	// Up
		std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
			<< " data points(" << pcl::getFieldsList(*cloud) << ")." << std::endl;

		// Remove nan points
		std::vector<int> idx;
		boost::shared_ptr<pcl::PointIndices> index = nullptr;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr no_nan_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::removeNaNFromPointCloud(*cloud, *no_nan_cloud, idx);
		if (no_nan_cloud == nullptr)
		{
			std::cout << "remove NaN from point cloud failed!" << std::endl;
			return 0;
		}
		//index->header = no_nan_cloud->header;  // if you want to set RadiusOutlierRemover.setIndices, so code this line to get an indices, this will make no_nan_cloud header change, so it makes no_nan_cloud nullptr 
		//index->indices = idx;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
		// Build filter
		pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outrem;
		outrem.setInputCloud(no_nan_cloud);
		outrem.setRadiusSearch(0.8);
		outrem.setMinNeighborsInRadius(2);
		//outrem.setKeepOrganized(true);       // 
		//outrem.setIndices(index);
		// Apply filter
		outrem.filter(*cloud_filtered);

    /* cloud_filtered is not organized, so PCLVisualizer won't work!*/
		//boost::shared_ptr<pcl::visualization::PCLVisualizer> filtered_viewer(new pcl::visualization::PCLVisualizer("filtered cloud"));
		//filtered_viewer->setCameraFieldOfView(1.02259994f);
		//filtered_viewer->spinOnce();
		//filtered_viewer->setPosition(400, 100);
		//filtered_viewer->setSize(cloud_filtered->width, cloud_filtered->height);
		//filtered_viewer->addPointCloud(cloud_filtered);
		//filtered_viewer->resetCameraViewpoint("filtered cloud");
		//filtered_viewer->setCameraPosition(
		//	0, 0, 0,		// Position
		//	0, 0, 1,		// Viewpoint
		//	0, -1, 0);	    // Up
		std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
			<< " data points(" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
	}

	return 0;
}
