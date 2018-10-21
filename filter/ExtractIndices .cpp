/*
In this tutorial we will learn how to use an pcl::ExtractIndices <pcl::ExtractIndices> filter
to extract a subset of points from a point cloud based on the indices output by a segmentation algorithm.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
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
		// Create the filtering object: downsample the dataset using a leaf size of 1cm
		pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(0.01f, 0.01f, 0.01f);
		sor.filter(*cloud_filtered);

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(0.01);

    // Write the cloud to disk
    pcl::PCDWriter writer;
		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		// create point cloud ptrs for processing 
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>);

		int i = 0, nr_points = (int)cloud_filtered->points.size();
		// While 30% of the original cloud is still there
		while (cloud_filtered->points.size() > 0.3 * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud(cloud_filtered);
			seg.segment(*inliers, *coefficients);
			if (inliers->indices.size() == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			// Extract the inliers
			extract.setInputCloud(cloud_filtered);
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*cloud_p);
			std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

      std::stringstream ss;
      ss << "table_scene_lms400_plane_" << i << ".pcd";
      writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_p, false);

			// Create the filtering object
			extract.setNegative(true);
			extract.filter(*cloud_f);
			cloud_filtered.swap(cloud_f);
			i++;
		}


	}

	return 0;
}
