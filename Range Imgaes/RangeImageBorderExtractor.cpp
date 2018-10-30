/*
This tutorial demonstrates how to extract borders (traversals from foreground to background) from a range image.
We are interested in three different kinds of points: object borders, 
which are the outermost visible points still belonging to an object, shadow borders, 
which are points in the background that adjoin occlusions, and veil points, 
interpolated points between the obstacle border and the shadow border, 
which are a typical phenomenon in 3D range data obtained by lidars.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
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
		const pcl::PointCloud<pcl::PointXYZRGBA>& point_cloud = *cloud;

		// set range image a 1deg angular resolution
		float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
		float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
		float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians

		Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		float noiseLevel = 0.00;
		float minRange = 0.0f;
		int borderSize = 1;

		// create range image
		pcl::RangeImage rangeImage;
		rangeImage.createFromPointCloud(point_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
			sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

		// open 3D viewer and add point cloud
		pcl::visualization::PCLVisualizer viewer("3D viewer");
		viewer.setBackgroundColor(1, 1, 1);
		viewer.addCoordinateSystem(1.0f, "global");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> point_cloud_color_handler(cloud, 0, 0, 0);
		viewer.addPointCloud(cloud, point_cloud_color_handler, "original point cloud");

		// extrct borders
		pcl::RangeImageBorderExtractor border_extractor(&rangeImage);
		pcl::PointCloud<pcl::BorderDescription> border_descriptors;
		border_extractor.compute(border_descriptors);

		// show points in 3D viewer
		pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
			veil_points_ptr(new  pcl::PointCloud<pcl::PointWithRange>),
			shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
		pcl::PointCloud<pcl::PointWithRange> &border_points = *border_points_ptr,
			&veil_points = *veil_points_ptr,
			&shadow_points = *shadow_points_ptr;

		for (int y = 0; y < (int)rangeImage.height; ++y)
		{
			for (int x = 0; x < (int)rangeImage.width; ++x)
			{
				if (border_descriptors.points[y*rangeImage.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
					border_points.points.push_back(rangeImage.points[y*rangeImage.width + x]);
				if (border_descriptors.points[y*rangeImage.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
					veil_points.points.push_back(rangeImage.points[y*rangeImage.width + x]);
				if (border_descriptors.points[y*rangeImage.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
					shadow_points.points.push_back(rangeImage.points[y*rangeImage.width + x]);
			}
		}

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler(border_points_ptr, 0, 255, 0);
		viewer.addPointCloud<pcl::PointWithRange>(border_points_ptr, border_points_color_handler, "border points");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler(veil_points_ptr, 255, 0, 0);
		viewer.addPointCloud<pcl::PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil_points");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler(shadow_points_ptr, 0, 255, 255);
		viewer.addPointCloud<pcl::PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow_points");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");

		// show points on range image
		pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
		range_image_borders_widget = pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(rangeImage, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false, border_descriptors, "Range image with borders");

		while (!viewer.wasStopped())
		{
			range_image_borders_widget->spinOnce();
			viewer.spinOnce();
			boost::this_thread::sleep(boost::posix_time::seconds(0.01));
		}

	}

	return 0;
}
