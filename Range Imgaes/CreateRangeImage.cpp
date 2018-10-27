/*
This tutorial demonstrates how to create a range image from a point cloud and a given sensor position. 
What is range image? It's main idea is that the three-dimensional point cloud is projected directly into two dimensional images.
We call this projection is  range_image.
*/

#include "stdafx.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
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

    // view range image
		pcl::visualization::RangeImageVisualizer viewer;
		viewer.showRangeImage(rangeImage);
		viewer.setSize(cloud->width, cloud->height);

		while (!viewer.wasStopped()) 
		{
			viewer.spinOnce();
		}

	}

	return 0;
}
