/*
This tutorial demonstrates how to extract NARF descriptors at NARF keypoint positions from a range image.
The executable enables us to load a point cloud from disc (or create it if not given), 
extract interest points on it and then calculate the descriptors at these positions.
It then visualizes these positions, both in an image and a 3D viewer.
*/

#include "stdafx.h"
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
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
		// get point cloud 
		const pcl::PointCloud<pcl::PointXYZRGBA>& point_cloud = *cloud;

		// Create a range image of a 1deg angular resolution
		float angularResolution = (float)(0.1f * (M_PI / 180.0f));  //   0.1 degree in radians
		float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
		float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians

		Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		float noiseLevel = 0.05;
		float minRange = 0.0f;
		int borderSize = 1;

		boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;
		range_image.createFromPointCloud(point_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
			sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
		
		pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
		range_image.integrateFarRanges(far_ranges);

		// -----Open 3D viewer and add point cloud-----
		// --------------------------------------------
		pcl::visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(1, 1, 1);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
		viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
		//viewer.addCoordinateSystem (1.0f, "global");
		//PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
		//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
		viewer.initCameraParameters();
		setViewerPose(viewer, range_image.getTransformationToWorldSystem());

		// -----Show range image-----
		// --------------------------
		pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
		range_image_widget.showRangeImage(range_image);

		// extract NARF keypoints
		pcl::RangeImageBorderExtractor range_image_border_extrctor;
		pcl::NarfKeypoint narf_keypoint_detector;
		narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extrctor);
		narf_keypoint_detector.setRangeImage(&range_image);
		narf_keypoint_detector.getParameters().support_size = 0.2f;

		pcl::PointCloud<int> keypoint_indices;
		narf_keypoint_detector.compute(keypoint_indices);
		std::cout << "Found" << keypoint_indices.size() << "key points.\n";

		// show keypoints in 3D viewer
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>& keypoints = *keypoints_ptr;
		keypoints.points.resize(keypoint_indices.points.size());
		for (size_t i= 0; i < keypoint_indices.points.size(); ++i)
		{
			keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
		viewer.addPointCloud<pcl::PointXYZRGBA>(keypoints_ptr, keypoints_color_handler, "keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

		// extract NARF descriptors for interet points
		std::vector<int> keypoint_indices2;
		keypoint_indices2.resize(keypoint_indices.size());
		for (size_t i = 0; i < keypoint_indices.size(); ++i)
		{
			keypoint_indices2[i] = keypoint_indices[i];
		}
		pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);
		narf_descriptor.getParameters().support_size = 0.2f;
		narf_descriptor.getParameters().rotation_invariant = true;
		pcl::PointCloud<pcl::Narf36> narf_descriptors;
		narf_descriptor.compute(narf_descriptors);

		std::cout << "Extracted" << narf_descriptors.size() << "descriptors for " 
			<< keypoint_indices.points.size() << " keypoints.\n";

		while (!viewer.wasStopped()) 
		{
			range_image_widget.spinOnce();
			viewer.spinOnce();
			boost::this_thread::sleep(boost::posix_time::microseconds(100));;
		}
	}

	return 0;
}
