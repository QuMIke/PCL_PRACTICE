#include "stdafx.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
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
	}

	boost::shared_ptr<pcl::io::openni2::Image> rgb_ptr = nullptr;
	rgb_ptr = v.getLatestImage();
	if (rgb_ptr == nullptr)
	{
		std::cout << "Get image failed!" << std::endl;
	}
	else
	{
		cv::Mat mImageRGB = cv::Mat(rgb_ptr->getHeight(), rgb_ptr->getWidth(), CV_8UC3, (void*)rgb_ptr->getData());
		// transform RGB to BGR
		cv::Mat cImageBGR;
		cv::cvtColor(mImageRGB, cImageBGR, CV_RGB2BGR);
		// display color image
		cv::imshow("Color Image", cImageBGR);
	}

	boost::shared_ptr<pcl::io::openni2::DepthImage> depth_ptr = nullptr;
	depth_ptr = v.getLatestDepth();
	if (depth_ptr == nullptr)
	{
		std::cout << "Get depth failed!" << std::endl;
	}
	else
	{
		cv::Mat mImageDepth = cv::Mat(depth_ptr->getHeight(), depth_ptr->getWidth(), CV_16UC1, (void*)depth_ptr->getData());
		double maxVal, minVal;
		cv::minMaxIdx(mImageDepth, &minVal, &maxVal);
		cv::Mat mScaledDepth;
		mImageDepth.convertTo(mScaledDepth, CV_8U, 255./maxVal);
		// display depth image
		cv::imshow("Depth Image", mScaledDepth);
	}

	cv::waitKey(0);
    	return 0;
}
