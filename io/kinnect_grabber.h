#pragma once
#ifndef  MY_POINT_CLOUD_GRABBER_H
#define MY_POINT_CLOUD_GRABBER_H

#include "basic_func.h"
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/console/print.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>


template <typename PointT>
class kinnectGrabber 
{
public:
	kinnectGrabber(pcl::io::OpenNI2Grabber& grabber)
		: grabber_(grabber)
		, cloud_viewer_(new pcl::visualization::PCLVisualizer("PCL OpenNI2 cloud"))
		, rgb_viewer_()
		, depth_viewer_()
	{
	};
	inline void setSaveFlag(bool flag)
	{
		if_save_cloud = flag;
	};
	inline void setVizCloudFlag(bool flag)
	{
		if_viz_depth = flag;
	};
	inline void setVizRGBFlag(bool flag)
	{
		if_viz_rgb = flag;
	};
	inline void setVizDepthFlag(bool flag)
	{
		if_viz_cloud = flag;
	};
public:
	void run(void);
	typename pcl::PointCloud<PointT>::ConstPtr getLatestCloud(void);	
	boost::shared_ptr<pcl::io::openni2::Image> getLatestRGBImg(void);
	boost::shared_ptr<pcl::io::openni2::DepthImage> getLatestDepthImg(void);	
	void saveCLoud2Disk();
	void saveRGBImg2Disk(const std::string& file_name);
	void saveDepthImg2Disk(const std::string& file_name);
private:
	kinnectGrabber(const kinnectGrabber&);
	kinnectGrabber& operator=(const kinnectGrabber&);
	void cloud_callback(typename const pcl::PointCloud<PointT>::ConstPtr cloud);
	void rgb_image_callback(const boost::shared_ptr<pcl::io::Image>& rgb);
	void depth_image_callback(const boost::shared_ptr<pcl::io::DepthImage>& depth);
	void mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void* cookie);
	void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void* cookie);
private:
	pcl::PCDWriter writer_;
	pcl::io::OpenNI2Grabber& grabber_;
private:
	boost::mutex cloud_mutex_;
	bool if_save_cloud = false;
	bool if_viz_cloud = true;
	typename pcl::PointCloud<PointT>::ConstPtr cloud_ = nullptr;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
private:
	boost::mutex rgb_mutex_;
	bool if_viz_rgb = true;
	bool if_save_rgb = false;
	boost::shared_ptr<pcl::io::openni2::Image> rgb_image_ = nullptr;
	boost::shared_ptr<pcl::visualization::ImageViewer> rgb_viewer_;
private:
	boost::mutex depth_mutex_;	
	bool if_viz_depth = true;
	bool if_save_depth = false;
	boost::shared_ptr<pcl::io::openni2::DepthImage> depth_image_ = nullptr;
	boost::shared_ptr<pcl::visualization::ImageViewer> depth_viewer_;
};


#endif // ! MY_POINT_CLOUD_GRABBER_H
