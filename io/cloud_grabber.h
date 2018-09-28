#pragma once
#ifndef  MY_POINT_CLOUD_GRABBER_H
#define MY_POINT_CLOUD_GRABBER_H

#include <sstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>

template <typename PointT>
class cloudGrabber 
{
public:
	inline bool setSaveFlag(bool flag)
	{
		if_save_cloud = flag;
	};
	inline bool setVizCloudFlag(bool flag)
	{
		if_viz_depth = flag;
	};
	inline bool setVizRGBFlag(bool flag)
	{
		if_viz_rgb = flag;
	};
	inline bool setVizDepthFlag(bool flag)
	{
		if_viz_cloud = flag;
	};
public:
	void run();
	void stop();
	static cloudGrabber* getInstance();
	typename pcl::PointCloud<PointT>::Ptr getLatestCloud();	
	boost::shared_ptr<pcl::io::Image> getLatestRGBImg();
	boost::shared_ptr<pcl::io::DepthImage> getLatestDepthImg();	
	void saveCLoud2Disk(typename pcl::PointCloud<PointT>::Ptr cloud);
	void saveRGBImg2Disk(typename pcl::PointCloud<PointT>::Ptr cloud);
	void saveDepthImg2Disk(typename pcl::PointCloud<PointT>::Ptr cloud);	
private:
	cloudGrabber();
	cloudGrabber(const cloudGrabber&);
	cloudGrabber& operator=(const cloudGrabber&);
	void cloud_callback(typename const pcl::PointCloud<PointT>::ConstPtr cloud);
	void rgb_image_callback(const boost::shared_ptr<pcl::io::Image>& rgb_image);
	void depth_image_callback(const boost::shared_ptr<pcl::io::DepthImage>& depth_image);
	void mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void* cookie);
	void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void* nothing);
private:
	boost::mutex rgb_mutex_;
	boost::mutex cloud_mutex_;
	boost::mutex depth_mutex_;
	static cloudGrabber* m_instance;
private:
	bool if_viz_rgb = true;
	bool if_viz_depth = true;
	bool if_viz_cloud = true;
	bool if_save_rgb = false;
	bool if_save_depth = false;
	bool if_save_cloud = false;
private:
	pcl::PCDWriter writer_;
	pcl::io::OpenNI2Grabber grabber_;
	pcl::visualization::ImageViewer rgb_viewer_;
	pcl::visualization::ImageViewer depth_viewer_;
	pcl::visualization::CloudViewer cloud_viewer_;
	typename pcl::PointCloud<PointT>::Ptr cloud_ = nullptr;
	boost::shared_ptr<pcl::io::Image> rgb_image_ = nullptr;
	boost::shared_ptr<pcl::io::DepthImage> depth_image_ = nullptr;
};

template<typename PointT> cloudGrabber<PointT>* cloudGrabber<PointT>::m_instance= nullptr;

#endif // ! MY_POINT_CLOUD_GRABBER_H
