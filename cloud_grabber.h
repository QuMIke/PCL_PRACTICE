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
		if_vis = flag;
	};
	inline bool setVizFlag(bool flag)
	{
		if_viz = flag;
	};
public:
	typename pcl::PointCloud<PointT>::Ptr getSourceCloud();
	typename pcl::PointCloud<PointT>::Ptr getFilteredCloud();
	void write2Disk(typename pcl::PointCloud<PointT>::Ptr cloud);
	void write2Disk(typename pcl::PointCloud<PointT>::ConstPtr cloud);
	void visualCloud(typename pcl::PointCloud<PointT>::Ptr cloud);
	void visualCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);
	void filtCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);
	void run();
	void stop();
	static cloudGrabber* getInstance();
private:
	cloudGrabber();
	cloudGrabber(const cloudGrabber&);
	cloudGrabber& operator=(const cloudGrabber&);
private:
	bool if_viz = true;
	bool if_save = false;
	static cloudGrabber* m_instance;
	boost::mutex bmutex_;
	pcl::PCDWriter writer_;
	pcl::io::OpenNI2Grabber grabber_;
	typename pcl::PointCloud<PointT>::Ptr cloud_ = nullptr;
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered = nullptr;
	typename pcl::StatisticalOutlierRemoval<PointT> sor;
	boost::shared_ptr<pcl::visualization::CloudViewer> viewer_src_cloud;
	boost::shared_ptr<pcl::visualization::CloudViewer> viewer_filtered_cloud;
};

template<typename PointT> cloudGrabber<PointT>* cloudGrabber<PointT>::m_instance= nullptr;

#endif // ! MY_POINT_CLOUD_GRABBER_H