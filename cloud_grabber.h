#pragma once
#ifndef  MY_POINT_CLOUD_GRABBER_H
#define MY_POINT_CLOUD_GRABBER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/print.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

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
	void write2Disk(typename pcl::PointCloud<PointT>::ConstPtr cloud);
	void visualCallback(typename pcl::PointCloud<PointT>::ConstPtr cloud);
	void processCallback(typename pcl::PointCloud<PointT>::ConstPtr cloud);
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
	pcl::io::OpenNI2Grabber grabber_;
	typename pcl::PointCloud<PointT>::Ptr cloud_;
	boost::shared_ptr<pcl::visualization::CloudViewer> viewer_;	
};

template<typename PointT> cloudGrabber<PointT>* cloudGrabber<PointT>::m_instance= nullptr;

#endif // ! MY_POINT_CLOUD_GRABBER_H
