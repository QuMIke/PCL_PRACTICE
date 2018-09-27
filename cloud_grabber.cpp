#include "stdafx.h"
#include "cloud_grabber.h"

template <typename PointT> typename pcl::PointCloud<PointT>::Ptr 
cloudGrabber<PointT>::getSourceCloud()
{
	return cloud_;
}

template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
cloudGrabber<PointT>::getFilteredCloud()
{
	return cloud_filtered;
}

template <typename PointT> void 
cloudGrabber<PointT>::write2Disk(typename pcl::PointCloud<PointT>::Ptr cloud)
{
	boost::mutex::scoped_lock cloud_lock(bmutex_);
	pcl::console::print_info("Writing remaining clouds in the buffer to disk...\n");
	std::stringstream ss;
	std::string time = boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time());
	ss << "frame-" << time << ".pcd";
	writer_.writeBinaryCompressed(ss.str(), *cloud);
}

template <typename PointT> void
cloudGrabber<PointT>::write2Disk(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
	boost::mutex::scoped_lock cloud_lock(bmutex_);
	pcl::console::print_info("Writing remaining clouds in the buffer to disk...\n");
	std::stringstream ss;
	std::string time = boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time());
	ss << "frame-" << time << ".pcd";
	writer_.writeBinaryCompressed(ss.str(), *cloud);
}

template<typename PointT> void
cloudGrabber<PointT>::visualCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
	boost::mutex::scoped_lock cloud_clock(bmutex_);
	pcl::console::print_info("Visualize the clouds...\n");
	while (if_viz && !viewer_->wasStopped())
	{
		viewer_->showCloud(cloud);
	}
}

template<typename PointT> void
cloudGrabber<PointT>::visualCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
	boost::mutex::scoped_lock cloud_clock(bmutex_);
	pcl::console::print_info("Visualize the clouds...\n");
	while (if_viz && !viewer_->wasStopped())
	{
		viewer_->showCloud(cloud);
	}
}

template<typename PointT> void 
cloudGrabber<PointT>::filtCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
	boost::mutex::scoped_lock cloud_clock(bmutex_);
	pcl::console::print_info("Process clouds...\n");
	*cloud_ = *cloud;
	sor.setInputCloud(cloud_);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
}

template<typename PointT> void
cloudGrabber<PointT>::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeyCode())
		std::cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << " )  was";
	else
		std::cout << "the special key \'" << event.getKeySym() << "\' was ";
	if (event.keyDown())
		std::cout << " pressed." << std::endl;
	else
		std::cout << " released." << std::endl;
}

template<typename PointT> void
cloudGrabber<PointT>::run()
{

}

template<typename PointT> void
cloudGrabber<PointT>::stop()
{

}

template<typename PointT> typename static cloudGrabber<PointT>*
cloudGrabber<PointT>::getInstance()
{
	if (m_instance == nullptr)
	{
		m_instance = new cloudGrabber();
	}
	return m_instance;
}
