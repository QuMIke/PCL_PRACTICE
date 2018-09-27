#include "stdafx.h"
#include "cloud_grabber.h"

template <typename PointT> void 
cloudGrabber<PointT>::write2Disk(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{

}

template<typename PointT> void
cloudGrabber<PointT>::visualCallback(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{

}

template<typename PointT> void 
cloudGrabber<PointT>::processCallback(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{

}

template<typename PointT> void
cloudGrabber<PointT>::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{

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