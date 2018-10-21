#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni2_grabber.h>
#include <pcl/console/parse.h>
#include <string>
#include "basic_func.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointType>
class KinnectGrabber
{
public:
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::ConstPtr CloudConstPtr;

	KinnectGrabber(pcl::io::OpenNI2Grabber& grabber)
		: grabber_(grabber)
		, rgb_data_(0), rgb_data_size_(0)
	{
	}

	void
		cloud_callback(const CloudConstPtr& cloud)
	{
		FPS_CALC("cloud callback");
		boost::mutex::scoped_lock lock(cloud_mutex_);
		cloud_ = cloud;
	}

	CloudConstPtr
		getLatestCloud()
	{
		//lock while we swap our cloud and reset it.
		boost::mutex::scoped_lock lock(cloud_mutex_);
		CloudConstPtr temp_cloud;
		temp_cloud.swap(cloud_); //here we set cloud_ to null, so that
								 //it is safe to set it again from our
								 //callback
		return (temp_cloud);
	}

	void
		image_callback(const boost::shared_ptr<pcl::io::openni2::Image>& image)
	{
		FPS_CALC("image callback");
		boost::mutex::scoped_lock lock(image_mutex_);
		image_ = image;

		if (image->getEncoding() != pcl::io::openni2::Image::RGB)
		{
			if (rgb_data_size_ < image->getWidth() * image->getHeight())
			{
				if (rgb_data_)
					delete[] rgb_data_;
				rgb_data_size_ = image->getWidth() * image->getHeight();
				rgb_data_ = new unsigned char[rgb_data_size_ * 3];
			}
			image_->fillRGB(image_->getWidth(), image_->getHeight(), rgb_data_);
		}
	}

	boost::shared_ptr<pcl::io::openni2::Image>
	getLatestImage()
	{
		boost::mutex::scoped_lock lock(image_mutex_);
		boost::shared_ptr<pcl::io::openni2::Image> temp_image;
		temp_image.swap(image_);
		return (temp_image);
	}

	void
		depth_callback(const boost::shared_ptr<pcl::io::openni2::DepthImage>& depth)
	{
		FPS_CALC("depth callback");
		boost::mutex::scoped_lock lock(depth_mutex_);
		depth_ = depth;
	}

	boost::shared_ptr<pcl::io::openni2::DepthImage>
		getLatestDepth()
	{
		boost::mutex::scoped_lock lock(depth_mutex_);
		boost::shared_ptr<pcl::io::openni2::DepthImage> temp_depth;
		temp_depth.swap(depth_);
		return (temp_depth);
	}

	void
		run()
	{
		is_start = true;

		boost::function<void(const CloudConstPtr&) > cloud_cb = boost::bind(&KinnectGrabber::cloud_callback, this, _1);
		cloud_connection = grabber_.registerCallback(cloud_cb);

		if (grabber_.providesCallback<void(const boost::shared_ptr<pcl::io::openni2::Image>&)>())
		{
			boost::function<void(const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb = boost::bind(&KinnectGrabber::image_callback, this, _1);
			image_connection = grabber_.registerCallback(image_cb);
		}

		if (grabber_.providesCallback<void(const boost::shared_ptr<pcl::io::openni2::DepthImage>&)>())
		{
			boost::function<void(const boost::shared_ptr<pcl::io::openni2::DepthImage>&) > depth_cb = boost::bind(&KinnectGrabber::depth_callback, this, _1);
			depth_connection = grabber_.registerCallback(depth_cb);
		}

		grabber_.start();
		boost::this_thread::sleep(boost::posix_time::seconds(2)); // important! interval time for io synchronize， if not  you may only get rgb without depth and cloud!!!
 		grabber_.stop();
		cloud_connection.disconnect();
		image_connection.disconnect();
		depth_connection.disconnect();
	}

private:	
	bool is_start = false;
	pcl::io::OpenNI2Grabber& grabber_;
private:
	CloudConstPtr cloud_;
	boost::mutex cloud_mutex_;
	boost::signals2::connection cloud_connection;

	boost::mutex image_mutex_;
	boost::signals2::connection image_connection;
	boost::shared_ptr<pcl::io::openni2::Image> image_;
	unsigned char* rgb_data_;
	unsigned rgb_data_size_;

	boost::mutex depth_mutex_;
	boost::signals2::connection depth_connection;
	boost::shared_ptr<pcl::io::openni2::DepthImage> depth_;
};
