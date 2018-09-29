#include "stdafx.h"
#include "cloud_grabber.h"

template<typename PointT> void
	cloudGrabber<PointT>::run()
	{
		std::string mouseMsg3D("Mouse coordinates in PCL Visualizer");
		std::string keyMsg3D("Key event for PCL Visualizer");
		cloud_viewer_.registerMouseCallback(&cloudGrabber<PointT>::mouse_callback, *this, (void*)(&mouseMsg3D));
		cloud_viewer_.registerKeyboardCallback(&cloudGrabber<PointT>::keyboard_callback, *this, (void*)(&keyMsg3D));
		boost::function<void(const CloudConstPtr&) > cloud_cb = boost::bind(&cloudGrabber<PointT>::cloud_callback, this, _1);
		boost::signals2::connection cloud_connection = grabber_.registerCallback(cloud_cb);

		boost::signals2::connection image_connection;
		if (grabber_.providesCallback<void(const boost::shared_ptr<openni_wrapper::Image>&)>())
		{
			string mouseMsg2D("Mouse coordinates in image viewer");
			string keyMsg2D("Key event for image viewer");
			image_viewer_.registerMouseCallback(&cloudGrabber<PointT>::mouse_callback, *this, (void*)(&mouseMsg2D));
			image_viewer_.registerKeyboardCallback(&cloudGrabber<PointT>::keyboard_callback, *this, (void*)(&keyMsg2D));
			boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind(&cloudGrabber<PointT>::rgb_image_callback, this, _1);
			image_connection = grabber_.registerCallback(image_cb);
		}

		grabber_.start();

		while (!cloud_viewer_.wasStopped(1))
		{
			if (cloud_)
			{
				FPS_CALC("drawing cloud");
				//the call to get() sets the cloud_ to null;
				cloud_viewer_.showCloud(getLatestCloud());
			}

			if (image_)
			{
				boost::shared_ptr<openni_wrapper::Image> image = getLatestImage();

				if (image->getEncoding() == openni_wrapper::Image::RGB)
				{
					image_viewer_.showRGBImage(image->getMetaData().Data(), image->getWidth(), image->getHeight());
				}
				else
				{
					if (rgb_data_size < image->getWidth() * image->getHeight())
					{
						rgb_data_size = image->getWidth() * image->getHeight();
						rgb_data = new unsigned char[rgb_data_size * 3];
					}
					image->fillRGB(image->getWidth(), image->getHeight(), rgb_data);
					image_viewer_.showRGBImage(rgb_data, image->getWidth(), image->getHeight());
				}
			}        
		}
	}

template<typename PointT> void
	cloudGrabber<PointT>::stop()
	{	
		grabber_.stop();
		cloud_connection.disconnect();
		image_connection.disconnect(); 
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

template <typename PointT> typename pcl::PointCloud<PointT>::ConstPtr 
	cloudGrabber<PointT>::getLatestCloud()
	{
		boost::mutex::scoped_lock lock(cloud_mutex_);
		typename pcl::PointCloud<PointT>::ConstPtr temp_cloud;
		temp_cloud.swap(cloud_); 
		return (temp_cloud);
	}

template <typename PointT> boost::shared_ptr<pcl::io::Image>
	cloudGrabber<PointT>::getLatestRGBImg()
	{	
		boost::mutex::scoped_lock lock(image_mutex_);
		boost::shared_ptr<pcl::io::Image> temp_image;
		temp_image.swap(rgb_image_);
		return (temp_image);
	}

template <typename PointT> boost::shared_ptr<pcl::io::DepthImage>
	cloudGrabber<PointT>::getLatestDepthImg()
	{
		boost::mutex::scoped_lock lock(depth_mutex_);
		boost::shared_ptr<pcl::io::DepthImage> temp_depth;
		temp_image.swap(depth_image_);
		return (temp_depth);
	}

template <typename PointT> void 
	cloudGrabber<PointT>::saveCLoud2Disk()
	{
		boost::mutex::scoped_lock cloud_lock(cloud_mutex_);
		pcl::console::print_info("Writing remaining clouds in the buffer to disk...\n");
		std::stringstream ss;
		std::string time = boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time());
		ss << "frame-" << time << ".pcd";
		writer_.writeBinaryCompressed(ss.str(), *cloud_);
	}

template <typename PointT> void
	cloudGrabber<PointT>::saveRGBImg2Disk(const std::string& file_name)
	{
		boost::mutex::scoped_lock cloud_lock(rgb_mutex_);
		pcl::console::print_info("Writing remaining RGB img in the buffer to disk...\n");
		int width = rgb_image_->getWidth();
		int height = rgb_image_->getHeight();
		const unsigned char* rgb_data = rgb_image_->getData();
		pcl::io::saveRgbPNGFile(file_name, rgb_data, width, height);
	}

template <typename PointT> void
	cloudGrabber<PointT>::saveDepthImg2Disk(const std::string& file_name)
	{
		boost::mutex::scoped_lock cloud_lock(depth_mutex_);
		pcl::console::print_info("Writing remaining depth img in the buffer to disk...\n");
		int width = rgb_image_->getWidth();
		int height = rgb_image_->getHeight();
		const unsigned char* depth_data = depth_image_->getData();
		pcl::io::saveShortPNGFile(file_name, depth_data, width, height, 1);

	}

template<typename PointT> void 
	cloudGrabber<PointT>::cloud_callback(typename const pcl::PointCloud<PointT>::ConstPtr cloud)
	{
		FPS_CALC("cloud callback");
		boost::mutex::scoped_lock lock(cloud_mutex_);
		cloud_ = cloud;
	}

template<typename PointT> void 
	cloudGrabber<PointT>::rgb_image_callback(const boost::shared_ptr<pcl::io::Image>& rgb)
	{
		FPS_CALC("rgb image callback");
		boost::mutex::scoped_lock lock(rgb_mutex_);
		rgb_image_ = rgb;
	}

template<typename PointT> void
	cloudGrabber<PointT>::depth_image_callback(const boost::shared_ptr<pcl::io::DepthImage>& depth)
	{
		FPS_CALC("depth image callback");
		boost::mutex::scoped_lock lock(depth_mutex_);
		depth_image_ = depth;
	}

template<typename PointT> void
	cloudGrabber<PointT>::mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void* cookie)
	{
		std::string* message = (string*)cookie;
		if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
		{
			std::cout << (*message) << " :: " << mouse_event.getX() << " , " << mouse_event.getY() << std::endl;
		}	
	}

template<typename PointT> void
	cloudGrabber<PointT>::keyboard_callback(const pcl::visualization::KeyboardEvent& event, void* cookie)
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
