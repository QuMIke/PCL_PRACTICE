#include "stdafx.h"
#include "kinnect_grabber.h"

template<typename PointT> void
	kinnectGrabber<PointT>::run(void)
	{
		std::string mouseMsg3D("Mouse coordinates in PCL Visualizer");
		std::string keyMsg3D("Key event for PCL Visualizer");
		cloud_viewer_.registerMouseCallback(&kinnectGrabber<PointT>::mouse_callback, *this, (void*)(&mouseMsg3D));
		cloud_viewer_.registerKeyboardCallback(&kinnectGrabber<PointT>::keyboard_callback, *this, (void*)(&keyMsg3D));
		boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&) > cloud_cb = boost::bind(&kinnectGrabber<PointT>::cloud_callback, this, _1);
		boost::signals2::connection cloud_connection = grabber_.registerCallback(cloud_cb);

		boost::signals2::connection rgb_image_connection;
		if (grabber_.providesCallback<void(const boost::shared_ptr<pcl::io::Image>&)>())
		{
			std::string mouseMsg2D("Mouse coordinates in rgb image viewer");
			std::string keyMsg2D("Key event for rgb image viewer");
			rgb_viewer_.registerMouseCallback(&kinnectGrabber<PointT>::mouse_callback, *this, (void*)(&mouseMsg2D));
			rgb_viewer_.registerKeyboardCallback(&kinnectGrabber<PointT>::keyboard_callback, *this, (void*)(&keyMsg2D));
			boost::function<void(const boost::shared_ptr<pcl::io::Image>&) > rgb_img_cb = boost::bind(&kinnectGrabber<PointT>::rgb_image_callback, this, _1);
			rgb_image_connection = grabber_.registerCallback(rgb_img_cb);
		}

		boost::signals2::connection depth_image_connection;
		if (grabber_.providesCallback<void(const boost::shared_ptr<pcl::io::DepthImage>&)>())
		{
			std::string mouseMsg2D("Mouse coordinates in depth image viewer");
			std::string keyMsg2D("Key event for depth image viewer");
			depth_viewer_.registerMouseCallback(&kinnectGrabber<PointT>::mouse_callback, *this, (void*)(&mouseMsg2D));
			depth_viewer_.registerKeyboardCallback(&kinnectGrabber<PointT>::keyboard_callback, *this, (void*)(&keyMsg2D));
			boost::function<void(const boost::shared_ptr<pcl::io::DepthImage>&) > depth_img_cb = boost::bind(&kinnectGrabber<PointT>::depth_image_callback, this, _1);
			depth_image_connection = grabber_.registerCallback(depth_img_cb);
		}

		grabber_.start();

		while (!cloud_viewer_.wasStopped(1) && !rgb_viewer_.wasStopped() && !depth_viewer_.wasStopped())
		{
			if (cloud_ && if_viz_cloud)
			{
				FPS_CALC("drawing cloud");
				//the call to get() sets the cloud_ to null;
				cloud_viewer_.showCloud(getLatestCloud());
			}

			if (rgb_image_ && if_viz_rgb)
			{
				boost::shared_ptr<pcl::io::Image> rgb_image = getLatestRGBImg();
				rgb_viewer_.showRGBImage(rgb_image->getData(), rgb_image->getWidth(), rgb_image->getHeight());
			}        

			if (depth_image_ && if_viz_depth)
			{
				boost::shared_ptr<pcl::io::DepthImage> depth_image = getLatestDepthImg();
				depth_viewer_.showRGBImage(depth_image->getData(), depth_image->getWidth(), depth_image->getHeight());
			}
		}
	}

template<typename PointT> void
	kinnectGrabber<PointT>::stop(void)
	{	
		grabber_.stop();
		cloud_connection.disconnect();
		rgb_image_connection.disconnect(); 
		depth_image_connection.disconnect();
	}

template <typename PointT> typename pcl::PointCloud<PointT>::ConstPtr 
	kinnectGrabber<PointT>::getLatestCloud(void)
	{
		boost::mutex::scoped_lock lock(cloud_mutex_);
		typename pcl::PointCloud<PointT>::ConstPtr temp_cloud;
		temp_cloud.swap(cloud_); 
		return (temp_cloud);
	}

template <typename PointT> boost::shared_ptr<pcl::io::Image>
	kinnectGrabber<PointT>::getLatestRGBImg(void)
	{	
		boost::mutex::scoped_lock lock(image_mutex_);
		boost::shared_ptr<pcl::io::Image> temp_image;
		temp_image.swap(rgb_image_);
		return (temp_image);
	}

template <typename PointT> boost::shared_ptr<pcl::io::DepthImage>
	kinnectGrabber<PointT>::getLatestDepthImg(void)
	{
		boost::mutex::scoped_lock lock(depth_mutex_);
		boost::shared_ptr<pcl::io::DepthImage> temp_depth;
		temp_image.swap(depth_image_);
		return (temp_depth);
	}

template <typename PointT> void 
	kinnectGrabber<PointT>::saveCLoud2Disk(void)
	{
		boost::mutex::scoped_lock cloud_lock(cloud_mutex_);
		pcl::console::print_info("Writing remaining clouds in the buffer to disk...\n");
		std::stringstream ss;
		std::string time = boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time());
		ss << "frame-" << time << ".pcd";
		writer_.writeBinaryCompressed(ss.str(), *cloud_);
	}

template <typename PointT> void
	kinnectGrabber<PointT>::saveRGBImg2Disk(const std::string& file_name)
	{
		boost::mutex::scoped_lock cloud_lock(rgb_mutex_);
		pcl::console::print_info("Writing remaining RGB img in the buffer to disk...\n");
		int width = rgb_image_->getWidth();
		int height = rgb_image_->getHeight();
		const unsigned char* rgb_data = rgb_image_->getData();
		pcl::io::saveRgbPNGFile(file_name, rgb_data, width, height);
	}

template <typename PointT> void
	kinnectGrabber<PointT>::saveDepthImg2Disk(const std::string& file_name)
	{
		boost::mutex::scoped_lock cloud_lock(depth_mutex_);
		pcl::console::print_info("Writing remaining depth img in the buffer to disk...\n");
		int width = rgb_image_->getWidth();
		int height = rgb_image_->getHeight();
		const unsigned char* depth_data = depth_image_->getData();
		pcl::io::saveShortPNGFile(file_name, depth_data, width, height, 1);

	}

template<typename PointT> void 
	kinnectGrabber<PointT>::cloud_callback(typename const pcl::PointCloud<PointT>::ConstPtr cloud)
	{
		FPS_CALC("cloud callback");
		boost::mutex::scoped_lock lock(cloud_mutex_);
		cloud_ = cloud;
	}

template<typename PointT> void 
	kinnectGrabber<PointT>::rgb_image_callback(const boost::shared_ptr<pcl::io::Image>& rgb)
	{
		FPS_CALC("rgb image callback");
		boost::mutex::scoped_lock lock(rgb_mutex_);
		rgb_image_ = rgb;
	}

template<typename PointT> void
	kinnectGrabber<PointT>::depth_image_callback(const boost::shared_ptr<pcl::io::DepthImage>& depth)
	{
		FPS_CALC("depth image callback");
		boost::mutex::scoped_lock lock(depth_mutex_);
		depth_image_ = depth;
	}

template<typename PointT> void
	kinnectGrabber<PointT>::mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void* cookie)
	{
		std::string* message = (string*)cookie;
		if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
		{
			std::cout << (*message) << " :: " << mouse_event.getX() << " , " << mouse_event.getY() << std::endl;
		}	
	}

template<typename PointT> void
	kinnectGrabber<PointT>::keyboard_callback(const pcl::visualization::KeyboardEvent& event, void* cookie)
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

