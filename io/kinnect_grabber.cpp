#include "stdafx.h"
#include "kinnect_grabber.h"

template<typename PointT> void
	kinnectGrabber<PointT>::run(void)
	{	
		cloud_viewer_->registerMouseCallback(&kinnectGrabber::mouse_callback, *this);
		cloud_viewer_->registerKeyboardCallback(&kinnectGrabber::keyboard_callback, *this);
		cloud_viewer_->setCameraFieldOfView(1.02259994f);
		boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&) > cloud_cb = boost::bind(&kinnectGrabber::cloud_callback, this, _1);
		boost::signals2::connection cloud_connection = grabber_.registerCallback(cloud_cb);

		boost::signals2::connection rgb_image_connection;
		if (grabber_.providesCallback<void(const boost::shared_ptr<pcl::io::openni2::Image>&)>())
		{
			std::string mouseMsg2D("Mouse coordinates in rgb image viewer");
			std::string keyMsg2D("Key event for rgb image viewer");
			rgb_viewer_.reset(new pcl::visualization::ImageViewer("PCL OpenNI rgb image"));
			rgb_viewer_->registerMouseCallback(&kinnectGrabber::mouse_callback, *this, (void*)(&mouseMsg2D));
			rgb_viewer_->registerKeyboardCallback(&kinnectGrabber::keyboard_callback, *this, (void*)(&keyMsg2D));
			boost::function<void(const boost::shared_ptr<pcl::io::openni2::Image>&) > rgb_img_cb = boost::bind(&kinnectGrabber::rgb_image_callback, this, _1);
			rgb_image_connection = grabber_.registerCallback(rgb_img_cb);
		}

		boost::signals2::connection depth_image_connection;
		if (grabber_.providesCallback<void(const boost::shared_ptr<pcl::io::openni2::DepthImage>&)>())
		{
			std::string mouseMsg2D("Mouse coordinates in depth image viewer");
			std::string keyMsg2D("Key event for depth image viewer");
			rgb_viewer_.reset(new pcl::visualization::ImageViewer("PCL OpenNI depth image"));
			depth_viewer_->registerMouseCallback(&kinnectGrabber<PointT>::mouse_callback, *this, (void*)(&mouseMsg2D));
			depth_viewer_->registerKeyboardCallback(&kinnectGrabber<PointT>::keyboard_callback, *this, (void*)(&keyMsg2D));
			boost::function<void(const boost::shared_ptr<pcl::io::openni2::DepthImage>&) > depth_img_cb = boost::bind(&kinnectGrabber<PointT>::depth_image_callback, this, _1);
			depth_image_connection = grabber_.registerCallback(depth_img_cb);
		}

		bool image_init = false, depth_init = false, cloud_init = false;
		grabber_.start();

		while (!cloud_viewer_->wasStopped() && !rgb_viewer_->wasStopped())
		{
			pcl::PointCloud<PointT>::ConstPtr cloud;
			cloud_viewer_->spinOnce();
			if (cloud_mutex_.try_lock())
			{
				cloud_.swap(cloud);
				cloud_mutex_.unlock();
			}
			if (cloud && if_viz_cloud)
			{
				FPS_CALC("drawing cloud");
				if (!cloud_init)
				{
					cloud_viewer_->setPosition(0, 0);
					cloud_viewer_->setSize(cloud->width, cloud->height);
					cloud_init = !cloud_init;
				}
				//the call to get() sets the cloud_ to null;
				if (!cloud_viewer_->updatePointCloud(cloud, "OpenNICloud"))
				{
					cloud_viewer_->addPointCloud(cloud, "OpenNICloud");
					cloud_viewer_->resetCameraViewpoint("OpenNICloud");
					cloud_viewer_->setCameraPosition(
						0, 0, 0,		// Position
						0, 0, 1,		// Viewpoint
						0, -1, 0);	// Up
				}
			}

			boost::shared_ptr<pcl::io::openni2::Image> image;
			if (rgb_mutex_.try_lock())
			{
				rgb_image_.swap(image);
				rgb_mutex_.unlock();
			}
			if (image && if_viz_rgb)
			{
				if (!image_init && cloud && cloud->width != 0)
				{
					rgb_viewer_->setPosition(cloud->width, 0);
					rgb_viewer_->setSize(cloud->width, cloud->height);
					image_init = !image_init;
				}

				if (image->getEncoding() == pcl::io::openni2::Image::RGB)
					rgb_viewer_->addRGBImage((const unsigned char*)image->getData(), image->getWidth(), image->getHeight());
				rgb_viewer_->spinOnce();
			}        

			boost::shared_ptr<pcl::io::openni2::DepthImage> depth;
			if (depth_mutex_.try_lock())
			{
				depth_image_.swap(depth);
				depth_mutex_.unlock();
			}
			if (depth_image_ && if_viz_depth)
			{
				if (!depth_init && cloud && cloud->width != 0)
				{
					depth_viewer_->setPosition(cloud->width, 0);
					depth_viewer_->setSize(cloud->width, cloud->height);
					depth_init = !depth_init;
				}
				depth_viewer_->addMonoImage((const unsigned char*)depth->getData(), depth->getWidth(), depth->getHeight());
				depth_viewer_->spinOnce();
			}
		}		

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
		boost::mutex::scoped_lock lock(rgb_mutex_);
		boost::shared_ptr<pcl::io::openni2::Image> temp_image;
		temp_image.swap(rgb_image_);
		return (temp_image);
	}

template <typename PointT> boost::shared_ptr<pcl::io::DepthImage>
	kinnectGrabber<PointT>::getLatestDepthImg(void)
	{
		boost::mutex::scoped_lock lock(depth_mutex_);
		boost::shared_ptr<pcl::io::DepthImage> temp_depth;
		temp_depth.swap(depth_image_);
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
		if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
		{
			std::cout << "left button pressed @ " << mouse_event.getX() << " , " << mouse_event.getY() << std::endl;
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
