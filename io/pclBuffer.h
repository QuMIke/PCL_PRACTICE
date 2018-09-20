 /* 
  *  Reference:pcl/io/tools/openni_pcd_recorder.cpp
  *  All rights reserved.
  */
#ifndef MY_PCL_BUFFER_H
#define MY_PCL_BUFFER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/condition.hpp>
#include <boost/circular_buffer.hpp>
#include <csignal>
#include <limits>

template <typename PointT>
class PCLBuffer
{
  public:
    PCLBuffer () {}

    bool 
    pushBack (typename pcl::PointCloud<PointT>::ConstPtr); // thread-save wrapper for push_back() method of ciruclar_buffer

    typename pcl::PointCloud<PointT>::ConstPtr 
    getFront (); // thread-save wrapper for front() method of ciruclar_buffer

    inline bool 
    isFull ()
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
      return (buffer_.full ());
    }

    inline bool
    isEmpty ()
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
      return (buffer_.empty ());
    }

    inline int 
    getSize ()
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
      return (int (buffer_.size ()));
    }

    inline int 
    getCapacity ()
    {
      return (int (buffer_.capacity ()));
    }

    inline void 
    setCapacity (int buff_size)
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
      buffer_.set_capacity (buff_size);
    }

  private:
    PCLBuffer (const PCLBuffer&); // Disabled copy constructor
    PCLBuffer& operator = (const PCLBuffer&); // Disabled assignment operator

    boost::mutex bmutex_;
    boost::condition_variable buff_empty_;
    boost::circular_buffer<typename pcl::PointCloud<PointT>::ConstPtr> buffer_;
};

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool 
PCLBuffer<PointT>::pushBack (typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
  bool retVal = false;
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
    if (!buffer_.full ())
      retVal = true;
    buffer_.push_back (cloud);
  }
  buff_empty_.notify_one ();
  return (retVal);
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename PointCloud<PointT>::ConstPtr 
PCLBuffer<PointT>::getFront ()
{
  typename PointCloud<PointT>::ConstPtr cloud;
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
    while (buffer_.empty ())
    {
      buff_empty_.wait (buff_lock);
    }
    cloud = buffer_.front ();
    buffer_.pop_front ();
  }
  return (cloud);
}

#endif
