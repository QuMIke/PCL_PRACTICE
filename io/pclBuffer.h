 /* 
  *  Reference:pcl/io/tools/openni_pcd_recorder.cpp
  *  All rights reserved.
  */
#ifndef MY_PCL_BUFFER_H
#define MY_PCL_BUFFER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <boost/thread/condition.hpp>
#include <boost/circular_buffer.hpp>
#include <csignal>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

//////////////////////////////////////////////////////////////////////////////////////////
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
    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    grabberCallBack (const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
      if (!pushBack (cloud))
      {
        {
          boost::mutex::scoped_lock io_lock (io_mutex);
          print_warn ("Warning! Buffer was full, overwriting data!\n");
        }
      }
      FPS_CALC ("cloud callback.", buf_);
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    grabAndSend ()
    {
      auto grabber = new OpenNI2Grabber ();
      boost::function<void (const typename pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&PCLBuffers<PointT>::grabberCallBack, this, _1);
      grabber->registerCallback (f);
      grabber->start ();

      while (true)
      {
        if (is_done)
          break;
        boost::this_thread::sleep (boost::posix_time::second(1));
      }
      interface->stop ();
    }

  public:
    Producer ()
    {
      thread_.reset (new boost::thread (boost::bind (&PCLBuffer<PointT>::grabAndSend, this)));
    }
    
  private:
    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    writeToDisk (const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
      stringstream ss;
      std::string time = boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());
      ss << "frame-" << time << ".pcd";
      writer_.writeBinaryCompressed (ss.str (), *cloud);
      FPS_CALC ("cloud write.", buf_);
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Consumer thread function
    void 
    receiveAndProcess ()
    {
      while (true)
      {
        if (is_done)
          break;
        writeToDisk (getFront ());
      }

      {
        boost::mutex::scoped_lock io_lock (io_mutex);
        print_info ("Writing remaining %ld clouds in the buffer to disk...\n", getSize ());
      }
      while (isEmpty ())
        writeToDisk (getFront ());
    }

  public:
    Consumer ()
    {
      thread_.reset (new boost::thread (boost::bind (&PCLBuffer<PointT>::receiveAndProcess, this)));
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      thread_c->join ();
      thread_p->join ();
      boost::mutex::scoped_lock io_lock (io_mutex);
      print_highlight ("PCLBuffer done.\n");
    }

  private:
    bool is_done = false;
    boost::mutex bmutex_; 
    boost::mutex io_mutex;
    boost::condition_variable buff_empty_;
    boost::circular_buffer<typename PointCloud<PointT>::ConstPtr> buffer_;
    PCDBuffer<PointT> &buf_;
    boost::shared_ptr<boost::thread> thread_p;
    boost::shared_ptr<boost::thread> thread_c;
    PCDWriter writer_;
};

#endif
