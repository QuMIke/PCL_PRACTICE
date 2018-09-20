#ifndef MY_KINNECT_AGENT_H
#define MY_KINNECT_AGENT_H
// Producer thread class
template <typename PointT>
class KinnectAgent
{
  private:
    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    grabberCallBack (const typename PointCloud<PointT>::ConstPtr& cloud)
    {
      if (!buf_.pushBack (cloud))
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
      OpenNIGrabber* grabber = new OpenNIGrabber ();
      grabber->getDevice ()->setDepthOutputFormat (depth_mode_);

      Grabber* interface = grabber;
      boost::function<void (const typename PointCloud<PointT>::ConstPtr&)> f = boost::bind (&Producer::grabberCallBack, this, _1);
      interface->registerCallback (f);
      interface->start ();

      while (true)
      {
        if (is_done)
          break;
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }
      interface->stop ();
    }

  public:
    KinnectProducer (PCDBuffer<PointT> &buf, openni_wrapper::OpenNIDevice::DepthMode depth_mode)
      : buf_ (buf),
        depth_mode_ (depth_mode)
    {
      thread_.reset (new boost::thread (boost::bind (&Producer::grabAndSend, this)));
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      thread_->join ();
      boost::mutex::scoped_lock io_lock (io_mutex);
      print_highlight ("Producer done.\n");
    }

  private:
    PCLBuffer<PointT> &buf_;
    openni_wrapper::OpenNIDevice::DepthMode depth_mode_;
    boost::shared_ptr<boost::thread> thread_;
};
#endif
