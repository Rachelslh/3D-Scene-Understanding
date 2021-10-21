#include "libfreenect2_grabber.hpp"

  Grabber::Grabber() {}

  bool Grabber::init_device() {

      libfreenect2::Freenect2Device::ColorCameraParams colorParams;
      libfreenect2::Freenect2Device::IrCameraParams IrParams;

      // FIND DEVICE
      if(freenect2.enumerateDevices() == 0)
      {
          std::cout << "no device connected!" << std::endl;
          return false;
      }
      if (serial == "")
      {
          serial = freenect2.getDefaultDeviceSerialNumber();
      }

      pipeline = new libfreenect2::CpuPacketPipeline();

      // OPEN AND CONFIGURE DEVICE
      dev = freenect2.openDevice(serial, pipeline);

      if(dev == 0)
      {
          std::cout << "failure opening device!" << std::endl;
          return false;
      }

      if (enable_rgb)
          types |= libfreenect2::Frame::Color;
      if (enable_depth)
          types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

      listener = new libfreenect2::SyncMultiFrameListener(types);

      dev->setColorFrameListener(listener);
      dev->setIrAndDepthFrameListener(listener);

      // START DEVICE
      if (enable_rgb && enable_depth)
      {
          if (!dev->start())
            return false;
      }
      else
      {
          if (!dev->startStreams(enable_rgb, enable_depth))
            return false;
      }

      colorParams = dev->getColorCameraParams();
      IrParams = dev->getIrCameraParams();

      // get the intrinsic matrix
      ColorIntrinsic.fx = colorParams.fx;
      ColorIntrinsic.fy = colorParams.fy;
      ColorIntrinsic.cx = colorParams.cx;
      ColorIntrinsic.cy = colorParams.cy;

      DepthIntrinsic.fx = IrParams.fx;
      DepthIntrinsic.fy = IrParams.fy;
      DepthIntrinsic.cx = IrParams.cx;
      DepthIntrinsic.cy = IrParams.cy;

      std::cout << "\nDepthIntrinsic:\n"<<"fx:"<<DepthIntrinsic.fx <<"\tfy:"<<DepthIntrinsic.fy
          <<"\tcx:"<<DepthIntrinsic.cx <<"\tcy:"<<DepthIntrinsic.cy<<'\n';

      registration = new libfreenect2::Registration(IrParams, colorParams);

      std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
      std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

      return true;
  }


  void Grabber::save_frame(ImgTypes imtype, const cv::Mat &img)
  {
      std::string typestr;

      switch(imtype) {
        case 1 : typestr = "rgb";
                break;
        case 2 : typestr = "depth";
                break;
      }

      std::string ImgPath = "data/" +typestr+"/" + typestr + ".png";
      cv::imwrite(ImgPath, img);
  }

  pcl::PointCloud<PointT>::Ptr Grabber::generate_organized_pcd_from_kinect2() {

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    float x, y, z, color;

    cloud->width = registered.width;
    cloud->height = registered.height;
    cloud->points.resize (cloud->width * cloud->height);

    for (int m = 0 ;  m < registered.width ; m++) {
      for (int n = 0 ; n < registered.height ; n++) {

        PointT p;
        registration->getPointXYZRGB(&undistorted, &registered, n, m, x, y, z, color);

        const uint8_t *c = reinterpret_cast<uint8_t*>(&color);
        uint8_t b = c[0];
        uint8_t g = c[1];
        uint8_t r = c[2];

        if (z<1.2 && y<0.2) // temporarily remove the unwanted points by qualifying xyz, and the point cloud segmentation is still learning. . .
        {
          p.z = -z;
          p.x = x;
          p.y = -y;
          p.b = b;
          p.g = g;
          p.r = r;

          cloud->points.push_back( p );
        }
      }
    }
    return cloud;
  }

  void Grabber::fetch() {

      cv::Mat tempRgb, tempDepth;
      //cv::namedWindow("ColorImage", cv::WINDOW_NORMAL | cv::WND_PROP_ASPECT_RATIO);
      //cv::namedWindow("DepthImage", WND_PROP_ASPECT_RATIO);

      // RECEIVE IMAGE FRAMES
      listener->waitForNewFrame(frames);

      libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
      cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(tempRgb);
      //cv::flip(tempRgb, mRgb, 1);
      //save_frame(_rgb, mRgb);

      libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
      cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(tempDepth);
      cv::flip(tempDepth, mDepth, 1);
      save_frame(_depth, mDepth );

      // REGISTRATION
      registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
      cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(tempRgb);
      cv::flip(tempRgb, mRgb, 1);
      save_frame(_rgb, mRgb);

      //std::cout << "M = " << std::endl << " "  << r << std::endl << std::endl;
      //cv::imshow("ColorImage", mRgb);
      //cv::imshow("DepthImage", mDepth / 4500.0f);

      //cv::waitKey(5000);
  }


  void Grabber::release_frames() {

    listener->release(frames);
  }


  void Grabber::shutdown() {

      // STOP DEVICE
      dev->stop();
      dev->close();

      delete registration;
  }

  libfreenect2::Registration* Grabber::get_registration_object() {

    return registration;
  }

  libfreenect2::Frame Grabber::get_undistorted_frame() { return undistorted; };

  libfreenect2::Frame Grabber::get_registered_frame() { return registered; };

  cv::Mat Grabber::get_depth_data() { return mDepth; };

  cv::Mat Grabber::get_rgb_data() { return mRgb; };
