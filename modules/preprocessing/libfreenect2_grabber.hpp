#ifndef LIBFREENECT2_GRABBER_HPP
#define LIBFREENECT2_GRABBER_HPP

// LIBFREENECT2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//OPENCV
#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGB PointT;

class Grabber {

  private:

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    libfreenect2::SyncMultiFrameListener *listener;

    libfreenect2::Registration *registration;
    libfreenect2::FrameMap frames;
    libfreenect2::Frame undistorted{libfreenect2::Frame(512, 424, 4)},
                        registered{libfreenect2::Frame(512, 424, 4)},
                        depth2rgb{libfreenect2::Frame(1920, 1080 + 2, 4)};

    cv::Mat mRgb, mDepth;

    std::string serial = "";
    bool enable_rgb = true;
    bool enable_depth = true;
    size_t framemax = -1;
    size_t framecount = 0;

    int types = 0;

    enum ImgTypes{ _rgb=1,_depth,_undistort} ;

  public:

    struct IntrinsicMatrix
    { // Default : Kinect v2 intrinsics
        float fx = 3.66161e+02;
        float fy = 3.66161e+02;
        float cx = 2.527e+02;
        float cy = 2.0129e+02;
    } ColorIntrinsic, DepthIntrinsic;

    Grabber();

    bool init_device();

    void save_frame(ImgTypes imtype, const cv::Mat &img);

    void fetch();

    pcl::PointCloud<PointT>::Ptr generate_organized_pcd_from_kinect2();

    void release_frames();

    void shutdown();

    libfreenect2::Registration* get_registration_object();

    libfreenect2::Frame get_undistorted_frame();

    libfreenect2::Frame get_registered_frame();

    cv::Mat get_depth_data();

    cv::Mat get_rgb_data();
};

#endif
