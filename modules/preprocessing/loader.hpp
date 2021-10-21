#include <fstream>
#include <iostream>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// OPENCV
#include <opencv2/opencv.hpp>

using namespace cv;

namespace io {

  typedef pcl::PointXYZRGB PointT;

  bool writeRawImage(const cv::Mat& image, const std::string& filename)
  {
    ofstream file;
    file.open (filename, ios::out|ios::binary);
    if (!file.is_open())
        return false;
    file.write(reinterpret_cast<const char *>(&image.rows), sizeof(int));
    file.write(reinterpret_cast<const char *>(&image.cols), sizeof(int));
    const int depth = image.depth();
    const int type  = image.type();
    const int channels = image.channels();
    file.write(reinterpret_cast<const char *>(&depth), sizeof(depth));
    file.write(reinterpret_cast<const char *>(&type), sizeof(type));
    file.write(reinterpret_cast<const char *>(&channels), sizeof(channels));
    int sizeInBytes = image.step[0] * image.rows;
    file.write(reinterpret_cast<const char *>(&sizeInBytes), sizeof(int));
    file.write(reinterpret_cast<const char *>(image.data), sizeInBytes);
    file.close();
    return true;
  }

  bool readRawImage(cv::Mat& image, const std::string& filename)
  {
    int rows, cols, data, depth, type, channels;
    ifstream file (filename, ios::in|ios::binary);
    if (!file.is_open())
        return false;
    try {
        file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
        file.read(reinterpret_cast<char *>(&cols), sizeof(cols));
        file.read(reinterpret_cast<char *>(&depth), sizeof(depth));
        file.read(reinterpret_cast<char *>(&type), sizeof(type));
        file.read(reinterpret_cast<char *>(&channels), sizeof(channels));
        file.read(reinterpret_cast<char *>(&data), sizeof(data));
        image = cv::Mat(rows, cols, type);
        file.read(reinterpret_cast<char *>(image.data), data);
    } catch (...) {
        file.close();
        return false;
    }

    file.close();
    return true;
  }

  pcl::PointCloud<PointT>::Ptr load_point_cloud() {

    pcl::PointCloud<PointT>::Ptr cloud_ (new pcl::PointCloud<PointT>);

    // Load .pcd file
    if (pcl::io::loadPCDFile<PointT> ("../data/PCD.pcd", *cloud_) == -1)
    {
        PCL_ERROR ("Couldn't read file PCD.pcd \n");
        exit(0);
    }
    std::cout << "Loaded "
        << cloud_->width * cloud_->height
        << " data points from PCD.pcd with the following fields: "
        << std::endl;

    return cloud_;
  }

}
