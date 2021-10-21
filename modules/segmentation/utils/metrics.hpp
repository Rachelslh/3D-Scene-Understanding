#include <iostream>
#include <cstdlib>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include "generic_point_cloud.hpp"

namespace base {


  std::vector<PointT> project_points_on_line_model(pcl::ModelCoefficients::Ptr line_model,
                                                std::vector<PointT> points)
  {
    // Create the filtering object
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_LINE);

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>),
                                cloud_projected (new pcl::PointCloud<PointT>);
    cloud->points.resize (points.size());

    int i = 0;
    for (auto& point: *cloud)
    {
      point = points[i];
      i ++;
    }

    proj.setInputCloud (cloud);
    proj.setModelCoefficients (line_model);
    proj.filter (*cloud_projected);

    points.clear();
    for (auto& point: *cloud_projected)
    {
      points.push_back(point);
    }

    return points;
  }


  double find_angle(PointT pre_pt, PointT pt, PointT next_pt) {

    Eigen::Vector3f p , q;
    double pq, p_mag, q_mag, angle;

    p = Eigen::Vector3f(pre_pt.x - pt.x, pre_pt.y - pt.y, pre_pt.z - pt.z);
    q = Eigen::Vector3f(next_pt.x - pt.x, next_pt.y - pt.y, next_pt.z - pt.z);

    pq = p.dot(q);
    p_mag = sqrt(pow(p[0], 2) + pow(p[1], 2)+ pow(p[2], 2));
    q_mag = sqrt(pow(q[0], 2) + pow(q[1], 2)+ pow(q[2], 2));

    angle = acos( pq / (p_mag * q_mag) );
    angle = angle * 180 / M_PI;

    return angle;
  }


  double find_angle_with_z_axis(PointT pt1, PointT pt2) {

    Eigen::Vector3f p;
    double p_mag, angle;

    p = Eigen::Vector3f(pt1.x - pt2.x, pt1.y - pt2.y, pt1.z - pt2.z);

    p_mag = sqrt(pow(p[0], 2) + pow(p[1], 2)+ pow(p[2], 2));

    angle = acos( p[2] / p_mag );
    angle = angle * 180 / M_PI;

    return angle;
  }


  int compare(const void* a, const void* b) {

    const double* angle_a = (double*) a;
    const double* angle_b = (double*) b;

    if (*angle_a > *angle_b)
      return 1;
    else if (*angle_a < *angle_b)
      return -1;

    return 0;
  }


  PointT get_centroid_of_segment(pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> indices) {

    pcl::CentroidPoint<PointT> centroid;
    PointT c;

    if (indices.size() == 1)
      return cloud->points[indices[0]];

    for (int i = 0; i < indices.size(); i++ )
      centroid.add(cloud->points[indices[i]]);

    centroid.get(c);

    return c;
  }


  PointT get_centroid_of_segment_using_points(std::vector<PointT> points) {

    pcl::CentroidPoint<PointT> centroid;
    PointT c;

    if (points.size() == 1)
      return points[0];

    for (int i = 0; i < points.size(); i++ )
      centroid.add(points[i]);

    centroid.get(c);

    return c;
  }


  float point2pointEuclideanDistance(PointT pt1, PointT pt2) {

    return pcl::squaredEuclideanDistance(pt1, pt2);
  }


  double point2PlaneDistance(PointT pt, pcl::ModelCoefficients coefficients){
      double f1 = fabs(coefficients.values[0]*pt.x+coefficients.values[1]*pt.y+coefficients.values[2]*pt.z+coefficients.values[3]);
      double f2 = sqrt(pow(coefficients.values[0],2)+pow(coefficients.values[1],2)+pow(coefficients.values[2],2));
      return f1/f2;
  }


  void compute_metrics(pcl::PointCloud<PointT>::Ptr cloud_,
                      pcl::PointIndices inliers,
                      pcl::ModelCoefficients coefficients,
                      std::vector<double> &err,
                      double mean_error, double max_error, double min_error)
  {

      // Iterate inliers
      for (int i=0;i<inliers.indices.size();i++) {

          // Get Point
          PointT pt = cloud_ -> points[inliers.indices[i]];

          // Compute distance
          double d = point2PlaneDistance(pt,coefficients)*1000;
          err.push_back(d);

          // Update statistics
          mean_error += d;
          if (d>max_error) max_error = d;
          if (d<min_error) min_error = d;
      }
  }


  void printPlaneTime(size_t indicesSize) {
      // Timer
      pcl::console::TicToc tt;

      pcl::console::print_info("[done, ");
      pcl::console::print_value("%g", tt.toc());
      pcl::console::print_info(" ms : ");
      pcl::console::print_value("%lu", indicesSize);
      pcl::console::print_info(" points]\n");
  }


  void displayPlaneInfo(pcl::PointCloud<PointT>::Ptr cloud_,
                      pcl::ModelCoefficients::Ptr coefficients,
                      int n_plane,
                      size_t indicesSize,
                      size_t originalSize,
                      double mean_error,
                      double sigma,
                      double max_error)
  {
      // Display info
      pcl::console::print_info("cloud: fitted plane %i: %fx%s%fy%s%fz%s%f=0 (inliers: %zu/%i)\n",
          n_plane,
          coefficients->values[0],(coefficients->values[1]>=0?"+":""),
          coefficients->values[1],(coefficients->values[2]>=0?"+":""),
          coefficients->values[2],(coefficients->values[3]>=0?"+":""),
          coefficients->values[3],
          indicesSize, originalSize);

      //pcl::console::print_info("cloud: mean error: %f(mm), standard deviation: %f (mm), max error: %f(mm)\n",mean_error,sigma,max_error);
      pcl::console::print_info("cloud: poitns left in cloud %i\n\n",(cloud_->width * cloud_->height) - indicesSize);
  }

}
