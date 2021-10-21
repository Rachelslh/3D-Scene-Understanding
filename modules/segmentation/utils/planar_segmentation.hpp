#ifndef PLANAR_SEGMENTATION_HPP
#define PLANAR_SEGMENTATION_HPP

// Basic
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
// Segmentation
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
// Contours and edges
#include <pcl/common/intersections.h>
// Utils
#include "metrics.hpp"
#include "segmentation/visualization/colorMaps.hpp"

#include "generic_point_cloud.hpp"


using namespace pcl::io;
using namespace base;

namespace segmentation {

  class plane_classifier : public generic_pcl {

      private:

          // VARS

          // Algorithm parameters
          double _min_percentage;
          bool _color_pc_with_error;
          double _max_distance;

          // Segmentation
          std::vector<pcl::ModelCoefficients> model_coefficients;
          std::vector<std::vector<int>> inlier_indices;

          // Planar segmentation object
          pcl::SACSegmentation<PointT> seg;
          pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_with_normals;

          // Timer
          pcl::console::TicToc tt;

          // Colors
          ColorPalette cp;

      public:

          plane_classifier(double min_percentage,
              double max_distance,
              bool color_pc_with_error) {

              _min_percentage = min_percentage;
              _max_distance = max_distance;
              _color_pc_with_error = color_pc_with_error;

              seg.setOptimizeCoefficients (true);
              seg.setModelType (pcl::SACMODEL_PLANE);
              seg.setMethodType (pcl::SAC_RANSAC);
              seg.setMaxIterations (100);
              seg.setDistanceThreshold(_max_distance);

              seg_with_normals.setOptimizeCoefficients (true);
              seg_with_normals.setModelType (pcl::SACMODEL_NORMAL_PLANE);
              seg_with_normals.setNormalDistanceWeight (0.1);
              seg_with_normals.setMethodType (pcl::SAC_RANSAC);
              seg_with_normals.setMaxIterations (100);
              seg_with_normals.setDistanceThreshold(_max_distance);
          }

          std::vector<std::vector<int>> get_inlier_indices() {
              return inlier_indices;
          }

          std::vector<pcl::ModelCoefficients> get_model_Cefficients() {
              return model_coefficients;
          }

          void set_color_palette(ColorPalette _cp) {
            cp = _cp;
          }


          pcl::PointCloud<PointT>::Ptr segment(int max_planes, bool with_normals) {

              // Create pointcloud to publish inliers
              pcl::PointCloud<PointT>::Ptr cloud_pub(new pcl::PointCloud<PointT>),
                                          cloudF(new pcl::PointCloud<PointT>);

              pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);

              pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
              pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

              // Error metrics
              double mean_error(0);
              double max_error(0);
              double min_error(100000);
              std::vector<double> err;
              int original_size(cloud->height * cloud->width);

              int n_planes(1), n_points = 0;
              while (cloud->height * cloud->width > original_size * _min_percentage/100 &&
                  n_planes <= max_planes)
              {

                  pcl::console::print_highlight(stderr, "Searching for the largest plane (%2.0d) ", n_planes);
                  tt.tic();
                  // Fit a plane
                  if (with_normals) {
                    seg_with_normals.setInputCloud(cloud);
                    seg_with_normals.setInputNormals (cloud_normals);
                    seg_with_normals.segment(*inliers, *coefficients);
                  }
                  else {
                    seg.setInputCloud(cloud);
                    seg.segment(*inliers, *coefficients);
                  }

                  // Check result
                  if (inliers->indices.empty())
                      break;

                  printPlaneTime(inliers->indices.size());

                  model_coefficients.push_back(*coefficients);

                  compute_metrics(cloud, *inliers, *coefficients, err, mean_error, max_error, min_error);
                  mean_error /= inliers->indices.size();

                  // Compute Standard deviation
                  std::vector<int> indices;
                  Color color(min_error, max_error);
                  double sigma(0);
                  for (std::vector<int>::const_iterator pit = inliers->indices.begin ();

                      pit != inliers->indices.end (); ++pit)
                  {
                      sigma += pow(err[*pit] - mean_error,2);

                      // Get Point
                      PointT pt = cloud -> points[*pit];

                      // Copy point to new cloud => cloud_pub
                      PointT pt_color;
                      uint32_t rgb;

                      pt_color.x = pt.x;
                      pt_color.y = pt.y;
                      pt_color.z = pt.z;

                      if (_color_pc_with_error)
                        rgb = color.getColor(err[*pit]);
                      else
                        rgb = cp.getColorPalette()[n_planes].getColor();

                      pt_color.rgb = *reinterpret_cast<float*>(&rgb);
                      cloud_pub->points.push_back(pt_color);

                      indices.push_back(n_points);
                      n_points++;
                    }

                  inlier_indices.push_back(indices);
                  // Standard deviation
                  sigma = sqrt(sigma / inliers->indices.size());

                  // Display info
                  displayPlaneInfo(cloud, coefficients, n_planes, inliers->indices.size(),original_size,
                      mean_error, sigma, max_error);

                  // Next iteration
                  n_planes++;

                  // Extract: Negative set to True -> Extract outliers
                  extract2PointCloud(cloud, cloudF, inliers, true);
                  cloud.swap(cloudF);

                  if (with_normals) {
                    extract2Normals(cloud_normals, cloud_normals2, inliers, true);
                    cloud_normals.swap(cloud_normals2);
                  }
              }

              cloud_pub->width    = cloud_pub->points.size();
              cloud_pub->height   = 1;
              cloud_pub->resize (cloud_pub->width * cloud_pub->height);

              return cloud_pub;
          }


          void reset_segmentation_obj() {
            model_coefficients.clear();
            inlier_indices.clear();
          }

  };

}

#endif
