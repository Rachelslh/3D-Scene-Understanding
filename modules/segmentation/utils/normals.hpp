#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/io.h>

namespace base {

  // For unorganized PCLs, normals can be calculated this way

  void calculate_normals_unorganized_pcl(pcl::PointCloud<PointT>::Ptr cloud,
                                      pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals) {

    
    // CALCULATE NORMALS
    pcl::NormalEstimation<PointT, pcl::Normal> ne;

    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);
  }


  // For organized PCLs, normals can be calculated this way

  void calculate_normals_organizd_pcl(pcl::PointCloud<PointT>::Ptr cloud,
                                    pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals) {

      pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
      ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
      ne.setMaxDepthChangeFactor(0.02f);
      ne.setNormalSmoothingSize(10.0f);
      ne.setInputCloud(cloud);
      ne.compute(*cloud_normals);

  }

}
