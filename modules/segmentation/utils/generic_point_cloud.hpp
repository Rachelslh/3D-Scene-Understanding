#ifndef GENERIC_POINT_CLOUD_HPP
#define GENERIC_POINT_CLOUD_HPP

// Basic
#include <iostream>
// PCL
#include <pcl/io/pcd_io.h>
// Extractions and clustering
#include <pcl/filters/extract_indices.h>

namespace base {

  typedef pcl::PointXYZRGB PointT;

  class generic_pcl {

  protected:

    // Input cloud
    pcl::PointCloud<PointT>::Ptr cloud;
    // Normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

    PointT centroid;

    // Extractors
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;

  public:


    void setInputCloud(pcl::PointCloud<PointT>::Ptr &_cloud) {
      cloud = _cloud;
    }

    void setInputNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals) {
      cloud_normals = normals;
    }

    void setCentroid(PointT _centroid) {
      centroid = _centroid;
    }

    pcl::PointCloud<PointT>::Ptr getInputCloud() {
      return cloud;
    }

    pcl::PointCloud<pcl::Normal>::Ptr getInputNormals() {
      return cloud_normals;
    }

    PointT getCentroid() {
      return centroid;
    }

    void extract2PointCloud(pcl::PointCloud<PointT>::Ptr &cloud_,
                            pcl::PointCloud<PointT>::Ptr &cloudF,
                            pcl::PointIndices::Ptr inliers,
                            bool negative)
    {
        // Extract
        extract.setInputCloud(cloud_);
        extract.setIndices(inliers);
        // Negative set to True ie: got outliers
        extract.setNegative(negative);
        extract.filter(*cloudF);
    }

    void extract2Normals(pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                            pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals2,
                            pcl::PointIndices::Ptr inliers,
                            bool negative)
    {
      extract_normals.setNegative (negative);
      extract_normals.setInputCloud (cloud_normals);
      extract_normals.setIndices (inliers);
      extract_normals.filter (*cloud_normals2);
    }

  };
}

#endif
