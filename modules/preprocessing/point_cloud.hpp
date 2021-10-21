#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include <iostream>
#include <math.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/filters/median_filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
// OPENCV
#include <opencv2/opencv.hpp>
// Internal Lib : Camera Intrinsics
#include "Depth-Cut_Based_Ground_Detection/include/camera.hpp"


using namespace cv;

namespace preprocessing {

  typedef pcl::PointXYZRGB PointT;
  typedef unsigned short ushort;

  // GENERATE POINT CLOUD OF RGB AND DEPTH FRAMES
  void generate_unorganized_pcd_from_data(Mat mColor, Mat mDepth,
                                Ground_Detection::Camera camera,
                                pcl::PointCloud<PointT> &cloud)
  {

    const double cameraFactor = 1000;

    const double fx = camera.get_fx();
    const double fy = camera.get_fy();
    const double cx = camera.get_cx();
    const double cy = camera.get_cy();

    for (int m = 0; m < mDepth.rows; m++) {
      for (int n = 0; n < mDepth.cols; n++) {
        //Get the value at (m, n) in the depth map
        float d = mDepth.ptr<float>(m)[n];
        Vec4b rgb = mColor.at<Vec4b>(m, n);

        // skip this point if there is no value
        if (d == 0)
          continue;

        PointT p;
        //Calculate the space coordinates of this point
        p.z =  d / cameraFactor;
        p.x = (n - cx) * p.z / fx;
        p.y = (m - cy) * p.z / fy;
        p.r = rgb[2];
        p.g = rgb[1];
        p.b = rgb[0];

        cloud.points.push_back(p);
      }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
  }


  // GENERATE POINT CLOUD OF RGB AND DEPTH FRAMES
  void generate_unorganized_pcds_from_data(Mat mColor, Mat mDepth,
                                Ground_Detection::Camera camera,
                                pcl::PointCloud<PointT>::Ptr &cloud,
                                pcl::PointCloud<PointT>::Ptr &noisy_ground_cloud,
                                std::map<int, bool> ground_indices)
  {

    const double cameraFactor = 1000;

    const double fx = camera.get_fx();
    const double fy = camera.get_fy();
    const double cx = camera.get_cx();
    const double cy = camera.get_cy();

    for (int m = 0; m < mDepth.rows; m++) {
      for (int n = 0; n < mDepth.cols; n++) {
        //Get the value at (m, n) in the depth map
        float d = mDepth.ptr<float>(m)[n];
        Vec4b rgb = mColor.at<Vec4b>(m, n);

        // skip this point if there is no value
        if (d == 0)
          continue;

        PointT p;
        //Calculate the space coordinates of this point
        p.z =  d / cameraFactor;
        p.x = (n - cx) * p.z / fx;
        p.y = (m - cy) * p.z / fy;
        p.r = rgb[2];
        p.g = rgb[1];
        p.b = rgb[0];

        if (ground_indices[m * mDepth.cols + n] == true)
          noisy_ground_cloud->points.push_back(p);
        else
          cloud->points.push_back(p);
      }
    }

    noisy_ground_cloud->width = noisy_ground_cloud->points.size();
    noisy_ground_cloud->height = 1;

    cloud->width = cloud->points.size();
    cloud->height = 1;
  }


  void flip_point_cloud(pcl::PointCloud<PointT>::Ptr &cloud) {

    Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();

    transform_y(1, 1) = -1;
    transform_y(2, 2) = -1;

    pcl::transformPointCloud(*cloud, *cloud, transform_y);

  }

  void rotate_point_cloud_around_z_axis(pcl::PointCloud<PointT>::Ptr &cloud, 
                                      pcl::ModelCoefficients &ground_model,
                                      Eigen::Vector4d centroid,
                                      bool clockWise) {

    Eigen::Matrix4f transform_M = Eigen::Matrix4f::Identity();
    Eigen::Matrix<float, 4, 1> rotated_ground_model;
    float theta;

    rotated_ground_model[0] = ground_model.values[0];
    rotated_ground_model[1] = ground_model.values[1];
    rotated_ground_model[2] = ground_model.values[2];
    rotated_ground_model[3] = ground_model.values[3];

    theta = acos(-centroid[2]/sqrt( pow(centroid[0],2) +
                                          pow(centroid[1],2) +
                                          pow(centroid[2],2) ));

    if (!clockWise)
      theta = - theta;

    transform_M (1,1) = cos(theta);
    transform_M (1,2) = sin(theta);
    transform_M (2,1) = -sin(theta);
    transform_M (2,2) = cos(theta);

    rotated_ground_model = transform_M * rotated_ground_model;

    ground_model.values[0] = rotated_ground_model[0];
    ground_model.values[1] = rotated_ground_model[1];
    ground_model.values[2] = rotated_ground_model[2];
    ground_model.values[3] = rotated_ground_model[3];

    pcl::transformPointCloud(*cloud, *cloud, transform_M);

    std::cout << "Point cloud rotated with a degree of " << theta << " around z-axis." <<'\n';
  }


  void apply_median_filter(pcl::PointCloud<PointT>::Ptr &cloud) {

    pcl::MedianFilter<PointT> medF;
    medF.setMaxAllowedMovement (2);
    medF.setWindowSize(5);
    medF.setInputCloud(cloud);
    medF.applyFilter (*cloud);
  }


  // CALCULATE THE CENTROID OF A POINT CLOUD
  Eigen::Vector4d center_pcd(pcl::PointCloud<PointT> &cloud_) {

    Eigen::Vector4d centroid;
    pcl::compute3DCentroid<PointT> (cloud_, centroid);
    std::cout << "centroid: " << centroid[0] << ", "
              << centroid[1] << ", " << centroid[2] << std::endl;

    // Center the point cloud around the origin
    pcl::demeanPointCloud <PointT, double> (cloud_, centroid, cloud_);

    return centroid;
  }


  void center_pcd_to_custom_point(pcl::PointCloud<PointT> &cloud_, Eigen::Vector4d centroid) {
    // Center the point cloud around the origin
    pcl::demeanPointCloud <PointT, double> (cloud_, centroid, cloud_);
  }


  void voxel_downsample(pcl::PointCloud<PointT>::Ptr &cloud) {

    // Create the filtering object: downsample the dataset using a leaf size of 100cm
    pcl::VoxelGrid<PointT> vgd;
    vgd.setInputCloud (cloud);
    vgd.setLeafSize (0.001, 0.001, 0.001);
    vgd.filter (*cloud);
  }


  void statistical_outlier_remover(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &outliers) {

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (5);
    // Get inliers only, if negation is set to True, then you'll get outliers only
    sor.filter (*cloud);
    //sor.setNegative (true);
    //sor.filter (*outliers);
  }


  pcl::PointCloud<PointT>::Ptr preprocess_pcd (pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &outliers)
  {

    //std::cout << "Point cloud flipped for correct visualization." << '\n';
    //flip_point_cloud(cloud);

    std::cout << "PointCloud before filtering: " << cloud->width * cloud->height
         << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    // Downsample : Voxel Grid
    voxel_downsample(cloud);

    std::cout << "PointCloud after filtering: " << cloud->width * cloud->height
         << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    // Statistical outliers removal
    statistical_outlier_remover(cloud, outliers);

    std::cout << "PointCloud after filtering: " << cloud->width * cloud->height
         << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    return cloud;
  }

}

#endif
