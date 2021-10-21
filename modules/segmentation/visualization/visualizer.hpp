#include <iostream>
#include <utility>
#include <string>
#include <map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "segmentation/visualization/colorMaps.hpp"

using namespace pcl::io;

typedef pcl::PointXYZRGB PointT;


pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, bool normals_mode)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->setBackgroundColor (255, 255, 255);

  //viewer->setPosition(0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, 211, 96, 96);
  viewer->setSize(cloud->width, cloud->height); // organized pcl
  viewer->addPointCloud(cloud, "scene");
  if (normals_mode)
    viewer->addPointCloudNormals<PointT, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->resetCameraViewpoint("scene");

  return (viewer);
}


void view_edges(pcl::visualization::PCLVisualizer::Ptr viewer,
                      pcl::PointCloud<PointT>::Ptr &cloud,
                      std::map<int, PointT> junctions,
                      std::map<int, std::vector<segmentation::edge_model>> edges)
{

  std::vector<segmentation::edge_model> v;
  std::pair<int, int> edge;
  std::vector<PointT> segment;
  PointT pt1, pt2, centroid;
  std::string type;
  int plane;

  for (auto const &data : edges) {
    plane = data.first;
    v = data.second;

    for (size_t i = 0; i < v.size(); i++) {

      edge = v[i].edge;

      if (v[i].dir == out) {
        pt2 = junctions[edge.first];
        pt1 = junctions[edge.second];
      }
      else {
        pt1 = junctions[edge.first];
        pt2 = junctions[edge.second];
      }

      segment.push_back(pt1);
      segment.push_back(pt2);
      centroid = get_centroid_of_segment_using_points(segment);

      if ( v[i].type == segmentation::concave ) {
        viewer->addLine(pt1, pt2, 255, 0, 255,
                    "intersection" + std::to_string(plane) + std::to_string(i), 0);
        type = "CC";
      }
      else if ( v[i].type == segmentation::convex ) {
        viewer->addLine(pt1, pt2, 255, 255, 0,
                    "intersection" + std::to_string(plane) + std::to_string(i), 0);
        type = "CV";
      }
      // Left boundary
      else if ( v[i].type == segmentation::left_boundary ) {
        viewer->addLine(pt1, pt2, 0, 255, 255,
                    "left boundary" + std::to_string(plane) + std::to_string(i), 0);
        viewer->addArrow(pt1, pt2, 0, 1, 0, false,
                    "boundary" + std::to_string(plane) + std::to_string(i), 0);
        type = "LB";
      }
      // Right boundary
      else if ( v[i].type == segmentation::right_boundary ) {
        viewer->addLine(pt1, pt2, 0, 255, 0,
                    "right boundary" + std::to_string(plane) + std::to_string(i), 0);
        viewer->addArrow(pt1, pt2, 0, 0, 1,false,
                    "boundary" + std::to_string(plane) + std::to_string(i), 0);
        type = "RB";
      }
      viewer->addText3D(type, centroid, 0.01, 1.0, 1.0, 1.0,
                  "type edge" + std::to_string(plane) + std::to_string(i), 0);

      segment.clear();
    }
  }

}


void view_contours_of_planes(pcl::visualization::PCLVisualizer::Ptr viewer,
              pcl::PointCloud<PointT>::Ptr &cloud,
              std::vector<std::vector<int>> inliers_map)
{

  uint32_t rgb;
  Color c(255,0,0);
  rgb = c.getColor();
  int i = 0, size;

  for (const auto &contour : inliers_map) {
    size = contour.size();
    for (size_t j = 1; j < size; j++){
      //viewer->addSphere(cloud -> points[v[j - 1]], 0.0025, 0, 255, 0, "plane contour" + std::to_string(i) + std::to_string(j), 0);
      viewer->addLine(cloud -> points[contour[j - 1]], cloud -> points[contour[j]], "contour" + std::to_string(i) + std::to_string(j));
    }

    //viewer->addSphere(cloud -> points[v[size - 1]], 0.0025, 0, 255, 0, "plane contour" + std::to_string(i) + std::to_string(size), 0);
    viewer->addLine(cloud -> points[contour[size - 1]], cloud -> points[contour[0]], "contour" + std::to_string(i) + std::to_string(size));
    i++;
  }

}

void view_centroids_of_planes (pcl::visualization::PCLVisualizer::Ptr viewer,
              std::vector<PointT> centroids)
{
  int i = 0;
  // draw centrois spheres on planes
  for (const auto &centroid : centroids) {
    //viewer->addSphere(c, 0.0025, "plane" + std::to_string(centroid.first), 0);
    viewer->addText3D(std::to_string(i + 1), centroid, 0.03, 1.0, 1.0, 1.0, std::to_string(i), 0);
    i ++;
  }

}


void view_junctions (pcl::visualization::PCLVisualizer::Ptr viewer,
              pcl::PointCloud<PointT>::Ptr &cloud,
              std::map<int, PointT> junctions)
{
  for (const auto &jct : junctions) {
    viewer->addText3D(std::to_string(jct.first), jct.second, 0.01, 1.0, 1.0, 1.0, "junction" + std::to_string(jct.first), 0);
    viewer->addSphere(jct.second, 0.008, "extreme pt" + std::to_string(jct.first), 0);
  }
}


void view_all_potential_junctions (pcl::visualization::PCLVisualizer::Ptr viewer,
              pcl::PointCloud<PointT>::Ptr &cloud,
              std::vector<std::vector<std::pair<PointT, std::vector<int>>>> pt_segments)
{
  std::vector<int> pts;
  int plane = 0, i = 0;

  for (const auto &segment : pt_segments)
    for (const auto &data : segment)
    {
        // Draw centroid of segment
        //viewer->addSphere(data.first, 0.005, 255, 255 , 255,
          //            "extreme pt " + std::to_string(plane) + std::to_string(i) + "centroid", 0);
        // Draw all segment
        pts = data.second;
        for (size_t j = 0; j < pts.size(); j++)
          viewer->addSphere(cloud->points[pts[j]], 0.005, 255, 0 , 255,
                        "extreme pt " + std::to_string(plane) + std::to_string(i) + std::to_string(j), 0);

        plane ++;
        i ++;
    }

}


void view_segments_of_junction_points (pcl::visualization::PCLVisualizer::Ptr viewer,
              pcl::PointCloud<PointT>::Ptr &cloud,
              std::vector<std::vector<std::pair<PointT, std::vector<int>>>> pt_segments,
              ColorPalette cp)
{
  std::vector<int> pts;
  uint32_t rgb;
  uint8_t r, g, b;
  int plane = 0, i = 0;

  for (const auto &segment : pt_segments)
    for (const auto &data : segment)
    {

        rgb = cp.getColorPalette()[i].getColor();
        r = (rgb >> 16) & 0x0000ff;
        g = (rgb >> 8)  & 0x0000ff;
        b = (rgb)       & 0x0000ff;

        // Draw centroid of segment
        //viewer->addSphere(data.first, 0.005, 255, 255 , 255,
          //            "extreme pt " + std::to_string(plane) + std::to_string(i) + "centroid", 0);
        // Draw all segment
        pts = data.second;
        for (size_t j = 0; j < pts.size(); j++)
          viewer->addSphere(cloud->points[pts[j]], 0.005, (double)r / 255, (double)g / 255 , (double)b / 255,
                        "extreme pt " + std::to_string(plane) + std::to_string(i) + std::to_string(j), 0);

        plane ++;
        i ++;
    }

}


void view_ground_intersections(pcl::visualization::PCLVisualizer::Ptr viewer,
              pcl::PointCloud<PointT>::Ptr &cloud,
              std::vector<pcl::ModelCoefficients> intersections) {

  int i = 0;
  for (auto const &lineCoeff : intersections) {
    viewer->addLine(lineCoeff, "ground intersect" + std::to_string(i), 0);
    i++;
  }
}
