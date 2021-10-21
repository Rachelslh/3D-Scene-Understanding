#ifndef EDGES_HPP
#define EDGES_HPP

// Basic
#include <iostream>
#include <utility>
#include <unordered_set>
#include <map>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/common/distances.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "generic_point_cloud.hpp"
#include "preprocessing/point_cloud.hpp"

using namespace base;

namespace segmentation {

  enum Direction {
    in,
    out,
    none
  };

  enum Type {

    convex,
    concave,
    left_boundary,
    right_boundary

  };

  struct edge_model {

    std::pair<int, int> surfaces;
    std::pair<int, int> edge;
    Type type;
    Direction dir;

  };

  class edge_classifier : public generic_pcl {

    private:
      // Params
      double threshold, tolerance, angular_tolerance = 0.1, boundary_thresh = 0.01;

      // Planar Segmentation Results
      std::vector<pcl::ModelCoefficients> model_coefficients;
      std::vector<std::vector<int>> inlier_indices;

      // Edges Per Planar Regions, These structures are for visualization purposes
      std::map<int, std::vector<edge_model>> edges_per_plane;
      std::map<int, PointT> all_junctions;
      std::vector<std::vector<int>> junctions_per_plane; // Indices for corner points for each plane contour

      std::map<int, std::vector<int>> boundary_edges; // For boundary edges (non intersections)
      // Segments of corner points and their centroids, Per plane
      std::vector<std::vector<std::pair<PointT, std::vector<int>>>> junction_segments;// euclidean clusters of probable junctions

      // Edges Per Junction
      std::map<int, std::vector<edge_model>> edges_per_junction;

      std::map<int, bool> intersected_planes;

      std::vector<std::vector<int>> hulls; // Concave hulls
      std::vector<PointT> centroids; // Holds the centroid of each plane

      // Boundary/Contour calculus for planes
      // Projection
      pcl::ProjectInliers<PointT> proj;

      // Concave hull construction
      pcl::ConcaveHull<PointT> chull;

    public:

      edge_classifier(double thresh, double tlr, double angular_tlr, double bound_thresh) : threshold { thresh },
                                                              tolerance { tlr },
                                                              angular_tolerance { angular_tlr },
                                                              boundary_thresh { bound_thresh }{}

      std::vector<std::vector<std::pair<PointT, std::vector<int>>>> get_corner_segments_and_centroids() {
        return junction_segments;
      }

      std::map<int, std::vector<edge_model>> get_edges() {
        return edges_per_plane;
      }

      std::map<int, std::vector<edge_model>> get_edges_per_junctions() {
        return edges_per_junction;
      }

      std::vector<std::vector<int>> get_concave_hulls() {
        return hulls;
      }

      std::map<int, PointT> get_junctions() {
        return  all_junctions;
      }

      std::vector<PointT> get_centroids() {
        return centroids;
      }

      // Overriding
      void setInputCloud(pcl::PointCloud<PointT>::Ptr &_cloud) {

        cloud = _cloud;

        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (cloud);

        chull.setAlpha (0.05);
        chull.setKeepInformation(true);
      }


      void setModelsAndIndices(std::vector<pcl::ModelCoefficients> model_coeffs,
                              std::vector<std::vector<int>> inlier_inds) {

        model_coefficients = model_coeffs;
        inlier_indices = inlier_inds;

        process_planes();
      }


      void find_intersections() {

        pcl::ModelCoefficients::Ptr line_coeff;
        Type type_intersect;
        edge_model intersec_obj;
        std::pair<int, int> edge_i, edge_j, surfaces;
        int pos1 = -1, pos2 = -1;

        for (int i = 0; i < model_coefficients.size() - 1; i++) {
          for(int j = i + 1; j < model_coefficients.size(); j++) {

            // Use this to check whether the planes are parallel or not
            line_coeff = get_intersection_line(model_coefficients[i], model_coefficients[j]);

            if (line_coeff != NULL && 
              junctions_per_plane[i].size() <= 8 &&
              junctions_per_plane[j].size() <= 8) {

              pos1 = locate_edge_junctions(*line_coeff, i, edge_i);
              pos2 = locate_edge_junctions(*line_coeff, j, edge_j);

              if ( pos1 > -1 && pos2 > -1 ) {
                
                // Erase edge_i of boundary_edges
                if (boundary_edges[edge_i.first].size() == 1)
                  boundary_edges.erase(edge_i.first);
                else
                  boundary_edges[edge_i.first].erase(boundary_edges[edge_i.first].begin() + pos1);
                // Erase edge_j of boundary_edges
                if (boundary_edges[edge_j.first].size() == 1)
                  boundary_edges.erase(edge_j.first);
                else
                  boundary_edges[edge_j.first].erase(boundary_edges[edge_j.first].begin() + pos2);

                std::cout << "Edges located" << '\n';
                std::cout <<  edge_i.first << '\t' << edge_i.second << '\n';
                std::cout <<  edge_j.first << '\t' << edge_j.second << '\n';

                update_junctions_wrt_intersection(line_coeff, i, j, edge_i, edge_j);

                std::cout << "[Found Intersection] : plane " << i + 1 << " , " << j + 1 << " : " << "\t" << "Point "
                          << line_coeff->values[0] << ' ' << line_coeff->values[1] << ' '
                          << line_coeff->values[2] << ", Direction : "  << line_coeff->values[3] << ' '
                          << line_coeff->values[4] << ' ' << line_coeff->values[5] << '\n';

                find_type_of_intersection(i, j, edge_i, type_intersect);

                surfaces.first = i;
                surfaces.second = j;

                intersec_obj = {surfaces, edge_i, type_intersect, none};
                edges_per_plane[i].push_back(intersec_obj);
                edges_per_junction[edge_i.first].push_back(intersec_obj);
                edges_per_junction[edge_i.second].push_back(intersec_obj);

                intersected_planes[i] = true;
                intersected_planes[j] = true;
              }
            }

          }
        }

      }

      void find_boundary_edges() {
        //intersected_planes[0] = true; For one plane cases, view from above
        for (auto const &pair : intersected_planes) {
          if (pair.second)
            set_boundary_edges_types(pair.first);
        }
      }


      void find_intersect_edges_with_ground(pcl::ModelCoefficients ground_model) {

        pcl::ModelCoefficients::Ptr line_coeff;
        edge_model intersec_obj;
        std::pair<int, int> edge_i, surfaces;
        int i, pos = -1;

        surfaces.second = -1;

        for (auto const &pair : intersected_planes) {
          
          //if (!pair.second)
            //continue;

          i = pair.first;
          line_coeff = get_intersection_line(model_coefficients[i], ground_model);

          if (line_coeff != NULL) {
            
            pos = locate_edge_junctions(*line_coeff, i, edge_i);
            
            if ( pos > -1 ) {

              // Erase edge_i of boundary_edges
              if (boundary_edges[edge_i.first].size() == 1)
                boundary_edges.erase(edge_i.first);
              else
                boundary_edges[edge_i.first].erase(boundary_edges[edge_i.first].begin() + pos);

              std::cout << "[Found Intersection] : plane " << i + 1 << " , ground" << " : " << "\t" << "Point "
                        << line_coeff->values[0] << ' ' << line_coeff->values[1] << ' '
                        << line_coeff->values[2] << ", Direction : "  << line_coeff->values[3] << ' '
                        << line_coeff->values[4] << ' ' << line_coeff->values[5] << '\n';

              surfaces.first = i;
              intersec_obj = {surfaces, edge_i, concave, none};
              edges_per_plane[i].push_back(intersec_obj);
              edges_per_junction[edge_i.first].push_back(intersec_obj);
              edges_per_junction[edge_i.second].push_back(intersec_obj);
            }
          }
        }

      }


      void process_planes() {

        std::vector<int> corners;

        for (int i = 0; i < model_coefficients.size(); i++) {
          // Find centroid of the plane
          centroids.push_back(get_centroid_of_segment(cloud, inlier_indices[i]));
          // Find boundary/contours of the plane i
          find_concave_hulls(i);
          // Find corner points
          find_potential_corner_points(i, corners);
          // Filter and process corners to find the exact junctions
          find_junctions(corners, i);
          // Form pairs of junctions representing edges
          extract_boundary_edges(i);
        }

      }


      void find_concave_hulls(int j) {

        pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT>),
                                    cloud_hull (new pcl::PointCloud<PointT>);
        pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr cluster_indices (new pcl::PointIndices);
        std::vector<int> v;

        cluster_indices->indices = inlier_indices[j];

        // Project the model inliers
        proj.setIndices (cluster_indices);

        *plane_coeff = model_coefficients[j];
        proj.setModelCoefficients (plane_coeff);
        proj.filter (*cloud_projected);

        std::cerr << "PointCloud after projection has: "
                  << cloud_projected->size () << " data points." << std::endl;

        // Create a Concave Hull representation of the projected inliers
        chull.setInputCloud (cloud_projected);

        chull.reconstruct (*cloud_hull);
        chull.getHullPointIndices(*cluster_indices);

        for (size_t k = 0; k < cluster_indices->indices.size(); k++)
          v.push_back(inlier_indices[j][cluster_indices->indices[k]]);

        hulls.push_back(v);

        std::cerr << "Cloud hull has: "
                  << cloud_hull->size () << " data points." << std::endl;
        std::cout << "\n" << '\n';

      }


      void find_potential_corner_points(int j, std::vector<int> &corners) {

        std::vector<int> v = hulls[j];
        PointT pt, pre_pt, next_pt;
        double angle;
        int m = 5, prec, next;

        for ( int i = 0; i < v.size(); i++ ){

          if (i - m < 0)
            prec = v.size() + (i - m);
          else
            prec = i - m;

          if (i + m >= v.size())
            next = m - (v.size() - i);
          else
            next = i + m;

          pre_pt = cloud->points[v[prec]];
          pt = cloud->points[v[i]];
          next_pt = cloud->points[v[next]];

          angle = find_angle(pre_pt, pt, next_pt);

          if (angle >= 70 && angle <= 150) {
            corners.push_back(v[i]);
          }
        }

      }


      void find_junctions(std::vector<int> &corners, int j) {

        std::vector<std::pair<PointT, std::vector<int>>> temp_vec;
        std::pair<PointT, std::vector<int>> data;
        std::map<int, PointT> unordered_junctions;
        std::vector<int> segment, points;

        pcl::PointCloud<PointT>::Ptr cloudF(new pcl::PointCloud<PointT>);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        PointT centroid;

        double temp, minDist = 1000;
        int indice, key = all_junctions.size();;

        inliers->indices = corners;
        extract2PointCloud(cloud, cloudF, inliers, false);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloudF);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (tolerance); // 2cm
        ec.setMinClusterSize (3);
        ec.setMaxClusterSize (30);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloudF);
        ec.extract (cluster_indices);

        corners.clear();

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          for (const auto& idx : it->indices)
            segment.push_back(inliers->indices[idx]);

          centroid = get_centroid_of_segment(cloud, segment);
          data.first = centroid;
          data.second = segment;
          temp_vec.push_back(data);

          unordered_junctions[segment[0]] = centroid;

          segment.clear();
        }

        junction_segments.push_back(temp_vec);

        // Order the obtained junctions
        segment = hulls[j];
        for (int i = 0; i < segment.size(); i++) {

          for (auto const &point : unordered_junctions) {
            indice = point.first;
            if (segment[i] == indice) {
              all_junctions[key] = point.second;
              points.push_back(key);
              unordered_junctions.erase(indice);

              key ++;
              break;
            }
          }
        }

        junctions_per_plane.push_back(points);

      }


      void extract_boundary_edges(int j) {

        std::vector<int> v = junctions_per_plane[j];
        int size = v.size();

        for (int i = 0; i < size - 1; i++) {
          boundary_edges[v[i]].push_back(v[i + 1]);
        }

        boundary_edges[v[size - 1]].push_back(v[0]);
      }


      void find_type_of_intersection(int i, int j,
                                    std::pair<int, int> edge,
                                    Type &type)
      {
        std::vector<int> all_indices_two_planes = inlier_indices[i];
        all_indices_two_planes.insert(all_indices_two_planes.end(), inlier_indices[j].begin(), inlier_indices[j].end());

        std::vector<PointT> v;
        v.push_back(all_junctions[edge.first]);
        v.push_back(all_junctions[edge.second]);

        PointT centroid = get_centroid_of_segment_using_points(v);
        PointT centroid_of_two_planes = get_centroid_of_segment(cloud, all_indices_two_planes);

        double angle = find_angle_with_z_axis(centroid_of_two_planes, centroid);

        std::cout << "Angle in-between" << '\t' << angle << '\n';

        if (angle > 95 )
          type = convex;
        else
          type = concave;
      }


      int locate_edge_junctions(pcl::ModelCoefficients line_coeff,
                                              int i,
                                              std::pair<int, int> &edge)
      {
          PointT pt1, pt2;
          std::vector<int> keys = junctions_per_plane[i];
          double distance1, distance2, minDist1 = threshold, minDist2 = threshold;
          int key_edge = -1, pos, j = 0;

          for (const auto &key : keys)
          {
            if (boundary_edges.find(key) == boundary_edges.end()) /*dosen't exist*/
              continue;
              
            for (const auto &value : boundary_edges[key]) {
              
              if (std::find(keys.begin(), keys.end(), value) == keys.end()) {
                j++;
                continue;
              }

              pt1 = all_junctions[key];
              pt2 = all_junctions[value];

              Eigen::Vector4f linePt(line_coeff.values[0], line_coeff.values[1], line_coeff.values[2], 0);
              Eigen::Vector4f lineDir(line_coeff.values[3], line_coeff.values[4], line_coeff.values[5], 0);

              distance1 = pcl::sqrPointToLineDistance(pt1.getVector4fMap(), linePt, lineDir);
              distance2 = pcl::sqrPointToLineDistance(pt2.getVector4fMap(), linePt, lineDir);

              //std::cout << minDist1 << '\t'<<distance1 << '\t' << minDist2<< '\t' <<distance2 << '\n';
              if (distance1 <= minDist1 && distance2 <= minDist2) {
                
                minDist1 = distance1;
                minDist2 = distance2;
                key_edge = key;
                pos = j;
              }

              j++;
            }

            j = 0;
          }

          if (key_edge == -1)
            return -1;
          else {
            edge.first = key_edge;
            edge.second = boundary_edges[key_edge][pos];
          }

          return pos;

      }


      void update_junctions_wrt_intersection(pcl::ModelCoefficients::Ptr line_coeff,
                                              int i, int j,
                                              std::pair<int, int> &edge_i,
                                              std::pair<int, int> &edge_j)
      {
        std::vector<PointT> points;
        std::pair<int, int> new_indices;
        PointT pt1 = all_junctions[edge_i.first],
              pt2 = all_junctions[edge_i.second],
              pt3 = all_junctions[edge_j.first],
              pt4 = all_junctions[edge_j.second];
        bool sense = true;

        points.push_back(pt1);
        points.push_back(pt2);
        points = project_points_on_line_model(line_coeff, points);

        pt1 = all_junctions[edge_i.first] = points[0];
        pt2 = all_junctions[edge_i.second] = points[1];

        if ( (edge_j.first != edge_i.first) && (edge_j.first != edge_i.second) )
          all_junctions.erase(edge_j.first);
        if ( (edge_j.second != edge_i.first) && (edge_j.second != edge_i.second) )
          all_junctions.erase(edge_j.second);

        if (point2pointEuclideanDistance(pt3, pt1) < point2pointEuclideanDistance(pt3, pt2)) {
          new_indices.first = edge_i.first;
          new_indices.second = edge_i.second;
        }
        else {
          sense = false;
          new_indices.first = edge_i.second;
          new_indices.second = edge_i.first;
        }

        // Proceed per junction
        if (new_indices.first > edge_j.first) {

          new_indices.first = edge_j.first;
          
          if (sense)
            edge_j.first = edge_i.first;
          else 
            edge_j.first = edge_i.second;
        }

        if(new_indices.second > edge_j.second) {
          new_indices.second = edge_j.second;

          if (sense)
            edge_j.second = edge_i.second;
          else 
            edge_j.second = edge_i.first;
        }

        // Update boundary_edges and junctions_per_plane where there are edge_j indices with the new_indices
        for (int k = 0; k <= junctions_per_plane[j].size(); k++ ) {

          if (junctions_per_plane[j][k] == edge_j.first)
            junctions_per_plane[j][k] = new_indices.first;
          else if (junctions_per_plane[j][k] == edge_j.second)
            junctions_per_plane[j][k] = new_indices.second;
        }

        std::vector<int> values;
        for(auto &pair : boundary_edges) {
          
          if (pair.first == edge_j.second)
            values = pair.second;

          for (auto &el: pair.second) {
            
            if (el == edge_j.first) {
              el = new_indices.first;
              break;
            }
          }
        }

        if (values.size() > 0) {
          boundary_edges.erase(edge_j.second);
          boundary_edges[new_indices.second].insert(boundary_edges[new_indices.second].end(),
                                                  values.begin(), values.end());
        }
        std::cout << "Edges updated" << '\n';
      }


      void set_boundary_edges_types(int i) {

        std::vector<int> keys = junctions_per_plane[i];
        edge_model intersec_obj;
        std::pair<int, int> edge, surfaces;
        Type type;
        Direction dir_1, dir_2;
        bool dir_found = false;
        int pos = -1, j = 0;
        
        surfaces.first = i;
        surfaces.second = -1;

        for (const auto key : keys)
        {
          if ( boundary_edges.find(key) == boundary_edges.end() ) /*key dosen't exist*/
            continue;

          for (const auto el : boundary_edges[key]) {

            if (!std::count(keys.begin(), keys.end(), el))
              continue;

            edge.first = key;
            edge.second = el;
        
            find_type_of_boundary_edge(i, edge, type);

            if (!dir_found) {
              find_direction_of_boundary_edge(edge, type, dir_1, dir_2);
              dir_found = true;
            }
  
            intersec_obj = {surfaces, edge, type, dir_1};
            edges_per_plane[i].push_back(intersec_obj);
            edges_per_junction[edge.first].push_back(intersec_obj);

            intersec_obj = {surfaces, edge, type, dir_2};
            edges_per_junction[edge.second].push_back(intersec_obj);

            pos = j;
            j++;

            break;
          }

          if (pos != - 1) {
            if (boundary_edges[key].size() == 1)
              boundary_edges.erase(key);
            else
            boundary_edges[key].erase(boundary_edges[key].begin() + pos);
          }

          pos = -1;
          j = 0;
        }

      }


      void find_type_of_boundary_edge(int i, std::pair<int, int> edge, Type &type) {

        PointT pt1, pt2, pt3;
        std::pair<double, double> dist_x, dist_y, dist_z;

        pt1 = all_junctions[edge.first];
        pt2 = all_junctions[edge.second];

        pt3 = centroids[i];

        dist_x.first = (pt3.x - pt1.x);
        dist_x.second = (pt3.x - pt2.x);

        dist_y.first = (pt3.y - pt1.y);
        dist_y.second = (pt3.y - pt2.y);

        dist_z.first = (pt3.z - pt1.z);
        dist_z.second = (pt3.z - pt2.z);

        if ( (pt3.x > pt1.x && pt3.x > pt2.x) &&
          (abs(dist_x.first) > boundary_thresh && abs(dist_x.second) > boundary_thresh) )
          
          type = left_boundary;

        else if ( (pt3.x < pt1.x && pt3.x < pt2.x) &&
          (abs(dist_x.first) > boundary_thresh && abs(dist_x.second) > boundary_thresh) )
        
          type = right_boundary;

        else if ( (pt3.y < pt1.y && pt3.y < pt2.y) &&
          (abs(dist_y.first) > boundary_thresh && abs(dist_y.second) > boundary_thresh) )
  
          type = right_boundary;

        else if ( (pt3.y > pt1.y && pt3.y > pt2.y) &&
          (abs(dist_y.first) > boundary_thresh && abs(dist_y.second) > boundary_thresh) )
              
          type = left_boundary;

        else if ( (pt3.z < pt1.z && pt3.z < pt2.z) &&
          (abs(dist_z.first) > boundary_thresh && abs(dist_z.second) > boundary_thresh))
  
          type = left_boundary;

        else if ( (pt3.z > pt1.z && pt3.z > pt2.z) &&
          (abs(dist_z.first) > boundary_thresh && abs(dist_z.second) > boundary_thresh) )
      
          type = right_boundary;

        else {
          if ( (dist_x.first - dist_x.second > boundary_thresh) )
            type = left_boundary;
          else if ( (dist_x.first - dist_x.second < boundary_thresh) )
            type = right_boundary;
          else if ( (dist_y.first - dist_y.second > boundary_thresh) )
            type = left_boundary;
          else if ( (dist_y.first - dist_y.second < boundary_thresh) )
            type = right_boundary;
          else if ( (dist_z.first - dist_z.second < boundary_thresh) )
            type = left_boundary;
          else if ( (dist_z.first - dist_z.second > boundary_thresh) )
            type = right_boundary;
        }

      }

      void find_direction_of_boundary_edge(std::pair<int, int> &edge, Type type, 
                                        Direction &dir_1, Direction &dir_2) {

        PointT pt1 = all_junctions[edge.first], pt2 = all_junctions[edge.second];
        double dist_x, dist_y, dist_z;

        dist_x = abs(pt2.x - pt1.x);
        dist_y = abs(pt2.y - pt1.y);
        dist_z = abs(pt2.z - pt1.z);
        
        //std::cout<< edge.first << '\t' << edge.second << '\n';

        bool res1, res2, res3;
        switch(type) {

          case left_boundary:

            if ( (pt1.z < pt2.z && dist_z > boundary_thresh * 10) || 
                (pt1.y > pt2.y && dist_y > boundary_thresh * 10)|| 
                (pt1.x > pt2.x && dist_x > boundary_thresh * 10) ) {
              dir_1 = in;
              dir_2 = out;
              res1 = pt1.z < pt2.z; res2 = pt1.y > pt2.y; res3 = pt1.x > pt2.x;
              //std::cout << res1 << '\t' << res2 << '\t' << res3 << '\n';
            }
            else  {
              dir_1 = out;
              dir_2 = in;
            }
            break;

          case right_boundary:

            if ( (pt1.z > pt2.z && dist_z > boundary_thresh * 10) || 
                (pt1.y < pt2.y && dist_y > boundary_thresh * 10)|| 
                (pt1.x > pt2.x && dist_x > boundary_thresh * 10) ) {
                  
              dir_1 = in;
              dir_2 = out;
              res1 = pt1.z > pt2.z; res2 = pt1.y < pt2.y; res3 = pt1.x > pt2.x;
              //std::cout << res1 << '\t' << res2 << '\t' << res3 << '\n';
            }
            else {
              dir_1 = out;
              dir_2 = in;
            }
            break;
        }

      }


      pcl::ModelCoefficients::Ptr get_intersection_line(pcl::ModelCoefficients planeA,
                                  pcl::ModelCoefficients planeB)
      {

          Eigen::Vector4f plane_a, plane_b;
          plane_a.x() = planeA.values[0];
          plane_a.y() = planeA.values[1];
          plane_a.z() = planeA.values[2];
          plane_a.w() = planeA.values[3];

          plane_b.x() = planeB.values[0];
          plane_b.y() = planeB.values[1];
          plane_b.z() = planeB.values[2];
          plane_b.w() = planeB.values[3];

          Eigen::VectorXf line;
          pcl::ModelCoefficients::Ptr line_coeff(new pcl::ModelCoefficients ());

          // Result will be False if planes are parallel
          bool result = pcl::planeWithPlaneIntersection(plane_a,plane_b,line,angular_tolerance);

          if (result) {
              line_coeff->values.resize(6);
              for (int i=0;i<6;i++)
              {
                  line_coeff->values[i]=line[i];
              }

              return line_coeff;
          }

          return NULL;
      }

  };

}

#endif
