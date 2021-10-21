#ifndef NODES_HPP
#define NODES_HPP

// Basic
#include <iostream>
#include <utility>
#include <map>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>

#include "generic_point_cloud.hpp"
#include "segmentation/utils/edges.hpp"

using namespace base;
using namespace segmentation;


namespace labeling {


  class label_edges {

    private:
      // Edges Per Junction
      std::map<int, std::vector<edge_model>> edges_per_junction;
      std::map<int, PointT> all_junctions;
      std::vector<PointT> centroids; // Holds the centroid of each plane
      // Totals
      std::map<int, std::vector<std::vector<edge_model>>> V_holder;
      std::map<int, std::vector<std::vector<edge_model>>> Y_holder;
      std::map<int, std::vector<std::vector<edge_model>>> W_holder;

    public:

      label_edges() {}

      void set_junctions(std::map<int, PointT> _all_junctions) {
        all_junctions = _all_junctions;
      }

      void set_centroids(std::vector<PointT> _centroids) {
        centroids = _centroids;
      }

      void set_edges_input(std::map<int, std::vector<edge_model>> _edges_per_junction) {
        edges_per_junction = _edges_per_junction;
      }

      void label() {

        int edges_tps[4] = {0, 0, 0, 0}; // convex, concave, in, out

        for (const auto &data : edges_per_junction) {

          get_present_types(data.second, edges_tps);
         
          if (data.second.size() == 2)
            check_for_V(data.second, edges_tps);

          else if (data.second.size() == 3) {
            check_for_W(data.second, edges_tps);
            check_for_Y(data.second, edges_tps);
          }

          edges_tps[0] = 0;
          edges_tps[1] = 0;
          edges_tps[2] = 0;
          edges_tps[3] = 0;
        }

      }


      void get_present_types(std::vector<edge_model> v, int edges_tps[4]) {

        for (const auto &edge : v) {

          if (edge.type == segmentation::convex)
            edges_tps[0] += 1;

          else if (edge.type == segmentation::concave)
            edges_tps[1] += 1;

          else if (edge.dir == segmentation::in)
            edges_tps[2] += 1;

          else if (edge.dir == segmentation::out)
            edges_tps[3] += 1;
        }
      }


      void print_all_nodes() {

        for (const auto &data: V_holder)
          for (const auto &edg : data.second)
            std::cout << "Type V" << data.first << '\t' <<
            "edge 1 : " << '\t' << edg[0].edge.first << ", " << edg[0].edge.second << '\t' <<
            "edge 2 : " << '\t' << edg[1].edge.first << ", " << edg[1].edge.second << '\n';

        for (const auto &data: W_holder)
          for (const auto &edg : data.second)
            std::cout << "Type W" << data.first << '\t' <<
            "edge 1 : " << '\t' << edg[0].edge.first << ", " << edg[0].edge.second << '\t' <<
            "edge 2 : " << '\t' << edg[1].edge.first << ", " << edg[1].edge.second << '\t' <<
            "edge 3 : " << '\t' << edg[2].edge.first << ", " << edg[2].edge.second << '\n';

        for (const auto &data: Y_holder)
          for (const auto &edg : data.second)
            std::cout << "Type Y" << data.first << '\t' <<
            "edge 1 : " << '\t' << edg[0].edge.first << ", " << edg[0].edge.second << '\t' <<
            "edge 2 : " << '\t' << edg[1].edge.first << ", " << edg[1].edge.second << '\t' <<
            "edge 3 : " << '\t' << edg[2].edge.first << ", " << edg[2].edge.second << '\n';

      }

      // TODO : Add position verification, is the other junction to left / right to the central one
      void check_for_V(std::vector<edge_model> v, int edges_tps[4]) {

        bool res;

        res = V1(v, edges_tps);
        if (!res) {
          res = V2(v, edges_tps);
          if (!res) {
            res = V3(v, edges_tps);
            if (!res)  {
              res = V4(v, edges_tps);
              if (!res) {
                res = V5(v, edges_tps);
                if (!res) 
                  V6(v, edges_tps);
              }
            }
          }
        }
      }


      void check_for_Y(std::vector<edge_model> v, int edges_tps[4])
      {
        bool res;

        res = Y1(v, edges_tps);
        if (!res) {
          res = Y2(v, edges_tps);
          if (!res)
            Y3(v, edges_tps);
        }
      }


      void check_for_W(std::vector<edge_model> v, int edges_tps[4])
      {
        bool res;

        res = W1(v, edges_tps);
        if (!res) {
          res = W2(v, edges_tps);
          if (!res)
            W3(v, edges_tps);
        }
      }


      bool V1(std::vector<edge_model> v, int edges_tps[4]) {

        if ( edges_tps[0] == 1 && edges_tps[2] == 1 && get_visible_surfaces(v) == 2)
        {
          V_holder[1].push_back(v);
          return true;
        }

        return false;
      }

      bool V2(std::vector<edge_model> v, int edges_tps[4]) {

        std::cout<<get_angle(v)<<'\n';
        if ( edges_tps[2] == 1 && edges_tps[3] == 1 && get_visible_surfaces(v) == 1 &&
        
          get_angle(v) > 100)
          
        {
          V_holder[2].push_back(v);
          return true;
        }
        
        return false;
      }

      bool V3(std::vector<edge_model> v, int edges_tps[4]) {

        if ( edges_tps[3] == 1 && edges_tps[0] == 1 && get_visible_surfaces(v) == 2)

        {
          V_holder[3].push_back(v);
          return true;
        }
        
        return false;
      }

      bool V4(std::vector<edge_model> v, int edges_tps[4]) {

        if (edges_tps[1] == 1 && edges_tps[3] == 1 && get_visible_surfaces(v) == 2)
        {

          V_holder[4].push_back(v);
          return true;
        }
        
        return false;
      }

      bool V5(std::vector<edge_model> v, int edges_tps[4]) {

        if ( edges_tps[2] == 1 && edges_tps[3] == 1 && get_visible_surfaces(v) == 1 &&
        
          get_angle(v) < 100)

        {
          V_holder[5].push_back(v);
          return true;
        }
        
        return false;
      }

      bool V6(std::vector<edge_model> v, int edges_tps[4]) {

        if ( edges_tps[1] == 1 && edges_tps[2] == 1 && get_visible_surfaces(v) == 2)

        {
          V_holder[6].push_back(v);
          return true;
        }
        
        return false;
      }

      bool Y1(std::vector<edge_model> v, int edges_tps[4]) {

        if (edges_tps[1] == 1 && edges_tps[2] == 1  && edges_tps[3] == 1 && edges_tps[1] == 1 && get_visible_surfaces(v) == 2)
        {
          Y_holder[1].push_back(v);
          return true;
        }

        return false;
      }

      bool Y2(std::vector<edge_model> v, int edges_tps[4]) {

        if (edges_tps[1] == 3 && get_visible_surfaces(v) == 3)
        {
          Y_holder[2].push_back(v);
          return true;
        }
        
        return false;
      }

      bool Y3(std::vector<edge_model> v, int edges_tps[4]) {

        if (edges_tps[0] == 3 && get_visible_surfaces(v) == 3)
        {
          Y_holder[3].push_back(v);
          return true;
        }
        
        return false;
      }

      bool W1(std::vector<edge_model> v, int edges_tps[4]) {

        if (edges_tps[0] == 2 && edges_tps[1] == 1 && get_visible_surfaces(v) == 3)
        {
          W_holder[1].push_back(v);
          return true;
        }

        return false;
      }

      bool W2(std::vector<edge_model> v, int edges_tps[4]) {

        if (edges_tps[1] == 2 && edges_tps[0] == 1 && get_visible_surfaces(v) == 3)
        {
          W_holder[2].push_back(v);
          return true;
        }

        return false;
      }

      bool W3(std::vector<edge_model> v, int edges_tps[4]) {

        if (edges_tps[2] == 1 && edges_tps[0] == 1 && edges_tps[3] == 1 &&

          get_visible_surfaces(v) == 2)
        {
          W_holder[3].push_back(v);
          return true;
        }

        return false;
      }

      int get_visible_surfaces(std::vector<edge_model> v) {

        std::vector<int> surfaces;

        for (const auto &edge : v) {

          if (edge.surfaces.first !=  -1 && 
              std::find(surfaces.begin(), surfaces.end(), edge.surfaces.first) == surfaces.end())
            surfaces.push_back(edge.surfaces.first);

          if (edge.surfaces.second !=  -1 && 
              std::find(surfaces.begin(), surfaces.end(), edge.surfaces.second) == surfaces.end())
            surfaces.push_back(edge.surfaces.second);
        }

        return surfaces.size();
      }


    double get_angle(std::vector<edge_model> v) {

      PointT pre_pt, pt, next_pt, node_c, plane_c;
      std::vector<PointT> seg;

      if (v[0].edge.first == v[1].edge.first) {
        pt = all_junctions[v[0].edge.first];
        pre_pt = all_junctions[v[0].edge.second];
        next_pt = all_junctions[v[1].edge.second];
      } 
      else if (v[0].edge.first == v[1].edge.second) {
        pt = all_junctions[v[0].edge.first];
        pre_pt = all_junctions[v[0].edge.second];
        next_pt = all_junctions[v[1].edge.first];
      } 
      else if (v[0].edge.second == v[1].edge.first) {
        pt = all_junctions[v[0].edge.second];
        pre_pt = all_junctions[v[0].edge.first];
        next_pt = all_junctions[v[1].edge.second];
      } 
      else if (v[0].edge.second == v[1].edge.second) {
        pt = all_junctions[v[0].edge.second];
        pre_pt = all_junctions[v[0].edge.first];
        next_pt = all_junctions[v[1].edge.first];
      } 

      seg.push_back(pre_pt);
      seg.push_back(next_pt);
      
      node_c = get_centroid_of_segment_using_points(seg);
      plane_c = centroids[v[0].surfaces.first];
      
      return find_angle(node_c, pt, plane_c);
    }

  };
}

#endif
