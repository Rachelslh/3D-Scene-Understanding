#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstdlib>
// Internal Libraries
// KINECT2 GRABBER
#include "preprocessing/libfreenect2_grabber.hpp"
// DCGD
#include "Depth-Cut_Based_Ground_Detection/include/ground_search.hpp"
// Segmentation
#include "segmentation/utils/planar_segmentation.hpp"
// Contours / Intersections
#include "segmentation/utils/edges.hpp"
// Labeling
#include "segmentation/utils/nodes.hpp"
// Utils
#include "segmentation/utils/normals.hpp"
#include "segmentation/visualization/visualizer.hpp"
#include "preprocessing/point_cloud.hpp"
#include "preprocessing/loader.hpp"

#include <pcl/io/ply_io.h>

pcl::visualization::PCLVisualizer::Ptr viewer;

int main (int argc, char** argv) {

    Grabber *kinect = new Grabber();
    segmentation::plane_classifier *seg;
    segmentation::edge_classifier *edg;
    labeling::label_edges *lbl;

    Ground_Detection::Camera *camera;
    Ground_Detection::DCGD *dcgd;

    ColorPalette cp;

    // Depth map and RGB
    cv::Mat mDepth, mColor;

    // Read from Kinect v2.0
    bool bDeviceStarted =  true;
    bool shutdown = false;

    try
    {
        //bDeviceStarted = kinect->init_device();
        seg = new segmentation::plane_classifier(10, 0.05, false);
        edg = new segmentation::edge_classifier(0.01, 0.03, 0.1, 0.01);
        lbl = new labeling::label_edges();

        Grabber::IntrinsicMatrix intrinsics = kinect->DepthIntrinsic;

        camera = new Ground_Detection::Camera(intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);
        dcgd = new Ground_Detection::DCGD(camera, 1, 7500, 50, 10, 20, false);
    }
    catch(const char* msg)
    {
        std::cerr << msg << endl;
    }

    pcl::PointCloud<PointT>::Ptr cloud_ (new pcl::PointCloud<PointT>),
                        outliers (new pcl::PointCloud<PointT>),
                        noisy_ground_cloud (new pcl::PointCloud<PointT>),
                        ground_cloud (new pcl::PointCloud<PointT>),
                        wall_cloud (new pcl::PointCloud<PointT>),
                        cloud_pub;

    pcl::PointCloud<PointT> init_cloud, dcgd_cloud, no_ground_cloud, preprocessed_cloud;      
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>),
                        noisy_ground_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients ground_model, wall_model;
    Eigen::Vector4d ground_centroid;

    //if(bDeviceStarted){

        std::cout<<"[Info] [Initial Message]: Device is opened and data transmission started..."<<endl;

      //  while(!shutdown) {

          // FETCH
          //kinect->fetch();

          cp.createColorPalette(50);
          seg->set_color_palette(cp);

          // READ FROM KINECT V2
          //mColor = kinect->get_rgb_data();
          //mDepth = kinect->get_depth_data();

          // SAVE TO DATA FOLDER
          //io::writeRawImage(mColor, "data/rgb/rgbMat2");
          //io::writeRawImage(mDepth, "data/depth/depthMat2");

          // OR READ FROM DATA FOLDER
          io::readRawImage(mColor, "data/rgb/rgbMat5");
          io::readRawImage(mDepth, "data/depth/depthMat5");
          
          // INITIAL PCD GENERATION
          preprocessing::generate_unorganized_pcd_from_data(mColor, mDepth, *camera, init_cloud);
          
          // DCGD
          dcgd->set_rgb_image(mColor);
          dcgd->run(mDepth);

          // PCD Generation
          preprocessing::generate_unorganized_pcds_from_data(mColor, mDepth, *camera, cloud_,
                                            noisy_ground_cloud, dcgd->get_ground_indices());
            
          preprocessing::flip_point_cloud(cloud_);
          preprocessing::flip_point_cloud(noisy_ground_cloud);

        
          dcgd_cloud = (*cloud_) + (*noisy_ground_cloud);

          // Detach ground from cloud by mapping between planar segmentation and dcgd output,
          // to get more accurate results
          // segment ground inliers to eliminate noise
          seg->setInputCloud(noisy_ground_cloud);
          ground_cloud = seg->segment(1, false); // Segment without using normals
          ground_model = seg->get_model_Cefficients()[0];
          // Merge cloud_ with the left non ground found points
          *cloud_ = (*cloud_) + (*seg->getInputCloud());

          *ground_cloud = (*ground_cloud) + (*cloud_);
          no_ground_cloud = *cloud_;

          //pcl::compute3DCentroid<PointT> (*ground_cloud, ground_centroid);

          // Apply rotation to face the z-axis
          //preprocessing::rotate_point_cloud_around_z_axis(cloud_, ground_model, ground_centroid, true);
          
          // Center PCD and calculate NORMALS for the new cloud
          // PREPROCESSING
          preprocessing::preprocess_pcd(cloud_, outliers);

          preprocessed_cloud = *cloud_;

          // Take off wall
          seg->reset_segmentation_obj();
          seg->setInputCloud(cloud_);
          seg->segment(1, false); // Segment without using normals
          *cloud_ = *seg->getInputCloud();

          // NORMALS CALCULUS
          // Note voxel-downsampling outputs an unorganized pcd anyways
          base::calculate_normals_unorganized_pcl (cloud_, normals);

          // PLANE SEGMENTATION => Generates planar regions
          seg->reset_segmentation_obj();
          seg->setInputCloud(cloud_);
          seg->setInputNormals(normals);
          cloud_pub = seg->segment(5, true); // Using normals

          // INTERSECTIONS AND CONTOURS
          edg->setInputCloud(cloud_pub);
          edg->setInputNormals(normals);
          edg->setModelsAndIndices(seg->get_model_Cefficients(), seg->get_inlier_indices());

          edg->find_intersections();
          edg->find_intersect_edges_with_ground(ground_model);
          edg->find_boundary_edges();

          // LABELING AND NODES
          lbl->set_edges_input(edg->get_edges_per_junctions());
          lbl->set_junctions(edg->get_junctions());
          lbl->set_centroids(edg->get_centroids());
          lbl->label();
          lbl->print_all_nodes();

          //kinect->release_frames();
        //}

        //kinect->shutdown();
    //}

/*
cv::namedWindow( "windowName_", cv::WINDOW_AUTOSIZE);
cv::imshow( "windowName_", mColor); // Show the image.

cv::waitKey(0); // Wait for a keystroke in the window
*/

    std::string arg1 = std::string(argv[1]);
    //std::string arg2 = std::string(argv[2]);

    if (arg1 == "-I"){
        viewer = simpleVis(init_cloud.makeShared(), normals, false);
    }
    else if (arg1 == "-D")
        viewer = simpleVis(dcgd_cloud.makeShared(), normals, false);
    else if (arg1 == "-G")
        viewer = simpleVis(ground_cloud, normals, false);
    else if (arg1 == "-N")
        viewer = simpleVis(no_ground_cloud.makeShared(), normals, false);
    else if (arg1 == "-P") {
        if (std::string(argv[2]) == "-N")
            viewer = simpleVis(preprocessed_cloud.makeShared(), normals, true);
        else
            viewer = simpleVis(preprocessed_cloud.makeShared(), normals, false);
    }
    else if (arg1 == "-S") {
        viewer = simpleVis(cloud_pub, normals, false);
        view_centroids_of_planes(viewer, edg->get_centroids());
        if (std::string(argv[2]) == "-H")
            view_contours_of_planes(viewer, cloud_pub, edg->get_concave_hulls());
        else if (std::string(argv[2]) == "-C")
            view_all_potential_junctions(viewer, cloud_pub, edg->get_corner_segments_and_centroids());
        else if (std::string(argv[2]) == "-S")
            view_segments_of_junction_points(viewer, cloud_pub, edg->get_corner_segments_and_centroids(), cp);
        else if (std::string(argv[2]) == "-J")
            view_junctions(viewer, cloud_pub, edg->get_junctions());
        else if (std::string(argv[2]) == "-E") {
            view_junctions(viewer, cloud_pub, edg->get_junctions());
            view_edges(viewer, cloud_pub, edg->get_junctions(), edg->get_edges());
        }
    }

    // Visualizer
    //viewer = simpleVis(cloud_, normals, false);
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(outliers, 240, 24, 5);
    //viewer->addPointCloud(outliers, single_color, "scene2");
    
    // Contours
    //view_contours_of_planes(viewer, cloud_pub, edg->get_concave_hulls());
    // Extreme Points
    //view_junctions(viewer, cloud_pub, edg->get_junctions());
    // Planes' centroids
    //view_centroids_of_planes(viewer, edg->get_centroids());
    // Intersections
    //view_edges(viewer, cloud_pub, edg->get_junctions(), edg->get_edges());
    //view_segments_of_junction_points(viewer, cloud_pub, edg->get_corner_segments_and_centroids(), cp);
    //view_ground_intersections(viewer, cloud_pub, edg->get_v());
    viewer->spin();
    viewer->close();

    return (0);
}
