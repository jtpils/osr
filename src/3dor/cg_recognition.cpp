/*
Author : Nitin Deshpande (Region Growing), Saad Abdullah (Visualisation PCD), Radiah Rivu (Visualisation PLY)
Correspondence Grouping - requires PCL 1.7
Referred literature -
"A Branch-and-Bound Approach to Correspondence and Grouping Problems"
"Object recognition in 3D scenes with occlusions and clutter by Hough voting"
"3D free-form object recognition in range images using local surface patches"
DISCLAIMER
The part of the code segment was borrowed from the PCL website - http://www.pointclouds.org/documentation/tutorials/correspondence_grouping.php
*/
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
// uncomment the following header if you are using PCL version > 1.8
// #include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

// declaring globals
std::string model_filename_;
std::string scene_filename_;

// Parameters supplied to algortihms
bool display_keypoints (false);
bool display_correspondences (false);
bool utilize_cloud_resolution (false);
bool use_hough_algo (true);
float uniform_sampling_radius (0.01f);
float scene_uniform_sampling_radius (0.03f);
float ref_frame_radius (0.015f);
float descrete_radius (0.02f);
float cluster_group_size (0.01f);
float cluster_group_threshold (5.0f);



// A simple function to display the usage information of the tool
void displayUsageGuide (char *filename)
{
  std::cout << "Instructions: " << filename << " model_filename.pcd scene_filename.pcd [Switches]" << std::endl << std::endl;
  std::cout << "Switches:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show used keypoints." << std::endl;
  std::cout << "     -c:                     Show used correspondences." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "                             each radius given by that value." << std::endl;
  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
}


// Simple fumction to read and parse the command line
void readCommandLine (int argc, char *argv[])
{
  // display usage guide
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    displayUsageGuide (argv[0]);
    exit (0);
  }

  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Cannot find filenames.\n";
    displayUsageGuide (argv[0]);
    exit (-1);
  }

  // get the model file
  model_filename_ = argv[filenames[0]];
  // get the scene filename
  scene_filename_ = argv[filenames[1]];

  // get the switch related parameters
  if (pcl::console::find_switch (argc, argv, "-k"))
  {
    // show keypoints
    display_keypoints = true;
  }
  if (pcl::console::find_switch (argc, argv, "-c"))
  {
    // get correspondnces between scene and model
    display_correspondences = true;
  }
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
    // use the cloud resolution to enhance the information
    utilize_cloud_resolution = true;
  }

  // choose the algorithm used
  std::string used_algorithm;
  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
  {
    // Hough algorithm
    if (used_algorithm.compare ("Hough") == 0)
    {
      use_hough_algo = true;
      // use geometric clustering
    }else if (used_algorithm.compare ("GC") == 0)
    {
      use_hough_algo = false;
    }
    else
    {
      std::cout << "Unable to find the algorithm specified.\n";
      displayUsageGuide (argv[0]);
      exit (-1);
    }
  }

  // get generic parmaters
  // employ the uniform sampling radius
  pcl::console::parse_argument (argc, argv, "--model_ss", uniform_sampling_radius);
  // employ the uniform scene sampling radius
  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_uniform_sampling_radius);
  // RF radius
  pcl::console::parse_argument (argc, argv, "--rf_rad", ref_frame_radius);
  // discrete radius
  pcl::console::parse_argument (argc, argv, "--descr_rad", descrete_radius);
  // cluster group size
  pcl::console::parse_argument (argc, argv, "--cg_size", cluster_group_size);
  // grouping limit
  pcl::console::parse_argument (argc, argv, "--cg_thresh", cluster_group_threshold);
}

// Performs computation for a PCD -> get the average dsistance between each point in the PCD and a neighbor neearest to it.
double calculateCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double resolution = 0.0;
  int number_of_points = 0;
  int n_resolution;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  // traverse through all the points in the cloud until all the points are visited
  for (size_t i = 0; i < cloud->size (); ++i)
  {
    // consider for faster computation
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    // consider the neighbor in the point cloud
    n_resolution = tree.nearestKSearch (i, 2, indices, sqr_distances);
    // use the square distance to decide if the point should be added to the cloud
    if (n_resolution == 2)
    {
      resolution += sqrt (sqr_distances[1]);
      ++number_of_points;
    }
  }
  // until the number of points in point cloud is not empty, keep computing the resolution
  // in the given cluster
  if (number_of_points != 0)
  {
    resolution /= number_of_points;
  }
  return resolution;
}

// Main module

int main (int argc, char *argv[])
{
  readCommandLine (argc, argv);

  // making use of the aliases
  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

  // load the pcd into memory
  if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    displayUsageGuide (argv[0]);
    return (-1);
  }
  if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    displayUsageGuide (argv[0]);
    return (-1);
  }

  //  resolution invariance - irrespective of the resolution, correspondences are computed
  if (utilize_cloud_resolution)
  {
    float resolution = static_cast<float> (calculateCloudResolution (model));
    if (resolution != 0.0f)
    {
      uniform_sampling_radius   *= resolution;
      scene_uniform_sampling_radius   *= resolution;
      ref_frame_radius     *= resolution;
      descrete_radius  *= resolution;
      cluster_group_size    *= resolution;
    }
  }

  // Get normals
  pcl::NormalEstimationOMP<PointType, NormalType> normal_estimation;

  normal_estimation.setKSearch (10);
  normal_estimation.setInputCloud (model);
  normal_estimation.compute (*model_normals);

  normal_estimation.setInputCloud (scene);
  normal_estimation.compute (*scene_normals);

  // get keypoints by downsampling
  pcl::UniformSampling<PointType> uniform_sample;

  uniform_sample.setInputCloud (model);
  uniform_sample.setRadiusSearch (uniform_sampling_radius);

  pcl::PointCloud<int> keypointIndices1;
  uniform_sample.compute(keypointIndices1);
  pcl::copyPointCloud(*model, keypointIndices1.points, *model_keypoints);
  std::cout << "total points in model: " << model->size () << "; taken Keypoints: " << model_keypoints->size () << std::endl;

  // associate a 3D feature descriptor to each model and scene keypoint
  // compute SHOT descriptors using SHOTEstimationOMP.
  // SHOTEstimationOMP estimates the Signature of Histograms of OrienTations (SHOT) descriptor for a given point cloud dataset
  // containing points and normals, in parallel, using the OpenMP standard.
  // http://docs.pointclouds.org/trunk/classpcl_1_1_s_h_o_t_estimation_o_m_p.html
  uniform_sample.setInputCloud (scene);
  uniform_sample.setRadiusSearch (scene_uniform_sampling_radius);
  //uniform_sampling.filter (*scene_keypoints);
  pcl::PointCloud<int> keypointIndices2;
  uniform_sample.compute(keypointIndices2);
  pcl::copyPointCloud(*scene, keypointIndices2.points, *scene_keypoints);
  std::cout << "total points in scene: " << scene->size () << "; taken Keypoints: " << scene_keypoints->size () << std::endl;


  // descriptor for keypoints
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descrete_radius);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  // use kd tree for correspondences between model and scene
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  /*
    Algo:
    for each scene keypt descriptor:
      find nearest neighbour
      push in model keypoint descriptor
      push to correspondences vector
  */
  for (size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Number of correspondences: " << model_scene_corrs->size () << std::endl;

  // ultimate stage - actual clustering of the previously found correspondences.
  // begin clustering algorithm
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //  Using Hough3D
  // Class implementing a 3D correspondence grouping algorithm that can deal with multiple instances of a model template found into a given scene.
  // http://docs.pointclouds.org/trunk/classpcl_1_1_hough3_d_grouping.html
  if (use_hough_algo)
  {
    // calculate reference frames for Hough
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (ref_frame_radius);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    // algorithm needs to associate a Local Reference Frame (LRF) for each keypoint belonging to the clouds which are passed as
    // arguments! In this example, we explicitly compute the set of LRFs using the BOARDLocalReferenceFrameEstimation estimator before calling the clustering algorithm
    // BOARDLocalReferenceFrameEstimation implements the BOrder Aware Repeatable Directions algorithm for local reference frame estimation
    // http://docs.pointclouds.org/trunk/classpcl_1_1_b_o_a_r_d_local_reference_frame_estimation.html
    // clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cluster_group_size);
    clusterer.setHoughThreshold (cluster_group_threshold);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    // get the model kps
    clusterer.setInputCloud (model_keypoints);
    // set ip ref frame - model
    clusterer.setInputRf (model_rf);
    // get the scene kps
    clusterer.setSceneCloud (scene_keypoints);
    // set ip ref frame - scene
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    clusterer.recognize (rototranslations, clustered_corrs);
  }
  // using GC
  else
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cluster_group_size);
    gc_clusterer.setGCThreshold (cluster_group_threshold);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  // visualise results and output computations

  // display the rotation and translation matrix
  // transformation matrix and the number of correspondences are obtained by the clustering method
  std::cout << "Number of model instances: " << rototranslations.size () << std::endl;
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    instances " << i + 1 << ":" << std::endl;
    std::cout << "      Model - Scene correspondences in this instance: " << clustered_corrs[i].size () << std::endl;
    // amalgamate rotation and translation matrix
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

  }

  // use the cloud viewer - http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer.html

  // If keypoint visualization is enabled,
  // then
  //       keypoints are displayed as blue dots
  // else if correspondence visualization is  enabled
  // then
  //       green line for each correspondence

  pcl::visualization::PCLVisualizer viewer ("correspondence Grouping");
  viewer.addPointCloud (scene, "scene_cloud");

  // pointers to store the clouds
  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  if (display_correspondences || display_keypoints)
  {
    //  translating the model  -> it doesn't end abruptly
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }
    // display kps
  if (display_keypoints)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

  // traverse all points in the transformation matrix to pass the required model and scene required
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    // represent rotated model
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    // represent translated model
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    // display rotated color model
    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    // continue adding points from point clouds into the stream for marking correspondences if any
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    if (display_correspondences)
    {
      for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }

  // simple callback to display the cloud results
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}
