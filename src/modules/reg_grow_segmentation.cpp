/**
 * Author : Nitin Deshpande (Region Growing)
 * Code segment to demonstrate Region Growing Monochrome segmentation on PCDs
 * requires PCL 1.7
 */

#include "./include/reg_grow_segmentation.h"

RGSeg::RGSeg()
{
    // do something
}

RGSeg::~RGSeg()
{
    // do nothing
}

void RGSeg::regionGrowingMonochrome (pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud, double k_parameter, int min_cluster_size, int max_cluster_size, unsigned int neighborhood_size, float smoothness_threshold, float curvature_threshold)
{
  // Does not check the validity and presence of a given point cloud.
  // Perform a check on the validity and presence of a given point cloud
  // manually.
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (pt_cloud);
  normal_estimator.setKSearch (k_parameter);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pt_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  /*
  check whether the point is neighbouring or not.
  if (dist(point) < threshold)
    neighbor
    clustersearhc()
  else
    not_a_neighbor
  */

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (min_cluster_size);
  reg.setMaxClusterSize (max_cluster_size);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (neighborhood_size);
  reg.setInputCloud (pt_cloud);
  //reg.setIndices (indices);
  //
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (smoothness_threshold);

  // reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (curvature_threshold);

  //??
  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  // display clusters
  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;

  int counter = 0;
  while (counter < clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
      std::cout << std::endl;
  }
  std::cout << std::endl;


  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("CLUSTERS - VISUALISE");
  viewer.showCloud (colored_cloud);

  while (!viewer.wasStopped ())
  {
  }
}

void RGSeg::regionGrowingRGB (pcl::PointCloud <pcl::PointXYZRGB>::Ptr pt_cloud, float dist_threshold, float pt_color_thresh, float reg_color_thresh, int min_cluster_size)
{

  // Does not check the validity and presence of a given point cloud.
  // Perform a check on the validity and presence of a given point cloud
  // manually.

  // set the search method -> KdTree
  pcl::search::Search <pcl::PointXYZRGB>::Ptr pcl_tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

  // store the indices of the point cloud
  pcl::IndicesPtr pc_indices (new std::vector <int>);

  // filtering a point cloud using passthrough filter
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (pt_cloud);

  // filter along z direction
  pass.setFilterFieldName ("z");

  // interval values are set to (0.0;1.0)
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*pc_indices);

  // create a region growing segmentation rgb object
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg_growing_seg;

  // set the input source cloud to process
  reg_growing_seg.setInputCloud (pt_cloud);

  // traverse along the provided indices
  reg_growing_seg.setIndices (pc_indices);

  // search along the Kd tree
  reg_growing_seg.setSearchMethod (pcl_tree);

  // set the distance threshold
  reg_growing_seg.setDistanceThreshold (dist_threshold);

  /*
  check whether the point is neighbouring or not.
  if (dist(point) < threshold)
  	neighbor
  	clustersearhc()
  else
  	not_a_neighbor
  */

  // specify the threshold value for color test between the points
  reg_growing_seg.setPointColorThreshold (pt_color_thresh);

  // specify the threshold value for color test between the region
  reg_growing_seg.setRegionColorThreshold (reg_color_thresh);

  // merge with the nearest neighbors
  reg_growing_seg.setMinClusterSize (min_cluster_size);

  // initialise clusters
  std::vector <pcl::PointIndices> pt_clusters;

  // when the  segmentation is finished return the array of clusters
  reg_growing_seg.extract (pt_clusters);

  // get colored cloud of the cluster
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg_growing_seg.getColoredCloud ();

  // region growing visualisation
  pcl::visualization::CloudViewer cloud_viewer ("RGB CLUSTERS - VISUALISE");

  // display cloud
  cloud_viewer.showCloud (colored_cloud);

  // callback like
  while (!cloud_viewer.wasStopped ())
  {
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

}


void RGSeg::printUsage (const char* progName)
{
  std::cout << "You may have encountered an error or a warning. Here is how you can use the tool" <<std::endl;
  std::cout << "Usage: "<< progName <<" filename.pcd"<< std::endl;
}
