/**
 * Author : Nitin Deshpande (Region Growing)
 * Code segment to demonstrate Region Growing Monochrome segmentation on PCDs
 * requires PCL 1.7
 */

#ifndef RGSEG_H
#define RGSEG_H

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>

class RGSeg
{
  private:

  public:

    // constructor
    RGSeg();

    // deconstructor
    ~RGSeg();

    /**
     * pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud = given point cloud
     * double k_parameter = used in determining the set of nearest neighbors of
     a point
     * int min_cluster_size = Set the minimum number of points that a cluster
     needs to contain in order to be considered valid
     * int max_cluster_size = Set the maximum number of points that a cluster
     needs to contain in order to be considered valid.
     * unsigned int neighborhood_size = set the number of neighbours
     * float smoothness_threshold = set smoothness threshold used for testing the
     points
     * float curvature_threshold = set curvature threshold used for testing the
     points
     */

    void regionGrowingMonochrome (pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud, double k_parameter, int min_cluster_size, int max_cluster_size, unsigned int neighborhood_size, float smoothness_threshold, float curvature_threshold);

    /**
     * pcl::PointCloud <pcl::PointXYZRGB>::Ptr pt_cloud = given color point cloud
     * float dist_threshold = set the distance threshold
     * float pt_color_thresh = specify the threshold value for color test
     between the points
     * float reg_color_thresh = specify the threshold value for color test
     between the region
     * int min_cluster_size = Set the minimum number of points that a cluster
     needs to contain in order to be considered valid
     */

     void regionGrowingRGB (pcl::PointCloud <pcl::PointXYZRGB>::Ptr pt_cloud, float dist_threshold, float pt_color_thresh, float reg_color_thresh, int min_cluster_size);

    // simple function to print the usage of the tool
    void printUsage (const char* progName);
};
#endif
