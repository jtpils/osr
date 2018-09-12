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
#include "./include/visualisation.h"
#include "./include/reg_grow_segmentation.h"

int main (int argc, char** argv)
{
  // get the filename of the pcd
  std::string filename;

  // create RGSeg class object
  RGSeg rgs;

  // create point cloud pointer
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud <pcl::PointXYZRGB>);

  // take into account the filename extension
	std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");

  // if the indices are empty -> then error, print the usage information - IE_010
	if (pcd_filename_indices.empty ())
	{
		 std::cout << "Could not read from pcd file. Are you sure you supplied a filename (*.pcd) while running the application?" << std::endl;
     rgs.printUsage(argv[0]);
		 return 0;
	}
  else
  {
    // else store the filename in a string, and pass it to loadPCDFile function
    std::string filename = argv[pcd_filename_indices[0]];
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (filename, *point_cloud_ptr) == -1)
    {
      std::cout << "Error! Could not read cloud data" << std::endl;
      rgs.printUsage(argv[0]);
      return (-1);
    }
  }

  // perform region growing monochrome operation

  // function prototype
  // regionGrowingRGB (pcl::PointCloud <pcl::PointXYZRGB>::Ptr pt_cloud,
  // float dist_threshold,
  // float pt_color_thresh,
  // float reg_color_thresh,
  // int min_cluster_size)
  rgs.regionGrowingRGB (point_cloud_ptr, 10.0, 6.0, 5.0, 600);

  return (0);
} // end main
