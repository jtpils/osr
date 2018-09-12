#include "visualisation.h"
#include <iostream>
#include <boost/thread/thread.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/console/parse.h>

int main (int argc, char** argv)
{
	// support to view two point clouds
	bool dual_view = false;

	// create Visualisation object
	Visualisation viz;

	// create two point cloud pointers
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr_1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr_2 (new pcl::PointCloud<pcl::PointXYZ>);

	// take into account the filename extensio
	std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");

	// if the indices are empty -> then error, print the usage information - IE_010
	if (pcd_filename_indices.empty ())
	{
		 viz.printUsage (argv[0]);
		 return 0;
	}
	else
	{ // else IE_010
		 // Read file .pcd
		 std::string filename = argv[pcd_filename_indices[0]];
		 if (pcl::io::loadPCDFile (filename, *point_cloud_ptr_1) == -1)
		 {
				std::cout << "Was not able to open file \""<<filename<<"\".\n";
				viz.printUsage (argv[0]);
				return 0;
		 }
		 // user provides more than one pcd
		 if (argc > 2)
		 {
				dual_view = true;
				// Read the second file .pcd
				std::string filename = argv[pcd_filename_indices[1]];
				if (pcl::io::loadPCDFile (filename, *point_cloud_ptr_2) == -1)
				{
					 std::cout << "Was not able to open file \""<<filename<<"\".\n";
					 viz.printUsage (argv[0]);
					 return 0;
				}
		 }
	}

	// shared pointer of type PCLVisualizer class
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	// check if the argument provided by the user
	if (dual_view) /* supports dual viewer */
	{
		viewer = viz.dualViewer(point_cloud_ptr_1,point_cloud_ptr_2);
	}
	else /* supports single viewer only */
	{
		viewer = viz.singleViewer(point_cloud_ptr_1);
	}

	// callback to visualize the point clouds
	viz.visualiseCloud (viewer);

} // end main
