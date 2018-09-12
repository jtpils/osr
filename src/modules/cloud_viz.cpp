/*
  Author: Saad Abdullah, Radiah Rivu
  Modified: Nitin Deshpande
  Code segment to display one or two PCDs - equivalent to plot or subplot.
  requires PCL 1.7
*/

#include <boost/thread/thread.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/console/parse.h>

//////////////////////////////////
// Help
//////////////////////////////////
void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" filename1.pcd [filename2.pcd]\n\n";
}

/////////////////////////////////////
// Open 3D viewer and add point cloud
/////////////////////////////////////
boost::shared_ptr<pcl::visualization::PCLVisualizer> View1 (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  return (viewer);
}

/////////////////////////////////////////////////////
// Open 3D viewer with two views and add point clouds
/////////////////////////////////////////////////////
boost::shared_ptr<pcl::visualization::PCLVisualizer> View2 (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Point Cloud1", 10, 10, "v1 text", v1);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Point Cloud2", 10, 10, "v2 text", v2);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "sample cloud2", v2);


  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  return (viewer);
}

//////////////////////////////////
// Main
//////////////////////////////////
int main (int argc, char** argv)
{
   bool single_view = true;


   pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud2_ptr (new pcl::PointCloud<pcl::PointXYZ>);


   //----------------------------------------------------
   // Parse Command Line Arguments: one or two files .pcd
   // and Read files .pcd
   //----------------------------------------------------
   std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
   if (pcd_filename_indices.empty ()) {
      printUsage (argv[0]);
      return 0;
   }
   else {
      // Read file .pcd
      std::string filename = argv[pcd_filename_indices[0]];
      if (pcl::io::loadPCDFile (filename, *point_cloud_ptr) == -1) {
         std::cout << "Was not able to open file \""<<filename<<"\".\n";
         printUsage (argv[0]);
         return 0;
      }

      if (argc > 2) {
         single_view = false;

         // Read the second file .pcd
         std::string filename = argv[pcd_filename_indices[1]];
         if (pcl::io::loadPCDFile (filename, *point_cloud2_ptr) == -1) {
            std::cout << "Was not able to open file \""<<filename<<"\".\n";
            printUsage (argv[0]);
            return 0;
         }
      }
   }

   //----------------------------------------------------
   // Visualize point clouds
   //----------------------------------------------------
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


   if (single_view)
      viewer = View1(point_cloud_ptr);
   else
      viewer = View2(point_cloud_ptr, point_cloud2_ptr);


   //----------------------------------------------------
   // Main loop
   //----------------------------------------------------
   while (!viewer->wasStopped ()) {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
}
