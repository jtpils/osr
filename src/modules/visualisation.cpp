/*
  Author: Saad Abdullah, Radiah Rivu
  Modified: Nitin Deshpande
  Code segment to display one or two PCDs - equivalent to plot or subplot.
  requires PCL 1.7
*/

#include "./include/visualisation.h"

// default constructor
Visualisation::Visualisation()
{
	// do something
}

// deconstructor
Visualisation::~Visualisation()
{
	// do nothing
}

// test function
void Visualisation::printUsage (const char* progName)
{
	std::cout << "You may have encountered an error or a warning. Here is how you can use the tool" <<std::endl;
	std::cout << "Usage: "<< progName <<" filename1.pcd | [filename2.pcd]"<< std::endl;
}

// type <class_name>::<method_name>()
boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualisation::singleViewer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	// add a 3D axes describing a coordinate system to screen at 0,0,0
	viewer->addCoordinateSystem (1.0);
 	viewer->initCameraParameters ();
	return (viewer);
}

// type <class_name>::<method_name>()
boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualisation::dualViewer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_2)
{
	// init viewer pointer to display the point cloud
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->initCameraParameters ();
	int v1(0); /* id of first viewport */
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor (0, 0, 0, v1);
	viewer->addText("PTCLD_1", 10, 10, "v1 text", v1);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_1, "Dual Viewer PTCLD 1 OF 2", v1);
	int v2(0); /* id of second viewport */
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
	viewer->addText("PTCLD_2", 10, 10, "v2 text", v2);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_2, "Dual Viewer PTCLD 2 OF 2", v2);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
	// add a 3D axes describing a coordinate system to screen at 0,0,0
	viewer->addCoordinateSystem (1.0);
	return (viewer);
}

// function to visualise the point cloud
// viewer : visualisation class object
void Visualisation::visualiseCloud (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	// callback like approach to visualise the cloud
	while (!viewer->wasStopped ())
	{
		// calls the interactor and updates the screen once
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}
