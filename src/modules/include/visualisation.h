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
#include <iostream>

#ifndef VISUALISATION_H
#define VISUALISATION_H
class Visualisation{
	private:
		// background color information
		double bg_red;
		double bg_blue;
		double bg_green;
		// viewport information
		double	vp_xmin;
		double	vp_ymin;
		double	vp_xmax;
		double	vp_ymax;
		int		vp_id;
	public:
		// def constructor
		Visualisation();
		// deconstructor
		~Visualisation();
		// test function to print usage information
		void printUsage (const char* progName);
		// function to display a single point cloud
		// pcl::PointXYZ : struct to represent (Euclidean) XYZ coordinate
		boost::shared_ptr<pcl::visualization::PCLVisualizer> singleViewer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
		// function to display two point clouds - side by side - equivalent to subplot
		boost::shared_ptr<pcl::visualization::PCLVisualizer> dualViewer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_2);
		// function to visualise the point cloud
		void visualiseCloud (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
};
#endif
