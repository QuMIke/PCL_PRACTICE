#include "stdafx.h"
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/PassThrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
	// Read data
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("test.pcd", *cloud);
	std::cerr << "Original point cloud size: " << cloud->points.size() << std::endl;

	//pcl::visualization::CloudViewer s_viewer("source cloud");
	//s_viewer.showCloud(cloud);

	// Pass through  to remove range out data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(1, 2);
	pass.filter(*cloud_filtered);
	std::cerr << "Point cloud after filtering has:" << cloud_filtered->points.size() << " data points." << std::endl;
	
	//pcl::visualization::CloudViewer f_viewer("filtered cloud");
	//f_viewer.showCloud(cloud_filtered);

	// Estimate normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(30);
	ne.compute(*cloud_normals);

	// Create the segmentation objects and set all params



	system("pause");

    return 0;
}

