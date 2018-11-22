#include "stdafx.h"
#include <stdlib.h>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/PassThrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
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

	// using BilateralFilter filtering point cloud before computing normals
	
	// Estimate normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(30);
	ne.compute(*cloud_normals);

	// Create the segmentation objects and set all params
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SACMODEL_PARALLEL_PLANE);
	seg.setModelType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);

	// Obtain the plane inliers and coefficients
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	seg.segment(*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// Obtain the plane point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planes(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_planes);
	
	pcl::visualization::CloudViewer p_viewer("paralle planes cloud");
	p_viewer.showCloud(cloud_planes);

	// Compute different angle
	float angle = acos(coefficients_plane->values[2]) * 180 / M_PI;
	if (coefficients_plane->values[0] < 0 && coefficients_plane->values[1]<0)
	{
		angle = -angle;
	}
	std::cerr << "Different angle between camera and object is: " << angle << std::endl;

		// Project the model inliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setIndices(inliers_plane);
	proj.setInputCloud(cloud_filtered);
	proj.setModelCoefficients(coefficients_plane);
	proj.filter(*cloud_projected);
	std::cerr << "PointCloud after projection has: "
		<< cloud_projected->points.size() << " data points." << std::endl;

	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_projected);
	chull.setAlpha(0.1);
	chull.reconstruct(*cloud_hull);
	std::cerr << "Concave hull has: " << cloud_hull->points.size()
		<< " data points." << std::endl;

	pcl::visualization::CloudViewer h_viewer("Hull cloud");
	h_viewer.showCloud(cloud_hull);
	
	system("pause");

    return 0;
}

