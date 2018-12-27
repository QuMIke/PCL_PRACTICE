// plane_process.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <stdlib.h>
#include <cmath>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/common.h>
#include <pcl/common/norms.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/PassThrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
	// Read data
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("10/PCD_1.pcd", *cloud);
	std::cerr << "Original point cloud size: " << cloud->points.size() << std::endl;
	//pcl::visualization::CloudViewer s_viewer("source cloud");
	//s_viewer.showCloud(cloud);

	// The same rotation matrix as before; theta radians around Z axis
	float theta = M_PI;
	Eigen::Affine3f transform_ = Eigen::Affine3f::Identity();
	transform_.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_);
	//pcl::visualization::CloudViewer v_viewer("voxel cloud");
	//v_viewer.showCloud(transformed_cloud);

	// get characteristics of points cloud
	pcl::PointXYZ min;
	pcl::PointXYZ max;
	pcl::getMinMax3D(*transformed_cloud, min, max);
	std::cout << "min.x - max.x: " << min.x << "-" << max.x << endl;
	std::cout << "min.y - max.y: " << min.y << "-" << max.y << endl;
	std::cout << "min.z - max.z: " << min.z << "-" << max.z << endl;

	// pass through to remove points which its altitude is below fork's altitude
	float fork_altitude = -570.f;
	float pallet_height = 100.f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(transformed_cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(fork_altitude, fork_altitude+ pallet_height);
	pass.filter(*cloud_y);
	std::cerr << "passThorugh point cloud below fork altitude, it left:" 
		<< cloud_y->points.size() << " data points.\n" << std::endl;
	//pcl::visualization::CloudViewer f_viewer("y axis filtered cloud");
	//f_viewer.showCloud(cloud_y);
	
	// Pass through to remove data which is out range of z axis
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
	pass.setInputCloud(cloud_y);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(1300.0, 3000.0);
	pass.filter(*cloud_z);
	std::cerr << "Point cloud after z axis passThorugh filtering has:" 
		<< cloud_z->points.size() << " data points.\n" << std::endl;
	//pcl::visualization::CloudViewer f_viewer("z axis filtered cloud");
	//f_viewer.showCloud(cloud_z);

	// filtering outliners according sigma via passthrough filter 
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud_z, centroid);
	Eigen::Matrix3f covariance_matrix;
	pcl::computeCovarianceMatrixNormalized(*cloud_z, centroid, covariance_matrix);
	std::cout << "center: \n" << centroid << std::endl;
	std::cout << "covariance: \n" << covariance_matrix << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_range(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::getMinMax3D(*cloud_z, min, max);
	float sigma_z = sqrt(covariance_matrix(2, 2));
	if (sigma_z > 150)
	{
		pass.setInputCloud(cloud_z);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(min.z, centroid[2]);
		pass.filter(*cloud_in_range);
		std::cerr << "Point cloud after passThorugh filtering has:" 
			<< cloud_in_range->points.size() << " data points.\n" << std::endl;
	}
	else
	{
		pcl::copyPoint(*cloud_z, *cloud_in_range);
	}
	//pcl::visualization::CloudViewer f_viewer("filtered cloud");
	//f_viewer.showCloud(cloud_inRange);
	
	// filtering by sigma principle
	pcl::compute3DCentroid(*cloud_in_range, centroid);
	pcl::computeCovarianceMatrixNormalized(*cloud_in_range, centroid, covariance_matrix);
	std::cout << "new center: \n" << centroid << std::endl;
	std::cout << "new covariance: \n"<< covariance_matrix << std::endl;
	std::vector<int> outliers_index;
	for (size_t i = 0; i <  cloud_in_range->points.size(); i++)
	{
		float distance_z =abs(cloud_in_range->points[i].z - centroid[2]);
		if (distance_z > sigma_z)
		{
			outliers_index.push_back(i);
		}
	}
	boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(outliers_index);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_closer_range(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_in_range);
	extract.setIndices(index_ptr);
	extract.setNegative(true);
	extract.filter(*cloud_closer_range);
	std::cerr << "Point cloud probably inliers has:" 
		<< cloud_closer_range->points.size() << " data points.\n" << std::endl;
	//pcl::visualization::CloudViewer f_viewer("filtered cloud");
	//f_viewer.showCloud(cloud_closer_range);

	pcl::getMinMax3D(*cloud_closer_range, min, max);
	std::cout << "filtered clouds min.x - max.x: " << min.x << "-" << max.x << endl;
	std::cout << "filtered clouds min.y - max.y: " << min.y << "-" << max.y << endl;
	std::cout << "filtered clouds min.z - max.z: " << min.z << "-" << max.z << endl;

	// collect points which have similar height with center
	float mean_y = (max.y + min.y) / 2.f;
	std::vector<pcl::PointXYZ> pts_array;

	for (size_t i = 0; i < cloud_closer_range->points.size(); i++)
	{
		if (abs(cloud_closer_range->points[i].y - mean_y) <= 5)
		{
			pts_array.push_back(cloud_closer_range->points[i]);
		}
	}
	std::cerr << "number of collect points which have similar heigh with center:"
		<< pts_array.size() << " data points.\n" << std::endl;

	// sort every point according its x value in array
	if (pts_array.size() <= 0)
	{
		std::cerr << "there is been mistake about points which have similar heigh with center" << std::endl;
	}
	std::sort(pts_array.begin(), pts_array.end(), [](const pcl::PointXYZ& s1, const pcl::PointXYZ& s2) { return s1.x < s2.x; });

	// Get distance array between two adjacent points and record the prob boundry points
	std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> dist_array;
	for (size_t i = 0; i < pts_array.size() - 1; i++)
	{
		float distance = pow((pts_array[i].x - pts_array[i + 1].x), 2)
				+ pow((pts_array[i].y - pts_array[i + 1].y), 2) 
				+ pow((pts_array[i].z - pts_array[i + 1].z), 2);
		distance = sqrt(distance);
		if (distance >= 100)
		{
			dist_array.push_back(std::make_pair(pts_array[i], pts_array[i + 1]));
		}
	}
	std::cerr << "dist_array size: " << dist_array.size() << std::endl;


	//// Create the voxelGrid filtering object
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::VoxelGrid<pcl::PointXYZ> sor;
	//sor.setInputCloud(cloud);
	//sor.setLeafSize(8.0f, 8.0f, 8.0f);
	//sor.filter(*cloud_voxel);
	//std::cerr << "PointCloud after voxelGrid filtering: " << cloud_voxel->width * cloud_voxel->height
	//	<< " data points (" << pcl::getFieldsList(*cloud_voxel) << ").\n";
	//pcl::visualization::CloudViewer v_viewer("voxel cloud");
	//v_viewer.showCloud(cloud_voxel);

	//// normal extratcion
	//pcl::PointCloud<pcl::Normal>::Ptr closer_range_normals(new pcl::PointCloud<pcl::Normal>);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	//ne.setSearchMethod(tree);
	//ne.setInputCloud(cloud_closer_range);
	//ne.setKSearch(50);
	//ne.compute(*closer_range_normals);

	//// Create the segmentation objects and set all params
	//pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	//seg.setOptimizeCoefficients(true);
	//seg.setMethodType(pcl::SACMODEL_PLANE);
	//seg.setModelType(pcl::SAC_RANSAC);
	//seg.setMaxIterations(100);
	//seg.setDistanceThreshold(0.01);
	//seg.setInputCloud(cloud_closer_range);
	//seg.setInputNormals(closer_range_normals);

	//// Obtain the plane inliers and coefficients
	//pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	//seg.segment(*inliers_plane, *coefficients_plane);
	//std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	//// Compute different angle
	//float angle = acos(coefficients_plane->values[2]) * 180 / M_PI;
	//if (coefficients_plane->values[0] < 0 && coefficients_plane->values[1]<0)
	//{
	//	angle = -angle;
	//}
	//std::cerr << "Different angle between camera and object is: " << angle << std::endl;

	//// Extract the planar inliers from the input cloud
	//pcl::ExtractIndices<pcl::PointXYZ> extract;
	//extract.setInputCloud(cloud_inRange);
	//extract.setIndices(inliers_plane);
	//extract.setNegative(false);

	//// Obtain the plane point cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planes(new pcl::PointCloud<pcl::PointXYZ>());
	//extract.filter(*cloud_planes);
	//pcl::visualization::CloudViewer p_viewer("paralle planes cloud");
	//p_viewer.showCloud(cloud_planes);


	//// Project the model inliers
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::ProjectInliers<pcl::PointXYZ> proj;
	//proj.setModelType(pcl::SACMODEL_PLANE);
	//proj.setIndices(inliers_plane);
	//proj.setInputCloud(cloud_inRange);
	//proj.setModelCoefficients(coefficients_plane);
	//proj.filter(*cloud_projected);
	//std::cerr << "PointCloud after projection has: "
	//	<< cloud_projected->points.size() << " data points." << std::endl;


	//// Compute centroid of projected plane
	//Eigen::Vector4f center;
	//pcl::compute3DCentroid(*cloud_projected, center);
	//std::cout << "The center  coordinates of the projected plane are: ("
	//	<< center[0] << ", "
	//	<< center[1] << ", "
	//	<< center[2] << ")." << std::endl;

	//// get AABB
	//pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	//feature_extractor.setInputCloud(cloud_projected);
	//feature_extractor.compute();
	//pcl::PointXYZ min_point_OBB;
	//pcl::PointXYZ max_point_OBB;
	//pcl::PointXYZ position_OBB;
	//Eigen::Matrix3f rotational_matrix_OBB;
	//feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	//std::cerr << "OBB center: " << position_OBB.x << " " << position_OBB.y << " " << position_OBB.z << std::endl;

	//// Create Center line
	//pcl::ModelCoefficients line_obb;
	//line_obb.values.resize(6);    // We need 6 values
	//line_obb.values[0] = position_OBB.x;
	//line_obb.values[1] = position_OBB.y;
	//line_obb.values[2] = position_OBB.z;
	//line_obb.values[3] = coefficients_plane->values[0];
	//line_obb.values[4] = coefficients_plane->values[1];
	//line_obb.values[5] = coefficients_plane->values[2];

	//// Collect all points which has the same y value as center
	//std::vector<pcl::PointXYZ> ptsArr;
	//for (size_t i = 0; i < cloud_projected->points.size(); i++)
	//{
	//	if (abs(cloud_projected->points[i].y - position_OBB.y) * 1000 <= 5)
	//	{
	//		ptsArr.push_back(cloud_projected->points[i]);
	//	}
	//}
	//std::cerr << "The points array's size is: " << ptsArr.size() << std::endl;

	//// sort every point according its x value in array
	//std::sort(ptsArr.begin(), ptsArr.end(), [](const pcl::PointXYZ& s1, const pcl::PointXYZ& s2) { return s1.x < s2.x; });

	//// Get distance array between two adjacent points
	//std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> dist_array;
	//for (size_t i = 0; i < ptsArr.size() - 1; i++)
	//{
	//	float distance = pow(1000.f * (ptsArr[i].x - ptsArr[i + 1].x), 2)
	//		+ pow(1000.f * (ptsArr[i].y - ptsArr[i + 1].y), 2) + pow(1000.f * (ptsArr[i].z - ptsArr[i + 1].z), 2);
	//	distance = sqrt(distance);
	//	if (distance > 200)
	//	{
	//		dist_array.push_back(std::make_pair(ptsArr[i], ptsArr[i + 1]));
	//	}
	//}
	//std::cerr << "dist_array size: " << dist_array.size() << std::endl;

	//// Visualizae the result 
	boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_closer_range, "cloud");
	//viewer->addLine(line_obb);
	//for (size_t i = 0; i < dist_array.size(); i++)
	//{
		std::string line_name = "line_" + std::to_string(0);
		viewer->addLine(dist_array[0].first, dist_array[0].second, 255, 0, 0, line_name);
		line_name = "line_" + std::to_string(1);
		viewer->addLine(dist_array[1].first, dist_array[1].second, 255, 0, 0, line_name);
	//}
	viewer->addCoordinateSystem(1.0, "global");
	viewer->initCameraParameters();
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce();
	//}

	system("pause");

	return (0);
}

