#include "stdafx.h"
#include <stdlib.h>
#include <cmath>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/PassThrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

//#include <pcl/surface/concave_hull.h>
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
	reader.read("1/PCD_1.pcd", *cloud);
	std::cerr << "Original point cloud size: " << cloud->points.size() << std::endl;
	//pcl::visualization::CloudViewer s_viewer("source cloud");
	//s_viewer.showCloud(cloud);

	// Create the voxelGrid filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(6.0f, 6.0f, 6.0f);
	sor.filter(*cloud_voxel);
	std::cerr << "PointCloud after voxelGrid filtering: " << cloud_voxel->width * cloud_voxel->height
		<< " data points (" << pcl::getFieldsList(*cloud_voxel) << ").\n";
	//pcl::visualization::CloudViewer v_viewer("voxel cloud");
	//v_viewer.showCloud(cloud_voxel);

	// Pass through to remove range out data 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_voxel);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(1300.0, 3000.0);
	pass.filter(*cloud_filtered);
	std::cerr << "Point cloud after passThorugh filtering has:" << cloud_filtered->points.size() << " data points.\n" << std::endl;
	//pcl::visualization::CloudViewer f_viewer("filtered cloud");
	//f_viewer.showCloud(cloud_filtered);

	// get range of points cloud
	pcl::PointXYZ min;
	pcl::PointXYZ max;
	pcl::getMinMax3D(*cloud_filtered, min, max);
	std::cout << "min.x - max.x: " << min.x << "-"<<max.x << endl;
	std::cout << "min.y - max.y: " << min.y << "-" << max.y << endl;
	std::cout << "min.z- max.z: " << min.z << "-" << max.z << endl;

	////// using BilateralFilter filtering point cloud before computing normals

	// Estimate normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// filtering groud
	std::vector<int> g_index;
	for (size_t i = 0; i < cloud_filtered->points.size(); i++)
	{
		if (abs(cloud_filtered->points[i].y - max.y) < 100)
		{
			Eigen::Vector3f ground_normal = Eigen::Vector3f(0, 1, 0);
			Eigen::Vector3f ground_normal_ = Eigen::Vector3f(0, -1, 0);
			Eigen::Vector3f point_normal = Eigen::Vector3f(cloud_normals->points[i].normal_x, cloud_normals->points[i].normal_y, cloud_normals->points[i].normal_z);
			if (pcl::getAngle3D(point_normal, ground_normal, true)<30.0f || pcl::getAngle3D(point_normal, ground_normal_, true)<45.0f)
			{
				g_index.push_back(i);
			}
		}
	}
	boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(g_index);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(index_ptr);
	extract.setNegative(true);
	extract.filter(*cloud_non_ground);
	std::cerr << "Point cloud non_ground has:" << cloud_non_ground->points.size() << " data points.\n" << std::endl;
	pcl::visualization::CloudViewer f_viewer("filtered cloud");
	f_viewer.showCloud(cloud_non_ground);

	//// Create the segmentation objects and set all params
	//pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	//seg.setOptimizeCoefficients(true);
	//seg.setMethodType(pcl::SACMODEL_PARALLEL_PLANE);
	//seg.setModelType(pcl::SAC_RANSAC);
	//seg.setMaxIterations(100);
	//seg.setDistanceThreshold(0.01);
	//seg.setInputCloud(cloud_filtered);
	//seg.setInputNormals(cloud_normals);

	//// Obtain the plane inliers and coefficients
	//pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	//seg.segment(*inliers_plane, *coefficients_plane);
	//std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	//// Extract the planar inliers from the input cloud
	//pcl::ExtractIndices<pcl::PointXYZ> extract;
	//extract.setInputCloud(cloud_filtered);
	//extract.setIndices(inliers_plane);
	//extract.setNegative(false);

	//// Obtain the plane point cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planes(new pcl::PointCloud<pcl::PointXYZ>());
	//extract.filter(*cloud_planes);
	////pcl::visualization::CloudViewer p_viewer("paralle planes cloud");
	////p_viewer.showCloud(cloud_planes);

	//// Compute different angle
	//float angle = acos(coefficients_plane->values[2]) * 180 / M_PI;
	//if (coefficients_plane->values[0] < 0 && coefficients_plane->values[1]<0)
	//{
	//	angle = -angle;
	//}
	//std::cerr << "Different angle between camera and object is: " << angle << std::endl;

	//// Project the model inliers
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::ProjectInliers<pcl::PointXYZ> proj;
	//proj.setModelType(pcl::SACMODEL_PLANE);
	//proj.setIndices(inliers_plane);
	//proj.setInputCloud(cloud_filtered);
	//proj.setModelCoefficients(coefficients_plane);
	//proj.filter(*cloud_projected);
	//std::cerr << "PointCloud after projection has: "
	//	<< cloud_projected->points.size() << " data points." << std::endl;

	////// Create a Concave Hull representation of the projected inliers
	////pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	////pcl::ConcaveHull<pcl::PointXYZ> chull;
	////chull.setInputCloud(cloud_projected);
	////chull.setAlpha(0.1);
	////chull.reconstruct(*cloud_hull);
	////std::cerr << "Concave hull has: " << cloud_hull->points.size()
	////	<< " data points." << std::endl;
	////pcl::visualization::CloudViewer h_viewer("Hull cloud");
	////h_viewer.showCloud(cloud_hull);

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
	//std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> disArr;
	//for (size_t i = 0; i < ptsArr.size() - 1; i++)
	//{
	//	float distance = pow(1000.f * (ptsArr[i].x - ptsArr[i + 1].x), 2)
	//		+ pow(1000.f * (ptsArr[i].y - ptsArr[i + 1].y), 2) + pow(1000.f * (ptsArr[i].z - ptsArr[i + 1].z), 2);
	//	distance = sqrt(distance);
	//	if (distance > 200)
	//	{
	//		disArr.push_back(std::make_pair(ptsArr[i], ptsArr[i + 1]));
	//	}
	//}
	//std::cerr << "DisArr size: " << disArr.size() << std::endl;

	//// Visualizae the result 
	//boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_projected, "cloud");
	//viewer->addLine(line_obb);
	//for (size_t i = 0; i < disArr.size(); i++)
	//{
	//	std::string line_name = "line_" + std::to_string(i);
	//	viewer->addLine(disArr[i].first, disArr[i].second, 255, 0, 0, line_name);
	//}
	//viewer->addCoordinateSystem(1.0, "global");
	//viewer->initCameraParameters();
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce();
	//}

	system("pause");

	return (0);
}
