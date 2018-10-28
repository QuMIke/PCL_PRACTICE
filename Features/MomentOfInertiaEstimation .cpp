/*
In this tutorial we will learn how to use the pcl::MomentOfInertiaEstimation class 
in order to obtain descriptors based on eccentricity and moment of inertia.
This class also allows to extract axis aligned and oriented bounding boxes of the cloud. 
But keep in mind that extracted OBB is not the minimal possible bounding box.
*/

#include "stdafx.h"
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "KinnectGrabber.h"

int main()
{
	pcl::io::OpenNI2Grabber grabber;
	KinnectGrabber<pcl::PointXYZRGBA> v(grabber);
	v.run();

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud = v.getLatestCloud();

	if (cloud == nullptr)
	{
		std::cout << "Get cloud failed!" << std::endl;
	}
	else
	{
		// create moment estimator and extratct
		pcl::MomentOfInertiaEstimation<pcl::PointXYZRGBA> feature_extractor;
		feature_extractor.setInputCloud(cloud);
		feature_extractor.compute();

		// define moment and eccentricity etc variable
		std::vector<float> moment_of_intertia;
		std::vector<float> eccentricity;
		pcl::PointXYZRGBA min_point_AABB;
		pcl::PointXYZRGBA max_point_AABB;
		pcl::PointXYZRGBA min_point_OBB;
		pcl::PointXYZRGBA max_point_OBB;
		pcl::PointXYZRGBA position_OBB;
		Eigen::Matrix3f  rotational_matrix_OBB;
		float major_value, middle_value, minor_value;
		Eigen::Vector3f major_vector, middle_vector, minor_vector;
		Eigen::Vector3f mass_center;

		// get moment and eccentricity
		feature_extractor.getMomentOfInertia(moment_of_intertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);
		feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
		feature_extractor.getEigenValues(major_value, middle_value, minor_value);
		feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
		feature_extractor.getMassCenter(mass_center);

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();
		viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "sample cloud");
		viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

		Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
		Eigen::Quaternionf quat(rotational_matrix_OBB);
		viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

		pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
		pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
		pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
		pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
		viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
		viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
		viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}
	
	return 0;
}
