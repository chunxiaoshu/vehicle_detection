#include <iostream>
#include <ctime>
#include <cmath>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <Eigen/src/Core/Array.h>

using namespace std;

#define PI 3.14159265

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;

int main(int argc, char *argv[]) {
	// get calculate time
	clock_t start_Total, finish_Total;
	start_Total = clock();

	// [1] load pcd data
	cout << "loading pcd data..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("../../pcd_data/test_pcd80.pcd", *cloud_origin)) {	
		cout << "loading pcd data failed" << endl << endl;
		return -1;
	}
	else {
		cout << "loading pcd data success" << endl << endl;
		cout << "cloud size origin: " << cloud_origin->size() << endl;
	}

	// [2] passthrough filter
	// x passthrough filter
	pcl::PointCloud<PointT>::Ptr cloud_after_x_filter(new pcl::PointCloud<PointT>());
	pcl::ConditionAnd<PointT>::Ptr x_passthrough_filter_cond(new pcl::ConditionAnd<PointT>());
	x_passthrough_filter_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
		new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GE, -9.5)));
	x_passthrough_filter_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
		new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LE, 1.5)));
	pcl::ConditionalRemoval<PointT> cloud_filter_x(false);
	cloud_filter_x.setCondition(x_passthrough_filter_cond);
	cloud_filter_x.setInputCloud(cloud_origin);
	cloud_filter_x.setKeepOrganized(true);
	cloud_filter_x.filter(*cloud_after_x_filter);
	// y passthrough filter
	pcl::PointCloud<PointT>::Ptr cloud_after_y_filter(new pcl::PointCloud<PointT>());
	pcl::ConditionAnd<PointT>::Ptr y_passthrough_filter_cond(new pcl::ConditionAnd<PointT>());
	y_passthrough_filter_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
		new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GE, -1.5)));
	y_passthrough_filter_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
		new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LE, 1.5)));
	pcl::ConditionalRemoval<PointT> cloud_filter_y(false);
	cloud_filter_y.setCondition(y_passthrough_filter_cond);
	cloud_filter_y.setInputCloud(cloud_after_x_filter);
	cloud_filter_y.setKeepOrganized(true);
	cloud_filter_y.filter(*cloud_after_y_filter);
	// nan passthrough filter
	pcl::PointCloud<PointT>::Ptr cloud_remove_nan(new pcl::PointCloud<PointT>());
	std::vector<int> indices_Pulse;
	pcl::removeNaNFromPointCloud(*cloud_after_y_filter, *cloud_remove_nan, indices_Pulse);
	cout << "cloud size after passthrough filter: " << cloud_remove_nan->size() << endl << endl;

	// [3] voxel grid downsample
	pcl::PointCloud<PointT>::Ptr cloud_downsampled(cloud_remove_nan);
	// pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>);
	// pcl::VoxelGrid<PointT> downsampled;
	// downsampled.setInputCloud(cloud_remove_nan);
	// downsampled.setLeafSize(0.01, 0.01, 0.01);
	// downsampled.filter(*cloud_downsampled);
	// cout << "cloud size after downsampled: " << cloud_downsampled->size() << endl << endl;

	// [4] calculate normal
	cout << "calculate normal..." << endl;
	clock_t normal_start = clock();
	pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	pcl::NormalEstimationOMP<PointT, NormalT> ne;
	ne.setNumberOfThreads(2);

	// pcl::NormalEstimation<PointT, NormalT> ne;
	ne.setInputCloud(cloud_downsampled);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.25);
	ne.compute(*cloud_normals);
	cout << "time to calculate normal： " << (double)(clock() - normal_start)/CLOCKS_PER_SEC << "s" << endl << endl;

	// [5] get horizontal point
	cout << "get trunk subface..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_horizontal(new pcl::PointCloud<PointT>);
	for (int i = 0; i < cloud_normals->size(); i++) {
		if (cloud_downsampled->points[i].z <= 2 && 
			(cloud_normals->points[i].normal_z >= 0.9 || cloud_normals->points[i].normal_z <= -0.9) ) {
			cloud_horizontal->push_back(cloud_downsampled->points[i]);
		}
	}

	// [6] segment trunk subface point 
	cout << "segmenting..." << endl << endl;
	pcl::PointCloud<PointT>::Ptr cloud_trunk_subface(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_except_trunk_subface(new pcl::PointCloud<PointT>());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr plane_coefficients_trunk_subface(new pcl::ModelCoefficients);

	pcl::SACSegmentation<PointT> plane_trunk_subface;
	plane_trunk_subface.setOptimizeCoefficients(true);
	plane_trunk_subface.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_subface.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_subface.setDistanceThreshold(0.01);
	plane_trunk_subface.setInputCloud(cloud_horizontal);
	plane_trunk_subface.segment(*inliers, *plane_coefficients_trunk_subface);

	pcl::ExtractIndices<PointT> extract_trunk_subface;
	extract_trunk_subface.setInputCloud(cloud_horizontal);
	extract_trunk_subface.setIndices(inliers);
	extract_trunk_subface.setNegative(false);
	extract_trunk_subface.filter(*cloud_trunk_subface);
	cout << "coefficients of trunk_subface plane " << endl;
	char coeff[4] = { 'a', 'b', 'c', 'd' };
	for (size_t i = 0; i < plane_coefficients_trunk_subface->values.size(); ++i) {
		cout << "	" << coeff[i] << ":";
		cout << "	" << plane_coefficients_trunk_subface->values[i] << endl;
	}


	// [7] trunk subface correction 
	float normal_start_x = plane_coefficients_trunk_subface->values[0];
	float normal_start_y = plane_coefficients_trunk_subface->values[1];
	float normal_start_z = plane_coefficients_trunk_subface->values[2];
	float normal_end_x = 0, normal_end_y = 0, normal_end_z = 1;
	float rotation_ceter_x = 0, rotation_ceter_y = 0, rotation_ceter_z = 0;
	float rotation_axis_x = (normal_end_y - normal_start_y) * (rotation_ceter_z - normal_start_z) - 
							(rotation_ceter_y - normal_start_y) * (normal_end_z - normal_start_z);
	float rotation_axis_y = (normal_end_z - normal_start_z) * (rotation_ceter_x - normal_start_x) - 
							(rotation_ceter_z - normal_start_z) * (normal_end_x - normal_start_x);
	float rotation_axis_z = (normal_end_x - normal_start_x) * (rotation_ceter_y - normal_start_y) - 
							(rotation_ceter_x - normal_start_x) * (normal_end_y - normal_start_y);

	float square_sum = rotation_axis_x * rotation_axis_x +
					   rotation_axis_y * rotation_axis_y +
					   rotation_axis_z * rotation_axis_z;
	square_sum = sqrt(square_sum);
	rotation_axis_x /= square_sum;
	rotation_axis_y /= square_sum;
	rotation_axis_z /= square_sum;
	cout << "旋转法向为：(" << rotation_axis_x << "," << rotation_axis_y << "," << rotation_axis_z << ")" << endl;

	float rotation_angle = acos(plane_coefficients_trunk_subface->values[2]);
	cout << "与旋转法向之间的偏转角度为：" << rotation_angle / M_PI * 180 << "度" << endl;

	float x, y, z;
	x = rotation_axis_x;
	y = rotation_axis_y;
	z = rotation_axis_z;
	Eigen::Matrix4f rotation_Matrix = Eigen::Matrix4f::Identity();
	rotation_Matrix(0, 0) = cos(rotation_angle) + (1 - cos(rotation_angle))*x*x;
	rotation_Matrix(0, 1) = (1 - cos(rotation_angle))*x*y - sin(rotation_angle)*z;
	rotation_Matrix(0, 2) = (1 - cos(rotation_angle))*x*z + sin(rotation_angle)*y;
	rotation_Matrix(1, 0) = (1 - cos(rotation_angle))*x*y + sin(rotation_angle)*z;
	rotation_Matrix(1, 1) = cos(rotation_angle) + (1 - cos(rotation_angle))*y*y;
	rotation_Matrix(1, 2) = (1 - cos(rotation_angle))*y*z - sin(rotation_angle)*x;
	rotation_Matrix(2, 0) = (1 - cos(rotation_angle))*x*z - sin(rotation_angle)*y;
	rotation_Matrix(2, 1) = (1 - cos(rotation_angle))*y*z + sin(rotation_angle)*x;
	rotation_Matrix(2, 2) = cos(rotation_angle) + (1 - cos(rotation_angle))*z*z;
	printf("旋转矩阵R为：\n");
	printf("            | %6.3f %6.3f %6.3f | \n", rotation_Matrix(0, 0), rotation_Matrix(0, 1), rotation_Matrix(0, 2));
	printf("        R = | %6.3f %6.3f %6.3f | \n", rotation_Matrix(1, 0), rotation_Matrix(1, 1), rotation_Matrix(1, 2));
	printf("            | %6.3f %6.3f %6.3f | \n", rotation_Matrix(2, 0), rotation_Matrix(2, 1), rotation_Matrix(2, 2));
	printf("\n");
	
	pcl::PointCloud<PointT>::Ptr cloud_downsampled_after_revise(new pcl::PointCloud<PointT>());
	pcl::transformPointCloud(*cloud_downsampled, *cloud_downsampled_after_revise, rotation_Matrix);
	
	// [8] get trunk subface height
	pcl::PointCloud<PointT>::Ptr cloud_trunk_subface_after_revise(new pcl::PointCloud<PointT>());
	pcl::transformPointCloud(*cloud_trunk_subface, *cloud_trunk_subface_after_revise, rotation_Matrix);
	float trunk_subface_height = 0.0;
	for (int i = 0; i < cloud_trunk_subface_after_revise->size(); ++i) {
		trunk_subface_height += cloud_trunk_subface_after_revise->points[i].z;
	}
	trunk_subface_height /= cloud_trunk_subface_after_revise->size();
	cout << "货车底面高度为" << trunk_subface_height << endl << endl;

	// [9] segment trunk side
	pcl::PointCloud<PointT>::Ptr potential_trunk_plane_back(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_plane_back(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_trunk_plane_front(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_plane_front(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_trunk_plane_right(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_plane_right(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_trunk_plane_left(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_plane_left(new pcl::PointCloud<PointT>());
	int deckNormalSize = cloud_normals->size();
	for (int i = 0; i < deckNormalSize; i++) {
		if (cloud_normals->points[i].normal_x >= 0.9) potential_trunk_plane_back->push_back(cloud_downsampled_after_revise->points[i]);
		if (cloud_normals->points[i].normal_x <= -0.9) potential_trunk_plane_front->push_back(cloud_downsampled_after_revise->points[i]);
		if (cloud_normals->points[i].normal_y >= 0.9) potential_trunk_plane_right->push_back(cloud_downsampled_after_revise->points[i]);
		if (cloud_normals->points[i].normal_y <= -0.9) potential_trunk_plane_left->push_back(cloud_downsampled_after_revise->points[i]);
	}

	float trunkplane_Xmin = 0.0, trunkplane_Xmax = 0.0, trunkplane_Ymin = 0.0, trunkplane_Ymax = 0.0;

	pcl::PointIndices::Ptr trunkplane_inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr trunkplane_coefficients(new pcl::ModelCoefficients);

	pcl::SACSegmentation<PointT> trunk_plane_seg;
	trunk_plane_seg.setOptimizeCoefficients(true);
	trunk_plane_seg.setModelType(pcl::SACMODEL_PLANE);
	trunk_plane_seg.setMethodType(pcl::SAC_RANSAC);
	trunk_plane_seg.setDistanceThreshold(0.01);

	pcl::ExtractIndices<PointT> trunkplane_extract;
	trunkplane_extract.setNegative(false);

	trunk_plane_seg.setInputCloud(potential_trunk_plane_back);
	trunk_plane_seg.segment(*trunkplane_inliers, *trunkplane_coefficients);
	trunkplane_extract.setInputCloud(potential_trunk_plane_back);
	trunkplane_extract.setIndices(trunkplane_inliers);
	trunkplane_extract.filter(*trunk_plane_back);
	for (int i = 0; i < trunk_plane_back->size(); i++) {
		trunkplane_Xmin += trunk_plane_back->points[i].x;
	}
	trunkplane_Xmin /= trunk_plane_back->size();
	cout << "货车后侧面x位置为" << trunkplane_Xmin << endl << endl;
	// pcl::io::savePCDFile("../../data/cloud_back.pcd", *trunk_plane_right, false);

	trunk_plane_seg.setInputCloud(potential_trunk_plane_front);
	trunk_plane_seg.segment(*trunkplane_inliers, *trunkplane_coefficients);
	trunkplane_extract.setInputCloud(potential_trunk_plane_front);
	trunkplane_extract.setIndices(trunkplane_inliers);
	trunkplane_extract.filter(*trunk_plane_front);
	for (int i = 0; i < trunk_plane_front->size(); i++) {
		trunkplane_Xmax += trunk_plane_front->points[i].x;
	}
	trunkplane_Xmax /= trunk_plane_front->size();
	cout << "货车前侧面x位置为" << trunkplane_Xmax << endl << endl;
	// pcl::io::savePCDFile("../../data/cloud_front.pcd", *trunk_plane_right, false);

	trunk_plane_seg.setInputCloud(potential_trunk_plane_right);
	trunk_plane_seg.segment(*trunkplane_inliers, *trunkplane_coefficients);
	trunkplane_extract.setInputCloud(potential_trunk_plane_right);
	trunkplane_extract.setIndices(trunkplane_inliers);
	trunkplane_extract.filter(*trunk_plane_right);
	for (int i = 0; i < trunk_plane_right->size(); i++) {
		trunkplane_Ymin += trunk_plane_right->points[i].y;
	}
	trunkplane_Ymin /= trunk_plane_right->size();
	cout << "货车左侧面y位置为" << trunkplane_Ymin << endl << endl;
	// pcl::io::savePCDFile("../../data/cloud_left.pcd", *trunk_plane_right, false);

	trunk_plane_seg.setInputCloud(potential_trunk_plane_left);
	trunk_plane_seg.segment(*trunkplane_inliers, *trunkplane_coefficients);
	trunkplane_extract.setInputCloud(potential_trunk_plane_left);
	trunkplane_extract.setIndices(trunkplane_inliers);
	trunkplane_extract.filter(*trunk_plane_left);
	for (int i = 0; i < trunk_plane_left->size(); i++) {
		trunkplane_Ymax += trunk_plane_left->points[i].y;
	}
	trunkplane_Ymax /= trunk_plane_left->size();
	cout << "货车右侧面y位置为" << trunkplane_Ymax << endl << endl;
	// pcl::io::savePCDFile("../../data/cloud_right.pcd", *trunk_plane_right, false);

	// [10] result
	float trunk_length = trunkplane_Xmax - trunkplane_Xmin;
	float trunk_width = trunkplane_Ymax - trunkplane_Ymin;
	cout << "货车的长度为" << trunk_length << endl << endl;
	cout << "货车的宽度为" << trunk_width << endl << endl;

	pcl::visualization::PCLVisualizer viewers("Cloud Viewer");
	viewers.addCoordinateSystem();
	viewers.setBackgroundColor(0.0, 0.0, 0.0);

	// viewers.addPointCloudNormals<pcl::PointXYZ, NormalT>(cloud_downsampled, cloud_normals);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> oricloud_color_handlers(cloud_downsampled_after_revise, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_downsampled_after_revise, oricloud_color_handlers, "original cloud");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud1_color_handlers(cloud_trunk_subface_after_revise, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_trunk_subface_after_revise, cloud1_color_handlers, "cloud1");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud2_color_handlers(trunk_plane_back, 0, 255, 0);
	viewers.addPointCloud<PointT>(trunk_plane_back, cloud2_color_handlers, "cloud2");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud3_color_handlers(trunk_plane_front, 0, 0, 255);
	viewers.addPointCloud<PointT>(trunk_plane_front, cloud3_color_handlers, "cloud3");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud4_color_handlers(trunk_plane_right, 255, 255, 0);
	viewers.addPointCloud<PointT>(trunk_plane_right, cloud4_color_handlers, "cloud4");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud5_color_handlers(trunk_plane_left, 0, 255, 255);
	viewers.addPointCloud<PointT>(trunk_plane_left, cloud5_color_handlers, "cloud5");

	viewers.spin();
	return 0;
}