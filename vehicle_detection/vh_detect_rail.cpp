#include <iostream>
#include <ctime>
#include <cmath>
#include <vector>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <Eigen/src/Core/Array.h>

#define SHOW_PLANE_COEFF
#define PI 3.14159265

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
using namespace std;

float get_height(pcl::PointCloud<PointT>::Ptr &cloud_plane, pcl::PointCloud<PointT>::Ptr &cloud_line) {
	const size_t high_lengh = 3000;
	const size_t height_step = 500;
	size_t height_cnt = 0;
	size_t height_max = 0;
	float height_ava = 0;
	float plane_high[high_lengh] = {0};
	size_t plane_high_idx[high_lengh] = {0};
	size_t plane_high_distribute[height_step] = {0};
	for (size_t i = 0; i < cloud_plane->size(); ++i) {
		size_t tmp = static_cast<int>((cloud_plane->points[i].x + 13) * 100);
		if (plane_high[tmp] < cloud_plane->points[i].z) {
			plane_high[tmp] = cloud_plane->points[i].z;
			plane_high_idx[tmp] = i;
		}
	}

	for (size_t i = 0; i < high_lengh; ++i) {
		if (plane_high[i] > 1.0 && plane_high[i] <= height_step) {
			size_t tmp = static_cast<int>(plane_high[i] * 100);
			++plane_high_distribute[tmp];
		}
	}

	for (size_t i = 1; i < height_step; ++i) {
		if (height_cnt < plane_high_distribute[i]) {
			height_cnt = plane_high_distribute[i];
			height_max = i;
		}
	}
	float height_tmp = static_cast<float>(height_max) / 100;

	for (size_t i = 0; i < cloud_plane->size(); ++i) {
		if (abs(cloud_plane->points[i].z - height_tmp) < 0.04) {
			cloud_line->push_back(cloud_plane->points[i]);
			height_ava += cloud_plane->points[i].z;
		}
	}

	return height_ava / cloud_line->size() + 0.005;
}

float get_length(pcl::PointCloud<PointT>::Ptr &cloud_plane, pcl::PointCloud<PointT>::Ptr &cloud_fb) {
	const size_t high_lengh = 3000;
  size_t num_max = 0;
  vector<float> half_max_x;
	size_t plane_high_distribute[high_lengh] = {0};
	for (size_t i = 0; i < cloud_plane->size(); ++i) {
		size_t tmp = static_cast<size_t>((cloud_plane->points[i].x + 13) * 100);
		++plane_high_distribute[tmp];
	}

	for (size_t i = 1; i < high_lengh; ++i) {
		if (num_max < plane_high_distribute[i]) {
			num_max = plane_high_distribute[i];
		}
	}

  num_max /= 3;
  // cout << "num_max" << num_max << endl;
  for (size_t i = 1; i < high_lengh; ++i) {
    // cout << "i" << i << "\t" << plane_high_distribute[i] << "\t";
		if (plane_high_distribute[i-1] < num_max && plane_high_distribute[i] > num_max
        || plane_high_distribute[i-1] > num_max && plane_high_distribute[i] < num_max) {
			half_max_x.push_back( static_cast<float>(i) / 100 - 13.0 );
		}
	}
  // cout << "half_max_x" << half_max_x.front() << endl;
  // cout << "half_max_x" << half_max_x.back() << endl;
	for (size_t i = 0; i < cloud_plane->size(); ++i) {
		if (abs(cloud_plane->points[i].x - half_max_x.back()) < 0.01 || abs(cloud_plane->points[i].x - half_max_x.front()) < 0.01) {
			cloud_fb->push_back(cloud_plane->points[i]);
		}
	}

	return half_max_x.back() - half_max_x.front();
}


int main(int argc, char *argv[]) {
	// get calculate time
	clock_t start_Total, finish_Total;
	start_Total = clock();

	// [1] load pcd data
	cout << "loading pcd data..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("../../pcd_data/test_pcd_rail.pcd", *cloud_origin)) {
		cout << "loading pcd data failed" << endl << endl;
		return -1;
	}
	else {
		cout << "loading pcd data success" << endl << endl;
		cout << "cloud size origin: " << cloud_origin->size() << endl;
	}

	// [2] remove points on the ground
	pcl::PointCloud<PointT>::Ptr cloud_remove_nan(new pcl::PointCloud<PointT>());
	std::vector<int> indices_Pulse;
	pcl::removeNaNFromPointCloud(*cloud_origin, *cloud_remove_nan, indices_Pulse);

	pcl::PointCloud<PointT>::Ptr cloud_remove_ground(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < cloud_remove_nan->size(); ++i) {
		if ( cloud_remove_nan->points[i].z > 0.5) {
			cloud_remove_ground->push_back(cloud_remove_nan->points[i]);
		}
	}
  cout << "cloud size remove ground: " << cloud_remove_ground->size() << endl;


	// [3] voxel grid downsample
	pcl::PointCloud<PointT>::Ptr cloud_downsampled(cloud_remove_ground);
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
	pcl::NormalEstimation<PointT, NormalT> ne;
	ne.setInputCloud(cloud_downsampled);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.25);
	ne.compute(*cloud_normals);
	cout << "time to calculate normal: " << (double)(clock() - normal_start)/CLOCKS_PER_SEC << "s" << endl;
	

	// [5] segment trunk subface point and get trunk subface height
	pcl::PointCloud<PointT>::Ptr cloud_subface(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < cloud_normals->size(); i++) {
		if (cloud_downsampled->points[i].z <= 2 && 
			(cloud_normals->points[i].normal_z >= 0.9 || cloud_normals->points[i].normal_z <= -0.9) ) {
			cloud_subface->push_back(cloud_downsampled->points[i]);
		}
	}

	pcl::PointCloud<PointT>::Ptr cloud_trunk_subface(new pcl::PointCloud<PointT>());
	pcl::PointIndices::Ptr inliers_subface(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr plane_coefficients_trunk_subface(new pcl::ModelCoefficients);

	pcl::SACSegmentation<PointT> plane_trunk_subface;
	plane_trunk_subface.setOptimizeCoefficients(true);
	plane_trunk_subface.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_subface.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_subface.setDistanceThreshold(0.01);
	plane_trunk_subface.setInputCloud(cloud_subface);
	plane_trunk_subface.segment(*inliers_subface, *plane_coefficients_trunk_subface);

	pcl::ExtractIndices<PointT> extract_trunk_subface;
	extract_trunk_subface.setInputCloud(cloud_subface);
	extract_trunk_subface.setIndices(inliers_subface);
	extract_trunk_subface.setNegative(false);
	extract_trunk_subface.filter(*cloud_trunk_subface);
#ifdef SHOW_PLANE_COEFF
	cout << "coefficients of trunk subface plane " << endl;
	const char coeff_plane[4] = { 'a', 'b', 'c', 'd' };
	for (size_t i = 0; i < plane_coefficients_trunk_subface->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << plane_coefficients_trunk_subface->values[i] << endl;
	}
#endif

	float trunk_subface_height = 0.0;
	for (size_t i = 0; i < cloud_trunk_subface->size(); ++i) {
		trunk_subface_height += cloud_trunk_subface->points[i].z;
	}
	trunk_subface_height /= cloud_trunk_subface->size();
	cout << "The height of trunk subface height: " << trunk_subface_height << endl;


  // [6] segment trunk head point and get trunk head height
	pcl::PointCloud<PointT>::Ptr cloud_head(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < cloud_normals->size(); i++) {
		if (cloud_downsampled->points[i].z > 2 && 
			(cloud_normals->points[i].normal_z >= 0.9 || cloud_normals->points[i].normal_z <= -0.9) ) {
			cloud_head->push_back(cloud_downsampled->points[i]);
		}
	}
	pcl::PointCloud<PointT>::Ptr cloud_trunk_head(new pcl::PointCloud<PointT>());
	pcl::PointIndices::Ptr inliers_head(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr plane_coefficients_trunk_head(new pcl::ModelCoefficients);

	pcl::SACSegmentation<PointT> plane_trunk_head;
	plane_trunk_head.setOptimizeCoefficients(true);
	plane_trunk_head.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_head.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_head.setDistanceThreshold(0.01);
	plane_trunk_head.setInputCloud(cloud_head);
	plane_trunk_head.segment(*inliers_head, *plane_coefficients_trunk_head);

	pcl::ExtractIndices<PointT> extract_trunk_head;
	extract_trunk_head.setInputCloud(cloud_head);
	extract_trunk_head.setIndices(inliers_head);
	extract_trunk_head.setNegative(false);
	extract_trunk_head.filter(*cloud_trunk_head);
#ifdef SHOW_PLANE_COEFF
	cout << "coefficients of trunk head plane " << endl;
	for (size_t i = 0; i < plane_coefficients_trunk_head->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << plane_coefficients_trunk_head->values[i] << endl;
	}
#endif

	float trunk_head_height = 0.0;
	for (size_t i = 0; i < cloud_trunk_head->size(); ++i) {
		trunk_head_height += cloud_trunk_head->points[i].z;
	}
	trunk_head_height /= cloud_trunk_head->size();
	cout << "The height of trunk head height: " << trunk_head_height << endl;


	// [7] segment trunk left and right plane
	pcl::PointCloud<PointT>::Ptr potential_trunk_plane_right(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_plane_right(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_trunk_plane_left(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_plane_left(new pcl::PointCloud<PointT>());
	for (size_t i = 0; i < cloud_normals->size(); ++i) {
		if (cloud_normals->points[i].normal_y >= 0.9) potential_trunk_plane_right->push_back(cloud_downsampled->points[i]);
		if (cloud_normals->points[i].normal_y <= -0.9) potential_trunk_plane_left->push_back(cloud_downsampled->points[i]);
	}

  pcl::ModelCoefficients::Ptr plane_coefficients_trunk_left(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_left(new pcl::PointIndices);

	pcl::SACSegmentation<PointT> plane_trunk_left;
	plane_trunk_left.setOptimizeCoefficients(true);
	plane_trunk_left.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_left.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_left.setDistanceThreshold(0.01);
  plane_trunk_left.setInputCloud(potential_trunk_plane_left);
	plane_trunk_left.segment(*inliers_left, *plane_coefficients_trunk_left);

	pcl::ExtractIndices<PointT> extract_trunk_left;
	extract_trunk_left.setNegative(false);
	extract_trunk_left.setInputCloud(potential_trunk_plane_left);
	extract_trunk_left.setIndices(inliers_left);
	extract_trunk_left.filter(*trunk_plane_left);

#ifdef SHOW_PLANE_COEFF
	cout << "coefficients of trunk left plane " << endl;
	for (size_t i = 0; i < plane_coefficients_trunk_left->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << plane_coefficients_trunk_left->values[i] << endl;
	}
#endif

	pcl::ModelCoefficients::Ptr plane_coefficients_trunk_right(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_right(new pcl::PointIndices);

	pcl::SACSegmentation<PointT> plane_trunk_right;
	plane_trunk_right.setOptimizeCoefficients(true);
	plane_trunk_right.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_right.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_right.setDistanceThreshold(0.01);
  plane_trunk_right.setInputCloud(potential_trunk_plane_right);
	plane_trunk_right.segment(*inliers_right, *plane_coefficients_trunk_right);

	pcl::ExtractIndices<PointT> extract_trunk_right;
	extract_trunk_right.setNegative(false);
	extract_trunk_right.setInputCloud(potential_trunk_plane_right);
	extract_trunk_right.setIndices(inliers_right);
	extract_trunk_right.filter(*trunk_plane_right);

#ifdef SHOW_PLANE_COEFF
	cout << "coefficients of trunk right plane " << endl;
	for (size_t i = 0; i < plane_coefficients_trunk_right->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << plane_coefficients_trunk_right->values[i] << endl;
	}
#endif

  // float trunk_width = abs(plane_coefficients_trunk_left->values[3] - plane_coefficients_trunk_right->values[3]);
  // cout << "The width of the trunk is " << trunk_width << endl;


  // [8] restruct trunk left anfd right plane
  pcl::PointCloud<PointT>::Ptr trunk_plane_left_restruct(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr trunk_plane_right_restruct(new pcl::PointCloud<PointT>());
  pcl::ProjectInliers<PointT> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  
  plane_coefficients_trunk_left->values[3] = 0.0;
  proj.setInputCloud(trunk_plane_left);
  proj.setModelCoefficients(plane_coefficients_trunk_left);
  proj.filter(*trunk_plane_left_restruct);
  
  plane_coefficients_trunk_right->values[3] = 0.0;
  proj.setInputCloud(trunk_plane_right);
  proj.setModelCoefficients(plane_coefficients_trunk_right);
  proj.filter(*trunk_plane_right_restruct);
  
  pcl::PointCloud<PointT>::Ptr trunk_plane_side_restruct(new pcl::PointCloud<PointT>());
  for (size_t i = 0; i < trunk_plane_left_restruct->size(); i++) {
    trunk_plane_side_restruct->push_back(trunk_plane_left_restruct->points[i]);
	}
  for (size_t i = 0; i < trunk_plane_right_restruct->size(); i++) {
    trunk_plane_side_restruct->push_back(trunk_plane_right_restruct->points[i]);
	}
  
  pcl::PointCloud<PointT>::Ptr trunk_plane_side_plane_restruct(new pcl::PointCloud<PointT>());
  pcl::ModelCoefficients::Ptr plane_coefficients_trunk_side(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_side(new pcl::PointIndices);

	pcl::SACSegmentation<PointT> plane_trunk_side;
	plane_trunk_side.setOptimizeCoefficients(true);
	plane_trunk_side.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_side.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_side.setDistanceThreshold(0.01);
  plane_trunk_side.setInputCloud(trunk_plane_side_restruct);
	plane_trunk_side.segment(*inliers_side, *plane_coefficients_trunk_side);
#ifdef SHOW_PLANE_COEFF
	cout << "coefficients of trunk side plane " << endl;
	for (size_t i = 0; i < plane_coefficients_trunk_side->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << plane_coefficients_trunk_side->values[i] << endl;
	}
#endif
 
  float left_D = 0.0, right_D = 0.0;
  for (size_t i = 0; i < trunk_plane_left->size(); i++) {
    left_D += plane_coefficients_trunk_side->values[0] * trunk_plane_left->points[i].x;
    left_D += plane_coefficients_trunk_side->values[1] * trunk_plane_left->points[i].y;
    left_D += plane_coefficients_trunk_side->values[2] * trunk_plane_left->points[i].z;
	}
  left_D /= trunk_plane_left->size();
  for (size_t i = 0; i < trunk_plane_right->size(); i++) {
    right_D += plane_coefficients_trunk_side->values[0] * trunk_plane_right->points[i].x;
    right_D += plane_coefficients_trunk_side->values[1] * trunk_plane_right->points[i].y;
    right_D += plane_coefficients_trunk_side->values[2] * trunk_plane_right->points[i].z;
	}
  right_D /= trunk_plane_right->size();
  
  float trunk_width = abs(right_D - left_D);
  cout << "The width of the trunk is " << trunk_width << endl;


  // [9] get trunk side height
	pcl::PointCloud<PointT>::Ptr trunk_line_left(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_line_right(new pcl::PointCloud<PointT>());

	float left_height = get_height(trunk_plane_left, trunk_line_left);
	cout << "The height of the trunk left side is " << left_height << endl;
	float right_height = get_height(trunk_plane_right, trunk_line_right);
	cout << "The height of the trunk right side is " << right_height << endl;
  float trunk_side_height = (left_height + right_height) / 2 ;
  cout << "The height of the trunk side is " << trunk_side_height << endl;


	// [10] get trunk length
  float normal_start_x = plane_coefficients_trunk_side->values[0];
	float normal_start_y = plane_coefficients_trunk_side->values[1];
	float normal_start_z = plane_coefficients_trunk_side->values[2];
	float normal_end_x = 0, normal_end_y = -1, normal_end_z = 0;
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
	cout << "rotation axis: (" << rotation_axis_x << "," << rotation_axis_y << "," << rotation_axis_z << ")" << endl;

	float rotation_angle = acos(abs(plane_coefficients_trunk_side->values[1]));
	cout << "rotation angel: " << rotation_angle / M_PI * 180 << "degree" << endl;

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
	printf("rotation matrix: \n");
	printf("            | %6.3f %6.3f %6.3f | \n", rotation_Matrix(0, 0), rotation_Matrix(0, 1), rotation_Matrix(0, 2));
	printf("        R = | %6.3f %6.3f %6.3f | \n", rotation_Matrix(1, 0), rotation_Matrix(1, 1), rotation_Matrix(1, 2));
	printf("            | %6.3f %6.3f %6.3f | \n", rotation_Matrix(2, 0), rotation_Matrix(2, 1), rotation_Matrix(2, 2));
	printf("\n");
	pcl::PointCloud<PointT>::Ptr cloud_trunk_subface_after_revise(new pcl::PointCloud<PointT>());
	pcl::transformPointCloud(*cloud_trunk_subface, *cloud_trunk_subface_after_revise, rotation_Matrix);
 
  pcl::PointCloud<PointT>::Ptr cloud_line_fb(new pcl::PointCloud<PointT>());
	float trunk_length = get_length(cloud_trunk_subface_after_revise, cloud_line_fb);
	cout << "The length of the trunk is " << trunk_length << endl;


  // [11] visualization
	pcl::visualization::PCLVisualizer viewers("Cloud Viewer");
	viewers.addCoordinateSystem();
	viewers.setBackgroundColor(0.0, 0.0, 0.0);

	// viewers.addPointCloudNormals<pcl::PointXYZ, NormalT>(cloud_downsampled, cloud_normals);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> oricloud_color_handlers(cloud_downsampled, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_downsampled, oricloud_color_handlers, "original cloud");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_trunk_subface_color_handlers(cloud_trunk_subface, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_trunk_subface, cloud_trunk_subface_color_handlers, "cloud trunk subface");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_trunk_head_color_handlers(cloud_trunk_head, 0, 255, 0);
	viewers.addPointCloud<PointT>(cloud_trunk_head, cloud_trunk_head_color_handlers, "cloud trunk head");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_plane_left_color_handlers(trunk_plane_left, 0, 0, 255);
	viewers.addPointCloud<PointT>(trunk_plane_left, trunk_plane_left_color_handlers, "cloud trunk plane left");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_plane_right_color_handlers(trunk_plane_right, 255, 255, 0);
	viewers.addPointCloud<PointT>(trunk_plane_right, trunk_plane_right_color_handlers, "cloud trunk plane right");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_line_left_color_handlers(trunk_line_left, 255, 255, 0);
	viewers.addPointCloud<PointT>(trunk_line_left, trunk_line_left_color_handlers, "cloud trunk line left");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_line_right_color_handlers(trunk_line_right, 0, 0, 255);
	viewers.addPointCloud<PointT>(trunk_line_right, trunk_line_right_color_handlers, "cloud trunk line right");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_line_fb_color_handlers(cloud_line_fb, 0, 255, 255);
	viewers.addPointCloud<PointT>(cloud_line_fb, trunk_line_fb_color_handlers, "cloud line front back");

	viewers.spin();
	return 0;
}

