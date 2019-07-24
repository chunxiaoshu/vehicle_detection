#include <iostream>
#include <ctime>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/src/Core/Array.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>


#define SHOW_PLANE_COEFF
#define PI 3.14159265

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
using namespace std;

float get_side_height(pcl::PointCloud<PointT>::Ptr &cloud_plane, pcl::PointCloud<PointT>::Ptr &cloud_line, float threshold) {
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
		if (abs(cloud_plane->points[i].z - height_tmp) < threshold) {
			cloud_line->push_back(cloud_plane->points[i]);
			height_ava += cloud_plane->points[i].z;
		}
	}

	return height_ava / cloud_line->size() + 0.005;
}

float get_fb_height(pcl::PointCloud<PointT>::Ptr &cloud_plane, pcl::PointCloud<PointT>::Ptr &cloud_line, float threshold) {
	const size_t high_lengh = 3000;
	const size_t height_step = 500;
	size_t height_cnt = 0;
	size_t height_max = 0;
	float height_ava = 0;
	float plane_high[high_lengh] = {0};
	size_t plane_high_idx[high_lengh] = {0};
	size_t plane_high_distribute[height_step] = {0};
	for (size_t i = 0; i < cloud_plane->size(); ++i) {
		size_t tmp = static_cast<int>((cloud_plane->points[i].y + 13) * 100);
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
		if (abs(cloud_plane->points[i].z - height_tmp) < threshold) {
			cloud_line->push_back(cloud_plane->points[i]);
			height_ava += cloud_plane->points[i].z;
		}
	}

	return height_ava / cloud_line->size() + 0.005;
}

float get_length(pcl::PointCloud<PointT>::Ptr &cloud_plane, float &subface_x_front, float &subface_x_back) {
	const size_t x_resolution = 3000;
  size_t num_max = 0;
  vector<float> half_max_x;
	vector<size_t> plane_x_distribute(x_resolution, 0);
	for (size_t i = 0; i < cloud_plane->size(); ++i) {
		size_t tmp = static_cast<size_t>((cloud_plane->points[i].x + 13) * 100);
		++plane_x_distribute[tmp];
	}

	for (size_t i = 1; i < x_resolution; ++i) {
		if (num_max < plane_x_distribute[i]) {
			num_max = plane_x_distribute[i];
		}
	}

  num_max -= 20;
  // cout << "num_max" << num_max << endl;
  for (size_t i = 1; i < x_resolution; ++i) {
    // cout << "i" << i << "\t" << plane_x_distribute[i] << "\t";
		if (plane_x_distribute[i-1] < num_max && plane_x_distribute[i] > num_max
        || plane_x_distribute[i-1] > num_max && plane_x_distribute[i] < num_max) {
			half_max_x.push_back( static_cast<float>(i) / 100 - 13.0 );
		}
	}
  // cout << "half_max_x" << half_max_x.front() << endl;
  // cout << "half_max_x" << half_max_x.back() << endl;
  subface_x_back = half_max_x.front();
  subface_x_front = half_max_x.back();
	
	return half_max_x.back() - half_max_x.front();
}

Eigen::Matrix4f get_rotation_matrix(Eigen::Vector3d normal_start, Eigen::Vector3d normal_end, float rotation_angle) {
  float normal_start_x = normal_start(0);
	float normal_start_y = normal_start(1);
	float normal_start_z = normal_start(2);
  float normal_end_x = normal_end(0);
	float normal_end_y = normal_end(1);
	float normal_end_z = normal_end(2);
	float rotation_ceter_x = 0, rotation_ceter_y = 0, rotation_ceter_z = 0;
	float rotation_axis_x = (normal_end_y - normal_start_y) * (rotation_ceter_z - normal_start_z) - 
							(rotation_ceter_y - normal_start_y) * (normal_end_z - normal_start_z);
	float rotation_axis_y = (normal_end_z - normal_start_z) * (rotation_ceter_x - normal_start_x) - 
							(rotation_ceter_z - normal_start_z) * (normal_end_x - normal_start_x);
	float rotation_axis_z = (normal_end_x - normal_start_x) * (rotation_ceter_y - normal_start_y) - 
							(rotation_ceter_x - normal_start_x) * (normal_end_y - normal_start_y);
	float square_sum = rotation_axis_x * rotation_axis_x + rotation_axis_y * rotation_axis_y +
					            rotation_axis_z * rotation_axis_z;
	square_sum = sqrt(square_sum);
	rotation_axis_x /= square_sum;
	rotation_axis_y /= square_sum;
	rotation_axis_z /= square_sum;
	Eigen::Matrix4f rotation_Matrix = Eigen::Matrix4f::Identity();
	rotation_Matrix(0, 0) = cos(rotation_angle) + (1 - cos(rotation_angle)) * rotation_axis_x * rotation_axis_x;
	rotation_Matrix(0, 1) = (1 - cos(rotation_angle)) * rotation_axis_x * rotation_axis_y - sin(rotation_angle) * rotation_axis_z;
	rotation_Matrix(0, 2) = (1 - cos(rotation_angle)) * rotation_axis_x * rotation_axis_z + sin(rotation_angle) * rotation_axis_y;
	rotation_Matrix(1, 0) = (1 - cos(rotation_angle)) * rotation_axis_x * rotation_axis_y + sin(rotation_angle) * rotation_axis_z;
	rotation_Matrix(1, 1) = cos(rotation_angle) + (1 - cos(rotation_angle)) * rotation_axis_y * rotation_axis_y;
	rotation_Matrix(1, 2) = (1 - cos(rotation_angle)) * rotation_axis_y * rotation_axis_z - sin(rotation_angle) * rotation_axis_x;
	rotation_Matrix(2, 0) = (1 - cos(rotation_angle)) * rotation_axis_x * rotation_axis_z - sin(rotation_angle) * rotation_axis_y;
	rotation_Matrix(2, 1) = (1 - cos(rotation_angle)) * rotation_axis_y * rotation_axis_z + sin(rotation_angle) * rotation_axis_x;
	rotation_Matrix(2, 2) = cos(rotation_angle) + (1 - cos(rotation_angle)) * rotation_axis_z * rotation_axis_z;

  cout << "rotation axis: (" << rotation_axis_x << "," << rotation_axis_y << "," << rotation_axis_z << ")" << endl;
	cout << "rotation angel: " << rotation_angle / M_PI * 180 << "  degree" << endl;
	printf("rotation matrix: \n");
	printf("            | %6.3f %6.3f %6.3f | \n", rotation_Matrix(0, 0), rotation_Matrix(0, 1), rotation_Matrix(0, 2));
	printf("        R = | %6.3f %6.3f %6.3f | \n", rotation_Matrix(1, 0), rotation_Matrix(1, 1), rotation_Matrix(1, 2));
	printf("            | %6.3f %6.3f %6.3f | \n", rotation_Matrix(2, 0), rotation_Matrix(2, 1), rotation_Matrix(2, 2));
	printf("\n");

  return rotation_Matrix;
}

int main(int argc, char *argv[]) {
	// get calculate time
	clock_t start_Total = clock();
  const char coeff_plane[4] = { 'a', 'b', 'c', 'd' };
  pcl::visualization::PCLVisualizer viewers("Cloud Viewer");
	viewers.addCoordinateSystem();
	viewers.setBackgroundColor(0.0, 0.0, 0.0);


	// [1] load pcd data
	cout << "loading pcd data..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud_origin)) {
	// if (pcl::io::loadPCDFile<PointT>("../../pcd_data/test_pcd_rail1.pcd", *cloud_origin)) {
		cout << "loading pcd data failed" << endl;
		return -1;
	}
	else {
		cout << "loading pcd data success" << endl;
		cout << "cloud size origin: " << cloud_origin->size() << endl;
	}


	// [2] remove points on the ground
	pcl::PointCloud<PointT>::Ptr cloud_remove_nan(new pcl::PointCloud<PointT>());
	std::vector<int> indices_Pulse;
	pcl::removeNaNFromPointCloud(*cloud_origin, *cloud_remove_nan, indices_Pulse);

	pcl::PointCloud<PointT>::Ptr cloud_remove_ground(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < cloud_remove_nan->size(); ++i) {
		if ( cloud_remove_nan->points[i].z > 0.8) {
			cloud_remove_ground->push_back(cloud_remove_nan->points[i]);
		}
	}
  cout << "cloud size remove ground: " << cloud_remove_ground->size() << endl;


  // [3] point cloud filter 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter_statisticaloutlier;
  filter_statisticaloutlier.setInputCloud(cloud_remove_ground);
  filter_statisticaloutlier.setMeanK(50);
  filter_statisticaloutlier.setStddevMulThresh(1.0);
  filter_statisticaloutlier.setNegative(false);
  filter_statisticaloutlier.filter(*cloud_filtered);
  cout << "cloud size after filter: " << cloud_filtered->size() << endl;


	// [4] voxel grid downsample
	pcl::PointCloud<PointT>::Ptr cloud_downsampled(cloud_filtered);
	// pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>);
	// pcl::VoxelGrid<PointT> downsampled;
	// downsampled.setInputCloud(cloud_filtered);
	// downsampled.setLeafSize(0.01, 0.01, 0.01);
	// downsampled.filter(*cloud_downsampled);
	// cout << "cloud size after downsampled: " << cloud_downsampled->size() << endl << endl;


	// [5] calculate normal
	cout << "calculate normal..." << endl;
	clock_t normal_start = clock();
  pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>);
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pcl::NormalEstimation<PointT, NormalT> ne;
	ne.setInputCloud(cloud_downsampled);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.15);
	ne.compute(*cloud_normals);
	cout << "time to calculate normal: " << (double)(clock() - normal_start)/CLOCKS_PER_SEC << "s" << endl;
  // viewers.addPointCloudNormals<pcl::PointXYZ, NormalT>(cloud_filtered, cloud_normals);
	

  // [6] project to ground
  pcl::ProjectInliers<pcl::PointXYZ> project_ground;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_ground(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr plane_coeff_trunk_head(new pcl::ModelCoefficients());
  plane_coeff_trunk_head->values.resize(4);
  plane_coeff_trunk_head->values[0] = 0.0;
  plane_coeff_trunk_head->values[1] = 0.0;
  plane_coeff_trunk_head->values[2] = 1.0;
  plane_coeff_trunk_head->values[3] = 0.0;
  project_ground.setModelType(pcl::SACMODEL_PLANE);
  project_ground.setInputCloud(cloud_downsampled);
  project_ground.setModelCoefficients(plane_coeff_trunk_head);
  project_ground.filter(*cloud_projected_ground);

  // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filter_color_handlers(cloud_projected_ground, 255, 255, 255);
  // viewers.addPointCloud<PointT>(cloud_projected_ground, cloud_filter_color_handlers, "filtered cloud");


  // [7] get trunk head height
  float trunk_pos_xmax = -100.0;
  for (size_t i = 0; i < cloud_downsampled->size(); ++i) {
		if (cloud_downsampled->points[i].x > trunk_pos_xmax) {
			trunk_pos_xmax = cloud_downsampled->points[i].x;
		}
	}

  pcl::PointCloud<PointT>::Ptr cloud_head_potential(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < cloud_downsampled->size(); ++i) {
		if (trunk_pos_xmax - cloud_downsampled->points[i].x < 1.0 && 
    (cloud_normals->points[i].normal_z >= 0.9 || cloud_normals->points[i].normal_z <= -0.9)) {
			cloud_head_potential->push_back(cloud_downsampled->points[i]);
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
	plane_trunk_head.setInputCloud(cloud_head_potential);
	plane_trunk_head.segment(*inliers_head, *plane_coefficients_trunk_head);

	pcl::ExtractIndices<PointT> extract_trunk_head;
	extract_trunk_head.setInputCloud(cloud_head_potential);
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

  // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_downsampled_color_handlers(cloud_downsampled, 255, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_downsampled, cloud_downsampled_color_handlers, "downsampled cloud");
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_head_potential_color_handlers(cloud_head_potential, 255, 0, 0);
	// viewers.addPointCloud<PointT>(cloud_head_potential, cloud_head_potential_color_handlers, "cloud head potential");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_head_color_handlers(cloud_trunk_head, 0, 255, 0);
	// viewers.addPointCloud<PointT>(cloud_trunk_head, cloud_head_color_handlers, "cloud head");


	// [8] segment horizontal point cloud
  pcl::PointCloud<PointT>::Ptr cloud_horizontal(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < cloud_downsampled->size(); ++i) {
		if (cloud_normals->points[i].normal_z >= 0.9 || cloud_normals->points[i].normal_z <= -0.9) {
			cloud_horizontal->push_back(cloud_downsampled->points[i]);
		}
	}

  vector<pcl::PointCloud<PointT>::Ptr> vec_cloud_horizontal;
  vector<pcl::ModelCoefficients::Ptr> vec_plane_coefficients_trunk_horizontal;
  pcl::PointCloud<PointT>::Ptr cloud_horizontal_extract(new pcl::PointCloud<PointT>);
  pcl::PointIndices::Ptr inliers_horizontal(new pcl::PointIndices);
	pcl::SACSegmentation<PointT> plane_trunk_horizontal;
  pcl::ExtractIndices<PointT> extract_trunk_horizontal;

  plane_trunk_horizontal.setOptimizeCoefficients(true);
  plane_trunk_horizontal.setModelType(pcl::SACMODEL_PLANE);
  plane_trunk_horizontal.setMethodType(pcl::SAC_RANSAC);
  plane_trunk_horizontal.setDistanceThreshold(0.01);
  plane_trunk_horizontal.setInputCloud(cloud_horizontal);
  extract_trunk_horizontal.setInputCloud(cloud_horizontal);

  for ( int i = 0; i < 5; ++i ) {
    pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr plane_coefficients_temp(new pcl::ModelCoefficients);
    vec_cloud_horizontal.push_back(cloud_temp);
    vec_plane_coefficients_trunk_horizontal.push_back(plane_coefficients_temp);
    plane_trunk_horizontal.segment( *inliers_horizontal, *(vec_plane_coefficients_trunk_horizontal[i]) );
    extract_trunk_horizontal.setIndices(inliers_horizontal);
    extract_trunk_horizontal.setNegative(false);
    extract_trunk_horizontal.filter( *(vec_cloud_horizontal[i]) );
    extract_trunk_horizontal.setNegative(true);
    extract_trunk_horizontal.filter(*cloud_horizontal_extract);
    // cout << "cloud size cloud_horizontal_extract: " << cloud_horizontal_extract->size() << endl;

#ifdef SHOW_PLANE_COEFF
    // cout << "coefficients of trunk horizontal plane " << endl;
    // for (size_t j = 0; j < vec_plane_coefficients_trunk_horizontal[i]->values.size(); ++j) {
    //   cout << "	" << coeff_plane[j] << ":";
    //   cout << "	" << vec_plane_coefficients_trunk_horizontal[i]->values[j] << endl;
    // }
#endif

    plane_trunk_horizontal.setInputCloud(cloud_horizontal_extract);
    extract_trunk_horizontal.setInputCloud(cloud_horizontal_extract);
  }

  int horizontal_min_idx = -1;
  float horizontal_min_height = 10.0;
  for (int i = 0; i < 5; ++i) {
    if ( abs(vec_plane_coefficients_trunk_horizontal[i]->values[3]) < horizontal_min_height ) {
      horizontal_min_height = abs(vec_plane_coefficients_trunk_horizontal[i]->values[3]);
      horizontal_min_idx = i;
    }
  }
  pcl::PointCloud<PointT>::Ptr cloud_trunk_subface(vec_cloud_horizontal[horizontal_min_idx]);

  // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_horizontal0_color_handlers(vec_cloud_horizontal[0], 255, 0, 0);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal[0], cloud_horizontal0_color_handlers, "horizontal0 cloud");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_horizontal1_color_handlers(vec_cloud_horizontal[1], 0, 255, 0);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal[1], cloud_horizontal1_color_handlers, "horizontal1 cloud");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_horizontal2_color_handlers(vec_cloud_horizontal[2], 0, 0, 255);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal[2], cloud_horizontal2_color_handlers, "horizontal2 cloud");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_horizontal3_color_handlers(vec_cloud_horizontal[3], 255, 255, 0);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal[3], cloud_horizontal3_color_handlers, "horizontal3 cloud");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_horizontal4_color_handlers(vec_cloud_horizontal[4], 0, 255, 255);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal[4], cloud_horizontal4_color_handlers, "horizontal4 cloud");


  // [9] calculate the centre line and get the angle
  float subface_x_front;
  float subface_x_back; 
  get_length(cloud_trunk_subface, subface_x_front, subface_x_back);
  
  pcl::PointCloud<PointT>::Ptr cloud_subface_line_front(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_subface_line_back(new pcl::PointCloud<PointT>);
  for (size_t i = 0; i < cloud_trunk_subface->size(); ++i) {
		if (abs(cloud_trunk_subface->points[i].x - subface_x_back) < 0.01)
			cloud_subface_line_back->push_back(cloud_trunk_subface->points[i]);
    if (abs(cloud_trunk_subface->points[i].x - subface_x_front) < 0.01)
			cloud_subface_line_front->push_back(cloud_trunk_subface->points[i]);
	}
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_downsampled_color_handlers(cloud_downsampled, 255, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_downsampled, cloud_downsampled_color_handlers, "downsampled cloud");
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_subface_color_handlers(cloud_trunk_subface, 255, 0, 0);
	// viewers.addPointCloud<PointT>(cloud_trunk_subface, cloud_subface_color_handlers, "cloud subface");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_subface_line_front_color_handlers(cloud_subface_line_front, 0, 255, 0);
  // viewers.addPointCloud<PointT>(cloud_subface_line_front, cloud_subface_line_front_color_handlers, "subface line front cloud");
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_subface_line_back_color_handlers(cloud_subface_line_back, 0, 255, 0);
	// viewers.addPointCloud<PointT>(cloud_subface_line_back, cloud_subface_line_back_color_handlers, "subface line back cloud");

  const size_t subface_x_resolution = 3000;
  int subface_front = (subface_x_front + 13.0) * 100;
  int subface_back = (subface_x_back + 13.0) * 100;
  
  vector<float> subface_y_max(subface_x_resolution, 0.0);
  vector<float> subface_y_min(subface_x_resolution, 0.0);
  for (size_t i = 0; i < cloud_projected_ground->size(); ++i) {
    int tmp = static_cast<int>((cloud_projected_ground->points[i].x + 13.0) * 100);
    if (tmp >= subface_back && tmp <= subface_front) {
      if (cloud_projected_ground->points[i].y > subface_y_max[tmp])
        subface_y_max[tmp] = cloud_projected_ground->points[i].y;
      if (cloud_projected_ground->points[i].y < subface_y_min[tmp])
        subface_y_min[tmp] = cloud_projected_ground->points[i].y;
    }
  }

  int num = 0;
  float sum_subface_x = 0.0;
  float sum_subface_y = 0.0;
  float sum_subface_xx = 0.0;
  float sum_subface_xy = 0.0;
  for (int i = 0; i < subface_x_resolution; ++i) {
    if (subface_y_max[i] - subface_y_min[i] > 0.5) {
      num ++;
      float x = static_cast<float>(i) / 100 - 13;
      float y = (subface_y_max[i] + subface_y_min[i]) /2;
      sum_subface_x += x; 
      sum_subface_y += y;
      sum_subface_xx += x * x;
      sum_subface_xy += x * y;
    }
  }
  
  float centre_line_k = (num * sum_subface_xy - sum_subface_x * sum_subface_y) / (num * sum_subface_xx - sum_subface_x * sum_subface_x);
  float centre_line_b = sum_subface_y - centre_line_b * sum_subface_x;
  cout << "centre line is y = " << centre_line_k << " + " << centre_line_b << endl; 


  // [10] get trunk subface marking point
  float x_marking_point;
  float y_marking_point;
  if (centre_line_k >= 0.0) {
    // at the left
    size_t marking_point_idx = 0;
    float sum_pos_xy_max = -100;
    for (size_t i = 0; i < cloud_trunk_subface->size(); ++i) {
      float tmp = cloud_trunk_subface->points[i].x + cloud_trunk_subface->points[i].y;
      if (tmp > sum_pos_xy_max) {
        sum_pos_xy_max = tmp;
        marking_point_idx = i;
      }
      x_marking_point = cloud_trunk_subface->points[marking_point_idx].x;
      y_marking_point = cloud_trunk_subface->points[marking_point_idx].y;     
    }
  }
  else {
    // at the right
    size_t marking_point_idx = 0;
    float sum_pos_xy_max = -100;
    for (size_t i = 0; i < cloud_trunk_subface->size(); ++i) {
      float tmp = cloud_trunk_subface->points[i].x - cloud_trunk_subface->points[i].y;
      if (tmp > sum_pos_xy_max) {
        sum_pos_xy_max = tmp;
        marking_point_idx = i;
      }
      x_marking_point = cloud_trunk_subface->points[marking_point_idx].x;
      y_marking_point = cloud_trunk_subface->points[marking_point_idx].y;     
    }
  }


  // [11] horizontal angle correction
  float rotation_angle_horizontal = atan(abs(centre_line_k));
  Eigen::Vector3d normal_start_horizontal(cos(rotation_angle_horizontal), sin(rotation_angle_horizontal), 0.0);
  Eigen::Vector3d normal_end_horizontal(1.0, 0.0, 0.0);
  
  Eigen::Matrix4f matrix_horizontal = get_rotation_matrix(normal_start_horizontal, normal_end_horizontal, rotation_angle_horizontal);

  pcl::PointCloud<PointT>::Ptr cloud_downsampled_rev(new pcl::PointCloud<PointT>());
	pcl::transformPointCloud(*cloud_downsampled, *cloud_downsampled_rev, matrix_horizontal);

  pcl::PointCloud<PointT>::Ptr cloud_subface_rev(new pcl::PointCloud<PointT>());
  pcl::transformPointCloud(*cloud_trunk_subface, *cloud_subface_rev, matrix_horizontal);


  // [12] calculate normal after revise
	cout << "calculate normal after revise..." << endl;
	clock_t normal_start_rev = clock();
  pcl::PointCloud<NormalT>::Ptr cloud_normals_rev(new pcl::PointCloud<NormalT>);
  pcl::search::KdTree<PointT>::Ptr tree_rev(new pcl::search::KdTree<PointT>());
	pcl::NormalEstimation<PointT, NormalT> ne_rev;
	ne_rev.setInputCloud(cloud_downsampled_rev);
	ne_rev.setSearchMethod(tree_rev);
	ne_rev.setRadiusSearch(0.15);
	ne_rev.compute(*cloud_normals_rev);
	cout << "time to calculate normal after revise: " << (double)(clock() - normal_start_rev)/CLOCKS_PER_SEC << "s" << endl;


  // [13] get subface width and length and height
  float subface_pos_xmin = 100.0;
  float subface_pos_xmax = -100.0;
  float subface_pos_ymin = 100.0;
  float subface_pos_ymax = -100.0;
  for (size_t i = 0; i < cloud_subface_rev->size(); ++i) {
    if (cloud_subface_rev->points[i].x > subface_pos_xmax)
      subface_pos_xmax = cloud_subface_rev->points[i].x;
    if (cloud_subface_rev->points[i].x < subface_pos_xmin)
      subface_pos_xmin = cloud_subface_rev->points[i].x;
    if (cloud_subface_rev->points[i].y > subface_pos_ymax)
      subface_pos_ymax = cloud_subface_rev->points[i].y;
    if (cloud_subface_rev->points[i].y < subface_pos_ymin)
      subface_pos_ymin = cloud_subface_rev->points[i].y;
  }
  float subface_width = subface_pos_ymax - subface_pos_ymin;
  float subface_length = subface_pos_xmax - subface_pos_xmin;
  // cout << "subface_pos_xmax: " << subface_pos_xmax << endl;
  // cout << "subface_pos_xmin: " << subface_pos_xmin << endl;
  // cout << "subface_pos_ymax: " << subface_pos_ymax << endl;
  // cout << "subface_pos_ymin: " << subface_pos_ymin << endl;

  pcl::PointCloud<PointT>::Ptr cloud_subface_tail(new pcl::PointCloud<PointT>());
  int num_point_subface_tail = 0;
  float subface_tail_height = 0.0;
  for (size_t i = 0; i < cloud_subface_rev->size(); ++i) {
    if (cloud_subface_rev->points[i].x - subface_pos_xmin <= 0.02) {
      num_point_subface_tail ++;
      subface_tail_height += cloud_subface_rev->points[i].z;
      cloud_subface_tail->push_back(cloud_subface_rev->points[i]);
    }
  }
  subface_tail_height = subface_tail_height / static_cast<float>(num_point_subface_tail);
  cout << "subface_height: " << subface_tail_height << endl;
  cout << "subface_width: " << subface_width << endl;
  cout << "subface_length: " << subface_length << endl;


	// [14] segment trunk left and right plane
	pcl::PointCloud<PointT>::Ptr trunk_plane_right(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_plane_left(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr trunk_plane_back(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr trunk_plane_front(new pcl::PointCloud<PointT>());
  float subface_y_centre = (subface_pos_ymax + subface_pos_ymin) / 2.0;
	for (size_t i = 0; i < cloud_normals_rev->size(); ++i) {
    if (cloud_downsampled_rev->points[i].x < subface_pos_xmax - 0.2 && cloud_downsampled_rev->points[i].x >= subface_pos_xmin + 0.2
        && cloud_downsampled_rev->points[i].z > subface_tail_height + 0.2) {
      if (cloud_downsampled_rev->points[i].y < subface_y_centre) 
        trunk_plane_right->push_back(cloud_downsampled_rev->points[i]);
      if (cloud_downsampled_rev->points[i].y > subface_y_centre) 
        trunk_plane_left->push_back(cloud_downsampled_rev->points[i]);
    }
    else if (cloud_downsampled_rev->points[i].x < subface_pos_xmin && cloud_downsampled_rev->points[i].z > subface_tail_height + 0.2) {
      trunk_plane_back->push_back(cloud_downsampled_rev->points[i]);
    }
    else if (cloud_downsampled_rev->points[i].x > subface_pos_xmax && cloud_downsampled_rev->points[i].x < subface_pos_xmax + 1.0
        && cloud_downsampled_rev->points[i].z > subface_tail_height + 0.2) {
      trunk_plane_front->push_back(cloud_downsampled_rev->points[i]);
    }
	}
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> oricloud_color_handlers(cloud_downsampled_rev, 255, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_downsampled_rev, oricloud_color_handlers, "original cloud");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_plane_left_color_handlers(trunk_plane_left, 255, 0, 0);
	// viewers.addPointCloud<PointT>(trunk_plane_left, trunk_plane_left_color_handlers, "cloud trunk plane left");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_plane_right_color_handlers(trunk_plane_right, 255, 0, 0);
	// viewers.addPointCloud<PointT>(trunk_plane_right, trunk_plane_right_color_handlers, "cloud trunk plane right");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_plane_back_color_handlers(trunk_plane_back, 0, 0, 255);
	// viewers.addPointCloud<PointT>(trunk_plane_back, trunk_plane_back_color_handlers, "cloud trunk plane back");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_plane_front_color_handlers(trunk_plane_front, 0, 0, 255);
	// viewers.addPointCloud<PointT>(trunk_plane_front, trunk_plane_front_color_handlers, "cloud trunk plane front");


  // [15] get trunk side height
	pcl::PointCloud<PointT>::Ptr trunk_line_left(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_line_right(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr trunk_line_back(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr trunk_line_front(new pcl::PointCloud<PointT>());

	float left_height = get_side_height(trunk_plane_left, trunk_line_left, 0.01);
	cout << "The height of the trunk left side is " << left_height << endl;
	float right_height = get_side_height(trunk_plane_right, trunk_line_right, 0.01);
	cout << "The height of the trunk right side is " << right_height << endl;
  float back_height = get_fb_height(trunk_plane_back, trunk_line_back, 0.01);
	cout << "The height of the trunk back side is " << back_height << endl;
  float front_height = get_fb_height(trunk_plane_front, trunk_line_front, 0.01);
	cout << "The height of the trunk front side is " << front_height << endl;
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_line_left_color_handlers(trunk_line_left, 0, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_line_left, trunk_line_left_color_handlers, "cloud trunk line left");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_line_right_color_handlers(trunk_line_right, 0, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_line_right, trunk_line_right_color_handlers, "cloud trunk line right");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_line_back_color_handlers(trunk_line_back, 255, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_line_back, trunk_line_back_color_handlers, "cloud trunk line back");
  // pcl::visualization::PointCloudColorHandlerCustom<PointT> trunk_line_front_color_handlers(trunk_line_front, 255, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_line_front, trunk_line_front_color_handlers, "cloud trunk line front");


  // [16] get trunk front height


  // [99] visualization
  pcl::visualization::PointCloudColorHandlerCustom<PointT> oricloud_color_handlers(cloud_downsampled_rev, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_downsampled_rev, oricloud_color_handlers, "original cloud");
   




	viewers.spin();
	return 0;
}

