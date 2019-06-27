#include <iostream>
#include <ctime>
#include <cmath>
#include <string>
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

#define SHOW_PLANE_COEFF

using namespace std;

#define PI 3.14159265

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;

int main(int argc, char *argv[]) {
	// get calculate time
	clock_t start_Total, finish_Total;
	start_Total = clock();


	// [1] load pcd data at three pos
	cout << "loading pcd data..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_pos1(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_pos2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_pos3(new pcl::PointCloud<PointT>);

	if (pcl::io::loadPCDFile<PointT>("../../pcd_data/pcd_data_multi/test_pcd1.pcd", *cloud_pos1)) {	
		cout << "loading pcd data cloud_pos1 failed" << endl;
		return -1;
	}
	else {
		cout << "loading pcd data cloud_pos1 success" << endl;
		cout << "cloud size cloud_pos1: " << cloud_pos1->size() << endl;
	}
	if (pcl::io::loadPCDFile<PointT>("../../pcd_data/pcd_data_multi/test_pcd2.pcd", *cloud_pos2)) {	
		cout << "loading pcd data cloud_pos2 failed" << endl;
		return -1;
	}
	else {
		cout << "loading pcd data cloud_pos2 success" << endl;
		cout << "cloud size cloud_pos2: " << cloud_pos2->size() << endl;
	}
	if (pcl::io::loadPCDFile<PointT>("../../pcd_data/pcd_data_multi/test_pcd3.pcd", *cloud_pos3)) {	
		cout << "loading pcd data cloud_pos3 failed" << endl;
		return -1;
	}
	else {
		cout << "loading pcd data cloud_pos3 success" << endl;
		cout << "cloud size cloud_pos3: " << cloud_pos3->size() << endl;
	}


  // [2] remove points on the ground   
  pcl::PointCloud<PointT>::Ptr cloud_pos1_trunk(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_pos2_trunk(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_pos3_trunk(new pcl::PointCloud<PointT>);

	for (size_t i = 0; i < cloud_pos1->size(); ++i) {
		if ( cloud_pos1->points[i].z > 0.5) {
			cloud_pos1_trunk->push_back(cloud_pos1->points[i]);
		}
	}
  cout << "cloud size cloud_pos1_trunk: " << cloud_pos1_trunk->size() << endl;
  for (size_t i = 0; i < cloud_pos2->size(); ++i) {
		if ( cloud_pos2->points[i].z > 0.5) {
			cloud_pos2_trunk->push_back(cloud_pos2->points[i]);
		}
	}
  cout << "cloud size cloud_pos2_trunk: " << cloud_pos2_trunk->size() << endl;
  for (size_t i = 0; i < cloud_pos3->size(); ++i) {
		if ( cloud_pos3->points[i].z > 0.5) {
			cloud_pos3_trunk->push_back(cloud_pos3->points[i]);
		}
	}
  cout << "cloud size cloud_pos3_trunk: " << cloud_pos3_trunk->size() << endl;


  // [3] get the height at the left and the right
  float cloud_pos1_left_height = 0.0;
  float cloud_pos1_right_height = 0.0;
  for (size_t i = 0; i < cloud_pos1_trunk->size(); ++i) {
    if (cloud_pos1_trunk->points[i].y > 0 && cloud_pos1_trunk->points[i].z > cloud_pos1_left_height ) {
      cloud_pos1_left_height = cloud_pos1_trunk->points[i].z;
    }
    else if (cloud_pos1_trunk->points[i].y < 0 && cloud_pos1_trunk->points[i].z > cloud_pos1_right_height ) {
      cloud_pos1_right_height = cloud_pos1_trunk->points[i].z;
    }
  }
  // cout << "The height at pos1 on left is : " << cloud_pos1_left_height << endl;
  // cout << "The height at pos1 on right is : " << cloud_pos1_right_height << endl;
  float cloud_pos1_height = (cloud_pos1_left_height + cloud_pos1_right_height) / 2;
  // cout << "The height at pos1 is : " << cloud_pos1_height << endl;

  float cloud_pos2_left_height = 0.0;
  float cloud_pos2_right_height = 0.0;
  for (size_t i = 0; i < cloud_pos2_trunk->size(); ++i) {
    if (cloud_pos2_trunk->points[i].y > 0 && cloud_pos2_trunk->points[i].z > cloud_pos2_left_height ) {
      cloud_pos2_left_height = cloud_pos2_trunk->points[i].z;
    }
    else if (cloud_pos2_trunk->points[i].y < 0 && cloud_pos2_trunk->points[i].z > cloud_pos2_right_height ) {
      cloud_pos2_right_height = cloud_pos2_trunk->points[i].z;
    }
  }
  // cout << "The height at pos2 on left is : " << cloud_pos2_left_height << endl;
  // cout << "The height at pos2 on right is : " << cloud_pos2_right_height << endl;
  float cloud_pos2_height = (cloud_pos2_left_height + cloud_pos2_right_height) / 2;
  // cout << "The height at pos2 is : " << cloud_pos2_height << endl;

  float trunk_height = (cloud_pos1_height + cloud_pos2_height) / 2;
  cout << "The height of trunk is : " << trunk_height << endl;
	

  // [4] get the line of the trunk line
  pcl::PointCloud<PointT>::Ptr cloud_line_pos1_plane(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos1_left(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos1_right(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos2_plane(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos2_left(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos2_right(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_plane(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_front(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_back(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_head(new pcl::PointCloud<PointT>());

  pcl::ModelCoefficients::Ptr coefficients_pos3_front(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coefficients_pos3_back(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coefficients_pos3_head(new pcl::ModelCoefficients);

  bool cloud_line_pos1_plane_flag = true;
  bool cloud_line_pos1_left_flag = true;
  bool cloud_line_pos1_right_flag = true;
  bool cloud_line_pos2_plane_flag = true;
  bool cloud_line_pos2_left_flag = true;
  bool cloud_line_pos2_right_flag = true;
  bool cloud_line_pos3_plane_flag = true;
  bool cloud_line_pos3_front_flag = true;
  bool cloud_line_pos3_back_flag = true;
  bool cloud_line_pos3_head_flag = true;

  pcl::PointCloud<PointT>::Ptr cloud_line(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_extract(new pcl::PointCloud<PointT>());
	pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_trunk_line(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointT> trunk_line;
  pcl::ExtractIndices<PointT> extract_line;

  trunk_line.setOptimizeCoefficients(true);
  trunk_line.setModelType(pcl::SACMODEL_LINE);
  trunk_line.setMethodType(pcl::SAC_RANSAC);
  trunk_line.setDistanceThreshold(0.01);

  trunk_line.setInputCloud(cloud_pos1_trunk);
  extract_line.setInputCloud(cloud_pos1_trunk);

  while ( cloud_line_pos1_plane_flag || cloud_line_pos1_left_flag || cloud_line_pos1_right_flag ) {
    trunk_line.segment(*inliers_line, *coefficients_trunk_line);
    extract_line.setIndices(inliers_line);
    extract_line.setNegative(false);
    extract_line.filter(*cloud_line);
    extract_line.setNegative(true);
    extract_line.filter(*cloud_line_extract);
    // cout << "cloud size cloud_line_extract: " << cloud_line_extract->size() << endl;

    if ( coefficients_trunk_line->values[5] < 0.1 && coefficients_trunk_line->values[5] > -0.1 ) {
      cloud_line_pos1_plane_flag = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos1_plane->push_back(cloud_line->points[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].y > 0 && cloud_line_pos1_left_flag ) {
      cloud_line_pos1_left_flag = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos1_left->push_back(cloud_line->points[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].y < 0 && cloud_line_pos1_right_flag ) {
      cloud_line_pos1_right_flag = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos1_right->push_back(cloud_line->points[i]);
      }
    }

    trunk_line.setInputCloud(cloud_line_extract);
    extract_line.setInputCloud(cloud_line_extract);
  }

  trunk_line.setInputCloud(cloud_pos2_trunk);
  extract_line.setInputCloud(cloud_pos2_trunk);

  while ( cloud_line_pos2_plane_flag || cloud_line_pos2_left_flag || cloud_line_pos2_right_flag ) {
    trunk_line.segment(*inliers_line, *coefficients_trunk_line);
    extract_line.setIndices(inliers_line);
    extract_line.setNegative(false);
    extract_line.filter(*cloud_line);
    extract_line.setNegative(true);
    extract_line.filter(*cloud_line_extract);
    // cout << "cloud size cloud_line_extract: " << cloud_line_extract->size() << endl;

    if ( coefficients_trunk_line->values[5] < 0.1 && coefficients_trunk_line->values[5] > -0.1 ) {
      cloud_line_pos2_plane_flag = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos2_plane->push_back(cloud_line->points[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].y > 0 && cloud_line_pos2_left_flag ) {
      cloud_line_pos2_left_flag = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos2_left->push_back(cloud_line->points[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].y < 0 && cloud_line_pos2_right_flag ) {
      cloud_line_pos2_right_flag = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos2_right->push_back(cloud_line->points[i]);
      }
    }

    trunk_line.setInputCloud(cloud_line_extract);
    extract_line.setInputCloud(cloud_line_extract);
  }

  trunk_line.setInputCloud(cloud_pos3_trunk);
  extract_line.setInputCloud(cloud_pos3_trunk);
  float height_temp = coefficients_trunk_line->values[2] + 0.2;

  while ( cloud_line_pos3_plane_flag || cloud_line_pos3_front_flag || cloud_line_pos3_back_flag 
        || cloud_line_pos3_head_flag ) {
    trunk_line.segment(*inliers_line, *coefficients_trunk_line);
    extract_line.setIndices(inliers_line);
    extract_line.setNegative(false);
    extract_line.filter(*cloud_line);
    extract_line.setNegative(true);
    extract_line.filter(*cloud_line_extract);
    // cout << "cloud size cloud_line_extract: " << cloud_line_extract->size() << endl;

    if ( coefficients_trunk_line->values[5] < 0.1 && coefficients_trunk_line->values[5] > -0.1 ) {
      cloud_line_pos3_plane_flag = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_plane->push_back(cloud_line->points[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].x > 0 && cloud_line_pos3_front_flag ) {
      cloud_line_pos3_front_flag = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_front->push_back(cloud_line->points[i]);
      }
      for (size_t i = 0; i < coefficients_trunk_line->values.size(); ++i) {
        coefficients_pos3_front->values.push_back(coefficients_trunk_line->values[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].x < 0 && cloud_line_pos3_back_flag ) {
      cloud_line_pos3_back_flag = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_back->push_back(cloud_line->points[i]);
      }
      for (size_t i = 0; i < coefficients_trunk_line->values.size(); ++i) {
        coefficients_pos3_back->values.push_back(coefficients_trunk_line->values[i]);
      }
    }

    if ( coefficients_trunk_line->values[5] < 0.1 && coefficients_trunk_line->values[5] > -0.1 
        && coefficients_trunk_line->values[2] > height_temp && cloud_line_pos3_head_flag ) {
      cloud_line_pos3_head_flag = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_head->push_back(cloud_line->points[i]);
      }
    }

    trunk_line.setInputCloud(cloud_line_extract);
    extract_line.setInputCloud(cloud_line_extract);
  }


  // [5] rebuild trunk subface plane
  pcl::PointCloud<PointT>::Ptr cloud_plane_subface(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_plane_subface_extract(new pcl::PointCloud<PointT>());
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr plane_coefficients_trunk_subface(new pcl::ModelCoefficients);

  for (size_t i = 0; i < cloud_line_pos1_plane->size(); ++i) {
    cloud_plane_subface->push_back(cloud_line_pos1_plane->points[i]);
  }
  for (size_t i = 0; i < cloud_line_pos2_plane->size(); ++i) {
    cloud_plane_subface->push_back(cloud_line_pos2_plane->points[i]);
  }
  for (size_t i = 0; i < cloud_line_pos3_plane->size(); ++i) {
    cloud_plane_subface->push_back(cloud_line_pos3_plane->points[i]);
  }

	pcl::SACSegmentation<PointT> plane_trunk_subface;
	plane_trunk_subface.setOptimizeCoefficients(true);
	plane_trunk_subface.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_subface.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_subface.setDistanceThreshold(0.01);
	plane_trunk_subface.setInputCloud(cloud_plane_subface);
	plane_trunk_subface.segment(*inliers_plane, *plane_coefficients_trunk_subface);

	pcl::ExtractIndices<PointT> extract_trunk_subface;
	extract_trunk_subface.setInputCloud(cloud_plane_subface);
	extract_trunk_subface.setIndices(inliers_plane);
	extract_trunk_subface.setNegative(false);
	extract_trunk_subface.filter(*cloud_plane_subface_extract);
#ifdef SHOW_PLANE_COEFF
	cout << "coefficients of trunk_subface plane " << endl;
	const char coeff_plane[4] = { 'a', 'b', 'c', 'd' };
	for (size_t i = 0; i < plane_coefficients_trunk_subface->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << plane_coefficients_trunk_subface->values[i] << endl;
	}
#endif


	// [6] get trunk subface and head height
	float trunk_subface_height = 0.0;
	for (size_t i = 0; i < cloud_plane_subface_extract->size(); ++i) {
		trunk_subface_height += cloud_plane_subface_extract->points[i].z;
	}
	trunk_subface_height /= cloud_plane_subface_extract->size();
	cout << "The height of the trunk subface is " << trunk_subface_height << endl;

  float trunk_head_height = 0.0;
	for (size_t i = 0; i < cloud_line_pos3_head->size(); ++i) {
		trunk_head_height += cloud_line_pos3_head->points[i].z;
	}
	trunk_head_height /= cloud_line_pos3_head->size();
	cout << "The height of the trunk head is " << trunk_head_height << endl;


  // [7] rebuild trunk left and right plane
  pcl::PointCloud<PointT>::Ptr cloud_plane_left(new pcl::PointCloud<PointT>());
  for (size_t i = 0; i < cloud_line_pos1_left->size(); ++i) {
    cloud_plane_left->push_back(cloud_line_pos1_left->points[i]);
  }
  for (size_t i = 0; i < cloud_line_pos2_left->size(); ++i) {
    cloud_plane_left->push_back(cloud_line_pos2_left->points[i]);
  }

  // pcl::PointCloud<PointT>::Ptr cloud_plane_left_extract(new pcl::PointCloud<PointT>());
  // pcl::ModelCoefficients::Ptr plane_coefficients_trunk_left(new pcl::ModelCoefficients);
  // pcl::SACSegmentation<PointT> plane_trunk_left;
	// plane_trunk_left.setOptimizeCoefficients(true);
	// plane_trunk_left.setModelType(pcl::SACMODEL_PLANE);
	// plane_trunk_left.setMethodType(pcl::SAC_RANSAC);
	// plane_trunk_left.setDistanceThreshold(0.01);
	// plane_trunk_left.setInputCloud(cloud_plane_left);
	// plane_trunk_left.segment(*inliers_plane, *plane_coefficients_trunk_left);

	// pcl::ExtractIndices<PointT> extract_trunk_left;
	// extract_trunk_left.setInputCloud(cloud_plane_left);
	// extract_trunk_left.setIndices(inliers_plane);
	// extract_trunk_left.setNegative(false);
	// extract_trunk_left.filter(*cloud_plane_left_extract);
	// cout << "coefficients of trunk left plane " << endl;
	// for (size_t i = 0; i < plane_coefficients_trunk_left->values.size(); ++i) {
  //  cout << "	" << coeff_plane[i] << ":";
	// 	cout << "	" << plane_coefficients_trunk_left->values[i] << endl;
	// }

  pcl::PointCloud<PointT>::Ptr cloud_plane_right(new pcl::PointCloud<PointT>());
  for (size_t i = 0; i < cloud_line_pos1_right->size(); ++i) {
    cloud_plane_right->push_back(cloud_line_pos1_right->points[i]);
  }
  for (size_t i = 0; i < cloud_line_pos2_right->size(); ++i) {
    cloud_plane_right->push_back(cloud_line_pos2_right->points[i]);
  }

  // pcl::PointCloud<PointT>::Ptr cloud_plane_right_extract(new pcl::PointCloud<PointT>());
  // pcl::ModelCoefficients::Ptr plane_coefficients_trunk_right(new pcl::ModelCoefficients);
  // pcl::SACSegmentation<PointT> plane_trunk_right;
	// plane_trunk_right.setOptimizeCoefficients(true);
	// plane_trunk_right.setModelType(pcl::SACMODEL_PLANE);
	// plane_trunk_right.setMethodType(pcl::SAC_RANSAC);
	// plane_trunk_right.setDistanceThreshold(0.01);
	// plane_trunk_right.setInputCloud(cloud_plane_right);
	// plane_trunk_right.segment(*inliers_plane, *plane_coefficients_trunk_right);

	// pcl::ExtractIndices<PointT> extract_trunk_right;
	// extract_trunk_right.setInputCloud(cloud_plane_right);
	// extract_trunk_right.setIndices(inliers_plane);
	// extract_trunk_right.setNegative(false);
	// extract_trunk_right.filter(*cloud_plane_right_extract);
	// cout << "coefficients of trunk right plane " << endl;
	// for (size_t i = 0; i < plane_coefficients_trunk_right->values.size(); ++i) {
	// 	cout << "	" << coeff_plane[i] << ":";
	// 	cout << "	" << plane_coefficients_trunk_right->values[i] << endl;
	// }

  pcl::ModelCoefficients::Ptr plane_coefficients_trunk_left_correction(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr plane_coefficients_trunk_right_correction(new pcl::ModelCoefficients);

  size_t num_left = cloud_plane_left->size();
  size_t num_right = cloud_plane_right->size();
  float sum_lx = 0.0;
  float sum_ly = 0.0;
  float sum_lxx = 0.0;
  float sum_lxy = 0.0;
  float sum_rx = 0.0;
  float sum_ry = 0.0;
  float sum_rxx = 0.0;
  float sum_rxy = 0.0;

  for (size_t i = 0; i < num_left; ++i) {
    sum_lx += cloud_plane_left->points[i].x;
    sum_ly += cloud_plane_left->points[i].y;
    sum_lxx += cloud_plane_left->points[i].x * cloud_plane_left->points[i].x;
    sum_lxy += cloud_plane_left->points[i].x * cloud_plane_left->points[i].y;
  }
  for (size_t i = 0; i < num_right; ++i) {
    sum_rx += cloud_plane_right->points[i].x;
    sum_ry += cloud_plane_right->points[i].y;
    sum_rxx += cloud_plane_right->points[i].x * cloud_plane_right->points[i].x;
    sum_rxy += cloud_plane_right->points[i].x * cloud_plane_right->points[i].y;
  }

  Eigen::Matrix<float, 3, 3> matrix_33;
  Eigen::Matrix<float, 3, 1> matrix_31;
  matrix_33 << sum_lx, num_left, 0.0, sum_rx, 0.0, num_right, sum_lxx + sum_rxx, sum_lx, sum_rx;
  matrix_31 << -sum_ly, -sum_ry, -(sum_lxy + sum_rxy);
  Eigen::Matrix<float, 3, 1> matrix_solve = matrix_33.inverse() * matrix_31;
  // cout << matrix_solve << endl << endl;

  float sym = -1;
  if ( matrix_solve(0, 0) < 0 ) {
    sym = 1;
  }
  float pow_k = matrix_solve(0, 0) * matrix_solve(0, 0);
  float A = sqrt( pow_k / ( pow_k + 1) ) * sym;
  float B = -sqrt( 1 / ( pow_k + 1) );
  float C = 0.0;
  float D_left = matrix_solve(1, 0) * B;
  float D_right = matrix_solve(2, 0) * B;

  plane_coefficients_trunk_left_correction->values.push_back(A);
  plane_coefficients_trunk_left_correction->values.push_back(B);
  plane_coefficients_trunk_left_correction->values.push_back(C);
  plane_coefficients_trunk_left_correction->values.push_back(D_left);

  plane_coefficients_trunk_right_correction->values.push_back(A);
  plane_coefficients_trunk_right_correction->values.push_back(B);
  plane_coefficients_trunk_right_correction->values.push_back(C);
  plane_coefficients_trunk_right_correction->values.push_back(D_right);

#ifdef SHOW_PLANE_COEFF
  cout << "coefficients of trunk_left plane " << endl;
	for (size_t i = 0; i < plane_coefficients_trunk_left_correction->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << plane_coefficients_trunk_left_correction->values[i] << endl;
	}
  cout << "coefficients of trunk_right plane " << endl;
	for (size_t i = 0; i < plane_coefficients_trunk_right_correction->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << plane_coefficients_trunk_right_correction->values[i] << endl;
	}
#endif

  // [8] get trunk weight
  // float trunk_weight = abs(plane_coefficients_trunk_left->values[3] - plane_coefficients_trunk_right->values[3]);
  float trunk_width = abs(D_left - D_right);
  cout << "The width of the trunk is " << trunk_width << endl;


  // [9] rebuild trunk front and back plane and get trunk length
  pcl::ModelCoefficients::Ptr plane_coefficients_trunk_front_correction(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr plane_coefficients_trunk_back_correction(new pcl::ModelCoefficients);

  size_t num_front = cloud_line_pos3_front->size();
  size_t num_back = cloud_line_pos3_back->size();
  float sum_faxbt = 0.0;
  float sum_baxbt = 0.0;

  for (size_t i = 0; i < num_front; ++i) {
    sum_faxbt += cloud_line_pos3_front->points[i].x * (-B) + cloud_line_pos3_front->points[i].y * A;
  }
  for (size_t i = 0; i < num_back; ++i) {
    sum_baxbt += cloud_line_pos3_back->points[i].x * (-B) + cloud_line_pos3_back->points[i].y * A;
  }
  sum_faxbt /= num_front;
  sum_baxbt /= num_back;

  plane_coefficients_trunk_front_correction->values.push_back(-B);
  plane_coefficients_trunk_front_correction->values.push_back(A);
  plane_coefficients_trunk_front_correction->values.push_back(C);
  plane_coefficients_trunk_front_correction->values.push_back(sum_faxbt);

  plane_coefficients_trunk_back_correction->values.push_back(-B);
  plane_coefficients_trunk_back_correction->values.push_back(A);
  plane_coefficients_trunk_back_correction->values.push_back(C);
  plane_coefficients_trunk_back_correction->values.push_back(sum_baxbt);

#ifdef SHOW_PLANE_COEFF
  cout << "coefficients of trunk_front plane " << endl;
	for (size_t i = 0; i < plane_coefficients_trunk_front_correction->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << plane_coefficients_trunk_front_correction->values[i] << endl;
	}
  cout << "coefficients of trunk_back plane " << endl;
	for (size_t i = 0; i < plane_coefficients_trunk_back_correction->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << plane_coefficients_trunk_back_correction->values[i] << endl;
	}
#endif

  float trunk_length = abs(sum_faxbt - sum_baxbt);
  cout << "The length of the trunk is " << trunk_length << endl;


  // [9] visualization
	pcl::visualization::PCLVisualizer viewers("Cloud Viewer");
	viewers.addCoordinateSystem();
	viewers.setBackgroundColor(0.0, 0.0, 0.0);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> pos1_cloud_color_handlers(cloud_pos1_trunk, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_pos1_trunk, pos1_cloud_color_handlers, "pos1 cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos2_cloud_color_handlers(cloud_pos2_trunk, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_pos2_trunk, pos2_cloud_color_handlers, "pos2 cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_cloud_color_handlers(cloud_pos3_trunk, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_pos3_trunk, pos3_cloud_color_handlers, "pos3 cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> plane_subface_color_handlers(cloud_plane_subface_extract, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_plane_subface_extract, plane_subface_color_handlers, "plane subface cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> plane_left_color_handlers(cloud_plane_left, 0, 255, 0);
	viewers.addPointCloud<PointT>(cloud_plane_left, plane_left_color_handlers, "pos1 line left cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> plane_right_color_handlers(cloud_plane_right, 0, 0, 255);
	viewers.addPointCloud<PointT>(cloud_plane_right, plane_right_color_handlers, "pos1 line right cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_line_front_color_handlers(cloud_line_pos3_front, 0, 255, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos3_front, pos3_line_front_color_handlers, "pos3 line front cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_line_back_color_handlers(cloud_line_pos3_back, 255, 255, 0);
	viewers.addPointCloud<PointT>(cloud_line_pos3_back, pos3_line_back_color_handlers, "pos3 line back cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_line_head_color_handlers(cloud_line_pos3_head, 255, 0, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos3_head, pos3_line_head_color_handlers, "pos3 line head cloud");

	viewers.spin();
	return 0;
}

