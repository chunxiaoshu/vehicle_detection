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

	for (int i = 0; i < cloud_pos1->size(); ++i) {
		if ( cloud_pos1->points[i].z > 0.5) {
			cloud_pos1_trunk->push_back(cloud_pos1->points[i]);
		}
	}
  cout << "cloud size cloud_pos1_trunk: " << cloud_pos1_trunk->size() << endl;
  for (int i = 0; i < cloud_pos2->size(); ++i) {
		if ( cloud_pos2->points[i].z > 0.5) {
			cloud_pos2_trunk->push_back(cloud_pos2->points[i]);
		}
	}
  cout << "cloud size cloud_pos2_trunk: " << cloud_pos2_trunk->size() << endl;
  for (int i = 0; i < cloud_pos3->size(); ++i) {
		if ( cloud_pos3->points[i].z > 0.5) {
			cloud_pos3_trunk->push_back(cloud_pos3->points[i]);
		}
	}
  cout << "cloud size cloud_pos3_trunk: " << cloud_pos3_trunk->size() << endl;


  // [3] get the height at the left and the right
  float cloud_pos1_left_height = 0.0;
  float cloud_pos1_right_height = 0.0;
  for (int i = 0; i < cloud_pos1_trunk->size(); ++i) {
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
  for (int i = 0; i < cloud_pos2_trunk->size(); ++i) {
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
      for (int i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos1_plane->push_back(cloud_line->points[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].y > 0 && cloud_line_pos1_left_flag ) {
      cloud_line_pos1_left_flag = false;
      for (int i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos1_left->push_back(cloud_line->points[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].y < 0 && cloud_line_pos1_right_flag ) {
      cloud_line_pos1_right_flag = false;
      for (int i = 0; i < cloud_line->size(); ++i) {
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
      for (int i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos2_plane->push_back(cloud_line->points[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].y > 0 && cloud_line_pos2_left_flag ) {
      cloud_line_pos2_left_flag = false;
      for (int i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos2_left->push_back(cloud_line->points[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].y < 0 && cloud_line_pos2_right_flag ) {
      cloud_line_pos2_right_flag = false;
      for (int i = 0; i < cloud_line->size(); ++i) {
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
      for (int i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_plane->push_back(cloud_line->points[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].x > 0 && cloud_line_pos3_front_flag ) {
      cloud_line_pos3_front_flag = false;
      for (int i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_front->push_back(cloud_line->points[i]);
      }
      for (size_t i = 0; i < coefficients_trunk_line->values.size(); ++i) {
        coefficients_pos3_front->values.push_back(coefficients_trunk_line->values[i]);
      }
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].x < 0 && cloud_line_pos3_back_flag ) {
      cloud_line_pos3_back_flag = false;
      for (int i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_back->push_back(cloud_line->points[i]);
      }
      for (size_t i = 0; i < coefficients_trunk_line->values.size(); ++i) {
        coefficients_pos3_back->values.push_back(coefficients_trunk_line->values[i]);
      }
    }

    if ( coefficients_trunk_line->values[5] < 0.1 && coefficients_trunk_line->values[5] > -0.1 
        && coefficients_trunk_line->values[2] > height_temp && cloud_line_pos3_head_flag ) {
      cloud_line_pos3_head_flag = false;
      for (int i = 0; i < cloud_line->size(); ++i) {
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

  for (int i = 0; i < cloud_line_pos1_plane->size(); ++i) {
    cloud_plane_subface->push_back(cloud_line_pos1_plane->points[i]);
  }
  for (int i = 0; i < cloud_line_pos2_plane->size(); ++i) {
    cloud_plane_subface->push_back(cloud_line_pos2_plane->points[i]);
  }
  for (int i = 0; i < cloud_line_pos3_plane->size(); ++i) {
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
	cout << "coefficients of trunk_subface plane " << endl;
	char coeff_subface[4] = { 'a', 'b', 'c', 'd' };
	for (size_t i = 0; i < plane_coefficients_trunk_subface->values.size(); ++i) {
		cout << "	" << coeff_subface[i] << ":";
		cout << "	" << plane_coefficients_trunk_subface->values[i] << endl;
	}


	// [6] get trunk subface and head height
	float trunk_subface_height = 0.0;
	for (int i = 0; i < cloud_plane_subface_extract->size(); ++i) {
		trunk_subface_height += cloud_plane_subface_extract->points[i].z;
	}
	trunk_subface_height /= cloud_plane_subface_extract->size();
	cout << "The height of the trunk subface is " << trunk_subface_height << endl;

  float trunk_head_height = 0.0;
	for (int i = 0; i < cloud_line_pos3_head->size(); ++i) {
		trunk_head_height += cloud_line_pos3_head->points[i].z;
	}
	trunk_head_height /= cloud_line_pos3_head->size();
	cout << "The height of the trunk head is " << trunk_head_height << endl;


  // [7] rebuild trunk left and right plane
  pcl::PointCloud<PointT>::Ptr cloud_plane_left(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_plane_left_extract(new pcl::PointCloud<PointT>());
  pcl::ModelCoefficients::Ptr plane_coefficients_trunk_left(new pcl::ModelCoefficients);
  for (int i = 0; i < cloud_line_pos1_left->size(); ++i) {
    cloud_plane_left->push_back(cloud_line_pos1_left->points[i]);
  }
  for (int i = 0; i < cloud_line_pos2_left->size(); ++i) {
    cloud_plane_left->push_back(cloud_line_pos2_left->points[i]);
  }

  pcl::SACSegmentation<PointT> plane_trunk_left;
	plane_trunk_left.setOptimizeCoefficients(true);
	plane_trunk_left.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_left.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_left.setDistanceThreshold(0.01);
	plane_trunk_left.setInputCloud(cloud_plane_left);
	plane_trunk_left.segment(*inliers_plane, *plane_coefficients_trunk_left);

	pcl::ExtractIndices<PointT> extract_trunk_left;
	extract_trunk_left.setInputCloud(cloud_plane_left);
	extract_trunk_left.setIndices(inliers_plane);
	extract_trunk_left.setNegative(false);
	extract_trunk_left.filter(*cloud_plane_left_extract);
	cout << "coefficients of trunk left plane " << endl;
  char coeff_left[4] = { 'a', 'b', 'c', 'd' };
	for (size_t i = 0; i < plane_coefficients_trunk_left->values.size(); ++i) {
		cout << "	" << coeff_left[i] << ":";
		cout << "	" << plane_coefficients_trunk_left->values[i] << endl;
	}

  pcl::PointCloud<PointT>::Ptr cloud_plane_right(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_plane_right_extract(new pcl::PointCloud<PointT>());
  pcl::ModelCoefficients::Ptr plane_coefficients_trunk_right(new pcl::ModelCoefficients);
  for (int i = 0; i < cloud_line_pos1_right->size(); ++i) {
    cloud_plane_right->push_back(cloud_line_pos1_right->points[i]);
  }
  for (int i = 0; i < cloud_line_pos2_right->size(); ++i) {
    cloud_plane_right->push_back(cloud_line_pos2_right->points[i]);
  }

  pcl::SACSegmentation<PointT> plane_trunk_right;
	plane_trunk_right.setOptimizeCoefficients(true);
	plane_trunk_right.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_right.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_right.setDistanceThreshold(0.01);
	plane_trunk_right.setInputCloud(cloud_plane_right);
	plane_trunk_right.segment(*inliers_plane, *plane_coefficients_trunk_right);

	pcl::ExtractIndices<PointT> extract_trunk_right;
	extract_trunk_right.setInputCloud(cloud_plane_right);
	extract_trunk_right.setIndices(inliers_plane);
	extract_trunk_right.setNegative(false);
	extract_trunk_right.filter(*cloud_plane_right_extract);
	cout << "coefficients of trunk right plane " << endl;
  char coeff_right[4] = { 'a', 'b', 'c', 'd' };
	for (size_t i = 0; i < plane_coefficients_trunk_right->values.size(); ++i) {
		cout << "	" << coeff_right[i] << ":";
		cout << "	" << plane_coefficients_trunk_right->values[i] << endl;
	}


  // [8] get trunk weight
  float trunk_weight_left = plane_coefficients_trunk_left->values[3] / 
        sqrt( plane_coefficients_trunk_left->values[0] * plane_coefficients_trunk_left->values[0]
            + plane_coefficients_trunk_left->values[1] * plane_coefficients_trunk_left->values[1]
            + plane_coefficients_trunk_left->values[2] * plane_coefficients_trunk_left->values[2] );
  float trunk_weight_right = plane_coefficients_trunk_right->values[3] / 
        sqrt( plane_coefficients_trunk_right->values[0] * plane_coefficients_trunk_right->values[0]
            + plane_coefficients_trunk_right->values[1] * plane_coefficients_trunk_right->values[1]
            + plane_coefficients_trunk_right->values[2] * plane_coefficients_trunk_right->values[2] );
  float trunk_weight = abs(trunk_weight_left - trunk_weight_right);
  cout << "The weight of the trunk head is " << trunk_weight << endl;


  // [9] rebuild trunk front and back plane
  





	pcl::visualization::PCLVisualizer viewers("Cloud Viewer");
	viewers.addCoordinateSystem();
	viewers.setBackgroundColor(0.0, 0.0, 0.0);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> pos1_cloud_color_handlers(cloud_pos1_trunk, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_pos1_trunk, pos1_cloud_color_handlers, "pos1 cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos2_cloud_color_handlers(cloud_pos2_trunk, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_pos2_trunk, pos2_cloud_color_handlers, "pos2 cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_cloud_color_handlers(cloud_pos3_trunk, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_pos3_trunk, pos3_cloud_color_handlers, "pos3 cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> plane_subface_color_handlers(cloud_plane_subface_extract, 0, 255, 0);
	viewers.addPointCloud<PointT>(cloud_plane_subface_extract, plane_subface_color_handlers, "plane subface cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> plane_left_color_handlers(cloud_plane_left_extract, 0, 255, 255);
	viewers.addPointCloud<PointT>(cloud_plane_left_extract, plane_left_color_handlers, "pos1 line left cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> plane_right_color_handlers(cloud_plane_right_extract, 255, 0, 255);
	viewers.addPointCloud<PointT>(cloud_plane_right_extract, plane_right_color_handlers, "pos1 line right cloud");


  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_line_front_color_handlers(cloud_line_pos3_front, 0, 255, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos3_front, pos3_line_front_color_handlers, "pos3 line front cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_line_back_color_handlers(cloud_line_pos3_back, 255, 0, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos3_back, pos3_line_back_color_handlers, "pos3 line back cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_line_head_color_handlers(cloud_line_pos3_head, 0, 0, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos3_head, pos3_line_head_color_handlers, "pos3 line head cloud");

  

	viewers.spin();
	return 0;
}

