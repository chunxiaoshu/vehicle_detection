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
#include <pcl/filters/project_inliers.h>
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

	if (pcl::io::loadPCDFile<PointT>("../../pcd_data/pcd_data_multi2/test_pcd1.pcd", *cloud_pos1)) {	
		cout << "loading pcd data at pos1 failed" << endl;
		return -1;
	}
	else {
		cout << "loading pcd data at pos1 success" << endl;
		cout << "cloud size at pos1: " << cloud_pos1->size() << endl;
	}
	if (pcl::io::loadPCDFile<PointT>("../../pcd_data/pcd_data_multi2/test_pcd2.pcd", *cloud_pos2)) {	
		cout << "loading pcd data at pos2 failed" << endl;
		return -1;
	}
	else {
		cout << "loading pcd data at pos2 success" << endl;
		cout << "cloud size at pos2: " << cloud_pos2->size() << endl;
	}
	if (pcl::io::loadPCDFile<PointT>("../../pcd_data/pcd_data_multi2/test_pcd3.pcd", *cloud_pos3)) {	
		cout << "loading pcd data at pos3 failed" << endl;
		return -1;
	}
	else {
		cout << "loading pcd data at pos3 success" << endl;
		cout << "cloud size at pos3: " << cloud_pos3->size() << endl;
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
  float height_cloud_pos1_left = 0.0;
  float height_cloud_pos1_right = 0.0;
  float height_cloud_pos2_left = 0.0;
  float height_cloud_pos2_right = 0.0;
  for (size_t i = 0; i < cloud_pos1_trunk->size(); ++i) {
    if (cloud_pos1_trunk->points[i].y > 0 && cloud_pos1_trunk->points[i].z > height_cloud_pos1_left ) {
      height_cloud_pos1_left = cloud_pos1_trunk->points[i].z;
    }
    else if (cloud_pos1_trunk->points[i].y < 0 && cloud_pos1_trunk->points[i].z > height_cloud_pos1_right ) {
      height_cloud_pos1_right = cloud_pos1_trunk->points[i].z;
    }
  }
  for (size_t i = 0; i < cloud_pos2_trunk->size(); ++i) {
    if (cloud_pos2_trunk->points[i].y > 0 && cloud_pos2_trunk->points[i].z > height_cloud_pos2_left ) {
      height_cloud_pos2_left = cloud_pos2_trunk->points[i].z;
    }
    else if (cloud_pos2_trunk->points[i].y < 0 && cloud_pos2_trunk->points[i].z > height_cloud_pos2_right ) {
      height_cloud_pos2_right = cloud_pos2_trunk->points[i].z;
    }
  }

  float height_cloud_left = (height_cloud_pos1_left + height_cloud_pos2_left) / 2;
  float height_cloud_right = (height_cloud_pos1_right + height_cloud_pos2_right) / 2;
  float height_trunk = (height_cloud_left + height_cloud_right) / 2;
  // cout << "The height at pos1 on left is : " << height_cloud_pos1_left << endl;
  // cout << "The height at pos1 on right is : " << height_cloud_pos1_right << endl;
  // cout << "The height at pos2 on left is : " << height_cloud_pos2_left << endl;
  // cout << "The height at pos2 on right is : " << height_cloud_pos2_right << endl;
  // cout << "The height at left is : " << height_cloud_left << endl;
  // cout << "The height at right is : " << height_cloud_right << endl;
  cout << "The height of trunk is : " << height_trunk << endl;
	

  // [4] get the line of the trunk line
  pcl::PointCloud<PointT>::Ptr cloud_line_pos1_plane(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos1_leave(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos2_plane(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos2_leave(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_plane(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_front(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_back(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_head(new pcl::PointCloud<PointT>());

  pcl::ModelCoefficients::Ptr coeff_line_pos3_front(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coeff_line_pos3_back(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coeff_line_pos3_head(new pcl::ModelCoefficients);

  bool flag_cloud_line_pos1_plane = true;
  bool flag_cloud_line_pos2_plane = true;
  bool flag_cloud_line_pos3_plane = true;
  bool flag_cloud_line_pos3_front = true;
  bool flag_cloud_line_pos3_back = true;
  bool flag_cloud_line_pos3_head = true;

  pcl::PointCloud<PointT>::Ptr cloud_line(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_extract(new pcl::PointCloud<PointT>());
	pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coeff_line(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointT> seg_line;
  pcl::ExtractIndices<PointT> extract_line;

  seg_line.setOptimizeCoefficients(true);
  seg_line.setModelType(pcl::SACMODEL_LINE);
  seg_line.setMethodType(pcl::SAC_RANSAC);
  seg_line.setDistanceThreshold(0.01);

  seg_line.setInputCloud(cloud_pos1_trunk);
  extract_line.setInputCloud(cloud_pos1_trunk);

  while ( flag_cloud_line_pos1_plane ) {
    seg_line.segment(*inliers_line, *coeff_line);
    extract_line.setIndices(inliers_line);
    extract_line.setNegative(false);
    extract_line.filter(*cloud_line);
    extract_line.setNegative(true);
    extract_line.filter(*cloud_line_extract);
    // cout << "cloud size cloud_line_extract: " << cloud_line_extract->size() << endl;

    if ( coeff_line->values[5] < 0.1 && coeff_line->values[5] > -0.1 ) {
      flag_cloud_line_pos1_plane = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos1_plane->push_back(cloud_line->points[i]);
      }
    }
    else {
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos1_leave->push_back(cloud_line->points[i]);
      }
    }

    seg_line.setInputCloud(cloud_line_extract);
    extract_line.setInputCloud(cloud_line_extract);
  }
  for (size_t i = 0; i < cloud_line_extract->size(); ++i) {
    cloud_line_pos1_leave->push_back(cloud_line_extract->points[i]);
  }

  seg_line.setInputCloud(cloud_pos2_trunk);
  extract_line.setInputCloud(cloud_pos2_trunk);

  while ( flag_cloud_line_pos2_plane ) {
    seg_line.segment(*inliers_line, *coeff_line);
    extract_line.setIndices(inliers_line);
    extract_line.setNegative(false);
    extract_line.filter(*cloud_line);
    extract_line.setNegative(true);
    extract_line.filter(*cloud_line_extract);
    // cout << "cloud size cloud_line_extract: " << cloud_line_extract->size() << endl;

    if ( coeff_line->values[5] < 0.1 && coeff_line->values[5] > -0.1 ) {
      flag_cloud_line_pos2_plane = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos2_plane->push_back(cloud_line->points[i]);
      }
    }
    else {
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos2_leave->push_back(cloud_line->points[i]);
      }
    }

    seg_line.setInputCloud(cloud_line_extract);
    extract_line.setInputCloud(cloud_line_extract);
  }
  for (size_t i = 0; i < cloud_line_extract->size(); ++i) {
    cloud_line_pos2_leave->push_back(cloud_line_extract->points[i]);
  }

  seg_line.setInputCloud(cloud_pos3_trunk);
  extract_line.setInputCloud(cloud_pos3_trunk);
  float height_temp = height_trunk - 0.2;

  while ( flag_cloud_line_pos3_plane || flag_cloud_line_pos3_front || flag_cloud_line_pos3_back 
       || flag_cloud_line_pos3_head ) {
    seg_line.segment(*inliers_line, *coeff_line);
    extract_line.setIndices(inliers_line);
    extract_line.setNegative(false);
    extract_line.filter(*cloud_line);
    extract_line.setNegative(true);
    extract_line.filter(*cloud_line_extract);
    // cout << "cloud size cloud_line_extract: " << cloud_line_extract->size() << endl;

    if ( flag_cloud_line_pos3_plane && coeff_line->values[2] < height_temp 
      && coeff_line->values[5] < 0.1 && coeff_line->values[5] > -0.1 ) {
      flag_cloud_line_pos3_plane = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_plane->push_back(cloud_line->points[i]);
      }
    }

    if ( flag_cloud_line_pos3_front && cloud_line->points[0].x > 0
      && (coeff_line->values[5] > 0.9 || coeff_line->values[5] < -0.9) ) {
      flag_cloud_line_pos3_front = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_front->push_back(cloud_line->points[i]);
      }
      for (size_t i = 0; i < coeff_line->values.size(); ++i) {
        coeff_line_pos3_front->values.push_back(coeff_line->values[i]);
      }
    }

    if ( flag_cloud_line_pos3_back && cloud_line->points[0].x < 0
      && (coeff_line->values[5] > 0.9 || coeff_line->values[5] < -0.9) ) {
      flag_cloud_line_pos3_back = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_back->push_back(cloud_line->points[i]);
      }
      for (size_t i = 0; i < coeff_line->values.size(); ++i) {
        coeff_line_pos3_back->values.push_back(coeff_line->values[i]);
      }
    }

    if ( flag_cloud_line_pos3_head && coeff_line->values[2] > height_temp 
      && coeff_line->values[5] < 0.1 && coeff_line->values[5] > -0.1 ) {
      flag_cloud_line_pos3_head = false;
      for (size_t i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_head->push_back(cloud_line->points[i]);
      }
    }

    seg_line.setInputCloud(cloud_line_extract);
    extract_line.setInputCloud(cloud_line_extract);
  }


  // [5] rebuild trunk subface plane
  pcl::PointCloud<PointT>::Ptr cloud_potential_plane_subface(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_plane_subface(new pcl::PointCloud<PointT>());
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coeff_plane_subface(new pcl::ModelCoefficients);

  for (size_t i = 0; i < cloud_line_pos1_plane->size(); ++i) {
    cloud_potential_plane_subface->push_back(cloud_line_pos1_plane->points[i]);
  }
  for (size_t i = 0; i < cloud_line_pos2_plane->size(); ++i) {
    cloud_potential_plane_subface->push_back(cloud_line_pos2_plane->points[i]);
  }
  for (size_t i = 0; i < cloud_line_pos3_plane->size(); ++i) {
    cloud_potential_plane_subface->push_back(cloud_line_pos3_plane->points[i]);
  }

	pcl::SACSegmentation<PointT> seg_plane_subface;
	seg_plane_subface.setOptimizeCoefficients(true);
	seg_plane_subface.setModelType(pcl::SACMODEL_PLANE);
	seg_plane_subface.setMethodType(pcl::SAC_RANSAC);
	seg_plane_subface.setDistanceThreshold(0.01);
	seg_plane_subface.setInputCloud(cloud_potential_plane_subface);
	seg_plane_subface.segment(*inliers_plane, *coeff_plane_subface);

	pcl::ExtractIndices<PointT> extract_plane_subface;
	extract_plane_subface.setInputCloud(cloud_potential_plane_subface);
	extract_plane_subface.setIndices(inliers_plane);
	extract_plane_subface.setNegative(false);
	extract_plane_subface.filter(*cloud_plane_subface);
#ifdef SHOW_PLANE_COEFF
	cout << "coefficients of trunk_subface plane " << endl;
	const char coeff_plane[4] = { 'a', 'b', 'c', 'd' };
	for (size_t i = 0; i < coeff_plane_subface->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << coeff_plane_subface->values[i] << endl;
	}
#endif


	// [6] get trunk subface and head height
	float height_subface = 0.0;
	for (size_t i = 0; i < cloud_plane_subface->size(); ++i) {
		height_subface += cloud_plane_subface->points[i].z;
	}
	height_subface /= cloud_plane_subface->size();
	cout << "The height of the trunk subface is " << height_subface << endl;

  float height_head = 0.0;
	for (size_t i = 0; i < cloud_line_pos3_head->size(); ++i) {
		height_head += cloud_line_pos3_head->points[i].z;
	}
	height_head /= cloud_line_pos3_head->size();
	cout << "The height of the trunk head is " << height_head << endl;


  // [7] rebuild trunk left and right plane and get trunk width
  pcl::PointCloud<PointT>::Ptr cloud_potential_plane_left(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_potential_plane_right(new pcl::PointCloud<PointT>());

  for (size_t i = 0; i < cloud_line_pos1_leave->size(); ++i) {
    if ( cloud_line_pos1_leave->points[i].y < 0 ) {
      cloud_potential_plane_left->push_back(cloud_line_pos1_leave->points[i]);
    }
    else {
      cloud_potential_plane_right->push_back(cloud_line_pos1_leave->points[i]);
    }
  }
  for (size_t i = 0; i < cloud_line_pos2_leave->size(); ++i) {
    if ( cloud_line_pos2_leave->points[i].y < 0 ) {
      cloud_potential_plane_left->push_back(cloud_line_pos2_leave->points[i]);
    }
    else {
      cloud_potential_plane_right->push_back(cloud_line_pos2_leave->points[i]);
    }
  }

  pcl::PointCloud<PointT>::Ptr cloud_plane_left(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_plane_left_extract(new pcl::PointCloud<PointT>());
  pcl::ModelCoefficients::Ptr coeff_plane_left(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointT> seg_plane_left;
  pcl::ExtractIndices<PointT> extract_plane_left;
	seg_plane_left.setOptimizeCoefficients(true);
	seg_plane_left.setModelType(pcl::SACMODEL_PLANE);
	seg_plane_left.setMethodType(pcl::SAC_RANSAC);
	seg_plane_left.setDistanceThreshold(0.01);
  
  seg_plane_left.setInputCloud(cloud_potential_plane_left);
  extract_plane_left.setInputCloud(cloud_potential_plane_left);
  while ( true ) {
    seg_plane_left.segment(*inliers_plane, *coeff_plane_left);
	  extract_plane_left.setIndices(inliers_plane);
    extract_plane_left.setNegative(false);
    extract_plane_left.filter(*cloud_plane_left);
    
    if ( coeff_plane_left->values[1] > 0.9 || coeff_plane_left->values[1] < -0.9 ) {
      break;
    }

    extract_plane_left.setNegative(true);
    extract_plane_left.filter(*cloud_plane_left_extract);
    seg_plane_left.setInputCloud(cloud_plane_left_extract);
    extract_plane_left.setInputCloud(cloud_plane_left_extract);
  }
#ifdef SHOW_PLANE_COEFF
	cout << "coefficients of trunk left plane " << endl;
	for (size_t i = 0; i < coeff_plane_left->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << coeff_plane_left->values[i] << endl;
	}
#endif

  pcl::PointCloud<PointT>::Ptr cloud_plane_right(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_plane_right_extract(new pcl::PointCloud<PointT>());
  pcl::ModelCoefficients::Ptr coeff_plane_right(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointT> seg_plane_right;
  pcl::ExtractIndices<PointT> extract_plane_right;
	seg_plane_right.setOptimizeCoefficients(true);
	seg_plane_right.setModelType(pcl::SACMODEL_PLANE);
	seg_plane_right.setMethodType(pcl::SAC_RANSAC);
	seg_plane_right.setDistanceThreshold(0.01);
  
  seg_plane_right.setInputCloud(cloud_potential_plane_right);
  extract_plane_right.setInputCloud(cloud_potential_plane_right);
  while ( true ) {
    seg_plane_right.segment(*inliers_plane, *coeff_plane_right);
	  extract_plane_right.setIndices(inliers_plane);
    extract_plane_right.setNegative(false);
    extract_plane_right.filter(*cloud_plane_right);
    
    if ( coeff_plane_right->values[1] > 0.9 || coeff_plane_right->values[1] < -0.9 ) {
      break;
    }

    extract_plane_right.setNegative(true);
    extract_plane_right.filter(*cloud_plane_right_extract);
    seg_plane_right.setInputCloud(cloud_plane_right_extract);
    extract_plane_right.setInputCloud(cloud_plane_right_extract);
  }
#ifdef SHOW_PLANE_COEFF
	cout << "coefficients of trunk right plane " << endl;
	for (size_t i = 0; i < coeff_plane_right->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << coeff_plane_right->values[i] << endl;
	}
#endif

  pcl::PointCloud<PointT>::Ptr cloud_plane_left_proj(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_plane_right_proj(new pcl::PointCloud<PointT>());
  pcl::ProjectInliers<PointT> proj_plane;
  proj_plane.setModelType(pcl::SACMODEL_PLANE);
  
  coeff_plane_left->values[3] = 0.0;
  proj_plane.setInputCloud(cloud_potential_plane_left);
  proj_plane.setModelCoefficients(coeff_plane_left);
  proj_plane.filter(*cloud_plane_left_proj);
  
  coeff_plane_right->values[3] = 0.0;
  proj_plane.setInputCloud(cloud_potential_plane_right);
  proj_plane.setModelCoefficients(coeff_plane_right);
  proj_plane.filter(*cloud_plane_right_proj);
  
  pcl::PointCloud<PointT>::Ptr cloud_plane_side_proj(new pcl::PointCloud<PointT>());
  for (size_t i = 0; i < cloud_plane_left_proj->size(); i++) {
    cloud_plane_side_proj->push_back(cloud_plane_left_proj->points[i]);
	}
  for (size_t i = 0; i < cloud_plane_right_proj->size(); i++) {
    cloud_plane_side_proj->push_back(cloud_plane_right_proj->points[i]);
	}

  pcl::PointCloud<PointT>::Ptr cloud_plane_side(new pcl::PointCloud<PointT>());
  pcl::ModelCoefficients::Ptr coeff_plane_side(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_side(new pcl::PointIndices);

	pcl::SACSegmentation<PointT> seg_plane_side;
	seg_plane_side.setOptimizeCoefficients(true);
	seg_plane_side.setModelType(pcl::SACMODEL_PLANE);
	seg_plane_side.setMethodType(pcl::SAC_RANSAC);
	seg_plane_side.setDistanceThreshold(0.01);
  seg_plane_side.setInputCloud(cloud_plane_side_proj);
	seg_plane_side.segment(*inliers_side, *coeff_plane_side);
#ifdef SHOW_PLANE_COEFF
	cout << "coefficients of trunk side plane " << endl;
	for (size_t i = 0; i < coeff_plane_side->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << coeff_plane_side->values[i] << endl;
	}
#endif
 
  float D_plane_left = 0.0, D_plane_right = 0.0;
  for (size_t i = 0; i < cloud_plane_left->size(); i++) {
    D_plane_left += coeff_plane_side->values[0] * cloud_plane_left->points[i].x;
    D_plane_left += coeff_plane_side->values[1] * cloud_plane_left->points[i].y;
    D_plane_left += coeff_plane_side->values[2] * cloud_plane_left->points[i].z;
	}
  D_plane_left /= cloud_plane_left->size();
  for (size_t i = 0; i < cloud_plane_right->size(); i++) {
    D_plane_right += coeff_plane_side->values[0] * cloud_plane_right->points[i].x;
    D_plane_right += coeff_plane_side->values[1] * cloud_plane_right->points[i].y;
    D_plane_right += coeff_plane_side->values[2] * cloud_plane_right->points[i].z;
	}
  D_plane_right /= cloud_plane_right->size();
  
  float width_trunk = abs(D_plane_left - D_plane_right);
  cout << "The width of the trunk is " << width_trunk << endl;


  // [8] rebuild trunk front and back plane and get trunk length
  pcl::ModelCoefficients::Ptr coeff_plane_front(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coeff_plane_back(new pcl::ModelCoefficients);

  float A_plane_fb = -coeff_plane_side->values[1];
  float B_plane_fb = coeff_plane_side->values[0];
  float C_plane_fb = 0.0;
  float D_plane_front = 0.0;
  float D_plane_back = 0.0;

  for (size_t i = 0; i < cloud_line_pos3_front->size(); ++i) {
    D_plane_front += cloud_line_pos3_front->points[i].x * A_plane_fb + cloud_line_pos3_front->points[i].y * B_plane_fb;
  }
  for (size_t i = 0; i < cloud_line_pos3_back->size(); ++i) {
    D_plane_back += cloud_line_pos3_back->points[i].x * A_plane_fb + cloud_line_pos3_back->points[i].y * B_plane_fb;
  }
  D_plane_front /= cloud_line_pos3_front->size();
  D_plane_back /= cloud_line_pos3_back->size();

  coeff_plane_front->values.push_back(A_plane_fb);
  coeff_plane_front->values.push_back(B_plane_fb);
  coeff_plane_front->values.push_back(C_plane_fb);
  coeff_plane_front->values.push_back(D_plane_front);

  coeff_plane_back->values.push_back(A_plane_fb);
  coeff_plane_back->values.push_back(B_plane_fb);
  coeff_plane_back->values.push_back(C_plane_fb);
  coeff_plane_back->values.push_back(D_plane_back);

#ifdef SHOW_PLANE_COEFF
  cout << "coefficients of trunk front plane " << endl;
	for (size_t i = 0; i < coeff_plane_front->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << coeff_plane_front->values[i] << endl;
	}
  cout << "coefficients of trunk back plane " << endl;
	for (size_t i = 0; i < coeff_plane_back->values.size(); ++i) {
		cout << "	" << coeff_plane[i] << ":";
		cout << "	" << coeff_plane_back->values[i] << endl;
	}
#endif

  float length_trunk = abs(D_plane_front - D_plane_back);
  cout << "The length of the trunk is " << length_trunk << endl;


  // [9] visualization
	pcl::visualization::PCLVisualizer viewers("Cloud Viewer");
	viewers.addCoordinateSystem();
	viewers.setBackgroundColor(0.0, 0.0, 0.0);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_pos1(cloud_pos1_trunk, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_pos1_trunk, color_handler_pos1, "cloud pos1");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_pos2(cloud_pos2_trunk, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_pos2_trunk, color_handler_pos2, "cloud pos2");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_pos3(cloud_pos3_trunk, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_pos3_trunk, color_handler_pos3, "cloud pos3");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_subface(cloud_plane_subface, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_plane_subface, color_handler_subface, "cloud subface");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_left(cloud_plane_left, 0, 255, 0);
	viewers.addPointCloud<PointT>(cloud_plane_left, color_handler_left, "cloud left");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_right(cloud_plane_right, 0, 0, 255);
	viewers.addPointCloud<PointT>(cloud_plane_right, color_handler_right, "cloud right");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_front(cloud_line_pos3_front, 0, 255, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos3_front, color_handler_front, "cloud front");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_back(cloud_line_pos3_back, 255, 255, 0);
	viewers.addPointCloud<PointT>(cloud_line_pos3_back, color_handler_back, "cloud back");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_head(cloud_line_pos3_head, 255, 0, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos3_head, color_handler_head, "cloud head");

	viewers.spin();
	return 0;
}

