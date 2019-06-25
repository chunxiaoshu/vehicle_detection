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

	// [1] load pcd data
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


  float cloud_pos1_left_height;
  float cloud_pos1_right_height;
  for (int i = 0; i < cloud_pos1_trunk->size(); ++i) {
    if ()

  }
	

  
  pcl::PointCloud<PointT>::Ptr cloud_line_pos1_left(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos1_right(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos1_plane(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos2_left(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos2_right(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos2_plane(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_front(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_back(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_line_pos3_plane(new pcl::PointCloud<PointT>());

  bool cloud_line_pos1_left_flag = true;
  bool cloud_line_pos1_right_flag = true;
  bool cloud_line_pos1_plane_flag = true;
  bool cloud_line_pos2_left_flag = true;
  bool cloud_line_pos2_right_flag = true;
  bool cloud_line_pos2_plane_flag = true;
  bool cloud_line_pos3_front_flag = true;
  bool cloud_line_pos3_back_flag = true;
  bool cloud_line_pos3_plane_flag = true;

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

  while ( cloud_line_pos1_left_flag || cloud_line_pos1_right_flag || cloud_line_pos1_plane_flag ) {
    trunk_line.segment(*inliers_line, *coefficients_trunk_line);
    extract_line.setIndices(inliers_line);
    extract_line.setNegative(false);
    extract_line.filter(*cloud_line);
    extract_line.setNegative(true);
    extract_line.filter(*cloud_line_extract);
    // cout << "cloud size cloud_line_extract: " << cloud_line_extract->size() << endl;

    if ( coefficients_trunk_line->values[5] < 0.1 && coefficients_trunk_line->values[5] > -0.1 
        && cloud_line_pos1_plane_flag ) {
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

  while ( cloud_line_pos2_left_flag || cloud_line_pos2_right_flag || cloud_line_pos2_plane_flag ) {
    trunk_line.segment(*inliers_line, *coefficients_trunk_line);
    extract_line.setIndices(inliers_line);
    extract_line.setNegative(false);
    extract_line.filter(*cloud_line);
    extract_line.setNegative(true);
    extract_line.filter(*cloud_line_extract);
    // cout << "cloud size cloud_line_extract: " << cloud_line_extract->size() << endl;

    if ( coefficients_trunk_line->values[5] < 0.1 && coefficients_trunk_line->values[5] > -0.1 
        && cloud_line_pos2_plane_flag ) {
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

  while ( cloud_line_pos3_front_flag || cloud_line_pos3_back_flag || cloud_line_pos3_plane_flag ) {
    trunk_line.segment(*inliers_line, *coefficients_trunk_line);
    extract_line.setIndices(inliers_line);
    extract_line.setNegative(false);
    extract_line.filter(*cloud_line);
    extract_line.setNegative(true);
    extract_line.filter(*cloud_line_extract);
    // cout << "cloud size cloud_line_extract: " << cloud_line_extract->size() << endl;

    if ( coefficients_trunk_line->values[5] < 0.1 && coefficients_trunk_line->values[5] > -0.1 
        && cloud_line_pos3_plane_flag ) {
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
    }

    if ( (coefficients_trunk_line->values[5] > 0.9 || coefficients_trunk_line->values[5] < -0.9) 
        && cloud_line->points[0].x < 0 && cloud_line_pos3_back_flag ) {
      cloud_line_pos3_back_flag = false;
      for (int i = 0; i < cloud_line->size(); ++i) {
        cloud_line_pos3_back->push_back(cloud_line->points[i]);
      }
    }

    trunk_line.setInputCloud(cloud_line_extract);
    extract_line.setInputCloud(cloud_line_extract);
  }



  
    // cout << "coefficients of trunk_subface plane " << endl;
    // string coeff[6] = { "point_x", "point_y", "point_z", "dir_x", "dir_y", "dir_z" };
    // for (size_t i = 0; i < coefficients_trunk_line->values.size(); ++i) {
    //   cout << "	" << coeff[i] << ":";
    //   cout << "	" << coefficients_trunk_line->values[i] << endl;
    // }





	pcl::visualization::PCLVisualizer viewers("Cloud Viewer");
	viewers.addCoordinateSystem();
	viewers.setBackgroundColor(0.0, 0.0, 0.0);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> pos1_cloud_color_handlers(cloud_pos1_trunk, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_pos1_trunk, pos1_cloud_color_handlers, "pos1 cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos1_line_plane_color_handlers(cloud_line_pos1_plane, 255, 255, 0);
	viewers.addPointCloud<PointT>(cloud_line_pos1_plane, pos1_line_plane_color_handlers, "pos1 line plane cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos1_line_left_color_handlers(cloud_line_pos1_left, 0, 255, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos1_left, pos1_line_left_color_handlers, "pos1 line left cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos1_line_right_color_handlers(cloud_line_pos1_right, 255, 0, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos1_right, pos1_line_right_color_handlers, "pos1 line right cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos2_cloud_color_handlers(cloud_pos2_trunk, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_pos2_trunk, pos2_cloud_color_handlers, "pos2 cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos2_line_plane_color_handlers(cloud_line_pos2_plane, 255, 255, 0);
	viewers.addPointCloud<PointT>(cloud_line_pos2_plane, pos2_line_plane_color_handlers, "pos2 line plane cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos2_line_left_color_handlers(cloud_line_pos2_left, 0, 255, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos2_left, pos2_line_left_color_handlers, "pos2 line left cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos2_line_right_color_handlers(cloud_line_pos2_right, 255, 0, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos2_right, pos2_line_right_color_handlers, "pos2 line right cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_cloud_color_handlers(cloud_pos3_trunk, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_pos3_trunk, pos3_cloud_color_handlers, "pos3 cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_line_plane_color_handlers(cloud_line_pos3_plane, 255, 255, 0);
	viewers.addPointCloud<PointT>(cloud_line_pos3_plane, pos3_line_plane_color_handlers, "pos3 line plane cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_line_front_color_handlers(cloud_line_pos3_front, 0, 255, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos3_front, pos3_line_front_color_handlers, "pos3 line front cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pos3_line_back_color_handlers(cloud_line_pos3_back, 255, 0, 255);
	viewers.addPointCloud<PointT>(cloud_line_pos3_back, pos3_line_back_color_handlers, "pos3 line back cloud");

	viewers.spin();
	return 0;
}

