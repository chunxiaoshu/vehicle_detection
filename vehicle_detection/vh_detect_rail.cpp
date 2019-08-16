#include <iostream>
#include <ctime>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/src/Core/Array.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_hull.h>
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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>


// #define SHOW_PLANE_COEFF
typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<PointT>::Ptr PointPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointConstPtr;
typedef pcl::PointCloud<NormalT>::Ptr NormalPtr;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> VisualizationHandle;

const double PI = 3.14159265;
const int HORIZONTAL_PLANE_NUM = 3;
const float GROUNG_ZMAX = 0.8;
const float TRUNK_X_OFFSET = 20.0;
const float TRUNK_Y_OFFSET = 5.0;
const float WIDTH_RESOLUTION = 0.1;

float get_side_height(PointPtr cloud, PointPtr &cloud_line, float threshold) {
	const size_t high_lengh = 3000;
	const size_t height_step = 500;
	size_t height_cnt = 0;
	size_t height_max = 0;
	float height_ava = 0;
	float plane_high[high_lengh] = {0};
	size_t plane_high_idx[high_lengh] = {0};
	size_t plane_high_distribute[height_step] = {0};
	for (size_t i = 0; i < cloud->size(); ++i) {
		size_t tmp = static_cast<int>((cloud->points[i].x + TRUNK_X_OFFSET) * 100);
		if (plane_high[tmp] < cloud->points[i].z) {
			plane_high[tmp] = cloud->points[i].z;
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

	for (size_t i = 0; i < cloud->size(); ++i) {
		if (abs(cloud->points[i].z - height_tmp) < threshold) {
			cloud_line->push_back(cloud->points[i]);
			height_ava += cloud->points[i].z;
		}
	}

	return height_ava / cloud_line->size() + 0.005;
}

float get_fb_height(PointPtr cloud, PointPtr &cloud_line, float threshold) {
	const size_t high_lengh = 3000;
	const size_t height_step = 500;
	size_t height_cnt = 0;
	size_t height_max = 0;
	float height_ava = 0;
	float plane_high[high_lengh] = {0};
	size_t plane_high_idx[high_lengh] = {0};
	size_t plane_high_distribute[height_step] = {0};
	for (size_t i = 0; i < cloud->size(); ++i) {
		size_t tmp = static_cast<int>((cloud->points[i].y + TRUNK_X_OFFSET) * 100);
		// std::cout << tmp << "   ";
		if (plane_high[tmp] < cloud->points[i].z) {
			plane_high[tmp] = cloud->points[i].z;
			plane_high_idx[tmp] = i;
		}
	}

	for (size_t i = 0; i < high_lengh; ++i) {
		if (plane_high[i] > 1.0 && plane_high[i] <= height_step) {
			size_t tmp = static_cast<int>(plane_high[i] * 100);
			++plane_high_distribute[tmp];
		}
		// std::cout << "plane_high " << i << "  " << plane_high[i] << "  ";
	}

	for (size_t i = 1; i < height_step; ++i) {
		if (height_cnt < plane_high_distribute[i]) {
			height_cnt = plane_high_distribute[i];
			height_max = i;
		}
	}
	float height_tmp = static_cast<float>(height_max) / 100;
	// std::cout << "height_tmp is " << height_tmp << std::endl;

	for (size_t i = 0; i < cloud->size(); ++i) {
		if (abs(cloud->points[i].z - height_tmp) < threshold) {
			cloud_line->push_back(cloud->points[i]);
			height_ava += cloud->points[i].z;
		}
	}

	return height_ava / cloud_line->size() + 0.005;
}

float get_length(PointPtr cloud_plane, float &subface_x_front, float &subface_x_back, float threshold) {
	const size_t x_resolution = 3000;
	size_t num_max = 0;
	std::vector<float> half_max_x;
	std::vector<size_t> plane_x_distribute(x_resolution, 0);
	for (size_t i = 0; i < cloud_plane->size(); ++i) {
		size_t tmp = static_cast<size_t>((cloud_plane->points[i].x + TRUNK_X_OFFSET) * 100);
		++plane_x_distribute[tmp];
	}

	for (size_t i = 1; i < x_resolution; ++i) {
		if (num_max < plane_x_distribute[i]) {
			num_max = plane_x_distribute[i];
		}
	}

	num_max = static_cast<size_t>( static_cast<float>(num_max) * threshold );
	// std::cout << "num_max" << num_max << std::endl;
	for (size_t i = 1; i < x_resolution; ++i) {
		// std::cout << "i" << i << "\t" << plane_x_distribute[i] << "\n";
		if (plane_x_distribute[i-1] < num_max && plane_x_distribute[i] > num_max
        || plane_x_distribute[i-1] > num_max && plane_x_distribute[i] < num_max) {
			half_max_x.push_back( static_cast<float>(i) / 100 - TRUNK_X_OFFSET );
		}
	}
	 // std::cout << "half_max_x   " << half_max_x.front() << std::endl;
	 // std::cout << "half_max_x   " << half_max_x.back() << std::endl;
	subface_x_back = half_max_x.front();
	subface_x_front = half_max_x.back();
	
	return subface_x_front - subface_x_back;
}

int get_marking_point(PointPtr cloud_plane, float &x_marking_point, float &y_marking_point, 
					  float rotation_angle_horizontal, float trunk_length) {
	if (rotation_angle_horizontal >= 0.0) {
    	// at the right back
		size_t marking_point_idx = 0;
		float max_pos_xy_max = -100;
		for (size_t i = 0; i < cloud_plane->size(); ++i) {
			float tmp = -cloud_plane->points[i].x - cloud_plane->points[i].y;
			if (tmp > max_pos_xy_max) {
				max_pos_xy_max = tmp;
				marking_point_idx = i;
			}
		}
		x_marking_point = cloud_plane->points[marking_point_idx].x;
		y_marking_point = cloud_plane->points[marking_point_idx].y;
	}
	else {
    	// at the left back
		size_t marking_point_idx = 0;
		float max_pos_xy_max = -100;
		for (size_t i = 0; i < cloud_plane->size(); ++i) {
			float tmp = -cloud_plane->points[i].x + cloud_plane->points[i].y;
			if (tmp > max_pos_xy_max) {
				max_pos_xy_max = tmp;
				marking_point_idx = i;
			}
		}
		x_marking_point = cloud_plane->points[marking_point_idx].x;
		y_marking_point = cloud_plane->points[marking_point_idx].y;
	}
	x_marking_point += trunk_length * cos(rotation_angle_horizontal);
	y_marking_point += trunk_length * sin(rotation_angle_horizontal);
	return 0;
}

int get_side_protrusion(PointPtr cloud_plane, float side_height, std::vector<PointPtr> &vec_cloud_side_protrusion,
	std::vector<float> &vec_trunk_side_protrusion_height, std::vector<float> &vec_trunk_side_protrusion_xmax,
	std::vector<float> &vec_trunk_side_protrusion_xmin, std::vector<float> &vec_trunk_side_protrusion_ymax,
	std::vector<float> &vec_trunk_side_protrusion_ymin) {

	PointPtr trunk_side_protrusion_potential(new pcl::PointCloud<PointT>());
	for (size_t i = 0; i < cloud_plane->size(); ++i) {
		if (cloud_plane->points[i].z > side_height) {
			trunk_side_protrusion_potential->push_back(cloud_plane->points[i]);
		}
	}

	// PointPtr trunk_side_protrusion_filter(new pcl::PointCloud<PointT>());
	// pcl::RadiusOutlierRemoval<PointT> filter_side_protrusion;
	// filter_side_protrusion.setInputCloud(trunk_side_protrusion_potential);
	// filter_side_protrusion.setRadiusSearch(0.05);
	// filter_side_protrusion.setMinNeighborsInRadius(10);
	// filter_side_protrusion.filter(*trunk_side_protrusion_filter);

	if (trunk_side_protrusion_potential->size() < 50)
		return -1;

	pcl::search::KdTree<PointT>::Ptr tree_side_protrusion(new pcl::search::KdTree<PointT>);
	std::vector<pcl::PointIndices> cluster_side_protrusion;
	pcl::EuclideanClusterExtraction<PointT> ecx_side_protrusion;
	tree_side_protrusion->setInputCloud(trunk_side_protrusion_potential);
	ecx_side_protrusion.setClusterTolerance(0.1);
	ecx_side_protrusion.setMinClusterSize(50);
	ecx_side_protrusion.setMaxClusterSize(1000);
	ecx_side_protrusion.setSearchMethod(tree_side_protrusion);
	ecx_side_protrusion.setInputCloud(trunk_side_protrusion_potential);
	ecx_side_protrusion.extract(cluster_side_protrusion);

	for (int i = 0; i < cluster_side_protrusion.size(); ++i) {
		float trunk_side_protrusion_height = 0.0;
		float trunk_side_protrusion_xmax = -100.0;
		float trunk_side_protrusion_xmin = 100.0;
		float trunk_side_protrusion_ymax = -100.0;
		float trunk_side_protrusion_ymin = 100.0;
		PointPtr cloud_cluster(new pcl::PointCloud<PointT>);
		pcl::PointIndices cluster_index = cluster_side_protrusion[i];
		for (std::vector<int>::const_iterator it = cluster_index.indices.begin(); it != cluster_index.indices.end(); ++it)
      		cloud_cluster->push_back(trunk_side_protrusion_potential->points[*it]);

		for (size_t i = 0; i < cloud_cluster->size(); ++i) {
			if (cloud_cluster->points[i].z > trunk_side_protrusion_height)
				trunk_side_protrusion_height = cloud_cluster->points[i].z;
			if (cloud_cluster->points[i].x > trunk_side_protrusion_xmax)
				trunk_side_protrusion_xmax = cloud_cluster->points[i].x;
			if (cloud_cluster->points[i].x < trunk_side_protrusion_xmin)
				trunk_side_protrusion_xmin = cloud_cluster->points[i].x;
			if (cloud_cluster->points[i].y > trunk_side_protrusion_ymax)
				trunk_side_protrusion_ymax = cloud_cluster->points[i].y;
			if (cloud_cluster->points[i].y < trunk_side_protrusion_ymin)
				trunk_side_protrusion_ymin = cloud_cluster->points[i].y;
		}

		vec_cloud_side_protrusion.push_back(cloud_cluster);
		vec_trunk_side_protrusion_height.push_back(trunk_side_protrusion_height);
		vec_trunk_side_protrusion_xmax.push_back(trunk_side_protrusion_xmax);
		vec_trunk_side_protrusion_xmin.push_back(trunk_side_protrusion_xmin);
		vec_trunk_side_protrusion_ymax.push_back(trunk_side_protrusion_ymax);
		vec_trunk_side_protrusion_ymin.push_back(trunk_side_protrusion_ymin);

		// std::cout << "point size of cluster" << i << "at side protrusion is: " << cloud_cluster->size() << std::endl;
		// std::cout << "trunk side protrusion of cluster " << i << " height: " << trunk_side_protrusion_height << std::endl;
		// std::cout << "trunk side protrusion of cluster " << i << " xmax: " << trunk_side_protrusion_xmax << std::endl;
		// std::cout << "trunk side protrusion of cluster " << i << " xmin: " << trunk_side_protrusion_xmin << std::endl;
		// std::cout << "trunk side protrusion of cluster " << i << " ymax: " << trunk_side_protrusion_ymax << std::endl;
		// std::cout << "trunk side protrusion of cluster " << i << " ymin: " << trunk_side_protrusion_ymin << std::endl;
	}
	return 0;
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

	std::cout << "rotation axis: (" << rotation_axis_x << "," << rotation_axis_y << "," << rotation_axis_z << ")" << std::endl;
	std::cout << "rotation angel: " << rotation_angle / M_PI * 180 << "  degree" << std::endl;
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
	bool trunk_stair = false;
	const char coeff_plane[4] = { 'a', 'b', 'c', 'd' };
	const char coeff_line[6] = { 'x', 'y', 'z', 'a', 'b', 'c' };
	pcl::visualization::PCLVisualizer viewers("Cloud Viewer");
	viewers.addCoordinateSystem();
	viewers.setBackgroundColor(0.0, 0.0, 0.0);
	float para = 0;
	float para1 = 0;
	if (argc >= 3)
		para = atof(argv[2]);
	if (argc >= 4)
		para1 = atof(argv[3]);


	// [1] load pcd data
	std::cout << "loading pcd data..." << std::endl;
	PointPtr cloud_origin(new pcl::PointCloud<PointT>());
	if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud_origin)) {
	// if (pcl::io::loadPCDFile<PointT>("../../pcd_data/test_pcd_rail.pcd", *cloud_origin)) {
	// if (pcl::io::loadPCDFile<PointT>("../../pcd_data/test_pcd_rail.pcd", *cloud_origin)) {
	// if (pcl::io::loadPCDFile<PointT>("../../pcd_data/truck_step1.pcd", *cloud_origin)) {
		std::cout << "loading pcd data failed" << std::endl;
		return -1;
	}
	else {
		std::cout << "loading pcd data success" << std::endl;
		std::cout << "cloud size origin: " << cloud_origin->size() << std::endl;
	}


	// [2] remove points on the ground
	PointPtr cloud_remove_nan(new pcl::PointCloud<PointT>());
	std::vector<int> indices_Pulse;
	pcl::removeNaNFromPointCloud(*cloud_origin, *cloud_remove_nan, indices_Pulse);

	PointPtr cloud_remove_ground(new pcl::PointCloud<PointT>());
	for (size_t i = 0; i < cloud_remove_nan->size(); ++i) {
		if ( cloud_remove_nan->points[i].z > GROUNG_ZMAX) {
			cloud_remove_ground->push_back(cloud_remove_nan->points[i]);
		}
	}
	std::cout << "cloud size remove ground: " << cloud_remove_ground->size() << std::endl;


	// [3] point cloud filter 
	PointPtr cloud_filtered( cloud_remove_ground );
	// PointPtr cloud_filtered(new pcl::PointCloud<PointT>());
	// pcl::StatisticalOutlierRemoval<PointT> filter_statisticaloutlier;
	// filter_statisticaloutlier.setInputCloud(cloud_remove_ground);
	// filter_statisticaloutlier.setMeanK(para);
	// filter_statisticaloutlier.setStddevMulThresh( static_cast<float>(para1) / 1000 );
	// filter_statisticaloutlier.setNegative(false);
	// filter_statisticaloutlier.filter(*cloud_filtered);
	// std::cout << "cloud size after filter: " << cloud_filtered->size() << std::endl;

	// VisualizationHandle cloud_origin_colorh(cloud_remove_ground, 255, 0, 0);
	// viewers.addPointCloud<PointT>(cloud_remove_ground, cloud_origin_colorh, "origin cloud");
	// VisualizationHandle cloud_filter_colorh(cloud_filtered, 0, 255, 0);
	// viewers.addPointCloud<PointT>(cloud_filtered, cloud_filter_colorh, "filtered cloud");


	// [4] voxel grid downsample
	PointPtr cloud_downsampled(cloud_filtered);
	// PointPtr cloud_downsampled(new pcl::PointCloud<PointT>());
	// pcl::VoxelGrid<PointT> downsampled;
	// downsampled.setInputCloud(cloud_filtered);
	// downsampled.setLeafSize(0.01, 0.01, 0.01);
	// downsampled.filter(*cloud_downsampled);
	// std::cout << "cloud size after downsampled: " << cloud_downsampled->size() << std::endl << std::endl;

	// VisualizationHandle cloud_filter_colorh(cloud_filtered, 255, 0, 0);
	// viewers.addPointCloud<PointT>(cloud_filtered, cloud_filter_colorh, "filtered cloud");
	// VisualizationHandle cloud_downsampled_colorh(cloud_downsampled, 0, 255, 0);
	// viewers.addPointCloud<PointT>(cloud_downsampled, cloud_downsampled_colorh, "downsampled cloud");


	// [5] horizontal position correction


	// [6] calculate normal
	std::cout << "calculate normal..." << std::endl;
	clock_t normal_start = clock();
	NormalPtr cloud_normals(new pcl::PointCloud<NormalT>);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	// pcl::NormalEstimation<PointT, NormalT> ne;
	pcl::NormalEstimationOMP<PointT, NormalT> ne;
	ne.setInputCloud(cloud_downsampled);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.15);
	ne.compute(*cloud_normals);
	std::cout << "time to calculate normal: " << (double)(clock() - normal_start)/CLOCKS_PER_SEC << "s" << std::endl;
	// viewers.addPointCloudNormals<PointT, NormalT>(cloud_filtered, cloud_normals);


	// [7] project to ground
	pcl::ProjectInliers<PointT> project_ground;
	PointPtr cloud_projected_ground(new pcl::PointCloud<PointT>);
	pcl::ModelCoefficients::Ptr plane_coeff_ground(new pcl::ModelCoefficients());
	plane_coeff_ground->values.resize(4);
	plane_coeff_ground->values[0] = 0.0;
	plane_coeff_ground->values[1] = 0.0;
	plane_coeff_ground->values[2] = 1.0;
	plane_coeff_ground->values[3] = 0.0;
	project_ground.setModelType(pcl::SACMODEL_PLANE);
	project_ground.setInputCloud(cloud_downsampled);
	project_ground.setModelCoefficients(plane_coeff_ground);
	project_ground.filter(*cloud_projected_ground);

	// VisualizationHandle cloud_projected_ground_colorh(cloud_projected_ground, 255, 0, 0);
	// viewers.addPointCloud<PointT>(cloud_projected_ground, cloud_projected_ground_colorh, "projected ground cloud");


	// [8] segment horizontal point cloud
	PointPtr cloud_horizontal(new pcl::PointCloud<PointT>());
	for (size_t i = 0; i < cloud_downsampled->size(); ++i) {
		if (cloud_normals->points[i].normal_z >= 0.95 || cloud_normals->points[i].normal_z <= -0.95) {
			cloud_horizontal->push_back(cloud_downsampled->points[i]);
		}
	}

	std::vector<PointPtr> vec_cloud_horizontal;
	std::vector<pcl::ModelCoefficients::Ptr> vec_plane_coeff_trunk_horizontal;
	PointPtr cloud_horizontal_extract(new pcl::PointCloud<PointT>());
	pcl::PointIndices::Ptr inliers_horizontal(new pcl::PointIndices);
	pcl::SACSegmentation<PointT> plane_trunk_horizontal;
	pcl::ExtractIndices<PointT> extract_trunk_horizontal;

	plane_trunk_horizontal.setOptimizeCoefficients(true);
	plane_trunk_horizontal.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_horizontal.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_horizontal.setDistanceThreshold(0.02);
	plane_trunk_horizontal.setInputCloud(cloud_horizontal);
	extract_trunk_horizontal.setInputCloud(cloud_horizontal);

	for ( int i = 0; i < HORIZONTAL_PLANE_NUM; ++i ) {
		PointPtr cloud_temp(new pcl::PointCloud<PointT>());
		pcl::ModelCoefficients::Ptr plane_coeff_temp(new pcl::ModelCoefficients);
		vec_cloud_horizontal.push_back(cloud_temp);
		vec_plane_coeff_trunk_horizontal.push_back(plane_coeff_temp);
		plane_trunk_horizontal.segment( *inliers_horizontal, *(vec_plane_coeff_trunk_horizontal[i]) );
		extract_trunk_horizontal.setIndices(inliers_horizontal);
		extract_trunk_horizontal.setNegative(false);
		extract_trunk_horizontal.filter( *(vec_cloud_horizontal[i]) );
		extract_trunk_horizontal.setNegative(true);
		extract_trunk_horizontal.filter(*cloud_horizontal_extract);
		// std::cout << "cloud size cloud_horizontal_extract: " << cloud_horizontal_extract->size() << std::endl;

#ifdef SHOW_PLANE_COEFF
		std::cout << "coefficients of trunk horizontal plane " << std::endl;
		for (size_t j = 0; j < vec_plane_coeff_trunk_horizontal[i]->values.size(); ++j) {
			std::cout << "	" << coeff_plane[j] << ":";
			std::cout << "	" << vec_plane_coeff_trunk_horizontal[i]->values[j] << std::endl;
		}
#endif

		plane_trunk_horizontal.setInputCloud(cloud_horizontal_extract);
		extract_trunk_horizontal.setInputCloud(cloud_horizontal_extract);
	}

	int horizontal_min_idx = -1;
	float horizontal_min_height = 10.0;
	for (int i = 0; i < vec_cloud_horizontal.size(); ++i) {
		std::cout << "heifhaISDU   " << abs(vec_plane_coeff_trunk_horizontal[i]->values[3]) << std::endl;
		if ( abs(vec_plane_coeff_trunk_horizontal[i]->values[3]) < horizontal_min_height ) {
			horizontal_min_height = abs(vec_plane_coeff_trunk_horizontal[i]->values[3]);
			horizontal_min_idx = i;
		}
	}
	PointPtr cloud_trunk_subface( vec_cloud_horizontal[horizontal_min_idx] );
	pcl::ModelCoefficients::Ptr plane_coeff_subface( vec_plane_coeff_trunk_horizontal[horizontal_min_idx] );

	// VisualizationHandle cloud_horizontal0_colorh(vec_cloud_horizontal[0], 255, 0, 255);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal[0], cloud_horizontal0_colorh, "horizontal0 cloud");
	// VisualizationHandle cloud_horizontal1_colorh(vec_cloud_horizontal[1], 0, 255, 0);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal[1], cloud_horizontal1_colorh, "horizontal1 cloud");
	// VisualizationHandle cloud_horizontal2_colorh(vec_cloud_horizontal[2], 0, 0, 255);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal[2], cloud_horizontal2_colorh, "horizontal2 cloud");
	// VisualizationHandle cloud_subface_colorh(cloud_trunk_subface, 255, 0, 0);
	// viewers.addPointCloud<PointT>(cloud_trunk_subface, cloud_subface_colorh, "cloud subface");


	// [9] calculate the centre line and get the angle
	float subface_x_front;
	float subface_x_back; 
	get_length(cloud_trunk_subface, subface_x_front, subface_x_back, 0.8);
	PointPtr cloud_subface_line_front(new pcl::PointCloud<PointT>());
	PointPtr cloud_subface_line_back(new pcl::PointCloud<PointT>());
	for (size_t i = 0; i < cloud_trunk_subface->size(); ++i) {
		if (abs(cloud_trunk_subface->points[i].x - subface_x_back) < 0.01)
			cloud_subface_line_back->push_back(cloud_trunk_subface->points[i]);
		if (abs(cloud_trunk_subface->points[i].x - subface_x_front) < 0.01)
			cloud_subface_line_front->push_back(cloud_trunk_subface->points[i]);
	}
	// VisualizationHandle cloud_downsampled_colorh(cloud_downsampled, 255, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_downsampled, cloud_downsampled_colorh, "downsampled cloud");
	// VisualizationHandle cloud_subface_colorh(cloud_trunk_subface, 255, 0, 0);
	// viewers.addPointCloud<PointT>(cloud_trunk_subface, cloud_subface_colorh, "cloud subface");
	// VisualizationHandle cloud_subface_line_front_colorh(cloud_subface_line_front, 0, 255, 0);
	// viewers.addPointCloud<PointT>(cloud_subface_line_front, cloud_subface_line_front_colorh, "subface line front cloud");
	// VisualizationHandle cloud_subface_line_back_colorh(cloud_subface_line_back, 0, 255, 0);
	// viewers.addPointCloud<PointT>(cloud_subface_line_back, cloud_subface_line_back_colorh, "subface line back cloud");

	const size_t subface_x_resolution = 3000;
	int subface_front = (subface_x_front + TRUNK_X_OFFSET) * 100;
	int subface_back = (subface_x_back + TRUNK_X_OFFSET) * 100;
	std::vector<float> subface_y_max(subface_x_resolution, 0.0);
	std::vector<float> subface_y_min(subface_x_resolution, 0.0);
	for (size_t i = 0; i < cloud_projected_ground->size(); ++i) {
		int tmp = static_cast<int>((cloud_projected_ground->points[i].x + TRUNK_X_OFFSET) * 100);
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
			float x = static_cast<float>(i) / 100 - TRUNK_X_OFFSET + 0.005;
			float y = (subface_y_max[i] + subface_y_min[i]) /2;
			sum_subface_x += x; 
			sum_subface_y += y;
			sum_subface_xx += x * x;
			sum_subface_xy += x * y;
		}
	}
  
	float centre_line_k = (num * sum_subface_xy - sum_subface_x * sum_subface_y) / 
						  (num * sum_subface_xx - sum_subface_x * sum_subface_x);
	float centre_line_b = sum_subface_y - centre_line_b * sum_subface_x;
	std::cout << "centre line is y = " << centre_line_k << " + " << centre_line_b << std::endl; 

// 	PointPtr cloud_centre_line(new pcl::PointCloud<PointT>());
// 	for (int i = 0; i < subface_x_resolution; ++i) {
// 		if (subface_y_max[i] - subface_y_min[i] > 0.5) {
// 			PointT newpoint;
// 			newpoint.x = static_cast<float>(i) / 100 - TRUNK_X_OFFSET + 0.005;
// 			newpoint.y = (subface_y_max[i] + subface_y_min[i]) /2;
// 			cloud_centre_line->push_back(newpoint);	
// 		}
// 	}

// 	pcl::ModelCoefficients::Ptr line_coeff_centre(new pcl::ModelCoefficients);
// 	pcl::PointIndices::Ptr inliers_centre_line(new pcl::PointIndices);
// 	pcl::SACSegmentation<PointT> seg_centre_line;
// 	seg_centre_line.setOptimizeCoefficients(true);
// 	seg_centre_line.setModelType(pcl::SACMODEL_LINE);
// 	seg_centre_line.setMethodType(pcl::SAC_RANSAC);
// 	seg_centre_line.setDistanceThreshold(0.01);
// 	seg_centre_line.setInputCloud(cloud_centre_line);
// 	seg_centre_line.segment( *inliers_centre_line, *line_coeff_centre );
// #ifdef SHOW_PLANE_COEFF
// 	std::cout << "coefficients of trunk centre line " << std::endl;
// 	for (size_t i = 0; i < line_coeff_centre->values.size(); ++i) {
// 		std::cout << "	" << coeff_line[i] << ":";
// 		std::cout << "	" << line_coeff_centre->values[i] << std::endl;
// 	}
// #endif
// 	float angle_centre_line = atan(line_coeff_centre->values[4] / line_coeff_centre->values[3]);
// 	std::cout << "rotation angel: " << angle_centre_line / M_PI * 180 << "  degree" << std::endl;


	// [10] horizontal angle correction
	float rotation_angle_horizontal = atan(abs(centre_line_k));
	Eigen::Vector3d normal_start_horizontal(cos(rotation_angle_horizontal), sin(rotation_angle_horizontal), 0.0);
	Eigen::Vector3d normal_end_horizontal(1.0, 0.0, 0.0);
	Eigen::Matrix4f matrix_horizontal = get_rotation_matrix(normal_start_horizontal, normal_end_horizontal, rotation_angle_horizontal);

	PointPtr cloud_downsampled_rev(new pcl::PointCloud<PointT>());
	pcl::transformPointCloud(*cloud_downsampled, *cloud_downsampled_rev, matrix_horizontal);

	std::vector<PointPtr> vec_cloud_horizontal_rev;
	std::vector<PointPtr> vec_cloud_horizontal_filtered_rev;
	pcl::RadiusOutlierRemoval<PointT> filter_outrem;
	for (int i = 0; i < vec_cloud_horizontal.size(); ++i) {
		PointPtr cloud_temp_rev(new pcl::PointCloud<PointT>());
		PointPtr cloud_temp_filtered(new pcl::PointCloud<PointT>);
		pcl::transformPointCloud(*(vec_cloud_horizontal[i]), *cloud_temp_rev, matrix_horizontal);
		vec_cloud_horizontal_rev.push_back( cloud_temp_rev );

		filter_outrem.setInputCloud( cloud_temp_rev );
		filter_outrem.setRadiusSearch(0.05);
		filter_outrem.setMinNeighborsInRadius(10);
		filter_outrem.filter( *cloud_temp_filtered );
		vec_cloud_horizontal_filtered_rev.push_back( cloud_temp_filtered );
	}

	PointPtr cloud_subface_rev( vec_cloud_horizontal_filtered_rev[horizontal_min_idx] );

	// VisualizationHandle cloud_downsampled_colorh(cloud_downsampled_rev, 255, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_downsampled_rev, cloud_downsampled_colorh, "downsampled cloud");
	// VisualizationHandle cloud_horizontal0_colorh(vec_cloud_horizontal_filtered_rev[0], 255, 0, 0);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal_filtered_rev[0], cloud_horizontal0_colorh, "horizontal0 cloud");
	// VisualizationHandle cloud_horizontal1_colorh(vec_cloud_horizontal_filtered_rev[1], 0, 255, 0);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal_filtered_rev[1], cloud_horizontal1_colorh, "horizontal1 cloud");
	// VisualizationHandle cloud_horizontal2_colorh(vec_cloud_horizontal_filtered_rev[2], 0, 0, 255);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal_filtered_rev[2], cloud_horizontal2_colorh, "horizontal2 cloud");
	// VisualizationHandle cloud_horizontal3_colorh(vec_cloud_horizontal_filtered_rev[3], 255, 255, 0);
	// viewers.addPointCloud<PointT>(vec_cloud_horizontal_filtered_rev[3], cloud_horizontal3_colorh, "horizontal3 cloud");


	// [11] calculate normal after revise
	std::cout << "calculate normal after revise..." << std::endl;
	clock_t normal_start_rev = clock();
	NormalPtr cloud_normals_rev(new pcl::PointCloud<NormalT>);
	pcl::search::KdTree<PointT>::Ptr tree_rev(new pcl::search::KdTree<PointT>());
	// pcl::NormalEstimation<PointT, NormalT> ne_rev;
	pcl::NormalEstimationOMP<PointT, NormalT> ne_rev;
	ne_rev.setInputCloud(cloud_downsampled_rev);
	ne_rev.setSearchMethod(tree_rev);
	ne_rev.setRadiusSearch(0.15);
	ne_rev.compute(*cloud_normals_rev);
	std::cout << "time to calculate normal after revise: " << (double)(clock() - normal_start_rev)/CLOCKS_PER_SEC << "s" << std::endl;


	// [12] get trunk head height
	float trunk_pos_xmax = -100.0;
	for (size_t i = 0; i < cloud_downsampled_rev->size(); ++i) {
		if (cloud_downsampled_rev->points[i].x > trunk_pos_xmax && cloud_downsampled_rev->points[i].z > 1.5) {
			trunk_pos_xmax = cloud_downsampled_rev->points[i].x;
		}
	}

	PointPtr cloud_head_plane_potential(new pcl::PointCloud<PointT>());
	for (size_t i = 0; i < cloud_downsampled_rev->size(); ++i) {
		if (trunk_pos_xmax - cloud_downsampled_rev->points[i].x < 1.5 && 
    	(cloud_normals_rev->points[i].normal_z >= 0.95 || cloud_normals_rev->points[i].normal_z <= -0.95)) {
			cloud_head_plane_potential->push_back(cloud_downsampled_rev->points[i]);
		}
	}

	PointPtr cloud_trunk_head(new pcl::PointCloud<PointT>());
	pcl::PointIndices::Ptr inliers_head(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr plane_coeff_trunk_head(new pcl::ModelCoefficients);

	pcl::SACSegmentation<PointT> plane_trunk_head;
	plane_trunk_head.setOptimizeCoefficients(true);
	plane_trunk_head.setModelType(pcl::SACMODEL_PLANE);
	plane_trunk_head.setMethodType(pcl::SAC_RANSAC);
	plane_trunk_head.setDistanceThreshold(0.01);
	plane_trunk_head.setInputCloud(cloud_head_plane_potential);
	plane_trunk_head.segment(*inliers_head, *plane_coeff_trunk_head);

	pcl::ExtractIndices<PointT> extract_trunk_head;
	extract_trunk_head.setInputCloud(cloud_head_plane_potential);
	extract_trunk_head.setIndices(inliers_head);
	extract_trunk_head.setNegative(false);
	extract_trunk_head.filter(*cloud_trunk_head);
#ifdef SHOW_PLANE_COEFF
	std::cout << "coefficients of trunk head plane " << std::endl;
	for (size_t i = 0; i < plane_coeff_trunk_head->values.size(); ++i) {
		std::cout << "	" << coeff_plane[i] << ":";
		std::cout << "	" << plane_coeff_trunk_head->values[i] << std::endl;
	}
#endif

	float trunk_head_height = 0.0;
	for (size_t i = 0; i < cloud_trunk_head->size(); ++i) {
		trunk_head_height += cloud_trunk_head->points[i].z;
	}
	trunk_head_height /= cloud_trunk_head->size();
	std::cout << "The height of trunk head: " << trunk_head_height << std::endl;


	// [13] get trunk head width
	PointPtr cloud_head_potential(new pcl::PointCloud<PointT>());
	for (size_t i = 0; i < cloud_downsampled_rev->size(); ++i) {
		if (trunk_pos_xmax - cloud_downsampled_rev->points[i].x < 1.5 && 
		abs(trunk_head_height - cloud_downsampled_rev->points[i].z) < 0.15 ) {
			cloud_head_potential->push_back(cloud_downsampled_rev->points[i]);
		}
	}

	pcl::ProjectInliers<PointT> project_head_ground;
	PointPtr cloud_head_projected_ground(new pcl::PointCloud<PointT>);
	project_head_ground.setModelType(pcl::SACMODEL_PLANE);
	project_head_ground.setInputCloud(cloud_head_potential);
	project_head_ground.setModelCoefficients(plane_coeff_ground);
	project_head_ground.filter(*cloud_head_projected_ground);

	float trunk_head_ymax = -100.0;
	float trunk_head_ymin = 100.0;
	for (size_t i = 0; i < cloud_head_projected_ground->size(); ++i) {
		if (cloud_head_projected_ground->points[i].y > trunk_head_ymax)
			trunk_head_ymax = cloud_head_projected_ground->points[i].y;
		if (cloud_head_projected_ground->points[i].y < trunk_head_ymin)
			trunk_head_ymin = cloud_head_projected_ground->points[i].y;
	}

	float trunk_head_width = trunk_head_ymax - trunk_head_ymin;
	std::cout << "The width of trunk head: " << trunk_head_width << std::endl;

	// VisualizationHandle cloud_downsampled_colorh(cloud_downsampled_rev, 255, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_downsampled_rev, cloud_downsampled_colorh, "downsampled cloud");
	// VisualizationHandle cloud_head_plane_potential_colorh(cloud_head_plane_potential, 255, 0, 0);
	// viewers.addPointCloud<PointT>(cloud_head_plane_potential, cloud_head_plane_potential_colorh, "head plane potential cloud");
	// VisualizationHandle cloud_head_colorh(cloud_trunk_head, 0, 255, 0);
	// viewers.addPointCloud<PointT>(cloud_trunk_head, cloud_head_colorh, "head cloud");
	// VisualizationHandle cloud_head_potential_colorh(cloud_head_potential, 0, 0, 255);
	// viewers.addPointCloud<PointT>(cloud_head_potential, cloud_head_potential_colorh, "head potential cloud");
	// VisualizationHandle cloud_head_projected_colorh(cloud_head_projected_ground, 255, 0, 255);
	// viewers.addPointCloud<PointT>(cloud_head_projected_ground, cloud_head_projected_colorh, "head projected cloud");


	// [14] get subface tail height and ymax ymin xmin xmax
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
	std::cout << "subface_pos_xmax: " << subface_pos_xmax << std::endl;
	std::cout << "subface_pos_xmin: " << subface_pos_xmin << std::endl;
	std::cout << "subface_pos_ymax: " << subface_pos_ymax << std::endl;
	std::cout << "subface_pos_ymin: " << subface_pos_ymin << std::endl;

	PointPtr cloud_subface_tail(new pcl::PointCloud<PointT>());
	int num_point_subface_tail = 0;
	float subface_tail_height = 0.0;
	for (size_t i = 0; i < cloud_subface_rev->size(); ++i) {
		if (cloud_subface_rev->points[i].x - subface_pos_xmin <= 0.03) {
			num_point_subface_tail ++;
			subface_tail_height += cloud_subface_rev->points[i].z;
			cloud_subface_tail->push_back(cloud_subface_rev->points[i]);
		}
	}
	subface_tail_height = subface_tail_height / static_cast<float>(num_point_subface_tail);
	std::cout << "The height of trunk subface tail: " << subface_tail_height << std::endl;
	float trunk_width_max = subface_pos_ymax - subface_pos_ymin;
	std::cout << "The max width of trunk subface: " << trunk_width_max << std::endl;

	// VisualizationHandle cloud_downsampled_colorh(cloud_downsampled_rev, 255, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_downsampled_rev, cloud_downsampled_colorh, "downsampled cloud");
	// VisualizationHandle cloud_subface_colorh(cloud_subface_rev, 255, 0, 0);
	// viewers.addPointCloud<PointT>(cloud_subface_rev, cloud_subface_colorh, "subface cloud");
	// VisualizationHandle cloud_subface_filtered_colorh(cloud_subface_filtered, 0, 255, 0);
	// viewers.addPointCloud<PointT>(cloud_subface_filtered, cloud_subface_filtered_colorh, "subface filtered cloud");
	// VisualizationHandle cloud_subface_tail_colorh(cloud_subface_tail, 0, 0, 255);
	// viewers.addPointCloud<PointT>(cloud_subface_tail, cloud_subface_tail_colorh, "subface tail cloud");


	// [15] judge the stair and get the stair height and length
	PointPtr cloud_trunk_stair;
	float trunk_stair_height = 0.0;
	float trunk_stair_length = 0.0;
	float trunk_stair_x_front = 0.0;
	float trunk_stair_x_back = 0.0;
	for (size_t i = 0; i < vec_cloud_horizontal.size(); ++i) {
		float plane_height = abs(vec_plane_coeff_trunk_horizontal[i]->values[3]);
		if ( plane_height > horizontal_min_height + 0.01 && plane_height < trunk_head_height - 0.1 ) {
			float max_plane_x = -100.0;
			float min_plane_x = 100.0;
			float max_plane_y = -100.0;
			float min_plane_y = 100.0;
			for (size_t j = 0; j < vec_cloud_horizontal_filtered_rev[i]->size(); ++j) {
				if (vec_cloud_horizontal_filtered_rev[i]->points[j].x > max_plane_x)
					max_plane_x = vec_cloud_horizontal_filtered_rev[i]->points[j].x;
				if (vec_cloud_horizontal_filtered_rev[i]->points[j].x < min_plane_x)
					min_plane_x = vec_cloud_horizontal_filtered_rev[i]->points[j].x;
				if (vec_cloud_horizontal_filtered_rev[i]->points[j].y > max_plane_y)
					max_plane_y = vec_cloud_horizontal_filtered_rev[i]->points[j].y;
				if (vec_cloud_horizontal_filtered_rev[i]->points[j].y < min_plane_y)
					min_plane_y = vec_cloud_horizontal_filtered_rev[i]->points[j].y;
			}
			// std::cout << "At " << i << ", min_plane_x: " << min_plane_x << std::endl;
			// std::cout << "At " << i << ", subface_pos_xmax: " << subface_pos_xmax << std::endl;
			// std::cout << "At " << i << ", min_plane_y: " << min_plane_y << std::endl;
			// std::cout << "At " << i << ", subface_pos_ymin: " << subface_pos_ymin << std::endl;
			// std::cout << "At " << i << ", max_plane_y: " << max_plane_y << std::endl;
			// std::cout << "At " << i << ", subface_pos_ymax: " << subface_pos_ymax << std::endl;
			if ( abs(min_plane_x - subface_pos_xmax) < 0.2 && abs(max_plane_y - subface_pos_ymax) < 0.2
			&&  abs(min_plane_y - subface_pos_ymin) < 0.2 ) {
				cloud_trunk_stair = vec_cloud_horizontal_filtered_rev[i];
				for (size_t k = 0; k < cloud_trunk_stair->size(); ++k) {
					trunk_stair_height += cloud_trunk_stair->points[k].z;
				}
				trunk_stair_height /= cloud_trunk_stair->size();

				// trunk_stair_length = get_length(cloud_trunk_stair, trunk_stair_x_front, trunk_stair_x_back, 0.8);
				trunk_stair_length = max_plane_y - min_plane_y;
				trunk_stair_x_front = max_plane_x;
				trunk_stair_x_back = min_plane_x;
				trunk_stair = true;
				break;
			}
		}
	}
	if (trunk_stair) {
		std::cout << "Trunk stair exist" << std::endl;
		std::cout << "stair height: " << trunk_stair_height << std::endl;
		std::cout << "stair length: " << trunk_stair_length << std::endl;
	}
	else {
		std::cout << "Trunk stair don't exist" << std::endl;
	}


	// [16] segment trunk left and right plane and get trunk side height
	PointPtr trunk_plane_right(new pcl::PointCloud<PointT>());
	PointPtr trunk_plane_left(new pcl::PointCloud<PointT>());
	PointPtr trunk_plane_back(new pcl::PointCloud<PointT>());
	PointPtr trunk_plane_front(new pcl::PointCloud<PointT>());
	if (trunk_stair) {
		for (size_t i = 0; i < cloud_normals_rev->size(); ++i) {
			if (cloud_downsampled_rev->points[i].x < trunk_stair_x_front && cloud_downsampled_rev->points[i].x > subface_pos_xmin
			&& cloud_downsampled_rev->points[i].z > trunk_stair_height + 0.2) {
				if (cloud_downsampled_rev->points[i].y < subface_pos_ymin + 0.01) 
					trunk_plane_right->push_back(cloud_downsampled_rev->points[i]);
				if (cloud_downsampled_rev->points[i].y > subface_pos_ymax - 0.01) 
					trunk_plane_left->push_back(cloud_downsampled_rev->points[i]);
			}
			else if (cloud_downsampled_rev->points[i].x <= subface_pos_xmin + 0.1 
			&& cloud_downsampled_rev->points[i].y < subface_pos_ymax && cloud_downsampled_rev->points[i].y > subface_pos_ymin 
			&& cloud_downsampled_rev->points[i].z > trunk_stair_height + 0.2) {
				trunk_plane_back->push_back(cloud_downsampled_rev->points[i]);
			}
			else if (cloud_downsampled_rev->points[i].x >= trunk_stair_x_front - 0.1 
			&& cloud_downsampled_rev->points[i].x < trunk_stair_x_front + 0.2
			&& cloud_downsampled_rev->points[i].y < subface_pos_ymax && cloud_downsampled_rev->points[i].y > subface_pos_ymin
			&& cloud_downsampled_rev->points[i].z > trunk_stair_height + 0.2) {
				trunk_plane_front->push_back(cloud_downsampled_rev->points[i]);
			}
		}
	}
	else {
		for (size_t i = 0; i < cloud_normals_rev->size(); ++i) {
			if (cloud_downsampled_rev->points[i].x < subface_pos_xmax - 0.1 && cloud_downsampled_rev->points[i].x > subface_pos_xmin
			&& cloud_downsampled_rev->points[i].z > subface_tail_height + 0.2) {
				if (cloud_downsampled_rev->points[i].y < subface_pos_ymin + 0.01) 
					trunk_plane_right->push_back(cloud_downsampled_rev->points[i]);
				if (cloud_downsampled_rev->points[i].y > subface_pos_ymax - 0.01) 
					trunk_plane_left->push_back(cloud_downsampled_rev->points[i]);
			}
			else if (cloud_downsampled_rev->points[i].x <= subface_pos_xmin + 0.1 
			&& cloud_downsampled_rev->points[i].y < subface_pos_ymax && cloud_downsampled_rev->points[i].y > subface_pos_ymin 
			&& cloud_downsampled_rev->points[i].z > subface_tail_height + 0.2) {
				trunk_plane_back->push_back(cloud_downsampled_rev->points[i]);
			}
			else if (cloud_downsampled_rev->points[i].x >= subface_pos_xmax - 0.1 
			&& cloud_downsampled_rev->points[i].x < subface_pos_xmax + 0.2
			&& cloud_downsampled_rev->points[i].y < subface_pos_ymax && cloud_downsampled_rev->points[i].y > subface_pos_ymin
			&& cloud_downsampled_rev->points[i].z > subface_tail_height + 0.2) {
				trunk_plane_front->push_back(cloud_downsampled_rev->points[i]);
			}
		}
	}

	// VisualizationHandle cloud_downsampled_colorh(cloud_downsampled_rev, 255, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_downsampled_rev, cloud_downsampled_colorh, "downsampled cloud");
	// VisualizationHandle trunk_plane_left_colorh(trunk_plane_left, 255, 0, 0);
	// viewers.addPointCloud<PointT>(trunk_plane_left, trunk_plane_left_colorh, "trunk plane left cloud");
	// VisualizationHandle trunk_plane_right_colorh(trunk_plane_right, 255, 0, 0);
	// viewers.addPointCloud<PointT>(trunk_plane_right, trunk_plane_right_colorh, "trunk plane right cloud");
	// VisualizationHandle trunk_plane_back_colorh(trunk_plane_back, 0, 0, 255);
	// viewers.addPointCloud<PointT>(trunk_plane_back, trunk_plane_back_colorh, "trunk plane back cloud");
	// VisualizationHandle trunk_plane_front_colorh(trunk_plane_front, 0, 0, 255);
	// viewers.addPointCloud<PointT>(trunk_plane_front, trunk_plane_front_colorh, "trunk plane front cloud");


	// [17] get trunk side height
	PointPtr trunk_line_height_left(new pcl::PointCloud<PointT>());
	PointPtr trunk_line_height_right(new pcl::PointCloud<PointT>());
	PointPtr trunk_line_height_back(new pcl::PointCloud<PointT>());
	PointPtr trunk_line_height_front(new pcl::PointCloud<PointT>());

	float left_height = get_side_height(trunk_plane_left, trunk_line_height_left, 0.01);
	std::cout << "The height of the trunk left side is " << left_height << std::endl;
	float right_height = get_side_height(trunk_plane_right, trunk_line_height_right, 0.01);
	std::cout << "The height of the trunk right side is " << right_height << std::endl;
	float back_height = get_fb_height(trunk_plane_back, trunk_line_height_back, 0.01);
	std::cout << "The height of the trunk back side is " << back_height << std::endl;
	float front_height = get_fb_height(trunk_plane_front, trunk_line_height_front, 0.01);
	std::cout << "The height of the trunk front side is " << front_height << std::endl;

	// VisualizationHandle trunk_line_left_colorh(trunk_line_height_left, 0, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_line_height_left, trunk_line_left_colorh, "trunk line left height cloud");
	// VisualizationHandle trunk_line_right_colorh(trunk_line_height_right, 0, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_line_height_right, trunk_line_right_colorh, "trunk line right height cloud");
	// VisualizationHandle trunk_line_back_colorh(trunk_line_height_back, 255, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_line_height_back, trunk_line_back_colorh, "trunk line back height cloud");
	// VisualizationHandle trunk_line_front_colorh(trunk_line_height_front, 255, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_line_height_front, trunk_line_front_colorh, "trunk line front height cloud");


	// [18] get trunk length
	PointPtr cloud_front_line_potential( trunk_plane_front );
	PointPtr cloud_back_line_potential( trunk_plane_back );

	float trunk_front = 0.0;
	float trunk_back = 0.0;
	float front_xposmax;
	float front_xposmin;
	float back_xposmax;
	float back_xposmin;
	get_length(cloud_front_line_potential, front_xposmax, front_xposmin, 0.4);
	get_length(cloud_back_line_potential, back_xposmax, back_xposmin, 0.4);
	PointPtr cloud_front_line_back(new pcl::PointCloud<PointT>());
	PointPtr cloud_back_line_front(new pcl::PointCloud<PointT>());
	for (size_t i = 0; i < cloud_front_line_potential->size(); ++i) {
		if (abs(cloud_front_line_potential->points[i].x - front_xposmin) <= 0.01) {
			cloud_front_line_back->push_back(cloud_front_line_potential->points[i]);
			trunk_front += cloud_front_line_potential->points[i].x;
		}
	}
	for (size_t i = 0; i < cloud_back_line_potential->size(); ++i) {
		if (abs(cloud_back_line_potential->points[i].x - back_xposmax) <= 0.01) {
			cloud_back_line_front->push_back(cloud_back_line_potential->points[i]);
			trunk_back += cloud_back_line_potential->points[i].x;
		}
	}
	trunk_front /= static_cast<float>( cloud_front_line_back->size() );
	trunk_back /= static_cast<float>( cloud_back_line_front->size() );
	float trunk_length = trunk_front - trunk_back;
	std::cout << "The xpos of front is " << trunk_front << std::endl;
	std::cout << "The xpos of back is " << trunk_back << std::endl;
	std::cout << "The length of the trunk is " << trunk_length << std::endl;

	// VisualizationHandle trunk_line_front_potential_colorh(cloud_front_line_potential, 0, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_front_line_potential, trunk_line_front_potential_colorh, "trunk line front potential cloud");
	// VisualizationHandle trunk_line_back_potential_colorh(cloud_back_line_potential, 0, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_back_line_potential, trunk_line_back_potential_colorh, "trunk line back potential cloud");
	// VisualizationHandle trunk_line_front_back_colorh(cloud_front_line_back, 255, 0, 255);
	// viewers.addPointCloud<PointT>(cloud_front_line_back, trunk_line_front_back_colorh, "trunk line front cloud");
	// VisualizationHandle trunk_line_back_front_colorh(cloud_back_line_front, 255, 0, 255);
	// viewers.addPointCloud<PointT>(cloud_back_line_front, trunk_line_back_front_colorh, "trunk line back cloud");


	// [19] get trunk width with a certain resolution
	int width_size = static_cast<int>(trunk_length / WIDTH_RESOLUTION) + 1;
	std::vector<float> vec_trunk_width(width_size, 0);
	std::vector<float> vec_trunk_ymax(width_size, -100.0);
	std::vector<float> vec_trunk_ymin(width_size, 100.0);

	// // [19.1] method 1
	// for (size_t i = 0; i < cloud_subface_rev->size(); ++i) {
	// 	if (cloud_subface_rev->points[i].x <= trunk_front && cloud_subface_rev->points[i].x >= trunk_back) {
	// 		int temp = static_cast<int>((trunk_front - cloud_subface_rev->points[i].x) / WIDTH_RESOLUTION);
	// 		if (cloud_subface_rev->points[i].y > vec_trunk_ymax[temp])
	// 			vec_trunk_ymax[temp] = cloud_subface_rev->points[i].y;
	// 		if (cloud_subface_rev->points[i].y < vec_trunk_ymin[temp])
	// 			vec_trunk_ymin[temp] = cloud_subface_rev->points[i].y;
	// 	}
	// }
	// for (int i = 0; i < width_size; ++i) {
	// 	vec_trunk_width[i] = vec_trunk_ymax[i] - vec_trunk_ymin[i];
	// 	if (trunk_width_max - vec_trunk_width[i] > 1) {
	// 		vec_trunk_width[i] = trunk_width_max;
	// 		vec_trunk_ymax[i] = subface_pos_ymax;
	// 		vec_trunk_ymin[i] = subface_pos_ymin;
	// 	}
	// 	// cout << "i    " << i << "    vec_trunk_width    " << vec_trunk_width[i] << endl;
	// }

	// [19.2] method 2
	PointPtr cloud_subface_hull(new pcl::PointCloud<PointT>());
	std::vector<pcl::Vertices> polygons_subface;
	pcl::ConcaveHull<PointT> concavehull_subface;
	concavehull_subface.setInputCloud(cloud_subface_rev); 
	concavehull_subface.setAlpha(1.0);
	concavehull_subface.setKeepInformation(false);
	concavehull_subface.setDimension(2); 
	concavehull_subface.reconstruct(*cloud_subface_hull, polygons_subface);

	float subface_hull_ymin = 100.0;
	float subface_hull_ymax = -100.0;
	for (size_t i = 0; i < cloud_subface_hull->size(); ++i) {
		if (cloud_subface_hull->points[i].y < subface_hull_ymin)
			subface_hull_ymin = cloud_subface_hull->points[i].y;
		if (cloud_subface_hull->points[i].y > subface_hull_ymax)
			subface_hull_ymax = cloud_subface_hull->points[i].y;
	}

	PointPtr cloud_hull_left_potential(new pcl::PointCloud<PointT>());
	PointPtr cloud_hull_right_potential(new pcl::PointCloud<PointT>());

	for (size_t i = 0; i < cloud_subface_hull->size(); ++i) {
		if (subface_hull_ymax - cloud_subface_hull->points[i].y < 0.1)
			cloud_hull_left_potential->push_back(cloud_subface_hull->points[i]);
		if (cloud_subface_hull->points[i].y - subface_hull_ymin < 0.1)
			cloud_hull_right_potential->push_back(cloud_subface_hull->points[i]);
	}

	pcl::ModelCoefficients::Ptr line_coeff_subface_left(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_subface_left(new pcl::PointIndices);
	pcl::SACSegmentation<PointT> seg_subface_left;
	seg_subface_left.setOptimizeCoefficients(true);
	seg_subface_left.setModelType(pcl::SACMODEL_LINE);
	seg_subface_left.setMethodType(pcl::SAC_RANSAC);
	seg_subface_left.setDistanceThreshold(0.01);
	seg_subface_left.setInputCloud(cloud_hull_left_potential);
	seg_subface_left.segment( *inliers_subface_left, *line_coeff_subface_left );
#ifdef SHOW_PLANE_COEFF
	std::cout << "coefficients of trunk subface left " << std::endl;
	for (size_t i = 0; i < line_coeff_subface_left->values.size(); ++i) {
		std::cout << "	" << coeff_line[i] << ":";
		std::cout << "	" << line_coeff_subface_left->values[i] << std::endl;
	}
#endif
	float subface_left_line_k = line_coeff_subface_left->values[4] / line_coeff_subface_left->values[3];
	float subface_left_line_b = line_coeff_subface_left->values[1] - line_coeff_subface_left->values[0] * subface_left_line_k;
	std::cout << "subface left line is y = " << subface_left_line_k << " + " << subface_left_line_b << std::endl; 

	pcl::ModelCoefficients::Ptr line_coeff_subface_right(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_subface_right(new pcl::PointIndices);
	pcl::SACSegmentation<PointT> seg_subface_right;
	seg_subface_right.setOptimizeCoefficients(true);
	seg_subface_right.setModelType(pcl::SACMODEL_LINE);
	seg_subface_right.setMethodType(pcl::SAC_RANSAC);
	seg_subface_right.setDistanceThreshold(0.01);
	seg_subface_right.setInputCloud(cloud_hull_right_potential);
	seg_subface_right.segment( *inliers_subface_right, *line_coeff_subface_right );
#ifdef SHOW_PLANE_COEFF
	std::cout << "coefficients of trunk subface right " << std::endl;
	for (size_t i = 0; i < line_coeff_subface_right->values.size(); ++i) {
		std::cout << "	" << coeff_line[i] << ":";
		std::cout << "	" << line_coeff_subface_right->values[i] << std::endl;
	}
#endif
	float subface_right_line_k = line_coeff_subface_right->values[4] / line_coeff_subface_right->values[3];
	float subface_right_line_b = line_coeff_subface_right->values[1] - line_coeff_subface_right->values[0] * subface_right_line_k;
	std::cout << "subface right line is y = " << subface_right_line_k << " + " << subface_right_line_b << std::endl; 

	for (int i = 0; i < width_size; ++i) {
		float posx = trunk_front + i * WIDTH_RESOLUTION + WIDTH_RESOLUTION / 2;
		vec_trunk_ymax[i] = subface_left_line_k * posx + subface_left_line_b;
		vec_trunk_ymin[i] = subface_right_line_k * posx + subface_right_line_b;
		vec_trunk_width[i] = vec_trunk_ymax[i] - vec_trunk_ymin[i];
		// cout << "i    " << i << "    vec_trunk_width    " << vec_trunk_width[i] << endl;
	}

	// // [19.3] method 3
	// std::vector<float> vec_trunk_width(width_size, 0);
	// std::vector<float> vec_trunk_ymax(width_size, 0.0);
	// std::vector<float> vec_trunk_ymin(width_size, 0.0);
	// std::vector<int> vec_trunk_width_left_num(width_size, 0);
	// std::vector<int> vec_trunk_width_right_num(width_size, 0);
	// for (size_t i = 0; i < trunk_line_height_left->size(); ++i) {
	// 	if (trunk_line_height_left->points[i].x <= trunk_front && trunk_line_height_left->points[i].x >= trunk_back) {
	// 		int temp = static_cast<int>((trunk_front - trunk_line_height_left->points[i].x) / WIDTH_RESOLUTION);
	// 		vec_trunk_ymax[temp] += trunk_line_height_left->points[i].y;
	// 		vec_trunk_width_left_num[temp]++;
	// 	}
	// }
	// for (size_t i = 0; i < trunk_line_height_right->size(); ++i) {
	// 	if (trunk_line_height_right->points[i].x <= trunk_front && trunk_line_height_right->points[i].x >= trunk_back) {
	// 		int temp = static_cast<int>((trunk_front - trunk_line_height_right->points[i].x) / WIDTH_RESOLUTION);
	// 		vec_trunk_ymin[temp] += trunk_line_height_right->points[i].y;
	// 		vec_trunk_width_right_num[temp]++;
	// 	}
	// }
	// for (int i = 0; i < width_size; ++i) {
	// 	vec_trunk_ymax[i] /= vec_trunk_width_left_num[i];
	// 	vec_trunk_ymin[i] /= vec_trunk_width_right_num[i];
	// 	vec_trunk_width[i] = vec_trunk_ymax[i] - vec_trunk_ymin[i];
	// 	cout << "i    " << i << "    vec_trunk_width    " << vec_trunk_width[i] << endl;
	// }

	// VisualizationHandle cloud_subface_colorh(cloud_subface_rev, 255, 255, 255);
	// viewers.addPointCloud<PointT>(cloud_subface_rev, cloud_subface_colorh, "subface cloud");
	// VisualizationHandle cloud_subface_hull_colorh(cloud_subface_hull, 0, 255, 0);
	// viewers.addPointCloud<PointT>(cloud_subface_hull, cloud_subface_hull_colorh, "subface convex hull cloud");
	// VisualizationHandle cloud_subface_hull_left_colorh(cloud_hull_left_potential, 255, 0, 0);
	// viewers.addPointCloud<PointT>(cloud_hull_left_potential, cloud_subface_hull_left_colorh, "subface convex hull left cloud");
	// VisualizationHandle cloud_subface_hull_right_colorh(cloud_hull_right_potential, 0, 0, 255);
	// viewers.addPointCloud<PointT>(cloud_hull_right_potential, cloud_subface_hull_right_colorh, "subface convex hull right cloud");


	// [20] get trunk subface marking point before and after revise
	float x_marking_point;
	float y_marking_point;
	get_marking_point(cloud_trunk_subface, x_marking_point, y_marking_point, rotation_angle_horizontal, trunk_length);
	std::cout << "The marking point is (" << x_marking_point << ", " << y_marking_point << ")" << std::endl;

	float x_marking_point_rev;
	float y_marking_point_rev;
	get_marking_point(cloud_subface_rev, x_marking_point_rev, y_marking_point_rev, 0.0, trunk_length);
	std::cout << "The marking point after revise is (" << x_marking_point_rev << ", " << y_marking_point_rev << ")" << std::endl;


	// [21] get trunk side protrusion
	std::vector<PointPtr> vec_cloud_left_protrusion;
	std::vector<float> vec_trunk_left_protrusion_height;
	std::vector<float> vec_trunk_left_protrusion_xmax;
	std::vector<float> vec_trunk_left_protrusion_xmin;
	std::vector<float> vec_trunk_left_protrusion_ymax;
	std::vector<float> vec_trunk_left_protrusion_ymin;
	int left_protrusion = get_side_protrusion(trunk_plane_left, left_height, 
											  vec_cloud_left_protrusion, vec_trunk_left_protrusion_height, 
											  vec_trunk_left_protrusion_xmax, vec_trunk_left_protrusion_xmin,
											  vec_trunk_left_protrusion_ymax, vec_trunk_left_protrusion_ymin);
	if (left_protrusion < 0) {
		std::cout << "No left protrusion" << std::endl;
	}
	else {
		for (int i = 0; i < vec_cloud_left_protrusion.size(); ++i) {
			std::cout << "point size of cluster " << i << " at left protrusion is: " << vec_cloud_left_protrusion[i]->size() << std::endl;
			std::cout << "trunk left protrusion of cluster " << i << " height: " << vec_trunk_left_protrusion_height[i] << std::endl;
			std::cout << "trunk left protrusion of cluster " << i << " xmax: " << vec_trunk_left_protrusion_xmax[i] << std::endl;
			std::cout << "trunk left protrusion of cluster " << i << " xmin: " << vec_trunk_left_protrusion_xmin[i] << std::endl;
			std::cout << "trunk left protrusion of cluster " << i << " ymax: " << vec_trunk_left_protrusion_ymax[i] << std::endl;
			std::cout << "trunk left protrusion of cluster " << i << " ymin: " << vec_trunk_left_protrusion_ymin[i] << std::endl;
		}
	}

	std::vector<PointPtr> vec_cloud_right_protrusion;
	std::vector<float> vec_trunk_right_protrusion_height;
	std::vector<float> vec_trunk_right_protrusion_xmax;
	std::vector<float> vec_trunk_right_protrusion_xmin;
	std::vector<float> vec_trunk_right_protrusion_ymax;
	std::vector<float> vec_trunk_right_protrusion_ymin;
	int right_protrusion = get_side_protrusion(trunk_plane_right, right_height, 
											   vec_cloud_right_protrusion, vec_trunk_right_protrusion_height, 
											   vec_trunk_right_protrusion_xmax, vec_trunk_right_protrusion_xmin,
											   vec_trunk_right_protrusion_ymax, vec_trunk_right_protrusion_ymin);
	if (right_protrusion < 0) {
		std::cout << "No right protrusion" << std::endl;
	}
	else {
		for (int i = 0; i < vec_cloud_right_protrusion.size(); ++i) {
			std::cout << "point size of cluster " << i << " at right protrusion is: " << vec_cloud_right_protrusion[i]->size() << std::endl;
			std::cout << "trunk right protrusion of cluster " << i << " height: " << vec_trunk_right_protrusion_height[i] << std::endl;
			std::cout << "trunk right protrusion of cluster " << i << " xmax: " << vec_trunk_right_protrusion_xmax[i] << std::endl;
			std::cout << "trunk right protrusion of cluster " << i << " xmin: " << vec_trunk_right_protrusion_xmin[i] << std::endl;
			std::cout << "trunk right protrusion of cluster " << i << " ymax: " << vec_trunk_right_protrusion_ymax[i] << std::endl;
			std::cout << "trunk right protrusion of cluster " << i << " ymin: " << vec_trunk_right_protrusion_ymin[i] << std::endl;
		}
	}

	std::vector<PointPtr> vec_cloud_back_protrusion;
	std::vector<float> vec_trunk_back_protrusion_height;
	std::vector<float> vec_trunk_back_protrusion_xmax;
	std::vector<float> vec_trunk_back_protrusion_xmin;
	std::vector<float> vec_trunk_back_protrusion_ymax;
	std::vector<float> vec_trunk_back_protrusion_ymin;
	int back_protrusion = get_side_protrusion(trunk_plane_back, back_height, 
											  vec_cloud_back_protrusion, vec_trunk_back_protrusion_height, 
											  vec_trunk_back_protrusion_xmax, vec_trunk_back_protrusion_xmin,
											  vec_trunk_back_protrusion_ymax, vec_trunk_back_protrusion_ymin);
	if (back_protrusion < 0) {
		std::cout << "No back protrusion" << std::endl;
	}
	else {
		for (int i = 0; i < vec_cloud_back_protrusion.size(); ++i) {
			std::cout << "point size of cluster " << i << " at back protrusion is: " << vec_cloud_back_protrusion[i]->size() << std::endl;
			std::cout << "trunk back protrusion of cluster " << i << " height: " << vec_trunk_back_protrusion_height[i] << std::endl;
			std::cout << "trunk back protrusion of cluster " << i << " xmax: " << vec_trunk_back_protrusion_xmax[i] << std::endl;
			std::cout << "trunk back protrusion of cluster " << i << " xmin: " << vec_trunk_back_protrusion_xmin[i] << std::endl;
			std::cout << "trunk back protrusion of cluster " << i << " ymax: " << vec_trunk_back_protrusion_ymax[i] << std::endl;
			std::cout << "trunk back protrusion of cluster " << i << " ymin: " << vec_trunk_back_protrusion_ymin[i] << std::endl;
		}
	}

	// VisualizationHandle trunk_plane_left_colorh(trunk_plane_left, 255, 0, 0);
	// viewers.addPointCloud<PointT>(trunk_plane_left, trunk_plane_left_colorh, "trunk plane left cloud");
	// VisualizationHandle trunk_plane_right_colorh(trunk_plane_right, 255, 0, 0);
	// viewers.addPointCloud<PointT>(trunk_plane_right, trunk_plane_right_colorh, "trunk plane right cloud");
	// VisualizationHandle trunk_line_left_colorh(trunk_line_height_left, 0, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_line_height_left, trunk_line_left_colorh, "trunk line left height cloud");
	// VisualizationHandle trunk_line_right_colorh(trunk_line_height_right, 0, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_line_height_right, trunk_line_right_colorh, "trunk line right height cloud");
	// VisualizationHandle trunk_left_prot_potential_colorh(trunk_left_protrusion_potential, 0, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_left_protrusion_potential, trunk_left_prot_potential_colorh, "trunk left protrusion potential");
	// VisualizationHandle trunk_right_prot_potential_colorh(trunk_right_protrusion_potential, 0, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_right_protrusion_potential, trunk_right_prot_potential_colorh, "trunk right protrusion potential");
	// VisualizationHandle trunk_back_prot_potential_colorh(trunk_back_protrusion_potential, 0, 255, 0);
	// viewers.addPointCloud<PointT>(trunk_back_protrusion_potential, trunk_back_prot_potential_colorh, "trunk back protrusion potential");
	// VisualizationHandle trunk_left_prot_filter_colorh(vec_cloud_left_protrusion[0], 255, 0, 0);
	// viewers.addPointCloud<PointT>(vec_cloud_left_protrusion[0], trunk_left_prot_filter_colorh, "trunk left protrusion filter");
	// VisualizationHandle trunk_right_prot_filter_colorh(vec_cloud_right_protrusion[0], 255, 0, 255);
	// viewers.addPointCloud<PointT>(vec_cloud_right_protrusion[0], trunk_right_prot_filter_colorh, "trunk right protrusion filter");
	// VisualizationHandle trunk_back0_prot_filter_colorh(vec_cloud_back_protrusion[0], 0, 255, 255);
	// viewers.addPointCloud<PointT>(vec_cloud_back_protrusion[0], trunk_back0_prot_filter_colorh, "trunk back0 protrusion filter");
	// VisualizationHandle trunk_back1_prot_filter_colorh(vec_cloud_back_protrusion[1], 255, 255, 0);
	// viewers.addPointCloud<PointT>(vec_cloud_back_protrusion[1], trunk_back1_prot_filter_colorh, "trunk back1 protrusion filter");


	// [22] get trunk inside protrusion
	PointPtr trunk_inside_protrusion_potential(new pcl::PointCloud<PointT>());
	if (trunk_stair) {
		for (size_t i = 0; i < cloud_downsampled_rev->size(); ++i) {
			if (cloud_downsampled_rev->points[i].x < subface_pos_xmax - 0.01 && cloud_downsampled_rev->points[i].x > trunk_back + 0.01 
			&& cloud_downsampled_rev->points[i].z > subface_tail_height + 0.1) {
				int temp = static_cast<int>((trunk_front - cloud_downsampled_rev->points[i].x) / WIDTH_RESOLUTION);
				if (cloud_downsampled_rev->points[i].y < vec_trunk_ymax[temp] - 0.05
					&& cloud_downsampled_rev->points[i].y > vec_trunk_ymin[temp] + 0.05) {
					trunk_inside_protrusion_potential->push_back(cloud_downsampled_rev->points[i]);
				}
			}
			else if (cloud_downsampled_rev->points[i].x < trunk_front - 0.01 && cloud_downsampled_rev->points[i].x > subface_pos_xmax + 0.01 
			&& cloud_downsampled_rev->points[i].z > trunk_stair_height + 0.1) {
				int temp = static_cast<int>((trunk_front - cloud_downsampled_rev->points[i].x) / WIDTH_RESOLUTION);
				if (cloud_downsampled_rev->points[i].y < vec_trunk_ymax[temp] - 0.05
				&& cloud_downsampled_rev->points[i].y > vec_trunk_ymin[temp] + 0.05) {
					trunk_inside_protrusion_potential->push_back(cloud_downsampled_rev->points[i]);
				}
			}
		}
	}
	else {
		for (size_t i = 0; i < cloud_downsampled_rev->size(); ++i) {
			if (cloud_downsampled_rev->points[i].x < trunk_front - 0.05 && cloud_downsampled_rev->points[i].x > trunk_back + 0.05 
			&& cloud_downsampled_rev->points[i].z > subface_tail_height + 0.1) {
				int temp = static_cast<int>((trunk_front - cloud_downsampled_rev->points[i].x) / WIDTH_RESOLUTION);
				if (cloud_downsampled_rev->points[i].y < vec_trunk_ymax[temp] - 0.05
				&& cloud_downsampled_rev->points[i].y > vec_trunk_ymin[temp] + 0.05) {
					trunk_inside_protrusion_potential->push_back(cloud_downsampled_rev->points[i]);
				}
			}
		}
	}

	PointPtr cloud_filter_inside_protrusion( trunk_inside_protrusion_potential );
	// pcl::RadiusOutlierRemoval<PointT> filter_inside_protrusion;
	// PointPtr cloud_filter_inside_protrusion(new pcl::PointCloud<PointT>);
	// filter_inside_protrusion.setInputCloud( trunk_inside_protrusion_potential );
	// filter_inside_protrusion.setRadiusSearch(0.05);
	// filter_inside_protrusion.setMinNeighborsInRadius(10);
	// filter_inside_protrusion.filter( *cloud_filter_inside_protrusion );


	if (cloud_filter_inside_protrusion->size() < 50) {
		std::cout << "No inside protrusion" << std::endl;
	}
	else {
		pcl::search::KdTree<PointT>::Ptr tree_inside_protrusion(new pcl::search::KdTree<PointT>);
		std::vector<pcl::PointIndices> cluster_inside_protrusion;
		pcl::EuclideanClusterExtraction<PointT> ecx_inside_protrusion;
		tree_inside_protrusion->setInputCloud(cloud_filter_inside_protrusion);
		ecx_inside_protrusion.setClusterTolerance(0.1);
		ecx_inside_protrusion.setMinClusterSize(50);
		ecx_inside_protrusion.setMaxClusterSize(3000);
		ecx_inside_protrusion.setSearchMethod(tree_inside_protrusion);
		ecx_inside_protrusion.setInputCloud(cloud_filter_inside_protrusion);
		ecx_inside_protrusion.extract(cluster_inside_protrusion);

		std::cout << "size cluster_inside_protrusion " << cluster_inside_protrusion.size() << std::endl;

		std::vector<PointPtr> vec_cloud_inside_protrusion;
		for (int i = 0; i < cluster_inside_protrusion.size(); ++i) {
			float trunk_inside_protrusion_height = 0.0;
			float trunk_inside_protrusion_xmax = -100.0;
			float trunk_inside_protrusion_xmin = 100.0;
			float trunk_inside_protrusion_ymax = -100.0;
			float trunk_inside_protrusion_ymin = 100.0;
			PointPtr cloud_cluster(new pcl::PointCloud<PointT>);
			pcl::PointIndices cluster_index = cluster_inside_protrusion[i];
			for (std::vector<int>::const_iterator it = cluster_index.indices.begin(); it != cluster_index.indices.end(); ++it)
				cloud_cluster->push_back(cloud_filter_inside_protrusion->points[*it]);

			for (size_t i = 0; i < cloud_cluster->size(); ++i) {
				if (cloud_cluster->points[i].z > trunk_inside_protrusion_height)
					trunk_inside_protrusion_height = cloud_cluster->points[i].z;
				if (cloud_cluster->points[i].x > trunk_inside_protrusion_xmax)
					trunk_inside_protrusion_xmax = cloud_cluster->points[i].x;
				if (cloud_cluster->points[i].x < trunk_inside_protrusion_xmin)
					trunk_inside_protrusion_xmin = cloud_cluster->points[i].x;
				if (cloud_cluster->points[i].y > trunk_inside_protrusion_ymax)
					trunk_inside_protrusion_ymax = cloud_cluster->points[i].y;
				if (cloud_cluster->points[i].y < trunk_inside_protrusion_ymin)
					trunk_inside_protrusion_ymin = cloud_cluster->points[i].y;
			}

			// if (trunk_inside_protrusion_ymax - trunk_inside_protrusion_ymin < 0.03)
			// 	continue;
			// if (trunk_inside_protrusion_xmax - trunk_inside_protrusion_xmin < 0.03)
			// 	continue;

			vec_cloud_inside_protrusion.push_back(cloud_cluster);

			std::cout << "point size of cluster " << i << " at inside protrusion is: " << cloud_cluster->size() << std::endl;
			std::cout << "trunk inside protrusion of cluster " << i << " height: " << trunk_inside_protrusion_height << std::endl;
			std::cout << "trunk inside protrusion of cluster " << i << " xmax: " << trunk_inside_protrusion_xmax << std::endl;
			std::cout << "trunk inside protrusion of cluster " << i << " xmin: " << trunk_inside_protrusion_xmin << std::endl;
			std::cout << "trunk inside protrusion of cluster " << i << " ymax: " << trunk_inside_protrusion_ymax << std::endl;
			std::cout << "trunk inside protrusion of cluster " << i << " ymin: " << trunk_inside_protrusion_ymin << std::endl;
		}
		
		// VisualizationHandle oricloud_colorh(cloud_downsampled_rev, 255, 255, 255);
		// viewers.addPointCloud<PointT>(cloud_downsampled_rev, oricloud_colorh, "original cloud");
		// VisualizationHandle inside_protrusion_protential_colorh(cloud_filter_inside_protrusion, 255, 0, 0);
		// viewers.addPointCloud<PointT>(cloud_filter_inside_protrusion, 
		// 							  inside_protrusion_protential_colorh, "inside protrusion protential cloud");
	}
	
	
	// [99] visualization
	VisualizationHandle oricloud_colorh(cloud_downsampled_rev, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_downsampled_rev, oricloud_colorh, "original cloud");
	VisualizationHandle plane_subface_colorh(cloud_subface_rev, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_subface_rev, plane_subface_colorh, "subface cloud");

	VisualizationHandle cloud_head_plane_potential_colorh(cloud_head_plane_potential, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_head_plane_potential, cloud_head_plane_potential_colorh, "head plane potential cloud");
	VisualizationHandle cloud_head_colorh(cloud_trunk_head, 0, 255, 0);
	viewers.addPointCloud<PointT>(cloud_trunk_head, cloud_head_colorh, "head cloud");
	VisualizationHandle cloud_head_potential_colorh(cloud_head_potential, 0, 0, 255);
	viewers.addPointCloud<PointT>(cloud_head_potential, cloud_head_potential_colorh, "head potential cloud");

	VisualizationHandle trunk_plane_left_colorh(trunk_plane_left, 0, 255, 0);
	viewers.addPointCloud<PointT>(trunk_plane_left, trunk_plane_left_colorh, "trunk plane left cloud");
	VisualizationHandle trunk_plane_right_colorh(trunk_plane_right, 0, 255, 0);
	viewers.addPointCloud<PointT>(trunk_plane_right, trunk_plane_right_colorh, "trunk plane right cloud");
	VisualizationHandle trunk_plane_back_colorh(trunk_plane_back, 0, 255, 0);
	viewers.addPointCloud<PointT>(trunk_plane_back, trunk_plane_back_colorh, "trunk plane back cloud");
	VisualizationHandle trunk_plane_front_colorh(trunk_plane_front, 0, 255, 0);
	viewers.addPointCloud<PointT>(trunk_plane_front, trunk_plane_front_colorh, "trunk plane front cloud");

	VisualizationHandle trunk_line_left_colorh(trunk_line_height_left, 0, 0, 255);
	viewers.addPointCloud<PointT>(trunk_line_height_left, trunk_line_left_colorh, "trunk line left height cloud");
	VisualizationHandle trunk_line_right_colorh(trunk_line_height_right, 0, 0, 255);
	viewers.addPointCloud<PointT>(trunk_line_height_right, trunk_line_right_colorh, "trunk line right height cloud");
	VisualizationHandle trunk_line_back_colorh(trunk_line_height_back, 0, 0, 255);
	viewers.addPointCloud<PointT>(trunk_line_height_back, trunk_line_back_colorh, "trunk line back height cloud");
	VisualizationHandle trunk_line_front_colorh(trunk_line_height_front, 0, 0, 255);
	viewers.addPointCloud<PointT>(trunk_line_height_front, trunk_line_front_colorh, "trunk line front height cloud");

	VisualizationHandle trunk_line_front_back_colorh(cloud_front_line_back, 255, 0, 255);
	viewers.addPointCloud<PointT>(cloud_front_line_back, trunk_line_front_back_colorh, "trunk line front cloud");
	VisualizationHandle trunk_line_back_front_colorh(cloud_back_line_front, 255, 0, 255);
	viewers.addPointCloud<PointT>(cloud_back_line_front, trunk_line_back_front_colorh, "trunk line back cloud");

	VisualizationHandle inside_protrusion_protential_colorh(cloud_filter_inside_protrusion, 255, 255, 0);
	viewers.addPointCloud<PointT>(cloud_filter_inside_protrusion, inside_protrusion_protential_colorh, "inside protrusion protential cloud");

	viewers.spin();
	return 0;
}

