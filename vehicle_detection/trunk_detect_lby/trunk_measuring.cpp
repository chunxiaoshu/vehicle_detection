#include <iostream>
#include <ctime>
#include <cmath>
#include <math.h>
#include <thread>
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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>


#define PI 3.14159265


typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
using namespace std;

bool step=false;
bool hydro=false;
int num_stuff=0;
int num_strip=0;
//double ans[17]={0};



////////////////////////////////////////////////////////////////////////////////////////////////////
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


////////////////////////////////////////////////////////////////////////////////////////
//计算两平面模型之间的夹角，返回角度制并取锐角
double findAngleBetweenPlanes(pcl::ModelCoefficients::Ptr a,pcl::ModelCoefficients::Ptr b){
    double cos_angle=((a->values[0])*(b->values[0])+(a->values[1])*(b->values[1])+(a->values[2])*(b->values[2]))/
                        (sqrt(((a->values[0])*(a->values[0]))+((a->values[1])*(a->values[1]))+((a->values[2])*(a->values[2])))+
                         sqrt(((b->values[0])*(b->values[0]))+((b->values[1])*(b->values[1]))+((b->values[2])*(b->values[2])))  );
    double angle=(acos(cos_angle))*180/PI;
    if(angle>90){
        return 180-angle;
    }
    else return angle;
}
/////////////////////////////////////////////////////////////////////////////////////////
//计算模型与点法线的夹角
double findAngleBetweenPlan_Point(pcl::ModelCoefficients::Ptr a,NormalT b){
    double cos_angle=((a->values[0])*(b.data_n[0])+(a->values[1])*(b.data_n[1])+(a->values[2])*(b.data_n[2]))/
                        (sqrt(((a->values[0])*(a->values[0]))+((a->values[1])*(a->values[1]))+((a->values[2])*(a->values[2])))+
                         sqrt(((b.data_n[0])*(b.data_n[0]))+((b.data_n[1])*(b.data_n[1]))+((b.data_n[2])*(b.data_n[2])))  );
    double angle=(acos(cos_angle))*180/PI;
    if(angle>90){
        return 180-angle;
    }
    else return angle;
}
/////////////////////////////////////////////////////////////////////////////////////////
//计算模型与法线的夹角
double findAngleBetweenPlan_normal3(pcl::ModelCoefficients::Ptr a,float x,float y,float z){
    struct normal_3{
        float data_n[3];
    };
    normal_3* b;
    b->data_n[0]=x;
    b->data_n[1]=y;
    b->data_n[2]=z;
    double cos_angle=((a->values[0])*(b->data_n[0])+(a->values[1])*(b->data_n[1])+(a->values[2])*(b->data_n[2]))/
                        (sqrt(((a->values[0])*(a->values[0]))+((a->values[1])*(a->values[1]))+((a->values[2])*(a->values[2])))+
                         sqrt(((b->data_n[0])*(b->data_n[0]))+((b->data_n[1])*(b->data_n[1]))+((b->data_n[2])*(b->data_n[2])))  );
    double angle=(acos(cos_angle))*180/PI;
    if(angle>90){
        return 180-angle;
    }
    else return angle;
}
//////////////////////////////////////////////////////////////////////////////////////////
//计算点到平面距离
double findDistanceBetweenPlan_point(pcl::ModelCoefficients::Ptr a,PointT b){
    double distance = ((a->values[0])*b.x+(a->values[1])*b.y+(a->values[2])*b.z+(a->values[3]))/
                        (sqrt((a->values[0])*(a->values[0])+(a->values[1])*(a->values[1])+(a->values[2])*(a->values[2])));
    return distance;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////   M   A   I   N   //////////////////////////////////////////////
//IMPORTANT PARAMETER:
//cloud_subface 车厢底面点云
//cloud_ground  地面点云
//coe_ground    地面模型
//subface_height 底面平均高
//Gamma  伽马偏角 弧度制
//trunk_head_height 车头高
//trunk_side_height 车侧高 
int main(int argc,char **argv){
    clock_t start_Total = clock();
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addCoordinateSystem();
	viewer.setBackgroundColor(0.0, 0.0, 0.0);

    
    
    //[1]load pcd data to cloud_origin
    cout << "loading pcd data..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud_origin)) {
		cout << "loading pcd data failed" << endl;
		return -1;
	}
	else {
		cout << "loading pcd data success" << endl;
		cout << "cloud size origin: " << cloud_origin->size() << endl;
	}

    // [2] remove points on the ground
    //  out:cloud_remove_ground,cloud_ground
	pcl::PointCloud<PointT>::Ptr cloud_remove_nan(new pcl::PointCloud<PointT>());
     pcl::PointCloud<PointT>::Ptr cloud_ground(new pcl::PointCloud<PointT>);
	std::vector<int> indices_Pulse;
	pcl::removeNaNFromPointCloud(*cloud_origin, *cloud_remove_nan, indices_Pulse);

	pcl::PointCloud<PointT>::Ptr cloud_remove_ground(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < cloud_remove_nan->size(); ++i) {
		if ( cloud_remove_nan->points[i].z > 0.8) {
			cloud_remove_ground->push_back(cloud_remove_nan->points[i]);
		}
        else
        {
            cloud_ground->push_back(cloud_remove_nan->points[i]);
        }
        
	}
  cout << "cloud size remove ground: " << cloud_remove_ground->size() << endl;


    //[3] caculate ground 
   pcl::ModelCoefficients::Ptr coe_ground (new pcl::ModelCoefficients);
   coe_ground->values.resize(4);
   coe_ground->values[0]=0;
   coe_ground->values[1]=0;
    coe_ground->values[2]=1;
    coe_ground->values[3]=0;
   //pcl::PointIndices::Ptr inliers_ground(new pcl::PointIndices);
   //pcl::SACSegmentation<pcl::PointT> seg_ground;
   //seg_ground.setOptimizeCoefficients(true);
   //seg_ground.setModelType(pcl::SACMODEL_PLANE);
   //seg_ground.setMethodType(pcl::SAC_RANSAC);
   //seg_ground.setDistanceThreshold(0.01);
   //seg_ground.setInputCloud(cloud_ground);
   //seg_ground.segement(*inliers_ground,*coe_ground);
   //if(inliers_ground->indices.size()==0){
   //    cout<<"can not find ground"<<endl;
   //    return -1;
   //}
    cout<<"ground caculated!"<<endl;
  // [4] point cloud filter 
  // out:cloud_filtered
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter_statisticaloutlier;
  filter_statisticaloutlier.setInputCloud(cloud_remove_ground);
  filter_statisticaloutlier.setMeanK(50);
  filter_statisticaloutlier.setStddevMulThresh(1.0);
  filter_statisticaloutlier.setNegative(false);
  filter_statisticaloutlier.filter(*cloud_filtered);
  cout << "cloud size after filter: " << cloud_filtered->size() << endl;
   


	// [5] voxel grid downsample
    // out:cloud_downsampled
	pcl::PointCloud<PointT>::Ptr cloud_downsampled(cloud_filtered);
	// pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>);
	// pcl::VoxelGrid<PointT> downsampled;
	// downsampled.setInputCloud(cloud_filtered);
	// downsampled.setLeafSize(0.01, 0.01, 0.01);
	// downsampled.filter(*cloud_downsampled);
	// cout << "cloud size after downsampled: " << cloud_downsampled->size() << endl << endl;


	// [6] calculate normal
    // out:cloud_normals
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
	
    // [7] project to ground
    pcl::ProjectInliers<pcl::PointXYZ> project_ground;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_ground(new pcl::PointCloud<pcl::PointXYZ>);
  project_ground.setModelType(pcl::SACMODEL_PLANE);
  project_ground.setInputCloud(cloud_downsampled);
  project_ground.setModelCoefficients(coe_ground);
  project_ground.filter(*cloud_projected_ground);
    cout<<"cloud has been projected to the ground"<<endl;

   // viewer.addPointCloud<pcl::PointXYZ>(cloud_projected_ground, "sample cloud");

  //[8] 计算整车几何重心（z=0）
  // 先对投影点云进行降采样，保证均匀性，再通过MomentOfInertiaEstimation计算重心
  pcl::PointCloud<PointT>::Ptr cloud_projected_ground_downsampled(new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> downsample_for8;
  downsample_for8.setInputCloud(cloud_projected_ground);
  downsample_for8.setLeafSize(0.05,0.05,0.05);
  downsample_for8.filter(*cloud_projected_ground_downsampled);

  Eigen::Vector3f mass_center_projected;

  pcl::MomentOfInertiaEstimation<PointT> center_extractor;
  center_extractor.setInputCloud(cloud_projected_ground_downsampled);
  center_extractor.compute();

  center_extractor.getMassCenter(mass_center_projected);

cout<<"caculate center_2D complete"<<"center of the projected cloud is: ";
for(int i=0 ; i<3 ; ++i){
    cout<<mass_center_projected[i]<<",";
}
cout<<endl;



// [9]提取水平与竖直点云
//out:cloud_horizontal，cloud_vertical
cout<<"begin to segement horizontal & vertical cloud"<<endl;
  pcl::PointCloud<PointT>::Ptr cloud_horizontal(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_vertical(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < cloud_downsampled->size(); ++i) {
		if (cloud_normals->points[i].normal_z>0.9||cloud_normals->points[i].normal_z<-0.9) {
			cloud_horizontal->push_back(cloud_downsampled->points[i]);
		}
        else if(cloud_normals->points[i].normal_z<0.1&&cloud_normals->points[i].normal_z>-0.1){
            cloud_vertical->push_back(cloud_downsampled->points[i]);
        }
	}

 

    cout<<"complete!"<<endl;
    // 创建一个容器存储所有上述点构成的平面
    //out:cluster_horizontal                平行相对平面的点云集合
    //    vec_plane_coefficients_trunk_horizontal   上述点云集合相对应的模型参数
  vector<pcl::PointCloud<PointT>::Ptr> cluster_horizontal;
  vector<pcl::ModelCoefficients::Ptr> vec_plane_coefficients_trunk_horizontal;
  pcl::PointCloud<PointT>::Ptr cloud_horizontal_extract(new pcl::PointCloud<PointT>);
  pcl::PointIndices::Ptr inliers_horizontal(new pcl::PointIndices);
  pcl::SACSegmentation<PointT> plane_trunk_horizontal;
  pcl::ExtractIndices<PointT> extract_trunk_horizontal;
  extract_trunk_horizontal.setNegative(false);

  plane_trunk_horizontal.setOptimizeCoefficients(true);
  plane_trunk_horizontal.setModelType(pcl::SACMODEL_PLANE);
  plane_trunk_horizontal.setMethodType(pcl::SAC_RANSAC);
  plane_trunk_horizontal.setDistanceThreshold(0.01);
  
  for ( int i = 0; i < 5; ++i ) {
    pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr plane_coefficients_temp(new pcl::ModelCoefficients);
    plane_trunk_horizontal.setInputCloud(cloud_horizontal);
    extract_trunk_horizontal.setInputCloud(cloud_horizontal);
    plane_trunk_horizontal.segment(*inliers_horizontal,*plane_coefficients_temp);
    vec_plane_coefficients_trunk_horizontal.push_back(plane_coefficients_temp);
    extract_trunk_horizontal.setIndices(inliers_horizontal);
    extract_trunk_horizontal.setNegative(false);
    extract_trunk_horizontal.filter(*cloud_temp);
    cluster_horizontal.push_back(cloud_temp);
    extract_trunk_horizontal.setNegative(true);
    extract_trunk_horizontal.filter(*cloud_horizontal); 
    }
cout<<"extract completed"<<endl;


//[10]确定车厢内底面
//out:cluster_horizontal[subface_index]
    pcl::PointCloud<PointT>::Ptr cloud_subface(new pcl::PointCloud<PointT>);
    double subface_height=100000;//as huge as possible
    int subface_index;
    for(int i=0;i<5;i++){
        pcl::PointCloud<PointT>::Ptr cloud_downsampled_temp(new pcl::PointCloud<PointT>);
	    pcl::VoxelGrid<PointT> downsampledForStep9;
	    downsampledForStep9.setInputCloud(cluster_horizontal[i]);
	    downsampledForStep9.setLeafSize(1,1,1);
	    downsampledForStep9.filter(*cloud_downsampled_temp);
        double sum=0;
        for(int j=0;j<cloud_downsampled_temp->size();++j){
            sum+=cloud_downsampled_temp->points[j].z;
        }
        double avg_height=sum/(cloud_downsampled_temp->size());
        if (avg_height<subface_height){
            subface_height=avg_height;
            subface_index=i;
        }
    }
////////////////////////////////////////////////////////////////////////
//[11]将subface投影到地面并提取OBB包络框 并将车体摆正，中心为底面中心
//out:cloud_stable  

  pcl::ProjectInliers<pcl::PointXYZ> subface_project_ground;
  pcl::PointCloud<pcl::PointXYZ>::Ptr subface_projected_ground(new pcl::PointCloud<pcl::PointXYZ>);
  subface_project_ground.setModelType(pcl::SACMODEL_PLANE);
  subface_project_ground.setInputCloud(cluster_horizontal[subface_index]);
  subface_project_ground.setModelCoefficients(coe_ground);
  subface_project_ground.filter(*subface_projected_ground);
    cout<<"subface has been projected to the ground"<<endl;

    PointT min_point_OBB_subface2D;
    PointT max_point_OBB_subface2D;
    PointT position_OBB_subface2D;
    float major_value_subface2D, middle_value_subface2D, minor_value_subface2D;
    Eigen::Vector3f major_vector_subface2D, middle_vector_subface2D, minor_vector_subface2D;
    Eigen::Matrix3f rotational_matrix_OBB_subface2D;
    Eigen::Vector3f mass_center_subface2D;
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> subface2D_OBB_extractor;
    subface2D_OBB_extractor.setInputCloud (subface_projected_ground);
    subface2D_OBB_extractor.compute ();
   
   subface2D_OBB_extractor.getMassCenter (mass_center_subface2D);
   subface2D_OBB_extractor.getEigenValues (major_value_subface2D, middle_value_subface2D, minor_value_subface2D);
   subface2D_OBB_extractor.getEigenVectors (major_vector_subface2D, middle_vector_subface2D, minor_vector_subface2D);
   subface2D_OBB_extractor.getOBB (min_point_OBB_subface2D, max_point_OBB_subface2D, position_OBB_subface2D, rotational_matrix_OBB_subface2D);

    Eigen::Vector3f position (position_OBB_subface2D.x, position_OBB_subface2D.y, position_OBB_subface2D.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB_subface2D);
    viewer.addCube (position, quat, max_point_OBB_subface2D.x - min_point_OBB_subface2D.x, max_point_OBB_subface2D.y - min_point_OBB_subface2D.y, max_point_OBB_subface2D.z - min_point_OBB_subface2D.z, "OBB");
    cout<<"OBB of subface2D added"<<endl;

    cout<<"max point(x,y,z):"<<max_point_OBB_subface2D.x<<","<<max_point_OBB_subface2D.y<<","<<max_point_OBB_subface2D.z<<","<<endl;
    cout<<"min point(x,y,z):"<<min_point_OBB_subface2D.x<<","<<min_point_OBB_subface2D.y<<","<<min_point_OBB_subface2D.z<<","<<endl;
   cout << "Quaternion1" << endl << quat.coeffs() << endl;
   float Gamma=quat.z();
   Eigen::Vector3f position_reverse(-position[0],-position[1],0);
   Eigen::Quaternionf quat_reverse(1,0,0,-Gamma);

   pcl::PointCloud<PointT>::Ptr cloud_stable(new pcl::PointCloud<PointT>);
   transformPointCloud (*cloud_downsampled, *cloud_stable, position_reverse, quat_reverse);

    //viewer.addPointCloud(cloud_stable,"aaa");
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//[11]cloud_stable normals
cout << "calculate normal..." << endl;

  pcl::PointCloud<NormalT>::Ptr cloud_stable_normals(new pcl::PointCloud<NormalT>);
  pcl::search::KdTree<PointT>::Ptr stable_tree(new pcl::search::KdTree<PointT>());
	ne.setInputCloud(cloud_stable);
	ne.setSearchMethod(stable_tree);
	ne.setRadiusSearch(0.15);
	ne.compute(*cloud_stable_normals);
	cout << "calculate done: " <<endl;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//[12] 提取车头
	pcl::PointCloud<PointT>::Ptr cloud_head(new pcl::PointCloud<PointT>);
	for (size_t i = 0; i < cloud_stable_normals->size(); i++) {
		if (cloud_stable->points[i].z>2 
            && (cloud_normals->points[i].normal_z >= 0.9 || cloud_normals->points[i].normal_z <= -0.9)
            && cloud_stable->points[i].x>max_point_OBB_subface2D.x) {
                cloud_head->push_back(cloud_stable->points[i]);
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
	

float trunk_head_height = 0.0;
for (size_t i = 0; i < cloud_trunk_head->size(); ++i) {
	trunk_head_height += cloud_trunk_head->points[i].z;
}
trunk_head_height /= cloud_trunk_head->size();
cout << "The height of trunk head height: " << trunk_head_height << endl;

//viewer.addPointCloud(cloud_trunk_head,"bbb");


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//提取车侧
	pcl::PointCloud<PointT>::Ptr potential_trunk_plane_right(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_plane_right(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_trunk_plane_left(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr trunk_plane_left(new pcl::PointCloud<PointT>());
	for (size_t i = 0; i < cloud_stable_normals->size(); ++i) {
		if (cloud_normals->points[i].normal_y >= 0.9) {
            potential_trunk_plane_right->push_back(cloud_stable->points[i]);
        }
        if (cloud_normals->points[i].normal_y <= -0.9) {
            potential_trunk_plane_left->push_back(cloud_stable->points[i]);
            
        }
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

//viewer.addPointCloud(trunk_plane_right,"ccc");
//viewer.addPointCloud(trunk_plane_left,"ccd");

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

//通过trunk_plane_side_restruct获得cloud_step_filtered 和 bool step
//[in] trunk_side_height subface_height threadhold trunk_plane_side_restruct for subface SAC:0.01
        //求trunk ——side_height max_x_side min_x_side
    float max_x_side=0,min_x_side=0;
    
    for(size_t i=0;i<trunk_plane_side_restruct->size();i++){
        if (trunk_plane_side_restruct->points[i].x>max_x_side)
            max_x_side=trunk_plane_side_restruct->points[i].x;
        if (trunk_plane_side_restruct->points[i].x<min_x_side)
            min_x_side=trunk_plane_side_restruct->points[i].x;
    }
    cout<<"max_x_side="<<max_x_side<<endl;
    cout<<"min_x_side="<<min_x_side<<endl;

    pcl::PointCloud<PointT>::Ptr potential_step(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_in_box(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_in_box_except_step(new pcl::PointCloud<PointT>);
    vector<size_t> potential_step_index_of_cloud_stable;

    for(size_t i=0;i<cloud_stable->size();i++){
        bool a1=false;
        bool a2=false;
        if((cloud_stable->points[i].z>(subface_height+0.03)) && cloud_stable->points[i].z<(trunk_side_height-0.2) && cloud_stable->points[i].x<(max_x_side-0.02) && cloud_stable->points[i].x>(min_x_side+0.02) && 
                (cloud_stable_normals->points[i].normal_z> 0.5 || cloud_stable_normals->points[i].normal_z< -0.5)&&(cloud_stable_normals->points[i].normal_z< 0.9 || cloud_stable_normals->points[i].normal_z> -0.9)){
               cloud_in_box->points.push_back(cloud_stable->points[i]);
               
            }
        if((cloud_stable->points[i].z>(subface_height+0.03)) && cloud_stable->points[i].x<(max_x_side-0.01) && cloud_stable->points[i].x>(min_x_side+0.01) && 
                (cloud_stable_normals->points[i].normal_z> 0.9 || cloud_stable_normals->points[i].normal_z< -0.9)){
                    potential_step->points.push_back(cloud_stable->points[i]);
                   
                }
       
    }
    cout<<"potential_step find:"<<potential_step->size()<<" points"<<endl;
    pcl::PointCloud<PointT>::Ptr cloud_step_plane(new pcl::PointCloud<PointT>);


    pcl::ModelCoefficients::Ptr plane_step(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_step(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> plane_step_SAC;
	plane_step_SAC.setOptimizeCoefficients(true);
	plane_step_SAC.setModelType(pcl::SACMODEL_PLANE);
	plane_step_SAC.setMethodType(pcl::SAC_RANSAC);
	plane_step_SAC.setDistanceThreshold(0.01);
    plane_step_SAC.setInputCloud(potential_step);
	plane_step_SAC.segment(*inliers_step, *plane_step);
    
    if(inliers_step->indices.size()!=0){
        pcl::ExtractIndices<PointT> extract_step;
        extract_step.setInputCloud(potential_step);
        extract_step.setIndices(inliers_step);
        extract_step.setNegative(false);
        extract_step.filter(*cloud_step_plane);
     
        // 欧几里得聚簇分离可能的step和高度相同的杂物
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_step (new pcl::search::KdTree<pcl::PointXYZ>);
        tree_step->setInputCloud (cloud_step_plane);
        std::vector<pcl::PointIndices> cluster_indices_step_plane;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.1); // 1dm,根据相机步进长度设置
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (250000);
        ec.setSearchMethod (tree_step);
        ec.setInputCloud (cloud_step_plane);
        ec.extract (cluster_indices_step_plane);
        //提取点最多的点云，是可能的step云
        pcl::PointCloud<PointT>::Ptr cloud_step_filtered(new pcl::PointCloud<PointT>);
        int step_max_point_nums=0;
        int counter_it=0;
        int select_step_index;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_step_plane.begin (); it != cluster_indices_step_plane.end (); ++it)
        {
            if(it->indices.size()>step_max_point_nums){
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                    cloud_step_filtered->points.push_back (cloud_step_plane->points[*pit]); 
                    }
                    cloud_step_filtered->width = cloud_step_filtered->points.size ();
                    cloud_step_filtered->height = 1;
                    cloud_step_filtered->is_dense = true;
            }
            
        }

        // 将cloud_step_filtered投射到ground，落在subface OBB中的点数小于cloud_step 总点数的5%，即证明是step
        // subfaceOBB 靠前两点：（max_point_OBB_subface2D.x,min_point_OBB_subface2D.y）;(max_point_OBB_subface2D.x,max_point_OBB_subface2D.y)
        pcl::PointCloud<pcl::PointXYZ>::Ptr step_projected_ground(new pcl::PointCloud<pcl::PointXYZ>);
        project_ground.setInputCloud(cloud_step_filtered);
        project_ground.setModelCoefficients(coe_ground);
        project_ground.filter(*step_projected_ground);
        cout<<"step has been projected to the ground"<<endl;
        int counter_error_point=0;
        for(size_t i=0;i<step_projected_ground->size();++i){
            if(step_projected_ground->points[i].x<max_point_OBB_subface2D.x){
                counter_error_point++;
            }
        }
        if((counter_error_point/step_projected_ground->size())<0.05){
            step=true;//判断有step
        }
        vector<int> inline_in_box_others;
        pcl::getApproximateIndices(cloud_in_box,cloud_step_filtered,inline_in_box_others);
        pcl::ExtractIndices<PointT> extract_others_in_box;
        extract_others_in_box.setInputCloud(cloud_in_box);
        extract_others_in_box.setIndices(inline_in_box_others);
        extract_others_in_box.setNegative(false);
        extract_others_in_box.filter(*cloud_in_box_except_step);

        viewer.addPointCloud(cloud_step_filtered,"aaaaa");

        

    }
    else{
        pcl::copyPointCloud(cloud_in_box,cloud_in_box_except_step);
    }if(step){
        cout<<"step found!"<<endl;
        float step_height=0;
        for(size_t i=0;size_t<cloud_step_filtered->size();i++){
            step_height+=cloud_step_filtered->points[i].z;
        }
        step_height/=cloud_step_filtered->size();
    }
    else{
        cout<<"step not found"<<endl;
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //若有阶梯  将cloud_step_filtered投影到ground，构成AABB包络，并将AABB包络通过反向变化得到原位姿的阶梯投影
    if(step){
        pcl::ProjectInliers<pcl::PointXYZ> project_step_ground;
        pcl::PointCloud<pcl::PointXYZ>::Ptr step_projected_ground_stable(new pcl::PointCloud<pcl::PointXYZ>);
        project_step_ground.setModelType(pcl::SACMODEL_PLANE);
        project_step_ground.setInputCloud(cloud_step_filtered);
        project_step_ground.setModelCoefficients(coe_ground);
        project_step_ground.filter(*step_projected_ground_stable);
        cout<<"step has been projected to the ground"<<endl;

        pcl::PointXYZ min_point_AABB_step;
        pcl::PointXYZ max_point_AABB_step;

        pcl::MomentOfInertiaEstimation <pcl::PointXYZ> step_AABB_extractor;
        step_AABB_extractor.setInputCloud (step_projected_ground_stable);
        step_AABB_extractor.compute ();

        step_AABB_extractor.getAABB (min_point_AABB_step, max_point_AABB_step);//此时计算的AABB是重新矫正后的坐标，要获得真实坐标需要进行旋转平移操作

        viewer.addCube (position, quat, max_point_AABB_step.x - min_point_AABB_step.x, max_point_AABB_step.y - min_point_AABB_step.y, max_point_AABB_step.z - min_point_AABB_step.z, "AABB");

    }























while (!viewer.wasStopped ())
{
  viewer.spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}




}