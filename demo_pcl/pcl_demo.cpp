#include <iostream>
#include <ctime>
#include <cmath>
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

int main(int argc, char *argv[])
{
	// get calculate time
	clock_t start_Total, finish_Total;
	start_Total = clock();

	// // [1] load pcd data
	// cout << "loading pcd data..." << endl;
	// pcl::PointCloud<PointT>::Ptr cloud_origin(new pcl::PointCloud<PointT>);
	// if (pcl::io::loadPCDFile<PointT>("../test_pcd80.pcd", *cloud_origin))
	// {
	// 	cout << "loading pcd data failed" << endl << endl;
	// 	return -1;
	// }
	// else
	// {
	// 	cout << "loading pcd data success" << endl << endl;
	// 	cout << "cloud size origin: " << cloud_origin->size() << endl;
	// }

	// // [2] passthrough filter
	// // x passthrough filter
	// pcl::PointCloud<PointT>::Ptr cloud_after_x_filter(new pcl::PointCloud<PointT>());
	// pcl::ConditionAnd<PointT>::Ptr x_passthrough_filter_cond(new pcl::ConditionAnd<PointT>());
	// x_passthrough_filter_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
	// 	new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GE, -6.5)));
	// x_passthrough_filter_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
	// 	new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LE, 4.5)));
	// pcl::ConditionalRemoval<PointT> cloud_filter_x(false);
	// cloud_filter_x.setCondition(x_passthrough_filter_cond);
	// cloud_filter_x.setInputCloud(cloud_origin);
	// cloud_filter_x.setKeepOrganized(true);
	// cloud_filter_x.filter(*cloud_after_x_filter);
	// // y passthrough filter
	// pcl::PointCloud<PointT>::Ptr cloud_after_y_filter(new pcl::PointCloud<PointT>());
	// pcl::ConditionAnd<PointT>::Ptr y_passthrough_filter_cond(new pcl::ConditionAnd<PointT>());
	// y_passthrough_filter_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
	// 	new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GE, -1.5)));
	// y_passthrough_filter_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
	// 	new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LE, 1.5)));
	// pcl::ConditionalRemoval<PointT> cloud_filter_y(false);
	// cloud_filter_y.setCondition(y_passthrough_filter_cond);
	// cloud_filter_y.setInputCloud(cloud_after_x_filter);
	// cloud_filter_y.setKeepOrganized(true);
	// cloud_filter_y.filter(*cloud_after_y_filter);
	// // nan passthrough filter
	// pcl::PointCloud<PointT>::Ptr cloud_remove_nan(new pcl::PointCloud<PointT>());
	// std::vector<int> indices_Pulse;
	// pcl::removeNaNFromPointCloud(*cloud_after_y_filter, *cloud_remove_nan, indices_Pulse);
	// cout << "cloud size after passthrough filter: " << cloud_remove_nan->size() << endl << endl;

	// // [3] voxel grid downsample
	// pcl::PointCloud<PointT>::Ptr cloud_downsampled(cloud_remove_nan);
	// // pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>);
	// // pcl::VoxelGrid<PointT> downsampled;
	// // downsampled.setInputCloud(cloud_remove_nan);
	// // downsampled.setLeafSize(0.01, 0.01, 0.01);
	// // downsampled.filter(*cloud_downsampled);
	// // cout << "cloud size after downsampled: " << cloud_downsampled->size() << endl << endl;

	// // [4] calculate normal
	// cout << "calculate normal..." << endl;
	// clock_t normal_start = clock();
	// pcl::NormalEstimation<PointT, NormalT> ne;
	// ne.setInputCloud(cloud_downsampled);
	// pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	// ne.setSearchMethod(tree);
	// pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>);
	// pcl::PointCloud<PointT>::Ptr normals2cloud(new pcl::PointCloud<PointT>);
	// ne.setRadiusSearch(0.25);
	// ne.compute(*cloud_normals);
	// cout << "time to calculate normal： " << (double)(clock() - normal_start)/CLOCKS_PER_SEC << "s" << endl << endl;
	// pcl::io::savePCDFile("./cloud_downsampled.pcd", *cloud_downsampled, true);
	// pcl::io::savePCDFile("./cloud_normals.pcd", *cloud_normals, true);


	pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile<PointT>("./cloud_downsampled.pcd", *cloud_downsampled);
	pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>);
	pcl::io::loadPCDFile<NormalT>("./cloud_normals.pcd", *cloud_normals);
	

	// [5] get horizontal point
	cout << "get trunk subface..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_horizontal(new pcl::PointCloud<PointT>);
	for (int i = 0; i < cloud_normals->size(); i++)
	{
		if (cloud_downsampled->points[i].z <= 2 && 
				(cloud_normals->points[i].normal_z >= 0.9 || cloud_normals->points[i].normal_z <= -0.9) )
			cloud_horizontal->push_back(cloud_downsampled->points[i]);
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
	plane_trunk_subface.setDistanceThreshold(0.18);
	plane_trunk_subface.setInputCloud(cloud_horizontal);
	plane_trunk_subface.segment(*inliers, *plane_coefficients_trunk_subface);

	pcl::ExtractIndices<PointT> extract_trunk_subface;
	extract_trunk_subface.setInputCloud(cloud_horizontal);
	extract_trunk_subface.setIndices(inliers);
	extract_trunk_subface.setNegative(false);
	extract_trunk_subface.filter(*cloud_trunk_subface);
	cout << "coefficients of trunk_subface plane " << endl;
	char coeff[4] = { 'a', 'b', 'c', 'd' };
	for (size_t i = 0; i < plane_coefficients_trunk_subface->values.size(); ++i)
	{
		cout << "	" << coeff[i] << ":";
		cout << "	" << plane_coefficients_trunk_subface->values[i] << endl;
	}

	// [7] segment trunk side
	pcl::PointCloud<PointT>::Ptr potential_cabin_plane_100(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cabin_plane_100(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_cabin_plane_n100(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cabin_plane_n100(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_cabin_plane_010(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cabin_plane_010(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_cabin_plane_n010(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cabin_plane_n010(new pcl::PointCloud<PointT>());
	int deckNormalSize = cloud_normals->size();
	for (int i = 0; i < deckNormalSize; i++)
	{
		if (cloud_normals->points[i].normal_x >= 0.9) potential_cabin_plane_100->push_back(cloud_downsampled->points[i]);
		if (cloud_normals->points[i].normal_x <= -0.9) potential_cabin_plane_n100->push_back(cloud_downsampled->points[i]);
		if (cloud_normals->points[i].normal_y >= 0.9) potential_cabin_plane_010->push_back(cloud_downsampled->points[i]);
		if (cloud_normals->points[i].normal_y <= -0.9) potential_cabin_plane_n010->push_back(cloud_downsampled->points[i]);
	}

	//船舱保存船舱平面轮廓区域的变量
	float cabinPlane_Xmin, cabinPlane_Xmax, cabinPlane_Ymin, cabinPlane_Ymax;
	cabinPlane_Xmax = cabinPlane_Xmin = cabinPlane_Ymax = cabinPlane_Ymin = 0.0;

	pcl::PointIndices::Ptr cabinPlane_inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr cabinPlane_coefficients(new pcl::ModelCoefficients);

	pcl::SACSegmentation<PointT> cabin_plane_seg;
	cabin_plane_seg.setOptimizeCoefficients(true);
	cabin_plane_seg.setModelType(pcl::SACMODEL_PLANE);
	cabin_plane_seg.setMethodType(pcl::SAC_RANSAC);
	cabin_plane_seg.setDistanceThreshold(0.18);

	// 抽取分割出的船舱平面点的索引
	pcl::ExtractIndices<PointT> cabinPlane_extract;
	cabinPlane_extract.setNegative(false);

	//分割法向为(1, 0, 0)的船舱平面
	cabin_plane_seg.setInputCloud(potential_cabin_plane_100);
	cabin_plane_seg.segment(*cabinPlane_inliers, *cabinPlane_coefficients);
	cabinPlane_extract.setInputCloud(potential_cabin_plane_100);
	cabinPlane_extract.setIndices(cabinPlane_inliers);
	cabinPlane_extract.filter(*cabin_plane_100);
	for (int i = 0; i < cabin_plane_100->size(); i++) cabinPlane_Xmin += cabin_plane_100->points[i].x;
	cabinPlane_Xmin /= cabin_plane_100->size();
	cout << "	完成法向为(1, 0, 0)的船舱平面分割！" << endl;
	cout << "	由法向为(1, 0, 0)的船舱平面可得船舱轮廓范围x轴方向最小值cabinPlane_Xmin = " << cabinPlane_Xmin << endl << endl;
	//分割法向为(-1, 0, 0)的船舱平面
	cabin_plane_seg.setInputCloud(potential_cabin_plane_n100);
	cabin_plane_seg.segment(*cabinPlane_inliers, *cabinPlane_coefficients);
	cabinPlane_extract.setInputCloud(potential_cabin_plane_n100);
	cabinPlane_extract.setIndices(cabinPlane_inliers);
	cabinPlane_extract.filter(*cabin_plane_n100);
	for (int i = 0; i < cabin_plane_n100->size(); i++) cabinPlane_Xmax += cabin_plane_n100->points[i].x;
	cabinPlane_Xmax /= cabin_plane_n100->size();
	cout << "	完成法向为(-1, 0, 0)的船舱平面分割！" << endl;
	cout << "	由法向为(-1, 0, 0)的船舱平面可得船舱轮廓范围x轴方向最大值cabinPlane_Xmax = " << cabinPlane_Xmax << endl << endl;
	//分割法向为(0, 1, 0)的船舱平面
	cabin_plane_seg.setInputCloud(potential_cabin_plane_010);
	cabin_plane_seg.segment(*cabinPlane_inliers, *cabinPlane_coefficients);
	cabinPlane_extract.setInputCloud(potential_cabin_plane_010);
	cabinPlane_extract.setIndices(cabinPlane_inliers);
	cabinPlane_extract.filter(*cabin_plane_010);
	for (int i = 0; i < cabin_plane_010->size(); i++) cabinPlane_Ymin += cabin_plane_010->points[i].y;
	cabinPlane_Ymin /= cabin_plane_010->size();
	cout << "	完成法向为(0, 1, 0)的船舱平面分割！" << endl;
	cout << "	由法向为(0, 1, 0)的船舱平面可得船舱轮廓范围y轴方向最小值cabinPlane_Ymin = " << cabinPlane_Ymin << endl << endl;
	//分割法向为(0, -1, 0)的船舱平面
	cabin_plane_seg.setInputCloud(potential_cabin_plane_n010);
	cabin_plane_seg.segment(*cabinPlane_inliers, *cabinPlane_coefficients);
	cabinPlane_extract.setInputCloud(potential_cabin_plane_n010);
	cabinPlane_extract.setIndices(cabinPlane_inliers);
	cabinPlane_extract.filter(*cabin_plane_n010);
	for (int i = 0; i < cabin_plane_n010->size(); i++) cabinPlane_Ymax += cabin_plane_n010->points[i].y;
	cabinPlane_Ymax /= cabin_plane_n010->size();
	cout << "	完成法向为(0, -1, 0)的船舱平面分割！" << endl;
	cout << "	由法向为(0, -1, 0)的船舱平面可得船舱轮廓范围y轴方向最大值cabinPlane_Ymin = " << cabinPlane_Ymax << endl << endl;




	pcl::visualization::PCLVisualizer viewers("Cloud Viewer");
	viewers.addCoordinateSystem();
	viewers.setBackgroundColor(0.0, 0.0, 0.0);

	// viewers.addPointCloudNormals<pcl::PointXYZ, NormalT>(cloud_downsampled, cloud_normals);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> oricloud_color_handlers(cloud_downsampled, 255, 255, 255);
	viewers.addPointCloud<PointT>(cloud_downsampled, oricloud_color_handlers, "original cloud");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud1_color_handlers(cloud_trunk_subface, 255, 0, 0);
	viewers.addPointCloud<PointT>(cloud_trunk_subface, cloud1_color_handlers, "cloud1");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud2_color_handlers(cabin_plane_100, 0, 255, 0);
	viewers.addPointCloud<PointT>(cabin_plane_100, cloud2_color_handlers, "cloud2");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud3_color_handlers(cabin_plane_n100, 0, 0, 255);
	viewers.addPointCloud<PointT>(cabin_plane_n100, cloud3_color_handlers, "cloud3");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud4_color_handlers(cabin_plane_010, 255, 255, 0);
	viewers.addPointCloud<PointT>(cabin_plane_010, cloud4_color_handlers, "cloud4");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud5_color_handlers(cabin_plane_n010, 0, 255, 255);
	viewers.addPointCloud<PointT>(cabin_plane_n010, cloud5_color_handlers, "cloud5");

	viewers.spin();
	return 0;






	// //【12】利用得到的船舱轮廓范围从未经过反射率滤波的原始点云数据中分割出物料区域
	// float delta = 0.5;
	// pcl::PointCloud<PointT>::Ptr cloud_dump_ori(new pcl::PointCloud<PointT>);
	// pcl::PointCloud<PointT>::Ptr cloud_dump(new pcl::PointCloud<PointT>);
	// pcl::ConditionAnd<PointT>::Ptr cloud_dump_range_cond(new pcl::ConditionAnd<PointT>());
	// cloud_dump_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LE, cabinPlane_Xmax - delta)));
	// cloud_dump_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GE, cabinPlane_Xmin + delta)));
	// cloud_dump_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LE, cabinPlane_Ymax - delta)));
	// cloud_dump_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GE, cabinPlane_Xmin + delta)));
	// // 创建滤波器
	// pcl::ConditionalRemoval<PointT> cloud_dump_condrem(false);
	// cloud_dump_condrem.setCondition(cloud_dump_range_cond);
	// cloud_dump_condrem.setInputCloud(cloud_downsampled_after_revise);
	// cloud_dump_condrem.setKeepOrganized(true);
	// // 应用滤波器
	// cloud_dump_condrem.filter(*cloud_dump_ori);
	// std::vector<int> indices_dump;
	// pcl::removeNaNFromPointCloud(*cloud_dump_ori, *cloud_dump, indices_dump);
	// cout << "完成煤堆物料区域区域分割！" << endl;

	// //【13】对分割出来的物料区域进行邻域滤波处理，滤除由于空气中灰尘杂质导致测量得到的离散点
	// pcl::PointCloud<PointT>::Ptr cloud_dump_after_filtering(new pcl::PointCloud<PointT>);
	// //邻域滤波
	// pcl::RadiusOutlierRemoval<PointT> dump_outrem; //创建滤波器
	// dump_outrem.setInputCloud(cloud_dump);
	// dump_outrem.setRadiusSearch(0.2);
	// dump_outrem.setMinNeighborsInRadius(3);
	// dump_outrem.filter(*cloud_dump_after_filtering);

/*	
	//【14】对分割出来的物料区域进行网格化操作
	PointT minPoint, maxPoint;
	pcl::getMinMax3D(*cloud_dump_after_filtering, minPoint, maxPoint);
	//vector<vector<float>> high_grid;                                                         //网格高度记录
	vector<Grid> high_grid;
	cout << "物料区域网格化结果如下：" << endl;
	GrabModel grabModel(0, 0, -36, 1);
	allGrab = &grabModel;
	high_grid = Gridding(cloud_dump_after_filtering, minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, minPoint.z, maxPoint.z, 35, 45, true, 0.2, grabModel);              //获取网格高度
	for (int i = 0; i < high_grid.size(); i++) 
	{	
		cout << "( " << high_grid[i].pos_x << "," << high_grid[i].pos_y << "," << high_grid[i].high << ")   ";
		if ((i + 1) % 10 == 0)
			cout << endl;
	}
	cout << endl;
	
	//计时
	finish_Total = clock();
	cout << "算法总共消耗时间为： " << (finish_Total - start_Total) / 1000.0 << "s" << endl << endl;
*/
	//***
	//【15】分割结果可视化
	// pcl::visualization::PCLVisualizer viewer("Cloud Viewer For Segmentation Of Ship Scene");
	// viewer.addCoordinateSystem();
	// viewer.setBackgroundColor(1.0, 0.5, 0.5);

	// pcl::visualization::PointCloudColorHandlerCustom<PointT> oricloud_color_handler(cloud_land_after_revise, 255, 255, 255);
	// viewer.addPointCloud<PointT>(cloud_land_after_revise, oricloud_color_handler, "original cloud");

	// pcl::visualization::PointCloudColorHandlerCustom<PointT> holderPlane_color_handler(holder_plane, 255, 255, 0);
	// viewer.addPointCloud<PointT>(holder_plane, holderPlane_color_handler, "holder plane");

	// pcl::visualization::PointCloudColorHandlerCustom<PointT> deckPlane_color_handler(deck_plane_after_revise, 255, 0, 255);
	// viewer.addPointCloud<PointT>(deck_plane_after_revise, deckPlane_color_handler, "deck plane");

	// //pcl::visualization::PointCloudColorHandlerCustom<PointT> above_deckPlane_color_handler(cloud_above_deckPlane_after_revise, 0, 0, 255);
	// //viewer.addPointCloud<PointT>(cloud_above_deckPlane_after_revise, above_deckPlane_color_handler, "above deck plane");

	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cabin_plane_h1_color_handler(cabin_plane_100, 255, 0, 0);
	// viewer.addPointCloud<PointT>(cabin_plane_100, cabin_plane_h1_color_handler, "cabin plane h1");
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cabin_plane_h2_color_handler(cabin_plane_n100, 0, 255, 0);
	// viewer.addPointCloud<PointT>(cabin_plane_n100, cabin_plane_h2_color_handler, "cabin plane h2");
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cabin_plane_v1_color_handler(cabin_plane_010, 0, 0, 255);
	// viewer.addPointCloud<PointT>(cabin_plane_010, cabin_plane_v1_color_handler, "cabin plane v1");
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cabin_plane_v2_color_handler(cabin_plane_n010, 0, 255, 255); //100 100 100
	// viewer.addPointCloud<PointT>(cabin_plane_n010, cabin_plane_v2_color_handler, "cabin plane v2");

	// pcl::visualization::PointCloudColorHandlerCustom<PointT> dump_color_handler(cloud_dump_after_filtering, 50, 50, 50);
	// viewer.addPointCloud<PointT>(cloud_dump_after_filtering, dump_color_handler, "dump");

	// pcl::visualization::PointCloudColorHandlerCustom<PointT> land_color_handler(cloud_land_rest, 156, 160, 202);
	// viewer.addPointCloud<PointT>(cloud_land_rest, land_color_handler, "land");
	// /***
	// pcl::visualization::CloudViewer Simpleviewer("Simple Cloud Viewer");
	// Simpleviewer.showCloud(cloud_pulse_filtered);
	// ***/
	// while (!viewer.wasStopped())
	// {
	// 	//在此处可以添加其他处理
	// 	viewer.spinOnce();
	// }

	return 0;
}

