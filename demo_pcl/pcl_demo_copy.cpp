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

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char *argv[])
{
	//计时
	clock_t start_Total, finish_Total;
	start_Total = clock();

	//【1】加载原始点云数据
	cout << "加载原始点云数据中..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_downsampled_after_revise(new pcl::PointCloud<PointT>);

	if (pcl::io::loadPCDFile<PointT>("../originpcd.pcd", *cloud))//加载原始场景点云数据
	{
		cout << "原始船舱场景点云数据加载失败！" << endl << endl;
		return -1;
	}
	else
		cout << "原始船舱场景点云数据加载成功！" << endl << endl;

	// pcl::visualization::PCLVisualizer viewers("Cloud Viewer For Segmentation Of Ship Scene");
	// viewers.addCoordinateSystem();
	// viewers.setBackgroundColor(1.0, 0.5, 0.5);

	// pcl::visualization::PointCloudColorHandlerCustom<PointT> oricloud_color_handlers(cloud, 255, 255, 255);
	// viewers.addPointCloud<PointT>(cloud, oricloud_color_handlers, "original cloud");
	// viewers.spin();
	// return 0;


	float Imin = 15000.0, Imax = 0.0;
	for (int i = 0; i < cloud->size(); i++)
	{
		if (cloud->points[i].intensity < Imin) Imin = cloud->points[i].intensity;
		if (cloud->points[i].intensity > Imax) Imax = cloud->points[i].intensity;
	}
	cout << "点云中最大反射度值为：" << Imax << endl;
	cout << "点云中最小反射度值为：" << Imin << endl << endl;

	//【2】降采样
	pcl::VoxelGrid<PointT> downsampled;
	downsampled.setInputCloud(cloud);
	downsampled.setLeafSize(0.08, 0.08, 0.08);
	downsampled.filter(*cloud_downsampled);
	cout << "降采样前点云数量为：" << cloud->size() << endl;
	cout << "降采样后点云数量为：" << cloud_downsampled->size() << endl << endl;

	//【3】利用反射率信息精简点云数量
	pcl::PointCloud<PointT>::Ptr cloud_pulse_filtered_Ori(new pcl::PointCloud<PointT>()); //创建保存标定区域数据的点云
	pcl::PointCloud<PointT>::Ptr cloud_pulse_filtered(new pcl::PointCloud<PointT>()); //创建保存标定区域数据的点云
	pcl::PointCloud<PointT>::Ptr cloud_pulse_filtered_after_revise(new pcl::PointCloud<PointT>()); //创建保存标定区域数据的点云
	pcl::ConditionAnd<PointT>::Ptr cloud_pulse_filter_range_cond(new pcl::ConditionAnd<PointT>());
	cloud_pulse_filter_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("intensity", pcl::ComparisonOps::GE, 0.45)));
	// 创建滤波器
	pcl::ConditionalRemoval<PointT> cloud_filter_condrem(false);
	cloud_filter_condrem.setCondition(cloud_pulse_filter_range_cond);
	cloud_filter_condrem.setInputCloud(cloud_downsampled);
	cloud_filter_condrem.setKeepOrganized(true);
	// 应用滤波器
	cloud_filter_condrem.filter(*cloud_pulse_filtered_Ori);
	std::vector<int> indices_Pulse;
	pcl::removeNaNFromPointCloud(*cloud_pulse_filtered_Ori, *cloud_pulse_filtered, indices_Pulse);
	cout << "利用反射率信息精简场景点云后点云数量为：" << cloud_pulse_filtered->size() << endl << endl;

	//【4】估算点云法线
	cout << "计算原始点云数据法线中..." << endl;
	clock_t start, finish;
	start = clock();
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud(cloud_pulse_filtered);
	//创建一个空的kdtree对象，并把它传递给法线估计对象
	//基于给出的输入数据集，kdtree将被建立
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	ne.setSearchMethod(tree);
	//输出数据集
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr normals2cloud(new pcl::PointCloud<PointT>);
	//使用半径在查询点周围25厘米范围内的所有邻元素
	ne.setRadiusSearch(0.25);
	//计算点云法线
	ne.compute(*cloud_normals);
	finish = clock();
	cout << "计算法向量消耗时间为： " << finish - start << "ms" << endl << endl;

	//【5】根据法线信息分割出可能为甲板平面的点云数据
	cout << "分割甲板平面中..." << endl;
	pcl::PointCloud<PointT>::Ptr potential_deck_plane(new pcl::PointCloud<PointT>);
	for (int i = 0; i < cloud_normals->size(); i++)
	{
		if (cloud_normals->points[i].normal_z >= 0.99) potential_deck_plane->push_back(cloud_pulse_filtered->points[i]);
	}

	//【6】利用RANSAC算法从可能为甲板平面的点云数据中分割出准确甲板平面
	pcl::PointCloud<PointT>::Ptr deck_plane_before_revise(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr deck_plane_after_revise(new pcl::PointCloud<PointT>());

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr deckPlane_before_revise_coefficients(new pcl::ModelCoefficients);

	pcl::SACSegmentation<PointT> deck_plane_seg;
	deck_plane_seg.setOptimizeCoefficients(true);
	deck_plane_seg.setModelType(pcl::SACMODEL_PLANE);
	deck_plane_seg.setMethodType(pcl::SAC_RANSAC);
	deck_plane_seg.setDistanceThreshold(0.18);
	deck_plane_seg.setInputCloud(potential_deck_plane);
	deck_plane_seg.segment(*inliers, *deckPlane_before_revise_coefficients);

	// 抽取分割出的甲板平面点的索引
	pcl::ExtractIndices<PointT> deck_extract;
	deck_extract.setInputCloud(potential_deck_plane);
	deck_extract.setIndices(inliers);
	deck_extract.setNegative(false);
	deck_extract.filter(*deck_plane_before_revise);
	cout << "完成甲板平面分割，甲板平面模型（ax+by+cz+d=0）系数如下：" << endl;
	char coeff[4] = { 'a', 'b', 'c', 'd' };
	for (size_t i = 0; i < deckPlane_before_revise_coefficients->values.size(); ++i)
	{
		cout << "	" << coeff[i] << ":";
		cout << "	" << deckPlane_before_revise_coefficients->values[i] << endl;
	}

	//计算倾角校正前甲板平面包围盒
	PointT minPt2, maxPt2;  //存储船舱平面包围盒极值的两个点
	pcl::getMinMax3D(*deck_plane_before_revise, minPt2, maxPt2);  //获取坐标极值
	std::cout << "倾角校正前甲板平面 minPoint3D:(" << minPt2.x << "," << minPt2.y << "," << minPt2.z << ")"
		<< endl << "倾角校正前甲板平面 maxPoint3D:(" << maxPt2.x << "," << maxPt2.y << "," << maxPt2.z << ")" << endl << endl;

	//【7】计算扫描仪倾角
	//计算倾角校正后甲板平面包围盒
	float normal_start_x = deckPlane_before_revise_coefficients->values[0], normal_start_y = deckPlane_before_revise_coefficients->values[1], normal_start_z = deckPlane_before_revise_coefficients->values[2];
	float normal_end_x = 0, normal_end_y = 0, normal_end_z = 1;
	float rotation_ceter_x = 0, rotation_ceter_y = 0, rotation_ceter_z = 0;
	float rotation_axis_x = (normal_end_y - normal_start_y) * (rotation_ceter_z - normal_start_z) - (rotation_ceter_y - normal_start_y) * (normal_end_z - normal_start_z);
	float rotation_axis_y = (normal_end_z - normal_start_z) * (rotation_ceter_x - normal_start_x) - (rotation_ceter_z - normal_start_z) * (normal_end_x - normal_start_x);
	float rotation_axis_z = (normal_end_x - normal_start_x) * (rotation_ceter_y - normal_start_y) - (rotation_ceter_x - normal_start_x) * (normal_end_y - normal_start_y);
	//normalize旋转法向
	float square_sum = rotation_axis_x * rotation_axis_x +
		rotation_axis_y * rotation_axis_y +
		rotation_axis_z * rotation_axis_z;
	square_sum = sqrt(square_sum);
	rotation_axis_x /= square_sum;
	rotation_axis_y /= square_sum;
	rotation_axis_z /= square_sum;
	cout << "旋转法向为：(" << rotation_axis_x << "," << rotation_axis_y << "," << rotation_axis_z << ")" << endl;

	//计算需要绕旋转法向旋转的角度
	float rotation_angle = acos(deckPlane_before_revise_coefficients->values[2]);
	cout << "与旋转法向之间的偏转角度为：" << rotation_angle / M_PI * 180 << "度" << endl;

	//计算旋转矩阵R
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

	//【8】对甲板平面进行扫描仪倾角校正
	//旋转原始甲板平面点云进行倾角校正
	pcl::transformPointCloud(*deck_plane_before_revise, *deck_plane_after_revise, rotation_Matrix);
	pcl::getMinMax3D(*deck_plane_after_revise, minPt2, maxPt2);  //获取坐标极值
	std::cout << "倾角校正后甲板平面 minPoint3D:(" << minPt2.x << "," << minPt2.y << "," << minPt2.z << ")"
		<< endl << "倾角校正后甲板平面 maxPoint3D:(" << maxPt2.x << "," << maxPt2.y << "," << maxPt2.z << ")" << endl << endl;

	//计算甲板高度
	float deckHeight = 0;
	int sizeDeckPlane = deck_plane_after_revise->size();
	for (int i = 0; i < sizeDeckPlane; i++) deckHeight += deck_plane_after_revise->points[i].z;
	deckHeight /= sizeDeckPlane;
	cout << "经过倾角校正后得到甲板平面高度为：" << -deckHeight << endl;
	cout << endl;

	//【9】利用甲板平面范围分割出原始点云场景中陆侧区域、甲板平面以上区域
	cout << "利用甲板平面范围分割场景点云中..." << endl << endl;
	pcl::PointCloud<PointT>::Ptr cloud_land_after_revise_ori(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_land_after_revise(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_above_deckPlane_after_revise_ori(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_above_deckPlane_after_revise(new pcl::PointCloud<PointT>);

	//从经过反射率滤波后的原始点云数据中分割出甲板平面以上区域
	pcl::transformPointCloud(*cloud_pulse_filtered, *cloud_pulse_filtered_after_revise, rotation_Matrix); //对经过反射率滤波后的原始点云数据进行扫描仪倾角校正
	pcl::ConditionAnd<PointT>::Ptr cloud_above_deckPlane_range_cond(new pcl::ConditionAnd<PointT>());
	cloud_above_deckPlane_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LE, maxPt2.x - 1.0)));
	cloud_above_deckPlane_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GE, minPt2.x + 1.0)));
	cloud_above_deckPlane_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LE, -0.5)));
	cloud_above_deckPlane_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GE, deckHeight)));
	// 创建滤波器
	pcl::ConditionalRemoval<PointT> cloud_above_deckPlane_condrem(false);
	cloud_above_deckPlane_condrem.setCondition(cloud_above_deckPlane_range_cond);
	cloud_above_deckPlane_condrem.setInputCloud(cloud_pulse_filtered_after_revise);
	cloud_above_deckPlane_condrem.setKeepOrganized(true);
	// 应用滤波器
	cloud_above_deckPlane_condrem.filter(*cloud_above_deckPlane_after_revise_ori);
	std::vector<int> indices_above_deckPlane;
	pcl::removeNaNFromPointCloud(*cloud_above_deckPlane_after_revise_ori, *cloud_above_deckPlane_after_revise, indices_above_deckPlane);
	cout << "	完成甲板平面以上区域分割！" << endl;

	//从未经过反射率滤波的原始点云数据中分割出陆侧区域（包含定位反射板，用于后续识别定位使用）
	pcl::transformPointCloud(*cloud_downsampled, *cloud_downsampled_after_revise, rotation_Matrix); //对经过反射率滤波后的原始点云数据进行扫描仪倾角校正
	pcl::ConditionAnd<PointT>::Ptr cloud_land_range_cond(new pcl::ConditionAnd<PointT>());
	cloud_land_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LE, minPt2.x - 1.0)));
	// 创建滤波器
	pcl::ConditionalRemoval<PointT> cloud_land_condrem(false);
	cloud_land_condrem.setCondition(cloud_land_range_cond);
	cloud_land_condrem.setInputCloud(cloud_downsampled_after_revise);
	cloud_land_condrem.setKeepOrganized(true);
	// 应用滤波器
	cloud_land_condrem.filter(*cloud_land_after_revise_ori);
	std::vector<int> indices_land;
	pcl::removeNaNFromPointCloud(*cloud_land_after_revise_ori, *cloud_land_after_revise, indices_land);
	cout << "	完成陆侧区域分割！" << endl << endl;

	//【10】对陆侧区域进行进一步处理，分割出其中卸船机支架平面，用于激光扫描仪的定位
	//估算陆侧区域点云法线
	cout << "计算陆侧区域点云数据法线中..." << endl;
	start = clock();
	ne.setInputCloud(cloud_land_after_revise);
	//输出数据集
	pcl::PointCloud<pcl::Normal>::Ptr cloud_land_normals(new pcl::PointCloud<pcl::Normal>);
	//计算点云法线
	ne.compute(*cloud_land_normals);
	finish = clock();
	cout << "计算陆侧区域点云数据法线消耗时间为： " << finish - start << "ms" << endl << endl;

	pcl::PointCloud<PointT>::Ptr cloud_land_rest(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_holder_plane(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr holder_plane(new pcl::PointCloud<PointT>());
	int landNormalSize = cloud_land_normals->size();
	for (int i = 0; i < landNormalSize; i++)
	{
		if (cloud_land_normals->points[i].normal_x >= 0.99) potential_holder_plane->push_back(cloud_land_after_revise->points[i]);
		else cloud_land_rest->push_back(cloud_land_after_revise->points[i]);
	}
	//分割出卸船机支架平面
	pcl::PointIndices::Ptr holder_inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr holderPlane_coefficients(new pcl::ModelCoefficients);

	pcl::SACSegmentation<PointT> holder_plane_seg;
	holder_plane_seg.setOptimizeCoefficients(true);
	holder_plane_seg.setModelType(pcl::SACMODEL_PLANE);
	holder_plane_seg.setMethodType(pcl::SAC_RANSAC);
	holder_plane_seg.setDistanceThreshold(0.18);
	holder_plane_seg.setInputCloud(potential_holder_plane);
	holder_plane_seg.segment(*holder_inliers, *holderPlane_coefficients);

	// 抽取分割出的甲板平面点的索引
	pcl::ExtractIndices<PointT> holder_extract;
	holder_extract.setInputCloud(potential_holder_plane);
	holder_extract.setIndices(holder_inliers);
	holder_extract.setNegative(false);
	holder_extract.filter(*holder_plane);

	float holderPlaneX = 0;
	for (int i = 0; i < holder_plane->size(); i++) holderPlaneX += holder_plane->points[i].x;
	holderPlaneX /= holder_plane->size();
	cout << "卸船机支架平面距离扫描仪原点的距离为：" << holderPlaneX << "m (可被用于确定扫描仪在世界坐标系中的位置)" << endl << endl;

	//【11】对甲板平面以上区域进行进一步处理，分割出其中的四个船舱平面
	//估算甲板平面以上区域点云法线
	cout << "计算甲板平面以上区域点云数据法线中..." << endl;
	start = clock();
	ne.setInputCloud(cloud_above_deckPlane_after_revise);
	//输出数据集
	pcl::PointCloud<pcl::Normal>::Ptr cloud_above_deckPlane_normals(new pcl::PointCloud<pcl::Normal>);
	//计算点云法线
	ne.compute(*cloud_above_deckPlane_normals);
	finish = clock();
	cout << "计算甲板平面以上区域点云数据法线消耗时间为： " << finish - start << "ms" << endl << endl;

	//创建保存四个船舱平面的指针变量
	cout << "利用法线信息对甲板平面进一步处理，分割四个船舱平面中..." << endl << endl;
	pcl::PointCloud<PointT>::Ptr potential_cabin_plane_100(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cabin_plane_100(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_cabin_plane_n100(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cabin_plane_n100(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_cabin_plane_010(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cabin_plane_010(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr potential_cabin_plane_n010(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cabin_plane_n010(new pcl::PointCloud<PointT>());
	int deckNormalSize = cloud_above_deckPlane_normals->size();
	for (int i = 0; i < deckNormalSize; i++)
	{
		if (cloud_above_deckPlane_normals->points[i].normal_x >= 0.99) potential_cabin_plane_100->push_back(cloud_above_deckPlane_after_revise->points[i]);
		if (cloud_above_deckPlane_normals->points[i].normal_x <= -0.99) potential_cabin_plane_n100->push_back(cloud_above_deckPlane_after_revise->points[i]);
		if (cloud_above_deckPlane_normals->points[i].normal_y >= 0.99) potential_cabin_plane_010->push_back(cloud_above_deckPlane_after_revise->points[i]);
		if (cloud_above_deckPlane_normals->points[i].normal_y <= -0.99) potential_cabin_plane_n010->push_back(cloud_above_deckPlane_after_revise->points[i]);
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

	//【12】利用得到的船舱轮廓范围从未经过反射率滤波的原始点云数据中分割出物料区域
	float delta = 0.5;
	pcl::PointCloud<PointT>::Ptr cloud_dump_ori(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_dump(new pcl::PointCloud<PointT>);
	pcl::ConditionAnd<PointT>::Ptr cloud_dump_range_cond(new pcl::ConditionAnd<PointT>());
	cloud_dump_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LE, cabinPlane_Xmax - delta)));
	cloud_dump_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GE, cabinPlane_Xmin + delta)));
	cloud_dump_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LE, cabinPlane_Ymax - delta)));
	cloud_dump_range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GE, cabinPlane_Xmin + delta)));
	// 创建滤波器
	pcl::ConditionalRemoval<PointT> cloud_dump_condrem(false);
	cloud_dump_condrem.setCondition(cloud_dump_range_cond);
	cloud_dump_condrem.setInputCloud(cloud_downsampled_after_revise);
	cloud_dump_condrem.setKeepOrganized(true);
	// 应用滤波器
	cloud_dump_condrem.filter(*cloud_dump_ori);
	std::vector<int> indices_dump;
	pcl::removeNaNFromPointCloud(*cloud_dump_ori, *cloud_dump, indices_dump);
	cout << "完成煤堆物料区域区域分割！" << endl;

	//【13】对分割出来的物料区域进行邻域滤波处理，滤除由于空气中灰尘杂质导致测量得到的离散点
	pcl::PointCloud<PointT>::Ptr cloud_dump_after_filtering(new pcl::PointCloud<PointT>);
	//邻域滤波
	pcl::RadiusOutlierRemoval<PointT> dump_outrem; //创建滤波器
	dump_outrem.setInputCloud(cloud_dump);
	dump_outrem.setRadiusSearch(0.2);
	dump_outrem.setMinNeighborsInRadius(3);
	dump_outrem.filter(*cloud_dump_after_filtering);

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
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer For Segmentation Of Ship Scene");
	viewer.addCoordinateSystem();
	viewer.setBackgroundColor(1.0, 0.5, 0.5);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> oricloud_color_handler(cloud_land_after_revise, 255, 255, 255);
	viewer.addPointCloud<PointT>(cloud_land_after_revise, oricloud_color_handler, "original cloud");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> holderPlane_color_handler(holder_plane, 255, 255, 0);
	viewer.addPointCloud<PointT>(holder_plane, holderPlane_color_handler, "holder plane");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> deckPlane_color_handler(deck_plane_after_revise, 255, 0, 255);
	viewer.addPointCloud<PointT>(deck_plane_after_revise, deckPlane_color_handler, "deck plane");

	//pcl::visualization::PointCloudColorHandlerCustom<PointT> above_deckPlane_color_handler(cloud_above_deckPlane_after_revise, 0, 0, 255);
	//viewer.addPointCloud<PointT>(cloud_above_deckPlane_after_revise, above_deckPlane_color_handler, "above deck plane");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cabin_plane_h1_color_handler(cabin_plane_100, 255, 0, 0);
	viewer.addPointCloud<PointT>(cabin_plane_100, cabin_plane_h1_color_handler, "cabin plane h1");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cabin_plane_h2_color_handler(cabin_plane_n100, 0, 255, 0);
	viewer.addPointCloud<PointT>(cabin_plane_n100, cabin_plane_h2_color_handler, "cabin plane h2");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cabin_plane_v1_color_handler(cabin_plane_010, 0, 0, 255);
	viewer.addPointCloud<PointT>(cabin_plane_010, cabin_plane_v1_color_handler, "cabin plane v1");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cabin_plane_v2_color_handler(cabin_plane_n010, 0, 255, 255); //100 100 100
	viewer.addPointCloud<PointT>(cabin_plane_n010, cabin_plane_v2_color_handler, "cabin plane v2");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> dump_color_handler(cloud_dump_after_filtering, 50, 50, 50);
	viewer.addPointCloud<PointT>(cloud_dump_after_filtering, dump_color_handler, "dump");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> land_color_handler(cloud_land_rest, 156, 160, 202);
	viewer.addPointCloud<PointT>(cloud_land_rest, land_color_handler, "land");
	/***
	pcl::visualization::CloudViewer Simpleviewer("Simple Cloud Viewer");
	Simpleviewer.showCloud(cloud_pulse_filtered);
	***/
	while (!viewer.wasStopped())
	{
		//在此处可以添加其他处理
		viewer.spinOnce();
	}

	return 0;
}

