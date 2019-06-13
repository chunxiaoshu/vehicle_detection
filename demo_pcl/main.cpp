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

#define PI 3.141592653

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


struct Grid {
	float pos_x;
	float pos_y;
	float half_x_width;
	float half_y_width;
	float high;
	Grid(float pos_xi, float pos_yi, float half_x_widthi, float half_y_widthi, float highi) :
		pos_x(pos_xi),
		pos_y(pos_yi),
		half_x_width(half_x_widthi),
		half_y_width(half_y_widthi),
		high(highi) {}
};
struct GrabModel {
	float pos_x;
	float pos_y;
	float pos_z;
	float half_width;
	GrabModel(float pos_xi, float pos_yi, float pos_zi, float half_widthi) :
		pos_x(pos_xi),
		pos_y(pos_yi),
		pos_z(pos_zi),
		half_width(half_widthi) {}
};

vector<Grid> Gridding(pcl::PointCloud<PointT>::Ptr inputPointCloud, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, int xnumGrid, int ynumGrid, bool yGradding, float threshold, GrabModel grabModel);
GrabModel *allGrab;

int main(int argc, char *argv[])
{
	//计时
	clock_t start_Total, finish_Total;
	start_Total = clock();

	//pcl::visualization::PCLVisualizer viewer("Cloud Viewer For Segmentation Of Ship Scene");
	//viewer.addCoordinateSystem();
	//viewer.setBackgroundColor(1.0, 0.5, 0.5);

	//【1】加载原始点云数据
	cout << "加载原始点云数据中..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_downsampled_after_revise(new pcl::PointCloud<PointT>);

	if (pcl::io::loadPCDFile<PointT>("originpcd.pcd", *cloud))//加载原始场景点云数据
	{
		cout << "原始船舱场景点云数据加载失败！" << endl << endl;
		return -1;
	}
	else
		cout << "原始船舱场景点云数据加载成功！" << endl << endl;

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

bool Judge_cross(Grid grid, GrabModel grabModel, float threshold) {                //判断抓斗与某网格是否有碰撞危险
	float grid_x = grid.pos_x;
	float grid_y = grid.pos_y;
	float grid_high = grid.high;
	float grid_x_width = grid.half_x_width;
	float grid_y_width = grid.half_y_width;
	float grab_x = grabModel.pos_x;
	float grab_y = grabModel.pos_y;
	float grab_z = grabModel.pos_z;
	float grab_width = grabModel.half_width;
	if (abs(grid_x - grab_x) <= grid_x_width + grab_width && abs(grid_y - grab_y) <= grid_y_width + grab_width) {
		if (abs(grid_high - grab_z) <= threshold)
			return true;
		else
			return false;
	}
	else
		return false;
}



void viewerAddGrab(pcl::visualization::PCLVisualizer& viewer) {
	float x_min = allGrab->pos_x - allGrab->half_width;
	float x_max = allGrab->pos_x + allGrab->half_width;
	float y_min = allGrab->pos_y - allGrab->half_width;
	float y_max = allGrab->pos_y + allGrab->half_width;
	float z_min = allGrab->pos_z - allGrab->half_width;
	float z_max = allGrab->pos_z + allGrab->half_width;
	viewer.addCube(x_min, x_max, y_min, y_max, z_min, z_max, 1, 0.5, 0.5, "GrabM", 0);
}

vector<Grid> Gridding(pcl::PointCloud<PointT>::Ptr inputPointCloud, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, int xnumGrid, int ynumGrid, bool yGradding, float threshold, GrabModel grabModel)
{
	vector<Grid> gridResult;
	//float shake_threshold = 2;      //防抖阈值
	//vector<float> highGrid;                                                     //每个网格的最大高度
	pcl::PassThrough<PointT> pass;
	pcl::PointCloud<PointT>::Ptr cloud_filtered_x(new pcl::PointCloud<PointT>); //船舱点云
	pcl::PointCloud<PointT>::Ptr cloud_filtered_y(new pcl::PointCloud<PointT>); //船舱点云
	pcl::PointCloud<PointT>::Ptr pointcloud_grid(new pcl::PointCloud<PointT>);  //显示用，网格点云
	pcl::PointCloud<PointT>::Ptr pointcloud_realgrid(new pcl::PointCloud<PointT>);

	if (!yGradding)                                                             //仅在x方向网格化
	{
		Grid nextGrab(0, 0, 0, 0, 0);
		pass.setInputCloud(inputPointCloud);
		pass.setFilterFieldName("x");                                           //根据甲板范围对点云进行分割
		pass.setFilterLimits(x_min, x_max);
		pass.filter(*cloud_filtered_x);                                         //得到分割后的点云
		float x_grid_size = (x_max - x_min) / (float)xnumGrid;                  //网格大小
		for (int i = 0; i < xnumGrid; i++)
		{
			pcl::PointCloud<PointT>::Ptr temp_grid(new pcl::PointCloud<PointT>);           //单个网格点云
			pcl::PointCloud<PointT>::Ptr temp_grid1(new pcl::PointCloud<PointT>);
			PointT minPoint, maxPoint;                                                     //网格内最大最小点
			pass.setFilterLimits(x_min + i*x_grid_size, x_min + (i + 1)*x_grid_size);      //根据网格范围进行单网格划分
			pass.filter(*temp_grid);                                                       //划分出当前网格
			pcl::copyPointCloud(*temp_grid, *temp_grid1);
			
			/***********************************显示用，后删*******************************************************/
			
			if (i % 2 == 0)                                                                          //着色
			{
				for (int j = 0; j < temp_grid1->points.size(); j++){ temp_grid1->points[j].intensity = 0; }
			}
			else
			{
				for (int j = 0; j < temp_grid->points.size(); j++){ temp_grid1->points[j].intensity = 255; }
			}
			
			*pointcloud_grid += *temp_grid;  //合并网格
			*pointcloud_realgrid += *temp_grid1;
			/*******************************************************************************************************/
			pcl::getMinMax3D(*temp_grid, minPoint, maxPoint);                                        //获取网格内边界点
			Grid grid(x_min + (2 * i + 1) * x_grid_size / 2,
				0,
				x_grid_size / 2,
				0,
				maxPoint.z);
			gridResult.push_back(grid);                                                              //记录网格结果
			if (nextGrab.high < abs(grid.high))
				nextGrab = grid;
		}
		cout << "Next Grab Point is " << nextGrab.pos_x << endl;
		vector<Grid> warnGrid;
		for (int i = 0; i < gridResult.size(); i++) {
			if (Judge_cross(gridResult[i], grabModel, 1))
				warnGrid.push_back(gridResult[i]);
		}
		if (warnGrid.size() == 0)
			cout << "No Collision Warning" << endl;
		else {
			for (int i = 0; i < warnGrid.size(); i++) {
				cout << "【Warning】 : With grid (" << warnGrid[i].pos_x << ") hight:" << warnGrid[i].high << endl;
			}
		}
	}
	else																						    //标定阶段进行所有区域网格化
	{
		pass.setInputCloud(inputPointCloud);
		pass.setFilterFieldName("x");                                                               //根据甲板x范围进行提取
		pass.setFilterLimits(x_min, x_max);
		pass.filter(*cloud_filtered_x);

		pass.setInputCloud(cloud_filtered_x);
		pass.setFilterFieldName("y");                                                                //根据甲板y范围进行提取
		pass.setFilterLimits(y_min, y_max);
		pass.filter(*cloud_filtered_y);

		float x_grid_size = (x_max - x_min) / (float)xnumGrid;                                       //网格大小
		float y_grid_size = (y_max - y_min) / (float)ynumGrid;
		for (int i = 0; i < xnumGrid; i++)
		{
			pcl::PointCloud<PointT>::Ptr x_temp_grid(new pcl::PointCloud<PointT>);                    //首先在x方向划分
			pass.setInputCloud(cloud_filtered_y);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(x_min + i*x_grid_size, x_min + (i + 1)*x_grid_size);
			pass.filter(*x_temp_grid);
			for (int j = 0; j < ynumGrid; j++)
			{
				PointT minPoint, maxPoint;
				pcl::PointCloud<PointT>::Ptr y_temp_grid_ori(new pcl::PointCloud<PointT>);             //再进行y方向的划分
				pcl::PointCloud<PointT>::Ptr y_temp_grid(new pcl::PointCloud<PointT>);
				pcl::PointCloud<PointT>::Ptr y_temp_realgrid_ori(new pcl::PointCloud<PointT>);
				pcl::PointCloud<PointT>::Ptr y_temp_realgrid(new pcl::PointCloud<PointT>);
				pass.setInputCloud(x_temp_grid);
				pass.setFilterFieldName("y");
				pass.setFilterLimits(y_min + j*y_grid_size, y_min + (j + 1)*y_grid_size);
				pass.filter(*y_temp_grid_ori);
				pcl::copyPointCloud(*y_temp_grid_ori, *y_temp_realgrid_ori);
				/***********************************网格化显示用*******************************************************/
				if (y_temp_realgrid_ori->size() > 0) 
				{
					if (i % 2 == j % 2)                                                                          //间隔着色
					{
						for (int k = 0; k < y_temp_realgrid_ori->points.size(); k++){ y_temp_realgrid_ori->points[k].intensity = 0; }
					}
					else
					{
						for (int k = 0; k < y_temp_realgrid_ori->points.size(); k++){ y_temp_realgrid_ori->points[k].intensity = 255; }
					}
					std::vector<int> indices_Plane1;
					//indices_Plane.clear();
					//y_temp_grid->clear();
					pcl::removeNaNFromPointCloud(*y_temp_realgrid_ori, *y_temp_realgrid, indices_Plane1);
					*pointcloud_realgrid += *y_temp_realgrid;
				}
				//***
				if (y_temp_grid_ori->size() > 0)
				{
					                                                                                     //按高程值着色
					float sumDEM = 0;
					float avgDEM = 0;
					for (int k = 0; k < y_temp_grid_ori->points.size(); k++)
					{
						sumDEM += y_temp_grid_ori->points[k].z;
					}
					avgDEM = sumDEM / y_temp_grid_ori->points.size();
					float coeff = (avgDEM - z_min) / (z_max - z_min);
					for (int k = 0; k < y_temp_grid_ori->points.size(); k++)
					{
						y_temp_grid_ori->points[k].intensity = coeff;
						//y_temp_grid_ori->points[k].g = round(255 * coeff);
						//y_temp_grid_ori->points[k].b = round(255 * coeff);
					}
					std::vector<int> indices_Plane;
					//indices_Plane.clear();
					//y_temp_grid->clear();
					pcl::removeNaNFromPointCloud(*y_temp_grid_ori, *y_temp_grid, indices_Plane);
					*pointcloud_grid += *y_temp_grid;                                                          //合并网格
				}
				//***/
				/*******************************************************************************************************/
				//std::vector<int> indices_Plane;
				//indices_Plane.clear();
				//y_temp_grid->clear();
				//pcl::removeNaNFromPointCloud(*y_temp_grid_ori, *y_temp_grid, indices_Plane);
				//*pointcloud_grid += *y_temp_grid_ori;                                                          //合并网格
				pcl::getMinMax3D(*y_temp_grid, minPoint, maxPoint);                                      //获取网格内边界点
				if (y_temp_grid->size() == 0)
				{
					Grid grid(x_min + (2 * i + 1) * x_grid_size / 2,
						y_min + (2 * j + 1) * y_grid_size / 2,
						x_grid_size / 2,
						y_grid_size / 2,
						0);
					gridResult.push_back(grid);
					//cout << "第(" << i << "," << j << ")" << "个网格完成高度为：" << 0 << endl;
				}
				else
				{
					Grid grid(x_min + (2 * i + 1) * x_grid_size / 2,
						y_min + (2 * j + 1) * y_grid_size / 2,
						x_grid_size / 2,
						y_grid_size / 2,
						maxPoint.z);
					gridResult.push_back(grid);
					//cout << "第(" << i << "," << j << ")" << "个网格完成高度为：" << maxPoint.z << endl;
				}
			}
		}
	}
	/**********************************显示用，后删**************************************/
	pcl::visualization::CloudViewer cloud_viewer("Cloud Viewer Of Gridding of Dump Area");
	//cloud_viewer.addCoordinateSystem();
	cloud_viewer.showCloud(pointcloud_realgrid);
	cloud_viewer.runOnVisualizationThreadOnce(viewerAddGrab);
	
	while (!cloud_viewer.wasStopped())
	{
		//cloud_viewer.spinOnce();
	}
	/**********************************显示用，后删**************************************/
	pcl::visualization::CloudViewer cloud_viewer1("Cloud Viewer Of Demm of Dump Area");
	//cloud_viewer.addCoordinateSystem();
	cloud_viewer1.showCloud(pointcloud_grid);

	while (!cloud_viewer1.wasStopped())
	{
		//cloud_viewer.spinOnce();
	}
	/***********************************************************************************/

	cout << "xGrid: " << xnumGrid << "    " << "yGrid: " << ynumGrid << endl;
	return gridResult;
}