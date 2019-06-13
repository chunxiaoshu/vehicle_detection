#include <iostream>  
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>

int main()
{	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("originpcd.pcd", *cloud))
	{
		PCL_ERROR("can't read pcd file.");
		return -1;
	}

	pcl::visualization::PCLVisualizer viewer("3D viewer");
	viewer.setBackgroundColor(1.0, 0.5, 0.5);//背景
	viewer.addCoordinateSystem(1.0f);//坐标轴放大倍率为1
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler(cloud, 255, 255, 255);
	viewer.addPointCloud(cloud, point_cloud_color_handler, "original point cloud");//原始点云白色
	
	//viewer.initCameraParameters();
	//viewer.spin();//处理3D窗口的当前事件
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_below(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pf_below;
	pf_below.setInputCloud(cloud);
	pf_below.setFilterFieldName("z");//选择z轴做过滤
	pf_below.setFilterLimits(-38, -35);//过滤范围
	pf_below.setFilterLimitsNegative(false);//false保留范围内的；true保留范围外的
	pf_below.filter(*cloud_below);

	pcl::ModelCoefficients::Ptr coefficients_cabin(new pcl::ModelCoefficients);//平面参数
	pcl::PointIndices::Ptr inliers_cabin(new pcl::PointIndices);//分割点索引矩阵
	pcl::SACSegmentation<pcl::PointXYZ> seg_cabin;
	seg_cabin.setOptimizeCoefficients(true);//设置最优参数
	seg_cabin.setModelType(pcl::SACMODEL_PLANE);//平面分割
	seg_cabin.setMethodType(pcl::SAC_RANSAC);//随机抽样一致算法
	seg_cabin.setDistanceThreshold(0.1);//距离阈值
	seg_cabin.setInputCloud(cloud_below);//输入一个点云的备份
	seg_cabin.segment(*inliers_cabin, *coefficients_cabin);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cabin(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract_cabin;
	extract_cabin.setInputCloud(cloud_below);
	extract_cabin.setIndices(inliers_cabin);
	extract_cabin.setNegative(false);//false得到索引点云，true得到索引之外的点云
	extract_cabin.filter(*cloud_cabin);//获得船舱平面


	//创建和重建获取点云对应多边形边界的点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_cabin);
	chull.setAlpha(0.1);
	chull.reconstruct(*cloud_hull);
	

	//std::cerr << "Plane coefficients: " << *coefficients_cabin << std::endl;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cabin_color_handler(cloud_cabin, 0, 255, 0);
	viewer.addPointCloud(cloud_cabin, cabin_color_handler, "cabin");//船舱平面点黑色
	std::cerr << "The size of the plane cloud is: " << cloud_cabin->points.size() << " data points." << std::endl;

	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cabin_color_handler1(cloud_hull, 255, 0, 0);
	viewer.addPointCloud(cloud_hull, cabin_color_handler1, "cabin1");//船舱平面点黑色
	std::cerr << "The size of the hull cloud is: " << cloud_hull->points.size() << " data points." << std::endl;



	viewer.initCameraParameters();
	viewer.spin();//处理3D窗口的当前事件





	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_above(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pf_above;
	pf_above.setInputCloud(cloud);
	pf_above.setFilterFieldName("z");//选择z轴做过滤
	pf_above.setFilterLimits(-25, 0);//过滤范围
	pf_above.setFilterLimitsNegative(false);//false保留范围内的；true保留范围外的
	pf_above.filter(*cloud_above);

	pcl::ModelCoefficients::Ptr coefficients_deck(new pcl::ModelCoefficients);//平面或圆柱参数
	pcl::PointIndices::Ptr inliers_deck(new pcl::PointIndices);//分割点索引矩阵
	pcl::SACSegmentation<pcl::PointXYZ> seg_deck;
	seg_deck.setOptimizeCoefficients(true);//设置最优参数
	seg_deck.setModelType(pcl::SACMODEL_PLANE);//平面分割
	seg_deck.setMethodType(pcl::SAC_RANSAC);//随机抽样一致算法
	seg_deck.setDistanceThreshold(0.1);//距离阈值

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> decks(10);

	int i = 0, j = 0;
	while (cloud_above->points.size() > 50000)
	{
		PCL_INFO("Iteration NO %d.\n", i);
		i++;
		std::cerr << "The size of the rest cloud is: " << cloud_above->points.size() << " data points." << std::endl;

		seg_deck.setInputCloud(cloud_above);//输入一个点云的备份
		seg_deck.segment(*inliers_deck, *coefficients_deck);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_deck(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ExtractIndices<pcl::PointXYZ> extract_deck;
		extract_deck.setInputCloud(cloud_above);
		extract_deck.setIndices(inliers_deck);
		extract_deck.setNegative(false);//false得到索引点云，true得到索引之外的点云
		extract_deck.filter(*cloud_deck);

		extract_deck.setNegative(true);//false得到索引点云，true得到索引之外的点云
		extract_deck.filter(*cloud_s);
		cloud_above = cloud_s;

		std::cerr << "Plane coefficients: " << *coefficients_deck << std::endl;
		std::cerr << "The size of plane cloud is: " << cloud_deck->points.size() << " data points." << std::endl;

		if (coefficients_deck->values[0] > 0.99 || coefficients_deck->values[0] < -0.99 ||
			coefficients_deck->values[1] > 0.99 || coefficients_deck->values[1] < -0.99 ||
			i == 2)
		{
			PCL_INFO("get the aim %d points.\n", j);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> deck_color_handler(cloud_deck, 255, 0, 0);
			viewer.addPointCloud(cloud_deck, deck_color_handler, "deck");
			//viewer.initCameraParameters();
			//viewer.spin();

			viewer.removePointCloud("deck");
			decks[j] = cloud_deck;
			j++;
		}

	}
	
	if (j >= 1)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> deck_color_handler1(decks[0], 255, 0, 0);
		viewer.addPointCloud(decks[0], deck_color_handler1, "deck1");
	}
	if (j >= 2)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> deck_color_handler2(decks[1], 0, 255, 0);
		viewer.addPointCloud(decks[1], deck_color_handler2, "deck2");
	}
	if (j >= 3)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> deck_color_handler3(decks[2], 0, 0, 255);
		viewer.addPointCloud(decks[2], deck_color_handler3, "deck3");
	}
	if (j >= 4)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> deck_color_handler4(decks[3], 255, 255, 0);
		viewer.addPointCloud(decks[3], deck_color_handler4, "deck4");
	}
	if (j >= 5)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> deck_color_handler5(decks[4], 255, 0, 255);
		viewer.addPointCloud(decks[4], deck_color_handler5, "deck5");
	}
	if (j >= 6)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> deck_color_handler6(decks[5], 0, 255, 255);
		viewer.addPointCloud(decks[5], deck_color_handler6, "deck6");
	}
	if (j >= 7)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> deck_color_handler7(decks[6], 127, 127, 127);
		viewer.addPointCloud(decks[6], deck_color_handler7, "deck7");
	}
	if (j >= 8)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> deck_color_handler8(decks[7], 127, 255, 0);
		viewer.addPointCloud(decks[7], deck_color_handler8, "deck8");
	}

	viewer.initCameraParameters();
	viewer.spin();//处理3D窗口的当前事件
	













	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("originpcd.pcd", *cloud))
	{
		PCL_ERROR("can't read pcd file.");
		return -1;
	}

	pcl::visualization::PCLVisualizer viewer("3D viewer");
	viewer.setBackgroundColor(1.0, 0.5, 0.5);//背景
	viewer.addCoordinateSystem(1.0f);//坐标轴放大倍率为1
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler(cloud, 255, 255, 255);
	viewer.addPointCloud(cloud, point_cloud_color_handler, "original point cloud");//原始点云白色
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_below(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pf_below;
	pf_below.setInputCloud(cloud);
	pf_below.setFilterFieldName("z");//选择z轴做过滤
	pf_below.setFilterLimits(-38, -35);//过滤范围
	pf_below.setFilterLimitsNegative(false);//false保留范围内的；true保留范围外的
	pf_below.filter(*cloud_below);

	pcl::ModelCoefficients::Ptr coefficients_cabin(new pcl::ModelCoefficients);//平面参数
	pcl::PointIndices::Ptr inliers_cabin(new pcl::PointIndices);//分割点索引矩阵
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::PointCloud<pcl::Normal>::Ptr normals_below(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_below);
	ne.setRadiusSearch(0.1);
	ne.compute(*normals_below);

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);//定义为平面分割
	seg.setNormalDistanceWeight(0.2);
	seg.setMethodType(pcl::SAC_RANSAC);//随机抽样一致算法
	seg.setMaxIterations(100);//最大迭代
	seg.setDistanceThreshold(0.1);
	seg.setInputCloud(cloud_below);
	seg.setInputNormals(normals_below);//输入法线
	seg.segment(*inliers_cabin, *coefficients_cabin);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cabin(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract_cabin;
	extract_cabin.setInputCloud(cloud_below);
	extract_cabin.setIndices(inliers_cabin);
	extract_cabin.setNegative(false);//false得到索引点云，true得到索引之外的点云
	extract_cabin.filter(*cloud_cabin);//获得船舱平面

	std::cerr << "Plane coefficients: " << *coefficients_cabin << std::endl;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cabin_color_handler(cloud_cabin, 0, 0, 0);
	viewer.addPointCloud(cloud_cabin, cabin_color_handler, "cabin");//船舱平面点黑色

	viewer.initCameraParameters();
	viewer.spin();//处理3D窗口的当前事件


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_above(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pf_above;
	pf_above.setInputCloud(cloud);
	pf_above.setFilterFieldName("z");//选择z轴做过滤
	pf_above.setFilterLimits(-25, 0);//过滤范围
	pf_above.setFilterLimitsNegative(false);//false保留范围内的；true保留范围外的
	pf_above.filter(*cloud_above);

	pcl::ModelCoefficients::Ptr coefficients_deck(new pcl::ModelCoefficients);//平面或圆柱参数
	pcl::PointIndices::Ptr inliers_deck(new pcl::PointIndices);//分割点索引矩阵
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_deck;
	pcl::PointCloud<pcl::Normal>::Ptr normals_above(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_above);
	ne.setRadiusSearch(0.03);
	ne.compute(*normals_above);

	seg_deck.setOptimizeCoefficients(true);
	seg_deck.setModelType(pcl::SACMODEL_NORMAL_PLANE);//定义为平面分割
	seg_deck.setNormalDistanceWeight(0.2);
	seg_deck.setMethodType(pcl::SAC_RANSAC);//随机抽样一致算法
	seg_deck.setMaxIterations(100);//最大迭代
	seg_deck.setDistanceThreshold(0.1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_s(new pcl::PointCloud<pcl::Normal>);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> decks(10);

	int i = 0, j = 0;
	while (cloud_above->points.size() > 50000)
	{
		PCL_INFO("Iteration NO %d.\n", i);
		i++;
		std::cerr << "The size of all cloud is: " << cloud_above->points.size() << " data points." << std::endl;

		seg_deck.setInputCloud(cloud_above);//输入一个点云的备份
		seg_deck.setInputNormals(normals_above);//输入法线
		seg_deck.segment(*inliers_deck, *coefficients_deck);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_deck(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ExtractIndices<pcl::PointXYZ> extract_deck;
		extract_deck.setInputCloud(cloud_above);
		extract_deck.setIndices(inliers_deck);
		extract_deck.setNegative(false);//false得到索引点云，true得到索引之外的点云
		extract_deck.filter(*cloud_deck);

		extract_deck.setNegative(true);//false得到索引点云，true得到索引之外的点云
		extract_deck.filter(*cloud_s);
		cloud_above = cloud_s;

		pcl::ExtractIndices<pcl::Normal> extract_deck_normal;
		extract_deck_normal.setInputCloud(normals_above);
		extract_deck_normal.setNegative(true);
		extract_deck_normal.setIndices(inliers_deck);
		extract_deck_normal.filter(*normal_s);
		normals_above = normal_s;

		std::cerr << "Plane coefficients: " << *coefficients_deck << std::endl;
		std::cerr << "The size of plane cloud is: " << cloud_deck->points.size() << " data points." << std::endl;

		if (coefficients_deck->values[0] > 0.99 || coefficients_deck->values[0] < -0.99 ||
			coefficients_deck->values[1] > 0.99 || coefficients_deck->values[1] < -0.99)
		{
			PCL_INFO("get the aim %d points.\n", j);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> deck_color_handler(cloud_deck, 255, 0, 0);
			viewer.addPointCloud(cloud_deck, deck_color_handler, "deck");
			viewer.initCameraParameters();
			viewer.spin();

			viewer.removePointCloud("deck");
			decks[j] = cloud_deck;
			j++;
		}
		
	}
	*/
	

	return 0;
}

