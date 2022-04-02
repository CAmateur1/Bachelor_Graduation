#include "pointcloudmethod.h"
#include "mainwindow.h"

#include <QString>
#include <QDebug>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>//直通滤波
#include <pcl/filters/voxel_grid.h>//体素滤波
#include <pcl/filters/statistical_outlier_removal.h>//离群点滤波
//区域生长算法使用的头文件
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h>//过滤缺失坐标信息的点
#include <pcl/segmentation/region_growing.h>

#include <pcl/visualization/cloud_viewer.h>

//构造函数
PointCloudMethod::PointCloudMethod()
{

}

//打开文件
void PointCloudMethod::loadPointCloudFile(QString str, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::string file_path = str.toStdString();
    if(str.endsWith(".ply"))
    {
        pcl::io::loadPLYFile(file_path, *cloud);
    }
    else if(str.endsWith(".pcd"))
    {
        pcl::io::loadPCDFile(file_path, *cloud);
    }
}

//体素滤波
void PointCloudMethod::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PCLPointCloud2::Ptr voxel_cloud (new pcl::PCLPointCloud2());//点云数据指针

    pcl::PCLPointCloud2::Ptr voxel_cloud_filtered (new pcl::PCLPointCloud2());

    pcl::toPCLPointCloud2(*cloud, *voxel_cloud);//点云数据类型转换

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;//滤波器对象

    sor.setInputCloud(voxel_cloud);

    sor.setLeafSize(0.01f, 0.01f, 0.01f);//边长为1厘米的体素点

    sor.filter(*voxel_cloud_filtered);//过滤，过滤后的点云存储到voxl_cloud_filtered中

    cloud->clear();

    pcl::fromPCLPointCloud2(*voxel_cloud_filtered, *cloud);//类型转换
}

//离群点滤波
void PointCloudMethod::outliersFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outliers_removal (new pcl::PointCloud<pcl::PointXYZ>);//输入点云容器

    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outliers_removal_filtered (new pcl::PointCloud<pcl::PointXYZ>);//过滤后点云容器

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_sor;//过滤器

    pcl::copyPointCloud(*cloud, *statistical_outliers_removal);

    statistical_sor.setInputCloud(statistical_outliers_removal);//输入点云

    statistical_sor.setMeanK(50);//设置邻点数

    statistical_sor.setStddevMulThresh(1.0);

    statistical_sor.filter(*statistical_outliers_removal_filtered);

    pcl::copyPointCloud(*statistical_outliers_removal_filtered, *cloud);
}

//区域生长算法分割点云区域
void PointCloudMethod::regionSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_point)
{
    qDebug()<<"开始分割";
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//创建kd树对象，方便算法索引点云

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);//法向量存储器

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;//法向量计算对象

    normal_estimator.setSearchMethod(tree);//设置搜索方法

    normal_estimator.setInputCloud(cloud);//添加点云

    normal_estimator.setKSearch(50);//设置法向量估计邻近点数量

    normal_estimator.compute(*normals);//计算法向量并存储进法向量存储器


    pcl::IndicesPtr indices (new std::vector<int>);//索引容器

    pcl::removeNaNFromPointCloud(*cloud, *indices);//去除没有坐标的点云
    //qDebug()<<*cloud->size();


    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> region;//区域生长算法对象

    region.setMinClusterSize(50);//类点数下限

    region.setMaxClusterSize(20000);//类点数上限

    region.setSearchMethod(tree);//设置搜索方法

    region.setNumberOfNeighbours(100);//设置邻点数30

    region.setInputCloud(cloud);//添加点云

    region.setIndices(indices);//添加索引，由去除无坐标点云函数得到

    region.setInputNormals(normals);//添加法向量

    region.setSmoothnessThreshold(3.0 / 180.0 * M_PI);//设置聚类角度阈值

    region.setCurvatureThreshold(1.0);//设置区域生长起始点曲率阈值


    std::vector<pcl::PointIndices> clusters;//分割后的索引

    region.extract(clusters);//提取索引

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = region.getColoredCloud();//为点云上色
    colored_point = region.getColoredCloud();
    //qDebug()<<*colored_point->size();

    //*colored_point = *colored_cloud;
   // qDebug()<<*colored_point->size();
}

//直通滤波
void PointCloudMethod::passThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;//直通滤波器

    pass.setInputCloud(cloud);//输入点云

    pass.setFilterFieldName("z");//过滤坐标轴

    pass.setFilterLimits(-1.5, 0.0);//设置过滤范围

    pass.filter(*cloud_filtered);//过滤并将结果存入cloud_filtered中

    cloud->clear();//清除类成员的内容

    pcl::copyPointCloud(*cloud_filtered, *cloud);//将点云复制给类成员指针
}
