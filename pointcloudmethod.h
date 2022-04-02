#ifndef POINTCLOUDMETHOD_H
#define POINTCLOUDMETHOD_H

//#include <QMainWindow>
#include <QThread>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "QVTKWidget.h"


class PointCloudMethod: public QObject
{

    Q_OBJECT

public:
    PointCloudMethod();

public slots:
    //打开文件
    void loadPointCloudFile(QString str, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    //体素滤波，降采样使用。
    void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    //离群点滤波
    void outliersFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    //区域生长算法分割点云
    void regionSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_point);
    //直通滤波
    void passThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
signals:
    void downSampleComplete();
};

#endif // POINTCLOUDMETHOD_H
