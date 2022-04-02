#ifndef PROTHREAD_H
#define PROTHREAD_H
#include <QObject>
#include <QThread>
#include "pointcloudmethod.h"

class ProThread:public QObject
{
    Q_OBJECT
private:


public:
    ProThread();
    ~ProThread();
    void downSampleThread(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void outliersFilterThread(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void regionSegmentationThread(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr *colored_point);

    QThread down_sample_thread;
    QThread outliers_filter_thread;
    QThread region_seg_thread;
signals:
    void downSampleSig();
    void outliersFilterSig();
    void regionSegmentationSig();
    
public slots:
    void emitDownSampleSig();
    void emitOutliersFilterSig();
    void emitRegionSegmentationSig();

};

#endif // PROTHREAD_H
