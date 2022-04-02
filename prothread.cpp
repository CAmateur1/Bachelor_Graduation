#include "prothread.h"
#include "QDebug"

ProThread::ProThread()
{

}

ProThread::~ProThread()
{
    down_sample_thread.quit();

    down_sample_thread.wait();

    outliers_filter_thread.quit();

    outliers_filter_thread.wait();

    region_seg_thread.quit();

    region_seg_thread.wait();
    qDebug()<<"88888888";
}

//降采样线程
void ProThread::downSampleThread(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    PointCloudMethod *method = new PointCloudMethod;//工作指针

    method->moveToThread(&down_sample_thread);//指定线程执行

    connect(&down_sample_thread, &QThread::finished, method, &QObject::deleteLater);//线程结束后退出

    connect(this, &ProThread::downSampleSig, method, [=]()//执行降采样
    {
        qDebug()<<"222222222";

        method->downSample(cloud);//降采样

        down_sample_thread.quit();//终止子线程

        down_sample_thread.wait();//等待子线程关闭完成

        qDebug()<<"88888888";
    });

    down_sample_thread.start();
}

//离群点滤波线程
void ProThread::outliersFilterThread(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    PointCloudMethod *method = new PointCloudMethod;

    method->moveToThread(&outliers_filter_thread);

    connect(&outliers_filter_thread, &QThread::finished, method, &QObject::deleteLater);

    connect(this, &ProThread::outliersFilterSig, method, [=]()
    {
        qDebug()<<"开始离群点滤波";

        method->outliersFilter(cloud);
        qDebug()<<"123";

        outliers_filter_thread.quit();

        outliers_filter_thread.wait();

        qDebug()<<"xianchengjieshu";
    });

    outliers_filter_thread.start();
}

//区域生长算法分割点云线程
void ProThread::regionSegmentationThread(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr *colored_point)
{
    qDebug()<<"00000";
    PointCloudMethod *method = new PointCloudMethod;

    method->moveToThread(&region_seg_thread);

    connect(&region_seg_thread, &QThread::finished, method, &QObject::deleteLater);

    connect(this, &ProThread::regionSegmentationSig, method, [=]()
    {
        qDebug()<<"分割线程启动";
        method->regionSegmentation(*cloud, *colored_point);

        region_seg_thread.quit();

        region_seg_thread.wait();
        qDebug()<<"分割线程结束";
    });

    region_seg_thread.start();
}

//槽函数
void ProThread::emitDownSampleSig()
{
    emit downSampleSig();
}

void ProThread::emitOutliersFilterSig()
{
    emit outliersFilterSig();
}

void ProThread::emitRegionSegmentationSig()
{
    emit regionSegmentationSig();
}
