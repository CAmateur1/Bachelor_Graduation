#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include <QThread>

#include "pointcloudmethod.h"
#include "prothread.h"

class QVTKWidget;
class PointCloudMethod;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;//ui界面

    QVTKWidget *qvtkWidget;//点云显示控件

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;//点云可视化指针

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;//加载点云的指针

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_point;//带颜色的点云

    PointCloudMethod processor;//点云数据处理对象

    ProThread cloud_process_thread;

public slots:
    //按钮开始槽函数
    void start();
    //直通滤波
    void passThrough();
    //降采样
    void downSample();
    //离群点滤波
    void outliersFilter();
    //点云属性获取
    void getProperties(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void getProperties(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    //分割函数
    void regionSegmentation();

    //添加点云文件
    void addDataFiles();
    //双击文件操作槽函数
    void onDataTreeDoubleClicked();
    //设置点云处理时禁止其他处理
    void setButtonDisabled();
    //打开按钮功能
    void setButtonEnabled();

signals:
    void startDownSampleSig();
    void startOutliersFilterSig();
    void startRegionSegmentationSig();
};
#endif // MAINWINDOW_H
