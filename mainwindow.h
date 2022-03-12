#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "QVTKWidget.h"

class QVTKWidget;

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

public slots:
    //按钮开始槽函数
    void start();
    //降采样
    void downSample();
    //离群点滤波
    void outliersFilter();
};
#endif // MAINWINDOW_H
