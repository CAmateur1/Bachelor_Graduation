#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QPushButton>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include "vtkRenderWindow.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //将点云显示窗口与点云显示指针连接起来
    qvtkWidget = new QVTKWidget(this);

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));

    qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());

    viewer->setupInteractor (qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());

    //设置点云显示窗口大小与位置
    qvtkWidget->resize(700,550);

    qvtkWidget->move(350,0);

    qvtkWidget->update ();

    connect(ui->startButton, &QPushButton::clicked, this, &MainWindow::start);

    connect(ui->closeButton, &QPushButton::clicked, this, &QMainWindow::close);

    connect(ui->down_sampleButton, &QPushButton::clicked, this, &MainWindow::downSample);
    connect(ui->outliers_filterButton, &QPushButton::clicked, this, &MainWindow::outliersFilter);

}
MainWindow::~MainWindow()
{
    delete ui;
}

//开始显示点云槽函数
void MainWindow::start()
{
    pcl::io::loadPCDFile("D:\\software_my\\CloudPoint\\example\\VoxelGrid_filter\\table_scene_lms400.pcd", *cloud);

    viewer->addPointCloud(cloud, "source_cloud");

    viewer->addCoordinateSystem(1.0, "cloud", 0);

    qvtkWidget->update();


}

//降采样
void MainWindow::downSample()
{
    pcl::PCLPointCloud2::Ptr voxel_cloud (new pcl::PCLPointCloud2());

    pcl::PCLPointCloud2::Ptr voxel_cloud_filtered (new pcl::PCLPointCloud2());

    pcl::toPCLPointCloud2(*cloud, *voxel_cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    sor.setInputCloud(voxel_cloud);

    sor.setLeafSize(0.01f, 0.01f, 0.01f);

    sor.filter(*voxel_cloud_filtered);

    viewer->removeAllPointClouds();

    pcl::fromPCLPointCloud2(*voxel_cloud_filtered, *cloud);

    viewer->addPointCloud(cloud, "down_sample_cloud");

    qvtkWidget->update();
}

//离群点过滤
void MainWindow::outliersFilter()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outliers_removal (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outliers_removal_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_sor;

    pcl::copyPointCloud(*cloud, *statistical_outliers_removal);

    statistical_sor.setInputCloud(statistical_outliers_removal);

    statistical_sor.setMeanK(50);

    statistical_sor.setStddevMulThresh(1.0);

    statistical_sor.filter(*statistical_outliers_removal_filtered);

    viewer->removeAllPointClouds();

    pcl::copyPointCloud(*statistical_outliers_removal_filtered, *cloud);

    viewer->addPointCloud(cloud, "outliers_removal");

    qvtkWidget->update();
}





