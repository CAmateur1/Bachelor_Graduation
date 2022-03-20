#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QPushButton>
#include <QString>
#include <QFileDialog>
#include <QStringList>
#include <QFileInfoList>
#include <QDir>
#include <QDebug>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QHeaderView>

//区域生长算法头文件
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h>//过滤缺失坐标信息的点
#include <pcl/segmentation/region_growing.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
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
    qvtkWidget = new QVTKWidget(ui->viewerWidget);

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());//点云指针初始化

    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));//点云窗口指针初始化

    qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());

    viewer->setupInteractor (qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());

    //设置点云显示窗口大小与位置
    qvtkWidget->resize(800,550);

    //设置垂直布局
    QVBoxLayout layout;

    layout.addWidget(qvtkWidget);

    layout.addWidget(ui->buttonWidget);

    ui->viewerWidget->setLayout(&layout);

    //qvtkWidget->move(350,26);

    //更新点云窗口
    qvtkWidget->update ();

    //为QTreeWidget设置水平滚动条
    ui->dataTree->header()->setSectionResizeMode(QHeaderView::ResizeToContents);

    ui->dataTree->header()->setStretchLastSection(false);


    //信号和槽函数的连接
    connect(ui->startButton, &QPushButton::clicked, this, &MainWindow::start);//开始按钮

    connect(ui->closeButton, &QPushButton::clicked, this, &QMainWindow::close);//关闭窗口按钮

    connect(ui->down_sampleButton, &QPushButton::clicked, this, &MainWindow::downSample);//降采样按钮

    connect(ui->outliers_filterButton, &QPushButton::clicked, this, &MainWindow::outliersFilter);//离群点滤波按钮

    connect(ui->segButton, &QPushButton::clicked, this, &MainWindow::regionSegmentation);//分割算法按钮

    connect(ui->actionopen, &QAction::triggered, this, &MainWindow::addDataFiles);//菜单栏打开文件按钮

    connect(ui->dataTree, &QTreeWidget::itemDoubleClicked, this, &MainWindow::onDataTreeDoubleClicked);//双击打开文件

}
MainWindow::~MainWindow()
{
    delete ui;
}

//开始显示点云槽函数
void MainWindow::start()
{
    QString path = QFileDialog::getOpenFileName(this, "打开文件", "D:\\software_my\\CloudPoint\\example", tr("PointCloud(*.ply)"));

    //qDebug()<<path.toUtf8().data();

    std::string file_path = path.toStdString();

    pcl::io::loadPLYFile(file_path, *cloud);

    viewer->addPointCloud(cloud, "source_cloud");//添加点云

    viewer->addCoordinateSystem(1.0, "cloud", 0);//添加坐标系

    qvtkWidget->update();

    getProperties(cloud);
}

//降采样
void MainWindow::downSample()
{
    pcl::PCLPointCloud2::Ptr voxel_cloud (new pcl::PCLPointCloud2());//点云数据指针

    pcl::PCLPointCloud2::Ptr voxel_cloud_filtered (new pcl::PCLPointCloud2());

    pcl::toPCLPointCloud2(*cloud, *voxel_cloud);//点云数据类型转换

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;//滤波器对象

    sor.setInputCloud(voxel_cloud);

    sor.setLeafSize(0.01f, 0.01f, 0.01f);//边长为1厘米的体素点

    sor.filter(*voxel_cloud_filtered);//过滤，过滤后的点云存储到voxl_cloud_filtered中

    viewer->removeAllPointClouds();//清楚窗口点云

    pcl::fromPCLPointCloud2(*voxel_cloud_filtered, *cloud);//类型转换

    viewer->addPointCloud(cloud, "down_sample_cloud");//添加新点云

    qvtkWidget->update();//更新窗口内容
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

//添加点云文件
void MainWindow::addDataFiles()
{
    QString dir_name = QFileDialog::getExistingDirectory(this, "打开目录", "D:\\software_my", QFileDialog::ShowDirsOnly);//获取路径名QString类型

    QDir dir(dir_name);//路径名，QDir类型

    QString file_name;//文件名

    QStringList filters;//过滤器

    QStringList root_path_name(dir_name);//路径名，QStringList类型

    filters<<"*.ply";//文件后缀过滤

    dir.setNameFilters(filters);//设置过滤器

    QFileInfoList fileInfoList = dir.entryInfoList();//筛选文件

    //ui->dataTree->clear();

    QTreeWidgetItem* root_path = new QTreeWidgetItem(ui->dataTree, root_path_name);//根节点（路径名）

    ui->dataTree->addTopLevelItem(root_path);

    for(int inf = 0; inf < fileInfoList.count(); inf++)//添加文件循环
    {
        file_name= fileInfoList.at(inf).fileName();//获取文件名

       // qDebug()<<file_name.toUtf8().data();

        QTreeWidgetItem* point_file_name = new QTreeWidgetItem(QStringList()<<file_name);//创建子项，使用读取到的名字初始化

        root_path->addChild(point_file_name);
    }
}


//双击文件显示点云
void MainWindow::onDataTreeDoubleClicked()
{
    QString file_name = ui->dataTree->currentItem()->text(0);//获取文件名

    QString file_path = ui->dataTree->currentItem()->parent()->text(0);//获取路径名

    file_path += '/';

    file_path += file_name;//合并成绝对路径

    std::string file_path_s =file_path.toStdString();//转换成string类型

    //qDebug()<<file_path.toUtf8().data();

    //重新显示点云数据
    cloud->clear();

    pcl::io::loadPLYFile(file_path_s, *cloud);

    viewer->removeAllPointClouds();

    viewer->addPointCloud(cloud, "source_cloud");//添加点云

    viewer->addCoordinateSystem(1.0, "cloud", 0);//添加坐标系

    qvtkWidget->update();

    getProperties(cloud);
}

void MainWindow::getProperties(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    ui->propertyTree->clear();//清空当前属性窗口

    int point_sum_num = point_cloud->size();//获取点云总数

    //创建QTreeWidget项目并显示
    QTreeWidgetItem* size = new QTreeWidgetItem(ui->propertyTree, QStringList()<<"Point"<<QString::number(point_sum_num));

    ui->propertyTree->addTopLevelItem(size);
}

//区域生长算法分割
void MainWindow::regionSegmentation()
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//创建kd树对象，方便算法索引点云

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);//法向量存储器

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;//法向量计算对象

    normal_estimator.setSearchMethod(tree);//设置搜索方法

    normal_estimator.setInputCloud(cloud);//添加点云

    normal_estimator.setKSearch(50);//设置法向量估计邻近点数量

    normal_estimator.compute(*normals);//计算法向量并存储进法向量存储器


    pcl::IndicesPtr indices (new std::vector<int>);//索引容器

    pcl::removeNaNFromPointCloud(*cloud, *indices);//去除没有坐标的点云


    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> region;//区域生长算法对象

    region.setMinClusterSize(50);//类点数下限

    region.setMaxClusterSize(100000);//类点数上限

    region.setSearchMethod(tree);//设置搜索方法

    region.setNumberOfNeighbours(30);//设置邻点数

    region.setInputCloud(cloud);//添加点云

    region.setIndices(indices);//添加索引，由去除无坐标点云函数得到

    region.setInputNormals(normals);//添加法向量

    region.setSmoothnessThreshold(3.0 / 180.0 * M_PI);//设置聚类角度阈值

    region.setCurvatureThreshold(1.0);//设置区域生长起始点曲率阈值


    std::vector<pcl::PointIndices> clusters;//分割后的索引

    region.extract(clusters);//提取索引


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = region.getColoredCloud();//为点云上色

    colored_point = colored_cloud;

    //更新点云显示窗口
    viewer->removeAllPointClouds();

    viewer->addPointCloud(colored_point, "source_cloud");//添加点云

    viewer->addCoordinateSystem(1.0, "cloud", 0);//添加坐标系

    qvtkWidget->update();

    getProperties(cloud);

}

