#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QPushButton>
#include <QString>
#include <QFileDialog>
#include <QStringList>
#include <QFileInfoList>
#include <QTreeWidget>//树形结构头文件
#include <QTreeWidgetItem>
#include <QDir>
#include <QDebug>
#include <QVBoxLayout>//布局头文件
#include <QHeaderView>//水平滚动条

#include "vtkRenderWindow.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
   // point_cloud_method_ui = new PointCloudMethod(ui);

    //将点云显示窗口与点云显示指针连接起来
    qvtkWidget = new QVTKWidget(ui->viewerWidget);

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());//点云指针初始化

    colored_point.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

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

    connect(ui->passthroughButton, &QPushButton::clicked, this, &MainWindow::passThrough);//直通滤波

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

//打开点云文件
void MainWindow::start()
{
    QString path = QFileDialog::getOpenFileName(this, "打开文件", "D:\\software_my\\CloudPoint\\example", tr("PointCloud(*.ply *.pcd)"));

    //qDebug()<<path.toUtf8().data();

    processor.loadPointCloudFile(path, cloud);

    viewer->addPointCloud(cloud, "source_cloud");//添加点云

    viewer->addCoordinateSystem(1.0, "cloud", 0);//添加坐标系

    qvtkWidget->update();

    getProperties(cloud);
}

//添加点云文件
void MainWindow::addDataFiles()
{
    QString dir_name = QFileDialog::getExistingDirectory(this, "打开目录", "D:\\software_my", QFileDialog::ShowDirsOnly);//获取路径名QString类型

    QDir dir(dir_name);//路径名，QDir类型

    QString file_name;//文件名

    QStringList filters;//过滤器

    QStringList root_path_name(dir_name);//路径名，QStringList类型

    filters<<"*.ply"<<"*.pcd";//文件后缀过滤

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

    //重新显示点云数据
    cloud->clear();

    processor.loadPointCloudFile(file_path, cloud);

    viewer->removeAllPointClouds();

    viewer->addPointCloud(cloud, "source_cloud");//添加点云

    viewer->addCoordinateSystem(1.0, "cloud", 0);//添加坐标系

    qvtkWidget->update();

    getProperties(cloud);
}

//获取点云属性
void MainWindow::getProperties(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    ui->propertyTree->clear();//清空当前属性窗口

    int point_sum_num = point_cloud->size();//获取点云总数

    //创建QTreeWidget项目并显示
    QTreeWidgetItem* size = new QTreeWidgetItem(ui->propertyTree, QStringList()<<"Point"<<QString::number(point_sum_num));

    ui->propertyTree->addTopLevelItem(size);
}

void MainWindow::getProperties(pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud)
{
    ui->propertyTree->clear();//清空当前属性窗口

    int point_sum_num = colored_cloud->size();//获取点云总数

    QTreeWidgetItem* size = new QTreeWidgetItem(ui->propertyTree, QStringList()<<"Point"<<QString::number(point_sum_num));

    ui->propertyTree->addTopLevelItem(size);
}

//设置按钮不可按
void MainWindow::setButtonDisabled()
{
    ui->startButton->setDisabled(true);

    ui->passthroughButton->setDisabled(true);

    ui->down_sampleButton->setDisabled(true);

    ui->outliers_filterButton->setDisabled(true);

    ui->segButton->setDisabled(true);
}

//开启按钮功能
void MainWindow::setButtonEnabled()
{
    ui->startButton->setDisabled(false);

    ui->passthroughButton->setDisabled(false);

    ui->down_sampleButton->setDisabled(false);

    ui->outliers_filterButton->setDisabled(false);

    ui->segButton->setDisabled(false);
}

//区域生长算法分割
void MainWindow::regionSegmentation()
{
//    processor.regionSegmentation(cloud, colored_point);

//    getProperties(colored_point);

//    qDebug()<<"分割点云窗口更新";
//    //更新点云显示窗口
//    viewer->removeAllPointClouds();
//    qDebug()<<"1";

//    viewer->addPointCloud(colored_point, "colored_cloud");//添加点云
//    qDebug()<<"2";

//    qvtkWidget->update();
//    qDebug()<<"3";

    pcl::PointCloud<pcl::PointXYZ>::Ptr *source_cloud = &cloud;//加载点云的指针

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr *colored_cloud = &colored_point;//带颜色的点云

    qDebug()<<"111111";

    cloud_process_thread.regionSegmentationThread(source_cloud, colored_cloud);

    connect(this, &MainWindow::startRegionSegmentationSig, &cloud_process_thread, &ProThread::emitRegionSegmentationSig);

    connect(&cloud_process_thread.region_seg_thread, &QThread::started, this, &MainWindow::setButtonDisabled);

    connect(&cloud_process_thread.region_seg_thread, &QThread::finished, this, [=]()
    {
        qDebug()<<"分割点云窗口更新";
        //更新点云显示窗口
        viewer->removeAllPointClouds();
        qDebug()<<"1";

        viewer->addPointCloud(colored_point, "colored_cloud");//添加点云
        qDebug()<<"2";

        qvtkWidget->update();
        qDebug()<<"3";

        setButtonEnabled();
    });

    emit startRegionSegmentationSig();
}

//降采样
void MainWindow::downSample()
{
    cloud_process_thread.downSampleThread(cloud);

    connect(this, &MainWindow::startDownSampleSig, &cloud_process_thread, &ProThread::emitDownSampleSig);

    connect(&cloud_process_thread.down_sample_thread, &QThread::started, this, &MainWindow::setButtonDisabled);

    connect(&cloud_process_thread.down_sample_thread, &QThread::finished, this, [=]()
    {
        viewer->removeAllPointClouds();//窗口显示处理

        viewer->addPointCloud(cloud, "down_sample");

        qvtkWidget->update();

        getProperties(cloud);

        setButtonEnabled();

        qDebug()<<"55555";
    });

    emit startDownSampleSig();
}

//离群点过滤
void MainWindow::outliersFilter()
{
    //processor.outliersFilter(cloud);//离群点滤波
    qDebug()<<"000000000";

    cloud_process_thread.outliersFilterThread(cloud);

    connect(this, &MainWindow::startOutliersFilterSig, &cloud_process_thread, &ProThread::emitOutliersFilterSig);

    connect(&cloud_process_thread.outliers_filter_thread, &QThread::started, this, &MainWindow::setButtonDisabled);

    connect(&cloud_process_thread.outliers_filter_thread, &QThread::finished, this, [=]()
    {
        qDebug()<<"离群点滤波完成";

        viewer->removeAllPointClouds();//窗口显示处理

        viewer->addPointCloud(cloud, "outliers_removal");

        qvtkWidget->update();

        setButtonEnabled();

        getProperties(cloud);
    });

    emit startOutliersFilterSig();
}

//直通滤波
void MainWindow::passThrough()
{
    processor.passThrough(cloud);

    viewer->removeAllPointClouds();

    viewer->addPointCloud(cloud, "passThrough_cloud");

    qvtkWidget->update();

    getProperties(cloud);
}
