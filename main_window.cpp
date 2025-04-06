#include "main_window.h"
#include "ui_main_window.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow),
    cloud(new pcl::PointCloud<pcl::PointXYZ>),
    filtered(new pcl::PointCloud<pcl::PointXYZ>)
{
    ui->setupUi(this);

    connect(ui->actionLoadPCD, &QAction::triggered, this, &MainWindow::loadCloud);
    connect(ui->actionSaveCurrPointCloud, &QAction::triggered, this, &MainWindow::saveCloud);
    connect(ui->actionCameraReset, &QAction::triggered, this, &MainWindow::resetCamera);

    connect(ui->btn_sor, &QPushButton::clicked, this, &MainWindow::SOR);
    connect(ui->btn_ror, &QPushButton::clicked, this, &MainWindow::ROR);
    connect(ui->btn_median, &QPushButton::clicked, this, &MainWindow::medianFilter);
    connect(ui->btn_voxel_grid, &QPushButton::clicked, this, &MainWindow::voxelGrid);
    connect(ui->btn_mls, &QPushButton::clicked, this, &MainWindow::MLS);

    connect(ui->btn_mor, &QPushButton::clicked, this, &MainWindow::plug);

    connect(ui->btn_ssn, &QPushButton::clicked, this, &MainWindow::plug);
    connect(ui->btn_shadow_points, &QPushButton::clicked, this, &MainWindow::plug);

    connect(ui->btn_reset_points, &QPushButton::clicked, this, &MainWindow::pointsReset);

    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    ui->qvtkWidget->setRenderWindow(viewer->getRenderWindow());
}

MainWindow::~MainWindow() {
}

void MainWindow::loadCloud() {
    QString filename = QFileDialog::getOpenFileName(this, "Открыть PCD файл", "", "*.pcd");
    if (filename.isEmpty()) return;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename.toStdString(), *cloud) == -1) {
        QMessageBox::warning(this, "Error", "Failed to upload PSD file.");
        return;
    }

    updateViewer(cloud, "original");
    // Новое облако может быть в других координатах
    resetCamera();
    QMessageBox::information(this, "Done", QString("PointCloud from \"%1\" uploaded.").arg(filename) );
}

void MainWindow::SOR() {
    if (!cloud || cloud->empty()) {
        QMessageBox::warning(this, "Error", "Uploaded point cloud.");
        return;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);

    sor.filter(*filtered);

    updateViewer(filtered, "filtered");
    QMessageBox::information(this, "Filtering", "SOR finished.");
}

void MainWindow::ROR() {
    if (!cloud || cloud->empty()) {
        QMessageBox::warning(this, "Error", "Uploaded point cloud.");
        return;
    }

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMinNeighborsInRadius(10);
    sor.setRadiusSearch(5.0);

    sor.filter(*filtered);

    updateViewer(filtered, "filtered");
    QMessageBox::information(this, "Filtering", "ROR finished.");
}

void MainWindow::medianFilter()
{
    if (!cloud || cloud->empty()) {
        QMessageBox::warning(this, "Error", "Uploaded point cloud.");
        return;
    }

    if(!cloud->isOrganized())
    {
        QMessageBox::warning(this, "Error", "Point cloud is not organized.");
        return;
    }

    pcl::MedianFilter<pcl::PointXYZ> median_filter;
    median_filter.setInputCloud(cloud);
    median_filter.setWindowSize(5);
    median_filter.setMaxAllowedMovement(0.1f);

    median_filter.filter(*filtered);

    updateViewer(filtered, "filtered");
    QMessageBox::information(this, "Filtering", "Median filter finished.");
}

void MainWindow::voxelGrid()
{
    if (!cloud || cloud->empty()) {
        QMessageBox::warning(this, "Error", "Uploaded point cloud.");
        return;
    }

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(2.f, 2.f, 2.f);

    filter.filter(*filtered);

    updateViewer(filtered, "filtered");
    std::cout << *cloud << std::endl;
    std::cout << *filtered << std::endl;
    QMessageBox::information(this, "Filtering",
                            QString{"Voxel grid finished."});
}

void MainWindow::MLS()
{
    if (!cloud || cloud->empty()) {
        QMessageBox::warning(this, "Error", "Uploaded point cloud.");
        return;
    }

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);

    filter.setSearchRadius(5);                   // Радиус поиска соседей
    filter.setPolynomialOrder(2);                   // Степень полинома (1 - линейный, 2 - квадратичный)
    filter.setUpsamplingMethod(filter.SAMPLE_LOCAL_PLANE); // Метод апсемплинга
    filter.setUpsamplingRadius(0.1);               // Радиус для апсемплинга (если нужно)
    filter.setUpsamplingStepSize(0.1);            // Шаг апсемплинга

    filter.process(*filtered);

    updateViewer(filtered, "filtered");
    std::cout << *cloud << std::endl;
    std::cout << *filtered << std::endl;
    QMessageBox::information(this, "Filtering",
                            QString{"MLS finished."}
                            );
}

void MainWindow::plug()
{
    QMessageBox::warning(this, "Warning", "This function has no implementation.");
}

void MainWindow::pointsReset()
{
    updateViewer(cloud, "original");
    filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    QMessageBox::information(this, "Reset", "Point cloud has been restored.");
}

void MainWindow::saveCloud() {
    if (!filtered || filtered->empty()) {
        QMessageBox::warning(this, "Error", "Only filtered clouds are saved. Please apply filter.");
        return;
    }

    QString filename = QFileDialog::getSaveFileName(this, "Save filtered point cloud", "", "*.pcd");
    if (filename.isEmpty()) return;

    pcl::io::savePCDFileBinary(filename.toStdString(), *filtered);
    QMessageBox::information(this, "Saved", QString{"File \"%1\" saved."}.arg(filename));
}

void MainWindow::resetCamera()
{
    viewer->resetCamera();
    ui->qvtkWidget->update();
}

void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);
    viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
}

void MainWindow::updateViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& id) {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color, id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
    ui->qvtkWidget->update();
}
