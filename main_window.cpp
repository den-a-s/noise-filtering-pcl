#include "main_window.h"
#include "ui_main_window.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include <QDialog>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>

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

    // === Создание модального диалога ===
    QDialog dialog(this);
    dialog.setWindowTitle("Set SOR Parameters");

    QFormLayout *formLayout = new QFormLayout(&dialog);

    QSpinBox *meanKSpin = new QSpinBox(&dialog);
    meanKSpin->setRange(1, 1000);
    meanKSpin->setValue(50);

    QDoubleSpinBox *stddevSpin = new QDoubleSpinBox(&dialog);
    stddevSpin->setDecimals(2);
    stddevSpin->setRange(0.01, 10.0);
    stddevSpin->setValue(1.0);

    formLayout->addRow("Number of nearest neighbors:", meanKSpin);
    formLayout->addRow("Standard deviation multiplier:", stddevSpin);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    formLayout->addWidget(buttonBox);

    QObject::connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    QObject::connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted)
        return;

    // === Получение значений и применение фильтра ===
    int meanK = meanKSpin->value();
    double stddev = stddevSpin->value();

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddev);

    filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered);

    updateViewer(filtered, "filtered");

    QMessageBox::information(this, "Filtering", "SOR finished.");
}


void MainWindow::ROR() {
    if (!cloud || cloud->empty()) {
        QMessageBox::warning(this, "Error", "Uploaded point cloud.");
        return;
    }

    // === Создание модального диалога ===
    QDialog dialog(this);
    dialog.setWindowTitle("Set ROR Parameters");

    QFormLayout *formLayout = new QFormLayout(&dialog);

    QSpinBox *minNeighborSpin = new QSpinBox(&dialog);
    minNeighborSpin->setRange(1, 1000);
    minNeighborSpin->setValue(10);

    QDoubleSpinBox *radiusSearchSpin = new QDoubleSpinBox(&dialog);
    radiusSearchSpin->setDecimals(3);
    radiusSearchSpin->setRange(0.001, 100.0);
    radiusSearchSpin->setValue(5);

    formLayout->addRow("Min neighbors in radius:", minNeighborSpin);
    formLayout->addRow("Radius search:", radiusSearchSpin);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    formLayout->addWidget(buttonBox);

    QObject::connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    QObject::connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted)
        return;

    // === Получение значений и применение фильтра ===
    int minNeighbor = minNeighborSpin->value();
    double radiusSearch = radiusSearchSpin->value();

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMinNeighborsInRadius(minNeighbor);
    sor.setRadiusSearch(radiusSearch);

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

    // === Создание модального диалога ===
    QDialog dialog(this);
    dialog.setWindowTitle("Set Median Parameters");

    QFormLayout *formLayout = new QFormLayout(&dialog);

    QSpinBox *windowSizeSpin = new QSpinBox(&dialog);
    windowSizeSpin->setRange(1, 100);
    windowSizeSpin->setValue(5);

    QDoubleSpinBox *maxMvmtSpin = new QDoubleSpinBox(&dialog);
    maxMvmtSpin->setDecimals(3);
    maxMvmtSpin->setRange(0.001, 100.0);
    maxMvmtSpin->setValue(0.1);

    formLayout->addRow("Window size(odd):", windowSizeSpin);
    formLayout->addRow("Max allowed movement:", maxMvmtSpin);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    formLayout->addWidget(buttonBox);

    QObject::connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    QObject::connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted)
        return;

    // === Получение значений и применение фильтра ===
    int windowSize = windowSizeSpin->value();
    double maxMvmt = maxMvmtSpin->value();

    pcl::MedianFilter<pcl::PointXYZ> median_filter;
    median_filter.setInputCloud(cloud);
    median_filter.setWindowSize(windowSize);
    median_filter.setMaxAllowedMovement(maxMvmt);

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

    // === Создание модального диалога ===
    QDialog dialog(this);
    dialog.setWindowTitle("Set Voxel Grid Parameters");

    QFormLayout *formLayout = new QFormLayout(&dialog);

    QDoubleSpinBox *leafSizeSpin = new QDoubleSpinBox(&dialog);
    leafSizeSpin->setDecimals(3);
    leafSizeSpin->setRange(0.001, 100.0);
    leafSizeSpin->setValue(0.1);

    formLayout->addRow("Voxel leaf size(cube):", leafSizeSpin);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    formLayout->addWidget(buttonBox);

    QObject::connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    QObject::connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted)
        return;

    // === Получение значений и применение фильтра ===
    double leafSize = leafSizeSpin->value();

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    // Воксели кубы, для упрощения
    filter.setLeafSize(leafSize, leafSize, leafSize);

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

    // === Создание модального диалога ===
    QDialog dialog(this);
    dialog.setWindowTitle("Set MLS (sample local plane) Parameters");

    QFormLayout *formLayout = new QFormLayout(&dialog);

    QDoubleSpinBox *radiusSearchSpin = new QDoubleSpinBox(&dialog);
    radiusSearchSpin->setDecimals(3);
    radiusSearchSpin->setRange(0.001, 100.0);
    radiusSearchSpin->setValue(5);

    QSpinBox *polyOrderSpin = new QSpinBox(&dialog);
    polyOrderSpin->setRange(1, 5);
    polyOrderSpin->setValue(2);

    QDoubleSpinBox *upsampleRadiusSpin = new QDoubleSpinBox(&dialog);
    upsampleRadiusSpin->setDecimals(3);
    upsampleRadiusSpin->setRange(0.001, 100.0);
    upsampleRadiusSpin->setValue(0.1);

    QDoubleSpinBox *upsampleStepSizeSpin = new QDoubleSpinBox(&dialog);
    upsampleStepSizeSpin->setDecimals(3);
    upsampleStepSizeSpin->setRange(0.001, 100.0);
    upsampleStepSizeSpin->setValue(0.1);

    formLayout->addRow("Radius search:", radiusSearchSpin);
    formLayout->addRow("Polynom order:", polyOrderSpin);
    formLayout->addRow("Upsampling radius:", upsampleRadiusSpin);
    formLayout->addRow("Upsampling step size:", upsampleStepSizeSpin);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    formLayout->addWidget(buttonBox);

    QObject::connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    QObject::connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted)
        return;

    // === Получение значений и применение фильтра ===
    double radiusSearch = radiusSearchSpin->value();
    int polyOrder = polyOrderSpin->value();
    double upsampleRadius = upsampleRadiusSpin->value();
    double upsampleStepSize = upsampleStepSizeSpin->value();

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);

    filter.setSearchRadius(radiusSearch);                   // Радиус поиска соседей
    filter.setPolynomialOrder(polyOrder);                   // Степень полинома (1 - линейный, 2 - квадратичный)
    filter.setUpsamplingMethod(filter.NONE);  // Метод апсемплинга
    // filter.setUpsamplingRadius(upsampleRadius);             // Радиус для апсемплинга (если нужно)
    // filter.setUpsamplingStepSize(upsampleStepSize);         // Шаг апсемплинга

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
