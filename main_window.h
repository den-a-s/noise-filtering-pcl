#pragma once

#include <QMainWindow>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void loadCloud();
    void saveCloud();
    /**
     * @brief Восстанавливает камеру относительно облака точек
     */
    void resetCamera();

    void SOR();
    void ROR();
    void medianFilter();
    void voxelGrid();
    void MLS();

    void plug();

    void pointsReset();

private:

    void showEvent(QShowEvent *event) override;

    void updateViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& id);

    std::unique_ptr<Ui::MainWindow> ui;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;

    pcl::visualization::PCLVisualizer::Ptr viewer;
};
