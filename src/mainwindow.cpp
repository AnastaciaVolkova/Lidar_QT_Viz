#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>
#include <QFileInfo>
#include <pcl/io/pcd_io.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkOpenGLRenderer.h>

using namespace pcl;
using namespace std;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , default_directory_(QDir(QCoreApplication::applicationDirPath()).relativeFilePath("../data/pcd/data_1/"))
    , default_pcd_file_(QDir(QCoreApplication::applicationDirPath()).relativeFilePath("../src/simpleHighway.pcd"))
    , cloud_(new pcl::PointCloud<pcl::PointXYZI>)
{
    ui->setupUi(this);

    SetDirectories();

    QDir cur = QDir(QCoreApplication::applicationDirPath());

    vtkNew<vtkOpenGLRenderer> r;

    pcl_viewer_.reset(new pcl::visualization::PCLVisualizer(r, ui->vtk_widget->renderWindow(), "3d view of point cloud", false));

    cloud_ = pcl_processor->loadPcd(ui->le_input->text().toStdString());
    pcl_viewer_->setupInteractor(ui->vtk_widget->interactor(), ui->vtk_widget->renderWindow());
    ui->vtk_widget->update();
    renderPointCloud(ui->le_input->text().toStdString());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::SetDirectories(){
    if (ui->rbtn_is_multi->isChecked())
        ui->le_input->setText(default_directory_);
    else
        ui->le_input->setText(default_pcd_file_);
    QFileInfo file_info(ui->le_input->text());
    if (!file_info.exists())
        ui->le_input->setText("");
}


void MainWindow::on_chkbox_filter_stateChanged(int arg1)
{
    if (ui->chkbox_filter->isChecked()){
        ui->chkbox_seg->setEnabled(true);
        ui->chkbox_clust->setEnabled(true);
        ui->chkbox_box->setEnabled(true);
    } else {
        ui->chkbox_seg->setEnabled(false);
        ui->chkbox_clust->setEnabled(false);
        ui->chkbox_box->setEnabled(false);
    }
}

void MainWindow::on_chkbox_seg_stateChanged(int arg1)
{
    if (ui->chkbox_seg->isChecked()){
        ui->chkbox_clust->setEnabled(true);
        ui->chkbox_box->setEnabled(true);
    } else {
        ui->chkbox_clust->setEnabled(false);
        ui->chkbox_box->setEnabled(false);
    }
}

void MainWindow::on_chkbox_clust_stateChanged(int arg1)
{
    if (ui->chkbox_clust->isChecked()){
        ui->chkbox_box->setEnabled(true);
    } else {
        ui->chkbox_box->setEnabled(false);
    }
}

void MainWindow::on_btn_input_pressed()
{
    QString file_dir = ui->le_input->text();
    QFileInfo file_info(file_dir);

    if (!ui->rbtn_is_multi->isChecked()){
        if (!file_info.exists())
            file_dir = default_pcd_file_;
        QString file_name = QFileDialog::getOpenFileName(this,
                                                         tr("Choose pcd file"),
                                                         file_dir, tr("Point cloud data (*.pcd)"));
        ui->le_input->setText(file_name);
    } else {
        if (!file_info.exists())
            file_dir = default_directory_;
        QString directory = QFileDialog::getExistingDirectory(this,
                                                              tr("Choose directory with pcf files"),
                                                              file_dir,
                                                              QFileDialog::ShowDirsOnly);
        ui->le_input->setText(directory);
    }
    cloud_ = pcl_processor->loadPcd(ui->le_input->text().toStdString());

    renderPointCloud(ui->le_input->text().toStdString());
}

void MainWindow::on_rbtn_is_multi_toggled(bool checked)
{
    if (checked)
        ui->le_input->setText(default_directory_);
    else
        ui->le_input->setText(default_pcd_file_);
}


void MainWindow::renderPointCloud(Color color, string name){
    pcl_viewer_->removeAllPointClouds();
    pcl_viewer_->addPointCloud<pcl::PointXYZI> (cloud_, name);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    pcl_viewer_->spin();
};


void MainWindow::renderPointCloud(string name){
    pcl_viewer_->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_,"intensity");
    pcl_viewer_->addPointCloud<pcl::PointXYZI>(cloud_, intensity_distribution, name);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    pcl_viewer_->spin();
};

void MainWindow::on_rbtn_show_intensity_toggled(bool checked)
{
    if (checked)
        renderPointCloud(ui->le_input->text().toStdString());
    else
        renderPointCloud({0,1,0}, ui->le_input->text().toStdString());
}
