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
    , default_pcd_file_(QDir(QCoreApplication::applicationDirPath()).relativeFilePath("../src/example.pcd"))
    , cloud_(new pcl::PointCloud<pcl::PointXYZI>)
{
    ui->setupUi(this);
    stage_chkbtns.push_back(ui->chkbox_filter);
    stage_chkbtns.push_back(ui->chkbox_seg);
    stage_chkbtns.push_back(ui->chkbox_clust);
    stage_chkbtns.push_back(ui->chkbox_box);

    stage_controls_.push_back({{"chkbox", ui->chkbox_filter}, {"tab", ui->filter_params_widget}});
    stage_controls_.push_back({{"chkbox", ui->chkbox_seg}, {"tab", ui->seg_params_widget}});
    stage_controls_.push_back({{"chkbox", ui->chkbox_clust}, {"tab", ui->clus_params_widget}});
    stage_controls_.push_back({{"chkbox", ui->chkbox_box}});

    SetDirectories();

    QDir cur = QDir(QCoreApplication::applicationDirPath());

    vtkNew<vtkOpenGLRenderer> r;

    pcl_viewer_.reset(new pcl::visualization::PCLVisualizer(r, ui->vtk_widget->renderWindow(), "3d view of point cloud", false));

    cloud_ = pcl_processor->loadPcd(ui->le_input->text().toStdString());
    pcl_viewer_->setupInteractor(ui->vtk_widget->interactor(), ui->vtk_widget->renderWindow());
    ui->vtk_widget->update();
    if (ui->chkbox_show_intensity->isChecked())
        renderPointCloud();
    else
        ProcessChain();
}

MainWindow::~MainWindow()
{
    delete ui;
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

    renderPointCloud();
}

void MainWindow::on_chkbox_show_intensity_toggled(bool checked)
{
    if (checked){
        ui->grpbox_proc_stages->setEnabled(false);
        renderPointCloud();
    }
    else {
        ui->grpbox_proc_stages->setEnabled(true);
        ProcessChain();
    }
}

void MainWindow::on_rbtn_is_multi_toggled(bool checked)
{
    if (checked)
        ui->le_input->setText(default_directory_);
    else
        ui->le_input->setText(default_pcd_file_);
}

void MainWindow::on_rbtn_clust_pcl_toggled(bool checked)
{
    ui->btn_apply->setEnabled(true);
}


void MainWindow::on_rbtn_clust_my_toggled(bool checked)
{
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_chkbox_filter_toggled(bool checked)
{
    SetButtonStage(stage_controls_.begin());
}

void MainWindow::on_chkbox_seg_toggled(bool checked)
{
    SetButtonStage(stage_controls_.begin()+1);
}

void MainWindow::on_chkbox_clust_toggled(bool checked)
{
    SetButtonStage(stage_controls_.begin()+2);
}

void MainWindow::on_chkbox_box_toggled(bool checked)
{
    SetButtonStage(stage_controls_.begin()+3);
}

void MainWindow::renderPointCloud(Color color){
    if (!pcl_viewer_->contains(ui->le_input->text().toStdString()))
        pcl_viewer_->addPointCloud<pcl::PointXYZI> (cloud_, ui->le_input->text().toStdString());
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, ui->le_input->text().toStdString());
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ui->le_input->text().toStdString());
    pcl_viewer_->spin();
};

void MainWindow::renderPointCloud(){
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_,"intensity");
    if (!pcl_viewer_->contains(ui->le_input->text().toStdString()))
        pcl_viewer_->addPointCloud<pcl::PointXYZI>(cloud_, intensity_distribution, ui->le_input->text().toStdString());
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ui->le_input->text().toStdString());
    pcl_viewer_->spin();
};

void MainWindow::renderPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, string name, Color color){
    if (!pcl_viewer_->contains(name))
        pcl_viewer_->addPointCloud<pcl::PointXYZI>(cloud, name);

    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    pcl_viewer_->spin();
};

void MainWindow::renderBox(Box& box, string name, Color color, float opacity)
{
	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;

    if (pcl_viewer_->contains(name))
        pcl_viewer_->removeShape(name);

    if (pcl_viewer_->contains(name + "_fill"))
        pcl_viewer_->removeShape(name + "_fill");

    pcl_viewer_->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, name);
    pcl_viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name);
    pcl_viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    pcl_viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, name);

    pcl_viewer_->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, name + "_fill");
    pcl_viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
    pcl_viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    pcl_viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, name);
};

void MainWindow::ProcessChain(){
    PointCloud<PointXYZI>::Ptr cloud_proc;
    pcl_viewer_->removeAllPointClouds();
    pcl_viewer_->removeAllShapes();
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> seg_res;
    PointCloud<PointXYZI>::Ptr cloud_to_display = cloud_;
    Color color_to_display = {1, 1, 1};
    if (ui->chkbox_filter->isChecked()){
        cloud_proc = pcl_processor->FilterCloud(cloud_, static_cast<float>(ui->sld_filter_res->value())/100.0f,
        Eigen::Vector4f{static_cast<float>(ui->sld_min_x->value()), static_cast<float>(ui->sld_min_y->value()), static_cast<float>(ui->sld_min_z->value()), 1},
        Eigen::Vector4f{static_cast<float>(ui->sld_max_x->value()), static_cast<float>(ui->sld_max_y->value()), static_cast<float>(ui->sld_max_z->value()), 1});
        cloud_to_display = cloud_proc;
        color_to_display = {0.0f, 1.0f, 1.0f};
    }

    if (ui->chkbox_seg->isChecked()){
        seg_res = pcl_processor->SegmentPlane(
            cloud_proc,
            ui->sld_max_iter->value(),
            static_cast<float>(ui->sld_dist_thr->value())/100.0f);
        cloud_proc = seg_res.first;
        cloud_to_display = cloud_proc;
        color_to_display = {0.0f, 1.0f, 1.0f};
        renderPointCloud(seg_res.second, "plain", {0, 1, 0});
    };

    if (ui->chkbox_clust->isChecked()){
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pcl_processor->Clustering(
            cloud_proc,
            static_cast<float>(ui->sld_clus_res->value())/100.0f,
            ui->sld_clus_mn_size->value(),
            ui->sld_clus_mx_size->value());
        int clusterId = 0;
        std::vector<Color> colors = {
            {65/256.0f, 105/256.0f, 225/256.0f},
            {100/256.0f,149/256.0f,237/256.0f},
            {238/256.0f,130/256.0f,238/256.0f}};
        for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
        {
            std::cout << "cluster size:" << cluster->points.size() << std::endl;
            renderPointCloud(
                cluster,
                ui->le_input->text().toStdString()+std::to_string(clusterId),
                colors[clusterId%colors.size()]);
            if (ui->chkbox_box->isChecked()){
                Box box = pcl_processor->BoundingBox(cluster);
                renderBox(box,"Box"+to_string(clusterId));
            }
            ++clusterId;
        }
    } else {
        renderPointCloud(cloud_to_display, ui->le_input->text().toStdString(), color_to_display);
    }
    ui->btn_apply->setEnabled(false);
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

void MainWindow::SetButtonStage(QList<QMap<QString, QWidget*>>::iterator i){
    if (!static_cast<QCheckBox*>((*i)[QString("chkbox")])->isChecked()){
        while( i<stage_controls_.end() ){
            QCheckBox* cb = static_cast<QCheckBox*>((*i)[QString("chkbox")]);
            QWidget* tab =  i->contains("tab")? (*i)[QString("tab")]: nullptr;
            cb->blockSignals(true);
            cb->setChecked(false);
            cb->blockSignals(false);
            if (tab != nullptr)
                tab->setEnabled(false);
            i++;
        }
    } else {
        while (i >= stage_controls_.begin()){
            QCheckBox* cb = static_cast<QCheckBox*>((*i)[QString("chkbox")]);
            QWidget* tab =  i->contains("tab")? (*i)[QString("tab")]: nullptr;
            cb->blockSignals(true);
            cb->setChecked(true);
            cb->blockSignals(false);
            if (tab != nullptr)
                tab->setEnabled(true);
            i--;
        }
    }

    ProcessChain();
}

void MainWindow::on_sld_min_x_valueChanged()
{
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_sld_min_y_valueChanged()
{
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_sld_min_z_valueChanged()
{
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_sld_max_x_valueChanged()
{
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_sld_max_y_valueChanged()
{
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_sld_max_z_valueChanged()
{
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_sld_filter_res_valueChanged(){
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_sld_dist_thr_valueChanged(int value)
{
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_sld_max_iter_valueChanged(int value)
{
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_sld_clus_res_valueChanged(int value)
{
    ui->btn_apply->setEnabled(true);
}

void MainWindow::on_sld_clus_mn_size_valueChanged(int value){
    ui->btn_apply->setEnabled(true);
};

void MainWindow::on_sld_clus_mx_size_valueChanged(int value){
    ui->btn_apply->setEnabled(true);
};

void MainWindow::on_btn_apply_clicked()
{
    ProcessChain();
}
