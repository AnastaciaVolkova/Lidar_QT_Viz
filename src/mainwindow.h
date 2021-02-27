#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QList>
#include <QCheckBox>
#include <memory>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include "processPointClouds.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

struct Color{float r, g, b; };

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_rbtn_is_multi_toggled(bool checked);

    void on_btn_input_pressed();

    void on_chkbox_show_intensity_toggled(bool checked);

    void on_chkbox_filter_toggled(bool checked);

    void on_chkbox_seg_toggled(bool checked);

    void on_chkbox_clust_toggled(bool checked);

    void on_chkbox_box_toggled(bool checked);

    void on_sld_min_x_valueChanged();

    void on_sld_min_y_valueChanged();

    void on_sld_min_z_valueChanged();

    void on_sld_max_x_valueChanged();

    void on_sld_max_y_valueChanged();

    void on_sld_max_z_valueChanged();

    void on_sld_filter_res_valueChanged();

    void on_btn_apply_clicked();

    void on_sld_dist_thr_valueChanged(int value);

    void on_sld_max_iter_valueChanged(int value);

private:
    Ui::MainWindow *ui;

    const QString default_directory_, default_pcd_file_;
    QList<QCheckBox*> stage_chkbtns;

    void SetDirectories();

    pcl::visualization::PCLVisualizer::Ptr pcl_viewer_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
    std::unique_ptr<ProcessPointClouds<pcl::PointXYZI>> pcl_processor;

    void renderPointCloud(Color color);
    void renderPointCloud();
    void renderPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string name, Color color);
    void renderBox(Box& box, std::string name, Color color={1,0,0}, float opacity=1.0f);
    void ProcessChain();
    void SetButtonStage(QList<QCheckBox*>::iterator i);
};
#endif // MAINWINDOW_H
