#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QList>
#include <QMap>
#include <QCheckBox>
#include <QTimer>
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

    void on_sld_clus_res_valueChanged(int value);

    void on_sld_clus_mn_size_valueChanged(int value);

    void on_sld_clus_mx_size_valueChanged(int value);

    void on_rbtn_clust_pcl_toggled(bool checked);

    void on_rbtn_clust_my_toggled(bool checked);

    void UpdatePCL();

private:
    Ui::MainWindow *ui;

    const QString default_directory_, default_pcd_file_;
    const double camera_pos_;
    QList<QCheckBox*> stage_chkbtns;
    QList<QMap<QString, QWidget*>> stage_controls_;
    QTimer *timer_;

    void SetDirectories();

    pcl::visualization::PCLVisualizer::Ptr pcl_viewer_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
    std::unique_ptr<ProcessPointClouds<pcl::PointXYZI>> pcl_processor_;

    void renderPointCloud(Color color);
    void renderPointCloud();
    void renderPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string name, Color color);
    void renderBox(Box& box, std::string name, Color color={1,0,0}, float opacity=1.0f);
    void ProcessChain();
    void SetButtonStage(QList<QMap<QString, QWidget*>>::iterator i);
    std::vector<boost::filesystem::path> stream_;
    std::vector<boost::filesystem::path>::iterator stream_it_;

    struct Pararameters{
      struct Filter{
            int res, min_x, max_x, min_y, max_y, min_z, max_z;
      } filter;
      struct Segmentation{
          int max_iter, threshold;
      } segmenation;
      struct Clusterisation{
          int res, mn_size, mx_size;
      } clusterisation;
    } parameters_;
};
#endif // MAINWINDOW_H
