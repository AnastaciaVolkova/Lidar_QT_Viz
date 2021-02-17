#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>
#include <QFileInfo>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , default_directory_("../data/pcd/data_1")
    , default_pcd_file_("../src/simpleHighway.pcd")
{
    ui->setupUi(this);
    SetDirectories();
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
}

void MainWindow::on_rbtn_is_multi_toggled(bool checked)
{
    if (checked)
        ui->le_input->setText(default_directory_);
    else
        ui->le_input->setText(default_pcd_file_);
}
