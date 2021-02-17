#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
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
