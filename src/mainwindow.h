#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_rbtn_is_multi_toggled(bool checked);

private slots:
    void on_btn_input_pressed();

private slots:
    void on_chkbox_clust_stateChanged(int arg1);

private slots:
    void on_chkbox_seg_stateChanged(int arg1);

private slots:
    void on_chkbox_filter_stateChanged(int arg1);

private:
    Ui::MainWindow *ui;

    const QString default_directory_, default_pcd_file_;

    void SetDirectories();
};
#endif // MAINWINDOW_H
