#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "myglwidget.h"
#include "utils.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void SignalDrawAxis(int arg);
    void SignalPointSize(double arg);
    void SignalCloudNow(std::string data_path);

private slots:
    void on_pushButtonLoad_clicked();
    void on_checkBox_toggled(bool checked);
    void on_doubleSpinBoxPointSize_valueChanged(double arg1);
    void on_spinBoxCloudNo_valueChanged(int arg1);

private:
    Ui::MainWindow *ui;
    QString data_dir_qstr;
    std::string data_dir_str;
    std::vector<std::string> data_paths;
    bool data_loaded;
    int data_num;

    bool onDataDirectorySet(std::string data_dir);

};
#endif // MAINWINDOW_H
