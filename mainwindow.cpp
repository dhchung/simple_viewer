#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow) {
    ui->setupUi(this);
    data_loaded = false;
    data_num = 0;

    // Connecting callbacks
    connect(this, SIGNAL(SignalDrawAxis(int)), ui->widgetRender, SLOT(SlotDrawAxis(int)));
    connect(this, SIGNAL(SignalPointSize(double)), ui->widgetRender, SLOT(SlotPointSize(double)));
    connect(this, SIGNAL(SignalCloudNow(std::string)), ui->widgetRender, SLOT(SlotCloudNow(std::string)));
}

MainWindow::~MainWindow() {
    delete ui;
}


void MainWindow::on_pushButtonLoad_clicked() {
    // Loading data
    // Select the folder containing binary files of point clouds
    ui->spinBoxCloudNo->setValue(0);

    data_dir_qstr = QFileDialog::getExistingDirectory(this, "Choose Directory", QDir::currentPath(), QFileDialog::ShowDirsOnly);
    QString vis_data_dir = QString("Dir: ") + data_dir_qstr;
    if(data_dir_qstr.isEmpty()) {
        data_loaded = false;
        data_num = 0;
        vis_data_dir = QString("Dir:");
        ui->labelDirectory->setText(vis_data_dir);
        return;
    }

    ui->labelDirectory->setText(vis_data_dir);
    data_dir_str = data_dir_qstr.toUtf8().constData();
    data_loaded = onDataDirectorySet(data_dir_str);

    if(data_loaded) {
        data_num = static_cast<int>(data_paths.size());
        ui->spinBoxCloudNo->setMaximum(data_num-1);
        emit SignalCloudNow(data_paths[0]);
    } else {
        data_num = 0;
        ui->spinBoxCloudNo->setMaximum(0);
        return;
    }


}

void MainWindow::on_checkBox_toggled(bool checked) {
    int arg = checked ? 1 : 0;
    emit SignalDrawAxis(arg);
}

void MainWindow::on_doubleSpinBoxPointSize_valueChanged(double arg1) {
    emit SignalPointSize(arg1);
}

void MainWindow::on_spinBoxCloudNo_valueChanged(int arg1) {
    if(data_paths.empty()) {
        return;
    } else {
        if(arg1 + 1 > static_cast<int>(data_paths.size())) {
            printf("[ERROR] argument is larger than data_path size\n");
            return;
        }
        emit SignalCloudNow(data_paths[arg1]);
    }
}


bool MainWindow::onDataDirectorySet(std::string data_dir) {

    // Load all the point cloud lists : i.e. 000000.bin ~ nnnnnn.bin
    // Sort the list in ascending order

    DIR * dir_sensor = opendir(data_dir.c_str());
    struct dirent *ent;
    data_paths.clear();

    if(dir_sensor!=NULL) {
        while((ent = readdir(dir_sensor))!= NULL) {
            std::string filename = std::string(ent->d_name);
            if(filename == "." || filename == "..") {
                continue;
            }

            //check file type
            std::string filetype;
            size_t dotPos = filename.find_last_of('.');
            if (dotPos != std::string::npos && dotPos < filename.length() - 1) {
                filetype = filename.substr(dotPos + 1);
            } else {
                filetype = "";
            }

            if(filetype == "bin") {
                std::string data_path = data_dir + "/" + filename;
                data_paths.push_back(data_path);
            }
        }
    } else {
        printf("Invaid Directory\n");
        return false;
    }

    if(data_paths.empty()) {
        printf("No point cloud in %s\n", data_dir.c_str());
        return false;
    } 

    std::stable_sort(data_paths.begin(), data_paths.end(),
                    [](std::string first,
                    std::string second) -> bool{
                        size_t dotPos_first = first.find_last_of('.');
                        size_t dotPos_second = second.find_last_of('.');
                        size_t slashPos_first = first.find_last_of('/');
                        size_t slashPos_second = second.find_last_of('/');

                        int num_first = std::stod(first.substr(slashPos_first+1, dotPos_first-slashPos_first));
                        int num_second = std::stod(second.substr(slashPos_second+1, dotPos_second-slashPos_second));

                        return first < second;
                    });

    return true;
}