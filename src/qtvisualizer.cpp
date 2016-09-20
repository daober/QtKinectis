#include "qtvisualizer.h"
#include "ui_qtvisualizer.h"
#include "grabber_impl.hpp"
#include "logger.hpp"


qtvisualizer::qtvisualizer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::qtvisualizer)
{
    ui->setupUi(this);
    this->setWindowTitle("Kinect Point Cloud Viewer");

    //setup the cloud pointer
    cloud_.reset(new PointCloudT);
    cloud_->points.resize(2000);

    viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer_->setBackgroundColor(0.1, 0.1, 0.1);

    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
    //set up the qvtk window using object of grabber_impl
    //f2g::proc pipeline;

    //const char* logfile = "$HOME/src/logs/grabberlog/f2glog.txt";
    //boost::shared_ptr<f2g::grabber_impl> grabimpl(new f2g::grabber_impl());
    //boost::shared_ptr<f2g::grablog> log(new f2g::grablog(logfile));

    //grabimpl->processColorizedPointCloud(pipeline);


    //connect callback function for ui (buttons)
    connect(ui->pushButton_save, SIGNAL(clicked()), this, SLOT(loadFileButtonPressed()));
    connect(ui->pushButton_load, SIGNAL(clicked()), this, SLOT(saveFileButtonPressed()));
}


qtvisualizer::~qtvisualizer()
{
    delete ui;
}

void qtvisualizer::loadFileButtonPressed(){

//HINT: needs to be changed if not on a unix platform!
    QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));

    PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
    PointCloudT::Ptr cloud_temp(new PointCloudT);

    if(filename.isEmpty()){
        //TODO: warning window would be useful
        return;
    }
}

void qtvisualizer::saveFileButtonPressed(){



}

void qtvisualizer::depthButtonPressed(){

}

void qtvisualizer::pipelineButtonPressed(){


}
