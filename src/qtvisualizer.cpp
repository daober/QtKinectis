#include "qtvisualizer.h"
#include "ui_qtvisualizer.h"

#include <boost/make_shared.hpp>

qtvisualizer::qtvisualizer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::qtvisualizer){
    ui->setupUi(this);
    this->setWindowTitle("Kinect Point Cloud Viewer");

    pipeline = f2g::proc::CPU;
    //setup the cloud pointer
    cloud_.reset(new PointCloudT);
    cloud_->points.resize(2000);

    viewer_.reset(new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer_->setBackgroundColor(0.0, 0.0, 0.0);

    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    grabimpl = boost::make_shared<f2g::grabber_impl>();

    grabimpl->processQtColorizedCloud(pipeline, viewer_);
    
    //connect callback function for ui (save and load buttons)
    connect(ui->pushButton_save, SIGNAL(clicked()), this, SLOT(loadFileButtonPressed()));
    connect(ui->pushButton_load, SIGNAL(clicked()), this, SLOT(saveFileButtonPressed()));
    //connect callback function foir ui (start and stop aquiring depth images)
    connect(ui->startButton, SIGNAL(clicked()), this, SLOT(startButtonPressed()));
    connect(ui->stopButton, SIGNAL(clicked()), this, SLOT(stopButtonPressed()));
}


qtvisualizer::~qtvisualizer(){
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

    int return_status;

    if(filename.endsWith(".pcd", Qt::CaseInsensitive)){
        return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_temp);
    }
    else{
        return_status = pcl::io::loadPLYFile(filename.toStdString(), *cloud_temp);
    }
    if(return_status != 0){
        PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
        return;
    }

      // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    if(cloud_temp->is_dense){
        pcl::copyPointCloud(*cloud_temp, *cloud_);
    }
    else{
        PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
        std::vector<int> vec;
        pcl::removeNaNFromPointCloud (*cloud_temp, *cloud_, vec);
    }

    viewer_->updatePointCloud (cloud_, "cloud");
    viewer_->resetCamera();
    ui->qvtkWidget->update();
}


void qtvisualizer::saveFileButtonPressed(){
    // You might want to change "/home/" if you're not on an *nix platform
    QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

    PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

    if(filename.isEmpty ()){
      return;
    }

    int return_status;

    if(filename.endsWith (".pcd", Qt::CaseInsensitive)){
      return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_);
    }
    else if(filename.endsWith (".ply", Qt::CaseInsensitive)){
      return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
    }
    else{
      filename.append(".ply");
      return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
    }

    if(return_status != 0){
      PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
      return;
    }
}


void qtvisualizer::depthSelected(){

}

void qtvisualizer::pipelineSelected(){

}

void qtvisualizer::startButtonPressed(){
    std::cout<<"start button pressed"<<std::endl;
}

void qtvisualizer::stopButtonPressed(){
    std::cout<<"stop button pressed"<<std::endl;
}
