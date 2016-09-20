#ifndef QTVISUALIZER_H
#define QTVISUALIZER_H

#include "grabber_impl.hpp"
#include "logger.hpp"

#include <QMainWindow>
#include <QFileDialog>

//point cloud library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/math/special_functions/round.hpp>

//visualization toolskit (vtk)
#include <vtkRenderWindow.h>

#include <iostream>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


namespace Ui {
class qtvisualizer;
}

class qtvisualizer : public QMainWindow
{
    Q_OBJECT

public:
    explicit qtvisualizer(QWidget *parent = 0);
    ~qtvisualizer();

public slots:
    void saveFileButtonPressed();
    void loadFileButtonPressed();

    void startButtonPressed();
    void stopButtonPressed();
    
    void depthSelected();
    void pipelineSelected();


protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    PointCloudT::Ptr cloud_;

private:
    Ui::qtvisualizer *ui;

    //set up the qvtk window using object of grabber_impl
    f2g::proc pipeline;
    const char* logfile;

    boost::shared_ptr<f2g::grabber_impl> grabimpl;
    boost::shared_ptr<f2g::grablog> log;

};

#endif // QTVISUALIZER_H
