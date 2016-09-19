/**
*** entrypoint freenect2 pcl grabber ***

Copyright 2016, Daniel Obermaier

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author Daniel Obermaier
*/

/*include own headers first*/
//#include "qtvisualizer.h"
#include "grabber_impl.hpp"
#include "logger.hpp"
#include "argparser.hpp"


int init(int argc, char** argv){

    f2g::proc pipeline;

    int isErr = 0;

    const char* logfile = "$HOME/src/logs/grabberlog/f2glog.txt";

    boost::shared_ptr<f2g::grabber_impl> grabimpl(new f2g::grabber_impl());
    boost::shared_ptr<f2g::grablog> log(new f2g::grablog(logfile));

    boost::shared_ptr<f2g::argparser> parse(new f2g::argparser());

    parse->init(argc, argv);


    switch(parse->getPipeline()){
        case 0:
            std::cout<<"packet pipeline CPU selected" <<std::endl;
            pipeline = f2g::proc::CPU;
            break;
        case 1:
            std::cout<<"packet pipeline OPENCL selected" <<std::endl;
            pipeline = f2g::proc::OPENCL;
            break;
        case 2:
            std::cout<<"packet pipeline OPENGL selected" <<std::endl;
            pipeline = f2g::proc::OPENGL;
            break;
        default:
            std::cout<<"packet pipeline (default = CPU) selected" <<std::endl;
            pipeline = f2g::proc::CPU;
            break;
    }


    if(parse->getDepth() == 0){
        grabimpl->processColorizedPointCloud(pipeline);
    }

    if(parse->getDepth() == 1){
        grabimpl->processUncolorizedPointCloud(pipeline);
    }

}
