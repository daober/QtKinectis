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
#include "grabber_impl.hpp"
#include "logger.hpp"

int main(int argc, char** argv){
    int isErr = 0;

    bool colorized = false;
    bool uncolorized = false;

    const char* logfile = "$HOME/src/logs/grabberlog/f2glog.txt";

    boost::shared_ptr<f2g::grabber_impl> grabimpl(new f2g::grabber_impl());
    boost::shared_ptr<f2g::grablog> log(new f2g::grablog(logfile));

    log->showUsage();

    f2g::proc pipeline;

    switch((*argv)[1]){
        case 0:
            pipeline = f2g::proc::CPU;
            break;
        case 1:
            pipeline = f2g::proc::OPENCL;
            break;
        case 2:
            pipeline = f2g::proc::OPENGL;
            break;
        default:
            pipeline = f2g::proc::CPU;
            break;
    }

    switch((*argv)[2]){
        case 0:
            //depth image with rgb colors aligned
            colorized = true;
        break;
            case 1:
            //uncolorized depth images
            uncolorized = false;
            break;
        default:
            colorized = true;
            break;
    }


    if(argc < 3){
        std::cout<<"wrong usage, please read: "<<std::endl;
        log->showUsage();

        isErr= -2;
        std::cout<<"exiting program -> Error number: " <<isErr <<std::endl;

        exit (-2);
    }

    if(colorized){
        grabimpl->processColorizedPointCloud(pipeline);
    }

    if(uncolorized){
        //not implemented
        grabimpl->processUncolorizedPointCloud(pipeline);
    }

    return (isErr);
}
