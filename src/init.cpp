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


int main(int argc, char** argv){
    int isErr = 0;

    bool showPointCloud = true;
    bool showRGBCameraOutput = false;

    const char* logfile = "$HOME/src/logs/grabberlog/f2glog.txt";

    f2g::proc pl = f2g::proc::OPENGL;

    boost::shared_ptr<f2g::grabber_impl> f2grab(new f2g::grabber_impl());

    //is not used for now
    if(argc < 1){
        f2grab->showUsage();
    }

    f2grab->processPointCloud(pl, showRGBCameraOutput, showPointCloud);

    return (isErr);
}
