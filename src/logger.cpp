/**
*** class definition for freenect2 custom logger for additional helper functions ***

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

#include "logger.hpp"

#include <iostream>


f2g::grablog::grablog(const char* filename){
    if (filename){
        logfile_.open(filename);
    }
    if(good()){
        /*do something*/
    }
}

bool f2g::grablog::good(){
    return logfile_.is_open() && logfile_.good();
}

void f2g::grablog::log(Level level, const std::string &message){
     logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
}


void f2g::grablog::showUsage(){
    std::cout << "Syntax is: progname [ 0 | 1 | 2 ] [ 0 | 1 ] correspond to [CPU | OPENCL | OPENGL] and [ COLORIZED | UNCOLORIZED ] " << std::endl;
    std::cout << "example usage: sudo ./freenect2 1 0 for OPENCL pipeline and colorized point cloud" << std::endl;
    std::cout << "Press 'ESC' or ctrl + c to close program" << std::endl;
    std::cout << "Press 'ctrl + y' to store a cloud" << std::endl;
}


f2g::proc_err::proc_err(int argc, char **argv){
    std::cout<< "object of class proc_err (error processor) started..." << std::endl;

    paramCntError(argc, argv);
}

int f2g::proc_err::paramCntError(int argc, char **argv){
    if(argc < 1){
        std::cerr << "Syntax: "<<argv[0] << "[-proc 0|1|2]" << std::endl;
        bool isErr = -1;
        exit (isErr);
    }
    else{
        bool isErr = 0;
        return (isErr);
    }
}

int f2g::proc_err::pclArgError(int args){
    /*empty for now*/
    return (0);
}


void f2g::grablog::enableErrorlog(void){
    std::cout<<"not yet implemented" << std::endl;
}


void f2g::grablog::disableErrorlog(void){
    std::cout<<"not yet implemented" << std::endl;
}
