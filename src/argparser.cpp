/**

*** header file  argument parser  ***

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

// include own header first
#include "argparser.hpp"

#include <iostream>
#include <iterator>
#include <string>


const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;

namespace po = boost::program_options;


int f2g::argparser::init(int argc, char **argv){

    // declare the supported options
    po::options_description desc("Allowed options");

    desc.add_options()
        ("help,h",                             " displays help message")
        ("pipeline,p",     po::value<int>(),   " specify pipeline options [ 0 | 1 | 2 ]")
        ("depth,d",        po::value<int>(),   " specify colorization options [ 0 | 1 ]")
        ("save,s",         po::value<int>(),   " specify save options [ 0 | 1 ]");

    po::variables_map vm;

    try{
        po::store(po::parse_command_line(argc, argv, desc), vm); 
        po::notify(vm);
    }

    catch(po::error& e){
        std::cerr << "ERROR: " <<e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;

        return ERROR_IN_COMMAND_LINE;
    }

    catch(std::exception& e){
        std::cerr << "Unhandled Exception reached the top of main: "
                  <<e.what() << ", application will now exit"<< std::endl;


        return ERROR_UNHANDLED_EXCEPTION;
    }

    if(vm.count("help")){
        std::cout << desc << std::endl;
        
        return SUCCESS;
    }

    if(vm.count("pipeline")){
        std::cout << "packet pipeline specified" <<std::endl; 
        pipeline_ = 0;
        return SUCCESS;
    }

    if(vm.count("depth")){
        std::cout << "depth specified" <<std::endl;
        depth_ = 0;
        return SUCCESS;
    }

    if(vm.count("save")){
        save_ = 0;   
        return SUCCESS;
    }

return SUCCESS;
}
