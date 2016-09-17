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

#include "argparser.hpp"
#include <iostream>


int f2g::argparser::initparser(int argc, char **argv){

    boost::program_options::options_description desc("Allowed options");

    desc.add_options()
        ("help", "produce help message")
        ("pipeline", boost::program_options::value<int>(), "specify pipeline options")
        ("depth", boost::program_options::value<int>(), "specify colorization options");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if(vm.count("help")){
        std::cout << desc << "\n";
        return 1;
    }

    if(vm.count("pipeline")){
        std::cout << "packet pipeline is set to: "
             << vm["compression"].as<int>() << ".\n";
    } else {
        std::cout << "packet pipeline is not set. \n";
    }

}
