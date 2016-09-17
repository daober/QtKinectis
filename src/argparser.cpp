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


const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;


int f2g::argparser::init(int argc, char **argv){

    /*namespace po = boost::program_options;

    po::options_description desc("Options");

    desc.add_options()
        ("help, h", "print help message")
        ("pipeline, p", po::value<int>(), "specify pipeline options")
        ("depth, d", po::value<int>(), "specify colorization options");

    po::variables_map varMap;*/

    /*try{
        po::store(po::parse_command_line(argc, argv, desc), varMap); *///throws on error

        /** -help option
        */
        /*if(varMap.count("help")){
            std::cout << "This is just a template app that should be modified"
                      << " and added to in order to create a useful help output/command"
                      << " line application" << std::endl << std::endl;

        return SUCCESS;
        }

        po::notify(varMap); // throws on error and
                            // after help in case there are any problems
    }

    catch(po::error& e){
        std::cerr << "ERROR: " <<e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;

        return ERROR_IN_COMMAND_LINE;
    }

    catch(std::exception& e){
        std::cerr << "Unhandled Exception reached the top of main: "
                  <<e.what() << ", application will no exit"<< std::endl;


        return ERROR_UNHANDLED_EXCEPTION;
    }

return SUCCESS;*/
}
