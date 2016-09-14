/**
*** header for freenect2 custom logger for additional helper functions ***

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
#pragma once

#include <libfreenect2/logger.h>

#include <fstream>
#include <cstdlib>

namespace f2g{

	class grablog : public libfreenect2::Logger  {

	public:

		grablog(const char *filename);

	private:

		std::ofstream logfile_;

		bool good();
		virtual void log(Level level, const std::string &message);

		void enableErrorlog(void);
		void disableErrorlog(void);

	};

	class proc_err{

	public:

		proc_err(int argc, char **argv);

	private:

		int paramCntError(int argc, char **argv);
		int pclArgError(int args);

		bool isErr;
	};

}
