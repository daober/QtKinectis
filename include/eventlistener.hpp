/**

*** header for freenect2 eventlistener ***

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

#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/mouse_event.h>


namespace f2g {

    class eventlistener {

    public:

       static void KeyboardInputEvent(char arg);

       static void closeKinectGrabber(void);

       static void pclSaveEvent(const pcl::visualization::KeyboardEvent &event, void* data);
       static void pclMiscEvent(const pcl::visualization::KeyboardEvent &event, void *data);
       static void pclMouseEvent(const pcl::visualization::MouseEvent &event, void* data);

       static void QTButtonClickedEvent();


    private:

        int errNo;



    };

}
