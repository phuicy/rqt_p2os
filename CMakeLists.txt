cmake_minimum_required(VERSION 2.8.3)

project(rqt_p2os)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp p2os_msgs)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_p2os_SRCS
  src/rqt_p2os/p2os.cpp
)

set(rqt_p2os_HDRS
  include/rqt_p2os/p2os.h
)

set(rqt_p2os_UIS
  src/rqt_p2os/p2os.ui
)

set(rqt_p2os_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
	INCLUDE_DIRS ${rqt_p2os_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp p2os_msgs
)
catkin_python_setup()

qt4_wrap_cpp(rqt_p2os_MOCS ${rqt_p2os_HDRS})
qt4_wrap_ui(rqt_p2os_UIS_H ${rqt_p2os_UIS})

include_directories(${rqt_p2os_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_p2os_SRCS} ${rqt_p2os_MOCS} ${rqt_p2os_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts/rqt_p2os
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
