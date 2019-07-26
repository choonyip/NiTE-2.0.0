cmake_minimum_required(VERSION 2.8)

project(bone)

set(OPENNI2_INC "/home/cy8182/backup/OpenNI-Linux-x64-2.3/Include")
set(NITE2_INC "/home/cy8182/backup/NiTE-2.0.0/Include")
set(OPENNI2_LIB "/home/cy8182/backup/OpenNI-Linux-x64-2.3/Redist")
set(NITE2_LIB "/home/cy8182/backup/NiTE-2.0.0/Redist")

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(
	${OPENNI2_INC}
    	${NITE2_INC}
	${OPENGL_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
	${OPENCV_INCLUDE}
    	${CMAKE_CURRENT_SOURCE_DIR}
	)

link_directories(
	${OPENCV_LIB}
	${OPENNI2_LIB}
	${NITE2_LIB}
	)

add_executable(${PROJECT_NAME}
	main.cpp
	Viewer.cpp
)

target_link_libraries(${PROJECT_NAME}
	OpenNI2
	NiTE2
	${OPENGL_LIBRARIES}
	${GLUT_LIBRARIES}
	${OpenCV_LIBS}
)
