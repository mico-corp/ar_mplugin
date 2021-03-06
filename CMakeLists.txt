##---------------------------------------------------------------------------------------------------------------------
##  MICO ar wrapper plugin
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------
cmake_minimum_required (VERSION 3.12 FATAL_ERROR)
project(mico VERSION 1.0 LANGUAGES C CXX)
set(PLUGIN_NAME ${PROJECT_NAME}-ar)


##################################################
######    Configure project               ########
##################################################

##################################################
######    Loading 3rd party libraries.    ########
##################################################

set(EXTRA_INCLUDES "")
set(EXTRA_LIBRARIES "")
set(EXTRA_COMPILE_DEFS "")

##################################################
###### Create project and configure ALIAS ########
##################################################

file(GLOB_RECURSE MICO_AR_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE MICO_AR_HEADER_FILES "include/*.h" "include/*.inl")
qt5_wrap_cpp(MOC_FILES src/ar/gl_helpers/VisualizerGlWidget.cpp include/mico/ar/gl_helpers/VisualizerGlWidget.h)
 
SET(MICO_MODULES ${MICO_MODULES} ar PARENT_SCOPE)
add_mplugin(    PLUGIN_NAME ${PLUGIN_NAME} 
                PLUGIN_SOURCES ${MICO_AR_SOURCE_FILES} ${MOC_FILES}
				PLUGIN_HEADERS  ${MICO_AR_HEADER_FILES} 
				MICO_DEPS core
				)

##################################################
######    Loading 3rd party libraries.    ########
##################################################

find_package(OpenCV REQUIRED COMPONENTS core aruco)
mplugin_include_directory( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_INCLUDES ${OpenCV_INCLUDE_DIRS})
mplugin_link_library( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES ${OpenCV_LIBS})
mplugin_compile_definition( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_DEFINITIONS HAS_OPENCV)


if(WIN32)
	find_package(FreeGLUT REQUIRED)
	mplugin_link_library(PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES FreeGLUT::freeglut)

	mplugin_link_library(PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES "${MICO_ROOT_DIR}/dependencies/lib/Release/x64/glew32.lib")
	mplugin_link_library(PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES "${MICO_ROOT_DIR}/dependencies/lib/Release/x64/glew32.lib")
	mplugin_link_library(PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES "${MICO_ROOT_DIR}/dependencies/lib/Release/x64/glew32s.lib")
elseif(UNIX)
	find_package(GLUT REQUIRED)
	find_package(OpenGL REQUIRED)
	mplugin_include_directory( PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_INCLUDES ${GLUT_INCLUDE_DIR})
	mplugin_link_library(PLUGIN_NAME ${PLUGIN_NAME} PLUGIN_LIBRARIES GL glut GLU GLEW ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
endif()


add_subdirectory(apps)
