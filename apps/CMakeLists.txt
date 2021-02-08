#---------------------------------------------------------------------------------------------------------------------
#  Vertical Engineering Solutions
#---------------------------------------------------------------------------------------------------------------------
# 
#  Copyright 2020 Vertical Engineering Solutions  - All Rights Reserved
# 
#  Unauthorized copying of this file, via any medium is strictly prohibited Proprietary and confidential.
# 
#  All information contained herein is, and remains the property of Vertical Engineering Solutions.  The 
#  intellectual and technical concepts contained herein are proprietary to Vertical Engineering Solutions 
#  and its suppliers and may be covered by UE and Foreign Patents, patents in process, and are protected 
#  by trade secret or copyright law. Dissemination of this information or reproduction of this material is 
#  strictly forbidden unless prior written permission is obtained from Vertical Engineering Solutions.
#
#---------------------------------------------------------------------------------------------------------------------
#
#  Maintainer: pramon@vengineerings.com
#
#---------------------------------------------------------------------------------------------------------------------


file(GLOB_RECURSE AEROXSUITE_INSPECTIONSOFTWARE_HDR "include/inspection_software/*.h" "include/inspection_software/*.inl")
file(GLOB_RECURSE AEROXSUITE_INSPECTIONSOFTWARE_SRC "src/inspection_software/*.cpp")

add_library(inspection_software_lib STATIC ${AEROXSUITE_INSPECTIONSOFTWARE_SRC} ${AEROXSUITE_INSPECTIONSOFTWARE_HDR})
target_link_libraries(inspection_software_lib LINK_PUBLIC aerox_suite aerox_vision)
target_include_directories(inspection_software_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_target_properties(inspection_software_lib PROPERTIES CXX_STANDARD 17)

# Add boost
find_package(Boost 1.66 REQUIRED program_options HINTS /usr/local)



##############################################################################
##                                                                          ##
##                                APPS                                      ##
##                                                                          ##
##############################################################################

add_executable(calibration_app calibration_app.cpp)
set_target_properties(calibration_app PROPERTIES CXX_STANDARD 17)

# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core highgui features2d tracking)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

target_include_directories(calibration_app PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calibration_app LINK_PUBLIC ${OpenCV_LIBS})