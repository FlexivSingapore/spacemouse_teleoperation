cmake_minimum_required(VERSION 3.18.6)

# ==================================================================================================
#      PROJECT SETUP
# ==================================================================================================
project(flexiv_teleoperation VERSION 0.8.0)

# ==================================================================================================
#      PROJECT OPTIONS
# ==================================================================================================
option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)

# ==================================================================================================
#      LIBRARY DEPENDENCIES
# ==================================================================================================
# Eigen3
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  message(STATUS "Found Eigen3: ${Eigen3_DIR}")
endif()

# X11 for spacenavd
find_package(X11 REQUIRED)
if(X11_FOUND)
  message(STATUS "Found X11: ${X11_DIR}")
endif()

# Qt5
find_package(Qt5 COMPONENTS Core Widgets OpenGL Network REQUIRED)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Find flexiv_tdk INTERFACE library
find_package(flexiv_tdk REQUIRED)

# ==================================================================================================
#      CREATE EXECUTABLE
# ==================================================================================================
# Create base library from source
file(GLOB_RECURSE SRC_FILES_TELEOPERATION_APP

  ${CMAKE_CURRENT_SOURCE_DIR}/src/devices/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/logger/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/*.cpp

   # Need to include hpp for Qt MOC to parse the files
   # Need to include hpp for Qt MOC to parse the files
   ${CMAKE_CURRENT_SOURCE_DIR}/include/flexiv/teleoperation/gui/*.hpp
   
   ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

add_executable(${PROJECT_NAME}_app ${SRC_FILES_TELEOPERATION_APP})

target_include_directories(${PROJECT_NAME}_app
  PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include

  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PUBLIC # Public headers included IDL headers
    $<BUILD_INTERFACE:${GENERATED_IDL_INCLUDE_DIRECTORIES}>
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/spacenavd/include>
)

target_link_libraries(${PROJECT_NAME}_app
  flexiv::flexiv_tdk  

  # Libraries for spacemouse device
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/spacenavd/lib/libspnav.a
  X11::X11

  # Libraries for Force dimension devices
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DHD/lib/lin-x86_64/libdrd.a
  usb-1.0

  # Qt5 libraries for GUI
  Qt5::Widgets
  Qt5::Network
)

# Set target properties
set_target_properties(${PROJECT_NAME}_app PROPERTIES 
  CXX_STANDARD 17 
  CXX_STANDARD_REQUIRED ON)

# ==================================================================================================
#      CREATE EXECUTABLE
# ==================================================================================================
# Create base library from source
file(GLOB_RECURSE SRC_FILES_TELEOPERATION_APP

${CMAKE_CURRENT_SOURCE_DIR}/src/devices/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/logger/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/gui/*.cpp

 # Need to include hpp for Qt MOC to parse the files
 # Need to include hpp for Qt MOC to parse the files
 ${CMAKE_CURRENT_SOURCE_DIR}/include/flexiv/teleoperation/gui/*.hpp
 
 ${CMAKE_CURRENT_SOURCE_DIR}/src/test.cpp
)

add_executable(${PROJECT_NAME}_test ${SRC_FILES_TELEOPERATION_APP})

target_include_directories(${PROJECT_NAME}_test
PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include

PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/spacenavd/include>
)

target_link_libraries(${PROJECT_NAME}_test
flexiv::flexiv_tdk

# Libraries for spacemouse device
${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/spacenavd/lib/libspnav.a
X11::X11

# Libraries for Force dimension devices
${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/DHD/lib/lin-x86_64/libdrd.a
usb-1.0

# Qt5 libraries for GUI
Qt5::Widgets
Qt5::Network
)

# Set target properties
set_target_properties(${PROJECT_NAME}_test PROPERTIES 
CXX_STANDARD 17 
CXX_STANDARD_REQUIRED ON)
