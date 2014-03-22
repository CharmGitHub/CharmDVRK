cmake_minimum_required (VERSION 2.8)
project (DVRKDemoApps)

# Directories to look for include files
include_directories(${DVRKDemoApps_SOURCE_DIR}/include)
include_directories(${DVRKDemoApps_SOURCE_DIR}/ui)
include_directories(${DVRKDemoApps_SOURCE_DIR}/../../Core-Build/include)

# Directories to look for libraries
LINK_DIRECTORIES(${DVRKDemoApps_SOURCE_DIR}/../../Core-Build/lib)


set(DVRKDemoApps_SOURCES
    src/main.cpp
    src/form_mainwindow.cpp
)

set(DVRKDemoApps_HEADERS
   include/form_mainwindow.h
)

set(DVRKDemoApps_FORMS
   ui/form_mainwindow.ui
)

add_definitions(-Wall)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
qt4_wrap_cpp(MOC_SOURCES ${DVRKDemoApps_HEADERS})
qt4_wrap_ui(FORMS_HEADER ${DVRKDemoApps_FORMS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

# Create example executables
add_executable(DVRKDemoApps ${DVRKDemoApps_SOURCES} ${DVRKDemoApps_HEADERS} ${MOC_SOURCES} ${FORMS_HEADER})
target_link_libraries(DVRKDemoApps ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} DVRKCore)
