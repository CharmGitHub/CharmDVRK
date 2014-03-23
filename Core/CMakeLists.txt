cmake_minimum_required (VERSION 2.8)
project (DVRKCore)

#add_subdirectory(src/io)
#add_subdirectory(src/xmlreader)
#add_subdirectory(src/robot)

# Directories to look for include files
include_directories(${DVRKCore_SOURCE_DIR}/include)

# Find the Libxml2 package
find_package(LibXml2 REQUIRED)
if (LIBXML2_FOUND)
   MESSAGE("LIBXML2 found!")
   MESSAGE("${LIBXML2_INCLUDE_DIR}")
   include_directories(${LIBXML2_INCLUDE_DIR})
   set(LIBS ${LIBS} ${LIBXML2_LIBRARIES})
else()
   MESSAGE("LIBXML2 not found!")
endif (LIBXML2_FOUND)

# CISST Package include
set(REQUIRED_CISST_LIBRARIES cisstCommon cisstVector cisstNumerical cisstRobot cisstCommonXML)
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})

# Define a variable io_source_files containing a list of the source
# files for the io library
set(io_source_files
  src/io/AmpIO.cpp
  src/io/FirewirePort.cpp
)

# Define a variable xmlreader_source_files containing a list of the source
# files for the io library
set(xmlreader_source_files
  src/xmlreader/xmlreader.cpp
)

# Define a variable robot_source_files containing a list of the source
# files for the robot library
set(robot_source_files
  src/robot/CDaVinciRobot.cpp
  src/robot/CDaVinciRobotMTM.cpp
  src/robot/CDaVinciRobotPSM.cpp
)

# Define a variable timer_source_files containing a list of the source
# files for the timer library
set(timer_source_files
  src/timer/CPrecisionClock.cpp
)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${DVRKCore_BINARY_DIR}/lib)

# Create a library called io which includes the source files defined
# in the robot_source_files
add_library(DVRKCore STATIC ${io_source_files} ${xmlreader_source_files} ${robot_source_files} ${timer_source_files})
target_link_libraries(DVRKCore raw1394)

# Copy the include directory to the build directory
file(COPY include DESTINATION ${DVRKCore_BINARY_DIR})

