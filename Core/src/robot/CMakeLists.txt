# Directories to look for include files
include_directories(${DVRKCore_SOURCE_DIR}/include)

# CISST Package include
set(REQUIRED_CISST_LIBRARIES cisstCommon cisstVector cisstNumerical cisstRobot cisstCommonXML)
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})

# Define a variable robot_source_files containing a list of the source
# files for the robot library
set(robot_source_files
  CDaVinciRobot.cpp
  CDaVinciRobotMTM.cpp
  CDaVinciRobotPSM.cpp
)

# Create a library called io which includes the source files defined
# in the robot_source_files
add_library(robot ${robot_source_files})
cisst_target_link_libraries (robot ${REQUIRED_CISST_LIBRARIES})
target_link_libraries(robot xmlreader)
