# Directories to look for include files
include_directories(${DVRKCore_SOURCE_DIR}/include)

find_package(LibXml2 REQUIRED)
if (LIBXML2_FOUND)
   MESSAGE("LIBXML2 found!")
   MESSAGE("${LIBXML2_INCLUDE_DIR}")
   include_directories(${LIBXML2_INCLUDE_DIR})
   set(LIBS ${LIBS} ${LIBXML2_LIBRARIES})
else()
   MESSAGE("LIBXML2 not found!")
endif (LIBXML2_FOUND)

# Define a variable xmlreader_source_files containing a list of the source
# files for the io library
set(xmlreader_source_files
  xmlreader.cpp
)

# Create a library called io which includes the source files defined
# in the xmlreader_source_files
add_library(xmlreader ${xmlreader_source_files})
target_link_libraries(xmlreader LIBS)
