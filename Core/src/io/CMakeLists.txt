# Directories to look for include files
include_directories(${DVRKCore_SOURCE_DIR}/include)

# Define a variable io_source_files containing a list of the source
# files for the io library
set(io_source_files
  AmpIO.cpp
  FirewirePort.cpp
)

# Create a library called io which includes the source files defined
# in the io_source_files
add_library(io ${io_source_files})
