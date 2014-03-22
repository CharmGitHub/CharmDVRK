#ifndef XMLREADER_H
#define XMLREADER_H

#include "DataTypes.h"

std::string xmlReaderGetRobotType(const char *a_filename);
void xmlReaderGetRobotInfo(const char *a_filename, IOInfo &a_ioInfo);

#endif
