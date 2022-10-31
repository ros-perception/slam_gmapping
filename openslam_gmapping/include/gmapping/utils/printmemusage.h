#ifndef PRINTMEMUSAGE_H
#define PRINTMEMUSAGE_H
#include <sys/types.h>
#ifndef _WIN32
  #include <unistd.h>
#endif
#include <iostream>
#include <fstream>
#include <string>
#include <gmapping/utils/utils_export.h>

namespace GMapping{
	void UTILS_EXPORT printmemusage();
};

#endif
