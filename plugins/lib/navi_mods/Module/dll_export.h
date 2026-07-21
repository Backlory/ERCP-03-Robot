#pragma once
#include "robot_config.h"
#include "Module/modules.h"

ROBOT_API bool CreateModule(navi::ModPtr &ptr);

ROBOT_API bool DisposeModule(navi::ModPtr obj);

ROBOT_API const char *GetModuleVersion();
