

#ifndef BRIDGE_TIME_CONVERSION_H
#define BRIDGE_TIME_CONVERSION_H

#include "cartographer/common/time.h"
#include <ctime>

namespace Cobot{
    ::cartographer::common::Time now();

    //ICU Universal Time 尺度是 "0001-01-01 00:00:00.0 + 0000",在Unix epoch之前有719162天
    ::cartographer::common::Time FromDouble(double time);
}

#endif
