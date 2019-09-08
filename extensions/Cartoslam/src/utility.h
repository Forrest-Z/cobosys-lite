
#ifndef LITEDRIVER_UTILITY_H
#define LITEDRIVER_UTILITY_H

#if defined(_USE_RRLOG)   //use rrlog
#include "librrlog.h"
#include "librrstat.h"
#endif

#if defined(_USE_RRLOG)   //use rrlog
#define Litelog(level, fmt, ...)\
    do{\
        rrlog(MOD_SLAM, level, fmt, ##__VA_ARGS__);\
    }while(0)
#else   //use printf

#define Litelog(level, fmt, ...)\
    do{\
        std::printf(fmt, ##__VA_ARGS__);\
    }while(0)
#endif

#endif //LITEDRIVER_UTILITY_H
