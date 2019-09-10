
#include "time_conversion.h"

namespace Cobot{
    ::cartographer::common::Time now(){
        struct timespec time_now;
        clock_gettime(CLOCK_REALTIME, &time_now);
        return ::cartographer::common::FromUniversal(
                (time_now.tv_sec + ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds)
                * 10000000ll + (time_now.tv_nsec + 50) / 100
        );
    }

    //ICU Universal Time 尺度是 "0001-01-01 00:00:00.0 + 0000",在Unix epoch之前有719162天
    ::cartographer::common::Time FromDouble(double time){
        ///FIXME:传入的时间可能不是绝对时间，是一个相对时间，后面需要测试转换
        ///FIXME:cartographer中使用的是int64
        auto sec = (int64_t)(time);
        auto nsec = int64_t ((time - double(sec)) * 1e9);
        return ::cartographer::common::FromUniversal(
                (sec +
                 ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                10000000ll +                //cartographer中的单位为10000000
                (nsec + 50) / 100);   //+50获得正确的截断
    }

}