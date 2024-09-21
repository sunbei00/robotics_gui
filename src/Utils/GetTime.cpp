//
// Created by root on 9/21/24.
//

#include "Utils/GetTime.h"

namespace Utils {
    long long getCurrentTimeInSeconds() {
        auto now = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
        return duration.count();
    }
}