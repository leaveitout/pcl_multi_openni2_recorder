#ifndef PCL_MULTI_OPENNI2_RECORDER_TIMER_H
#define PCL_MULTI_OPENNI2_RECORDER_TIMER_H

#include <string>
#include <limits>
#include <pcl/common/time.h>

#include "PCDBuffer.h"
#include "Logger.h"


template <typename PointT>
class Timer {
private:
    double last_;
    std::string description_;
    int count_;

public:
    Timer(const std::string& description)
            : description_ (description) {
        last_ = pcl::getTime();
    }

    void time(PCDBuffer<PointT>& buff) {
        double now = pcl::getTime();
        ++count_;
        if (now - last_ > 1.0) {
            std::stringstream ss;
            ss << "Average framerate (" << description_ << "): " << double(count_)/(now - last_) <<
                    " Hz. Queue size: " << buff.getSize() << std::endl;
            Logger::log(Logger::INFO, std::move(ss.str()));
            count_ = 0;
            last_ = now;
        }
    }
};

#endif
