#ifndef PCL_MULTI_OPENNI2_RECORDER_PCD_BUFFER_H
#define PCL_MULTI_OPENNI2_RECORDER_PCD_BUFFER_H

#include <pcl/point_cloud.h>
#include <pcl/io/boost.h>
#include <boost/circular_buffer.hpp>

#include "CloudRecord.h"

using namespace pcl;

template <typename PointT>
class PCDBuffer
{
public:
    PCDBuffer () {}

    static constexpr int DEFAULT_WAKEUP_TIME = 10000;


    // thread-safe wrapper for push_back() method of ciruclar_buffer
    bool pushBack(const boost::shared_ptr<CloudRecord<PointT>> record);

    // thread-safe wrapper for front() method of ciruclar_buffer
    const boost::shared_ptr<CloudRecord <PointT>> getFront();

    inline bool isFull () {
        boost::mutex::scoped_lock buff_lock (bmutex_);
        return (buffer_.full ());
    }

    inline bool isEmpty () {
        boost::mutex::scoped_lock buff_lock (bmutex_);
        return (buffer_.empty ());
    }

    inline int getSize () {
        boost::mutex::scoped_lock buff_lock (bmutex_);
        return (int (buffer_.size ()));
    }

    inline int getCapacity () {
        return (int (buffer_.capacity ()));
    }

    inline void setCapacity (int buff_size) {
        boost::mutex::scoped_lock buff_lock (bmutex_);
        buffer_.set_capacity (buff_size);
    }

private:
    boost::mutex bmutex_;
    boost::condition_variable buff_empty_;
    boost::circular_buffer<boost::shared_ptr<CloudRecord<PointT>>> buffer_;

    PCDBuffer (const PCDBuffer&) = delete; // Disabled copy constructor
    PCDBuffer& operator = (const PCDBuffer&) = delete; // Disabled assignment operator
};

template <typename PointT>
bool PCDBuffer<PointT>::pushBack(const boost::shared_ptr<CloudRecord<PointT>> record) {
    bool overwrite = false;
    {
        boost::mutex::scoped_lock buff_lock(bmutex_);
        if(!buffer_.full())
            overwrite = true;
        buffer_.push_back(record);
    }
    buff_empty_.notify_one();
    return overwrite;
}

template <typename PointT>
const boost::shared_ptr<CloudRecord <PointT>> PCDBuffer<PointT>::getFront() {
    boost::shared_ptr<CloudRecord<PointT>> record;
    {
        boost::mutex::scoped_lock buff_lock(bmutex_);
        while (buffer_.empty()) {
            boost::chrono::microseconds period( DEFAULT_WAKEUP_TIME );
            boost::chrono::system_clock::time_point wake_up_time = boost::chrono::system_clock::now() + period;
            if(buff_empty_.wait_until(buff_lock, wake_up_time) == boost::cv_status::timeout)
                return nullptr;
        }

        record = buffer_.front();
        buffer_.pop_front();
    }
    return record;
}

#endif