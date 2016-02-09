#ifndef PCL_MULTI_OPENNI2_RECORDER_CONSUMER_H
#define PCL_MULTI_OPENNI2_RECORDER_CONSUMER_H

#include <pcl/point_cloud.h>
#include <pcl/io/boost.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include "PCDBuffer.h"
#include "Logger.h"

using namespace pcl;

template <typename PointT>
class Consumer
{
private:
    void writeToDisk (const boost::shared_ptr<CloudRecord<PointT>> record) {
        if(record) {
            std::stringstream ss;
            ss << id_ + 1 << "/" << id_ + 1 << "-" << record->getName() << ".pcd";
            writer_.writeBinaryCompressed (std::move(ss.str()), *(record->getCloud()));
            writer_timer_->time(buf_);
        }
    }

    void receiveAndProcess () {
        c_mutex_.lock();
        while (!is_done_) {
            c_mutex_.unlock();
            writeToDisk(buf_.getFront());
            c_mutex_.lock();
        }
        c_mutex_.unlock();

        Logger::log(Logger::INFO, "Writing remaining %ld clouds in the buffer to disk...\n", buf_.getSize());

        while (!buf_.isEmpty ())
            writeToDisk (buf_.getFront ());
    }

public:
    Consumer (PCDBuffer<PointT> &buf, size_t id)
            : buf_ (buf)
            , id_ (id)
            , is_done_ (false) {
    }

    void start() {
        std::stringstream ss;
        ss << "Device " << id_ << ", write callback";
        std::stringstream ss1;
        ss1 << id_ + 1;
        mkdir(ss1.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        writer_timer_.reset (new Timer<PointT>(ss.str()));
        thread_.reset (new boost::thread (boost::bind (&Consumer::receiveAndProcess, this)));
    }

    void stop () {
        {
            boost::mutex::scoped_lock lock(c_mutex_);
            is_done_ = true;
        }
        thread_->join();
        Logger::log(Logger::INFO, "Consumer done.\n");
    }

private:
    PCDBuffer<PointT> &buf_;
    boost::shared_ptr<boost::thread> thread_;
    PCDWriter writer_;
    size_t id_;
    boost::shared_ptr<Timer<PointT>> writer_timer_;
    bool is_done_;
    boost::mutex c_mutex_;
};

#endif
