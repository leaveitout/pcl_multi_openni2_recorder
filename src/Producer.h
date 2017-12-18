#ifndef PCL_MULTI_OPENNI2_RECORDER_PRODUCER_H
#define PCL_MULTI_OPENNI2_RECORDER_PRODUCER_H

#include <pcl/point_cloud.h>
#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <pcl/io/openni2_grabber.h>
#include <boost/thread/condition.hpp>
#include "PCDBuffer.h"
#include "Timer.h"

using namespace pcl;

typedef pcl::io::OpenNI2Grabber NI2Grabber;

template <typename PointT>
class Producer {
private:
    void grabberCallBack (const typename PointCloud<PointT>::ConstPtr& cloud) {
        // TODO: Remove next line once checked
//        std::stringstream ss;
//        ss << "Producer " << id_ << " operating on thread " << boost::this_thread::get_id() << std::endl;
//        Logger::log(Logger::DEBUG, ss.str());

        std::string time = boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::local_time ());
        boost::shared_ptr<CloudRecord<PointT>> cloud_record(new CloudRecord<PointT>(cloud, std::move(time)));

        if (!buf_.pushBack (cloud_record))
            Logger::log(Logger::WARN, "Warning! Buffer was full, overwriting data!\n");

        cloud_timer_->time(buf_);
    }

public:
    Producer(NI2Grabber::Ptr grabber, PCDBuffer<PointT> &buf, size_t id)
            : grabber_ (grabber)
            , buf_ (buf)
            , id_ (id) {
        boost::function<void (const typename PointCloud<PointT>::ConstPtr&)> cloud_callback =
                boost::bind (&Producer::grabberCallBack, this, _1);
        cloud_connection_ = grabber_->registerCallback (cloud_callback);
    }

    void start() {
        std::stringstream ss;
        ss << "Device " << id_ << ", cloud callback";
        cloud_timer_.reset (new Timer<PointT>(ss.str()));
    }

    void stop () {
        cloud_connection_.disconnect();
        Logger::log(Logger::INFO, "Producer done.\n");
    }

private:
    NI2Grabber::Ptr grabber_;
    PCDBuffer<PointT> &buf_;
    size_t id_;
    boost::shared_ptr<Timer<PointT>> cloud_timer_;
    boost::signals2::connection cloud_connection_;
};

#endif

