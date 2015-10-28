#ifndef PCL_MULTI_OPENNI2_RECORDER_RECORDER_NODE_H
#define PCL_MULTI_OPENNI2_RECORDER_RECORDER_NODE_H

#include <pcl/point_cloud.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/boost.h>

#include "PCDBuffer.h"
#include "Producer.h"
#include "Consumer.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;

typedef pcl::io::OpenNI2Grabber NI2Grabber;
typedef pcl::io::openni2::OpenNI2Device NI2Device;
typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;

template <typename PointT>
class RecorderNode {
private:
    NI2Grabber::Ptr grabber_;
    std::unique_ptr<Producer<PointT>> producer_;
    std::unique_ptr<Consumer<PointT>> consumer_;
    PCDBuffer<PointT> buffer_;
    bool grabber_was_running_;
    std::unique_ptr<boost::thread> start_thread_;
    std::unique_ptr<boost::thread> stop_thread_;
    size_t id_;
    const static int DEFAULT_BUFFER_SIZE = 200;

public:

    RecorderNode<PointT>(NI2Grabber::Ptr grabber, size_t id, int buffer_size = DEFAULT_BUFFER_SIZE)
        : grabber_ (grabber)
        , id_ (id)
        , grabber_was_running_ (false) {
        if(grabber_->isRunning())
            grabber_was_running_ = true;
        buffer_.setCapacity(buffer_size);
    }

    void stop() {
        stop_thread_.reset(new boost::thread(boost::bind(&RecorderNode::finish, this)));
    }

    void start() {
        start_thread_.reset(new boost::thread(boost::bind ( &RecorderNode::run, this)));
    }

    ~RecorderNode() {
        stop_thread_->join();
    }

private:
    void run() {
        producer_.reset(new Producer<PointT>(grabber_, buffer_, id_));
        consumer_.reset(new Consumer<PointT>(buffer_, id_));
        producer_->start();
        if(!grabber_was_running_)
            grabber_->start();
        stringstream ss;
        ss << "RecorderNode " << id_ << " operating on thread " << boost::this_thread::get_id() << std::endl;
        Logger::log(Logger::INFO, ss.str());
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        consumer_->start();
    }

    void finish() {
        Logger::log(Logger::INFO, "Stop signal received by node %i.\n", id_);
        if(!grabber_was_running_)
            grabber_->stop();
        producer_->stop();
        start_thread_->join();
        consumer_->stop();
    }
};
#endif


