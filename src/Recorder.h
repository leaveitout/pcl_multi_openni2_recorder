//
// Created by sean on 27/10/15.
//

#ifndef PCL_MULTI_OPENNI2_RECORDER_RECORDER_H
#define PCL_MULTI_OPENNI2_RECORDER_RECORDER_H

#include <pcl/point_cloud.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/boost.h>

#include "RecorderNode.h"

typedef pcl::io::OpenNI2Grabber NI2Grabber;
typedef pcl::io::openni2::OpenNI2Device NI2Device;
typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;
typedef boost::signals2::signal<void ()>  Signal;

template <typename PointT>
class Recorder {
private:
    const static int DEFAULT_BUFFER_SIZE = 500;

    std::vector<NI2Grabber::Ptr> grabbers_;
    std::vector<boost::shared_ptr <RecorderNode <PointT>>> nodes_;

    Signal start_signal_;
    Signal stop_signal_;
    std::vector<boost::signals2::connection> connections_;

public:
    Recorder<PointT>(std::vector<NI2Grabber::Ptr>& grabbers, int total_buffer_size = DEFAULT_BUFFER_SIZE)
            : grabbers_ (grabbers) {
        int buffer_size = total_buffer_size / static_cast<int>(grabbers.size());
        size_t id = 0;
        for(auto grabber : grabbers) {
            boost::shared_ptr<RecorderNode <PointT>> new_node(new RecorderNode<PointT>(grabber, id, buffer_size));

            new_node->start ();
//            boost::function<void (void)> start_callback =
//                    boost::bind(&RecorderNode<PointT>::start, new_node);
//            connections_.push_back(connectStart(start_callback));

            boost::function<void (void)> stop_callback = boost::bind(&RecorderNode<PointT>::stop, new_node);
            connections_.push_back(connectStop(stop_callback));
            nodes_.push_back(new_node);
            id++;
        }
    }

    ~Recorder<PointT>() {
        for(auto connection : connections_)
            connection.disconnect();
    }

    void start() {
        start_signal_();
    }

    void stop() {
        stop_signal_();
    }

private:
    // Connect a slot to the signal which will be emitted when start called.
    boost::signals2::connection connectStart(const Signal::slot_type &subscriber) {
        return start_signal_.connect(subscriber);
    }

    // Connect a slot to the signal which will be emitted when stop called.
    boost::signals2::connection connectStop(const Signal::slot_type &subscriber) {
        return stop_signal_.connect(subscriber);
    }
};

#endif //PCL_MULTI_OPENNI2_RECORDER_RECORDER_H
