#ifndef PCL_MULTI_OPENNI2_RECORDER_CLOUD_RECORD_H
#define PCL_MULTI_OPENNI2_RECORDER_CLOUD_RECORD_H

#include <string>
#include <pcl/point_cloud.h>

using namespace pcl;

template <typename PointT>
class CloudRecord {

private:
    typename PointCloud<PointT>::ConstPtr cloud_;
    std::string name_;

public:
    CloudRecord(typename PointCloud<PointT>::ConstPtr cloud, const std::string& name)
            : cloud_ (cloud)
            , name_ (std::move(name)) { }

    CloudRecord() { }

    inline const std::string& getName() {
       return name_;
    }

    inline const typename PointCloud<PointT>::ConstPtr getCloud() {
        return cloud_;
    }

private:
    // Disabled constructors
    CloudRecord (const CloudRecord&) = delete;
    CloudRecord& operator = (const CloudRecord&) = delete;

};

#endif
