//
// Created by sean on 15/01/16.
//

#ifndef PCL_CLOUD_REGISTRATION_CAMERA_INTRINSICS_LOADER_HPP
#define PCL_CLOUD_REGISTRATION_CAMERA_INTRINSICS_LOADER_HPP

#include <pcl/io/openni2_grabber.h>

namespace CameraIntrinsicsLoader {
    bool applyIntrinsics(size_t camera_num, pcl::io::OpenNI2Grabber::Ptr grabber);
    bool copyIntrinsicsToOutputDir(size_t camera_num);
    bool copyExtrinsicsToOutputDir();
};

#endif //PCL_CLOUD_REGISTRATION_CAMERAINTRINSICSLOADER_HPP
