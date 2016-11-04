/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Sudarshan Srinivasan <sudarshan85@gmail.com>
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2015, Seán Bruton <sbruton@tcd.ie>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <csignal>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include "Util.h"
#include "Logger.h"
#include "Recorder.h"
#include "CameraIntrinsicsLoader.hpp"

using namespace std;
using namespace pcl;
using namespace pcl::console;

typedef pcl::io::OpenNI2Grabber NI2Grabber;
typedef pcl::io::openni2::OpenNI2Device NI2Device;
typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;

size_t getSystemMemory();

const size_t DEFAULT_BUFFER_SIZE = 200;

// TODO: Add in the amount needed for the string name
const size_t TOTAL_BUFFER_SIZE = max( DEFAULT_BUFFER_SIZE,
                                size_t(getSystemMemory() / (2*640 * 480 * sizeof(pcl::PointXYZRGBA))));


//////////////////////////////////////////////////////////////////////////////////////////

boost::condition_variable stop_recording;
bool recording_done = false;
boost::mutex rmutex;

void ctrlC (int) {
    Logger::log(Logger::INFO, "\nCtrl-C detected, exit condition set to true.\n");
    boost::mutex::scoped_lock lock(rmutex);
    recording_done = true;
    stop_recording.notify_one();
}

//////////////////////////////////////////////////////////////////////////////////////////
void printHelp (int default_buff_size, int, char **argv) {
    using pcl::console::print_error;
    using pcl::console::print_info;

    print_error ("Syntax is: %s ((<device_id>... | <path-to-oni-file>) [-xyz] [-shift] [-buf X]  | -l [<device_id>] | -h | --help)]\n", argv [0]);
    print_info ("%s -h | --help : shows this help\n", argv [0]);
    print_info ("%s -xyz : save only XYZ data, even if the device is RGB capable\n", argv [0]);
    print_info ("%s -shift : use OpenNI shift values rather than 12-bit depth\n", argv [0]);
    print_info ("%s -noi : do not use saved intrinsics when recording\n", argv [0]);
    print_info ("%s -buf X ; use a buffer size of X frames (default: ", argv [0]);
    print_value ("%d", default_buff_size); print_info (")\n");
    print_info ("%s -l : list all available devices\n", argv [0]);
    print_info ("%s -l <device-id> :list all available modes for specified device\n", argv [0]);
    print_info ("\t\t<device_id> may be \"#1\", \"#2\", ... for the first, second etc device in the list\n");
#ifndef _WIN32
    print_info ("\t\t                   bus@address for the device connected to a specific usb-bus / address combination\n");
    print_info ("\t\t                   <serial-number>\n");
#endif
    print_info ("\n\nexamples:\n");
    print_info ("%s \"#1\"\n", argv [0]);
    print_info ("\t\t uses the first device.\n");
    print_info ("%s \"#1\" \"#2\"\n", argv [0]);
    print_info ("\t\t uses the first and second devices.\n");
    print_info ("%s  \"./temp/test.oni\"\n", argv [0]);
    print_info ("\t\t uses the oni-player device to play back oni file given by path.\n");
    print_info ("%s -l\n", argv [0]);
    print_info ("\t\t list all available devices.\n");
    print_info ("%s -l \"#2\"\n", argv [0]);
    print_info ("\t\t list all available modes for the second device.\n");
#ifndef _WIN32
    print_info ("%s A00361800903049A\n", argv [0]);
    print_info ("\t\t uses the device with the serial number \'A00361800903049A\'.\n");
    print_info ("%s 1@16\n", argv [0]);
    print_info ("\t\t uses the device on address 16 at USB bus 1.\n");
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
int main (int argc, char** argv) {
    print_highlight ("PCL OpenNI Recorder for saving buffered PCD (binary compressed to disk). "
                             "See %s -h for options.\n", argv[0]);

    std::vector<std::string> device_ids;

    std::string device_id ("");

    int buff_size = (int)TOTAL_BUFFER_SIZE;

    if (argc >= 2) {
        device_id = argv[1];
        if (device_id == "--help" || device_id == "-h") {
            printHelp (buff_size, argc, argv);
            return 0;
        }
        else if (device_id == "-l") {
            if (argc >= 3) {
                NI2Grabber grabber(argv[2]);
                boost::shared_ptr<NI2Device> device = grabber.getDevice();
                cout << "Supported depth modes for device: " << device->getVendor() << " , " <<
                        device->getName() << endl;
                auto depth_modes = grabber.getAvailableDepthModes();
                for (auto mode: depth_modes) {
                    cout << mode.first << " = " << mode.second.x_resolution_ << " x " <<
                            mode.second.y_resolution_ << " @ " << mode.second.frame_rate_ << endl;
                }
                auto color_modes = grabber.getAvailableImageModes();
                for (auto mode: color_modes) {
                    cout << mode.first << " = " << mode.second.x_resolution_ << " x " <<
                            mode.second.y_resolution_ << " @ " << mode.second.frame_rate_ << endl;
                }
            }
            else {
                auto device_manager = NI2DeviceManager::getInstance();
                auto device_infos = device_manager->getConnectedDeviceInfos();
                if(!device_infos->empty()) {
                    int device_idx = 1;
                    for(auto info: *device_infos) {
                        cout << "Device: " << device_idx << ", vendor: " << info.vendor_ << ", product: " << info.name_ <<
                        ", connected:" << info.uri_ << ", serial: " <<
                        device_manager->getDeviceByIndex(device_idx -1)->getStringID() << endl;
                        ++device_idx;
                    }
                }
                else
                    cout << "No devices connected." << endl;

                cout <<"Virtual Devices available: ONI player" << endl;
            }
            return 0;
        }
        else {
            device_ids.push_back(device_id);

            if(argc>2) {
                int argv_idx = 2;
                while(argv_idx < argc) {
                    if(argv[argv_idx][0] == '-')
                        break;

                    std::string next_device_id(argv[argv_idx]);
                    device_ids.push_back(next_device_id);
                    argv_idx++;
                }
            }
        }
    }
    else {
        auto device_manager = NI2DeviceManager::getInstance();
        if(device_manager->getNumOfConnectedDevices() > 0) {
            cout << "Device Id not set, using first device." << endl;
            device_ids.push_back(device_manager->getAnyDevice()->getUri());
        }
        else {
            cout << "No devices connected." << endl;
        }
    }

    bool just_xyz = find_switch (argc, argv, "-xyz");
    bool use_saved_intrinsics = !find_switch (argc, argv, "-noi");
    // TODO: Check that this is the default one, i.e. OpenNIDevice::OpenNI_12_bit_depth
//    auto depth_mode = pcl::io::openni2::PIXEL_FORMAT_DEPTH_1_MM;
//    if (find_switch (argc, argv, "-shift"))
//         TODO: Check that this is the correct one, i.e. OpenNIDevice::OpenNI_shift_values
//        depth_mode = pcl::io::openni2::PIXEL_FORMAT_SHIFT_9_3;

    if (parse_argument (argc, argv, "-buf", buff_size) != -1)
        print_highlight ("Setting buffer size to %d frames.\n", buff_size);
    else
        print_highlight ("Using default buffer size of %d frames.\n", buff_size);

    print_highlight ("Starting the producer and consumer threads... Press Ctrl+C to end\n");

    vector<NI2Grabber::Ptr> grabbers;

    for(auto id: device_ids) {
        NI2Grabber::Ptr grabber(new NI2Grabber(id));
        std::stringstream ss;
        ss << "Device string: (" << id << ")" << std::endl;
        Logger::log(Logger::INFO, ss.str());
        int id_num = std::stoi(id.substr(1));
        if(use_saved_intrinsics) {
            CameraIntrinsicsLoader::applyIntrinsics((size_t)id_num, grabber);
            CameraIntrinsicsLoader::copyIntrinsicsToOutputDir((size_t)id_num);
        }
        grabbers.push_back(grabber);
        cout << "Device id " << grabber->getName() << endl;
        Logger::log(Logger::INFO, id);
    }

    if(device_ids.size() == 3)
        CameraIntrinsicsLoader::copyExtrinsicsToOutputDir();

    // TODO: Resolve this
    // int buff_size = (int)(TOTAL_BUFFER_SIZE / grabbers.size());

    // Only xyz if any of the grabbers does not provide xyzrgba
    for(auto grabber: grabbers) {
        if(just_xyz)
            break;
        just_xyz = just_xyz || !grabber->providesCallback<NI2Grabber::sig_cb_openni_point_cloud_rgba>();
    }

    if(just_xyz) {
        print_highlight ("PointXYZ enabled.\n");
        Recorder<PointXYZ> recorder(grabbers, buff_size);
        recorder.start();
        signal(SIGINT, ctrlC);
        boost::mutex::scoped_lock lock(rmutex);
        while (!recording_done)
            stop_recording.wait(lock);
        recorder.stop();
    }
    else {
        print_highlight ("PointXYZRGBA enabled.\n");
        Recorder<PointXYZRGBA> recorder(grabbers, buff_size);
        recorder.start();
        signal(SIGINT, ctrlC);
        boost::mutex::scoped_lock lock(rmutex);
        while (!recording_done)
            stop_recording.wait(lock);
        recorder.stop();
    }

    // Copy the saved extrinsics to the same folder

    return (0);
}