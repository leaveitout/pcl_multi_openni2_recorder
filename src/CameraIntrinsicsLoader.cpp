//
// Created by sean on 15/01/16.
//

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <fstream>
#include "CameraIntrinsicsLoader.hpp"

bool CameraIntrinsicsLoader::applyIntrinsics(size_t camera_num, pcl::io::OpenNI2Grabber::Ptr grabber) {
    std::string path = "/home/sean/Documents/cameraparams/";
    std::stringstream ss1, ss2;
    ss1 << path << camera_num << "rgb.yml";
    ss2 << path << camera_num << "ir.yml";
    std::string rgb_filename = ss1.str();
    std::string ir_filename = ss2.str();

    cv::FileStorage fs_rgb;
    if(!fs_rgb.open(rgb_filename.c_str(), cv::FileStorage::READ) )
        return false;
    cv::FileStorage fs_ir;
    if(!fs_ir.open(ir_filename.c_str(), cv::FileStorage::READ) )
        return false;

    cv::Mat C_rgb = cv::Mat_<double>::zeros(3, 3);
    cv::Mat C_ir = cv::Mat_<double>::zeros(3, 3);

    fs_rgb["camera_matrix"] >> C_rgb;
    fs_ir["camera_matrix"] >> C_ir;

    double fx_rgb = C_rgb.at<double>(0, 0);
    double fy_rgb = C_rgb.at<double>(1, 1);
    double cx_rgb = C_rgb.at<double>(0, 2);
    double cy_rgb = C_rgb.at<double>(1, 2);

    double fx_ir = C_ir.at<double>(0, 0);
    double fy_ir = C_ir.at<double>(1, 1);
    double cx_ir = C_ir.at<double>(0, 2);
    double cy_ir = C_ir.at<double>(1, 2);

    grabber->setRGBCameraIntrinsics(fx_rgb, fy_rgb, cx_rgb, cy_rgb);
    grabber->setDepthCameraIntrinsics(fx_ir, fy_ir, cx_ir, cy_ir);

    return true;
}

bool CameraIntrinsicsLoader::copyIntrinsicsToOutputDir(size_t camera_num) {
    std::string path = "/home/sean/Documents/cameraparams/";
    std::stringstream ss1, ss2, ss3, ss4;

    ss1 << path << camera_num << "rgb.yml";
    ss2 << path << camera_num << "ir.yml";
    ss3 << camera_num << "rgb.yml";
    ss4 << camera_num << "ir.yml";
    std::string rgb_input_path = ss1.str();
    std::string ir_input_path = ss2.str();
    std::string rgb_output_filename = ss3.str();
    std::string ir_output_filename = ss4.str();

    std::ifstream  src1(rgb_input_path, std::ios::binary);
    std::ofstream  dst1(rgb_output_filename, std::ios::binary);

    dst1 << src1.rdbuf();

    src1.close();
    dst1.close();

    std::ifstream  src2(ir_input_path, std::ios::binary);
    std::ofstream  dst2(ir_output_filename,   std::ios::binary);

    dst2 << src2.rdbuf();

    src2.close();
    dst2.close();

    return true;
}

bool ::CameraIntrinsicsLoader::copyExtrinsicsToOutputDir() {
    std::string stereo_1_2_filepath = "/home/sean/Documents/cameraparams/12stereo.yml";
    std::string stereo_3_2_filepath = "/home/sean/Documents/cameraparams/32stereo.yml";
    std::string stereo_1_2_output = "12stereo.yml";
    std::string stereo_3_2_output = "32stereo.yml";

    std::ifstream  src1(stereo_1_2_filepath , std::ios::binary);
    std::ofstream  dst1(stereo_1_2_output, std::ios::binary);

    dst1 << src1.rdbuf();

    src1.close();
    dst1.close();

    std::ifstream  src2(stereo_3_2_filepath, std::ios::binary);
    std::ofstream  dst2(stereo_3_2_output, std::ios::binary);

    dst2 << src2.rdbuf();

    src2.close();
    dst2.close();
}
