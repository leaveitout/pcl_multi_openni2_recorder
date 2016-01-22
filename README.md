# pcl_multi_openni2_recorder
Program to record pcd files from multiple OpenNI2 compatible devices simultaneously.

This is an extension to the pcl_openni_pcd_recorder in the Point Cloud Library. 

It has been extended to work with multiple cameras and to use the newer OpenNI2 library.
It uses a separate buffer for each camera, which is accessed through consumer and producer threads for each camera.
It allows for real-time recording from multiple cameras (depending on hardware).

Requirements are:
C++11 capable compiler
PCL install
