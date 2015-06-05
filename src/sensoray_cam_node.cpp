
/*********************************************************************
*
*    File:    sensoray_cam_node
* Package:    sensoray_cam
*  Author:    Weikun Zhen (weikunz AT cmu DOT edu)
*    Date:    2015-05-26
*  Detail:    This program is a driver for sensoray model 2255
*             video capture box in ROS .
*
**********************************************************************/

#include <sensoray_cam/sensoray_cam.h>

// For communication to ROS
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

sig_atomic_t stopFlag = 0;

void handler( int )
{
    stopFlag = 1;
}

// the main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensoray_cam");
    ros::NodeHandle nh;

    // parameters
    std::string dev_name1, dev_name2, io_method_name, camera_name1, camera_name2, camera_info_url1, camera_info_url2;
    int image_width, image_height, framerate;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo1, cinfo2;

    // shared image message
    sensor_msgs::Image img1,img2;
    image_transport::CameraPublisher img_pub1,img_pub2;
    image_transport::ImageTransport it(nh);

    img_pub1 = it.advertiseCamera("/sensoray_cam_stereo/left/image_raw",1);
    img_pub2 = it.advertiseCamera("/sensoray_cam_stereo/right/image_raw",1);

    // grab the parameters
    nh.param("video_device1", dev_name1, std::string("/dev/video0"));
    nh.param("video_device2", dev_name2, std::string("/dev/video1"));

    // possible values: mmap, read, userptr
    nh.param("io_method", io_method_name, std::string("mmap"));
    nh.param("image_width", image_width, 640);
    nh.param("image_height", image_height, 480);
    nh.param("framerate", framerate, 10);

    // load the camera info
    nh.param("camera_frame_id1", img1.header.frame_id, std::string("stereo_left"));
    nh.param("camera_frame_id2", img2.header.frame_id, std::string("stereo_right"));
    nh.param("camera_name1", camera_name1, std::string("stereo_left"));
    nh.param("camera_name2", camera_name2, std::string("stereo_right"));
    nh.param("camera_info_url1", camera_info_url1, std::string("file://${ROS_HOME}/camera_info/calib_left.yaml"));
    nh.param("camera_info_url2", camera_info_url2, std::string("file://${ROS_HOME}/camera_info/calib_right.yaml"));
    cinfo1.reset(new camera_info_manager::CameraInfoManager(nh, camera_name1, camera_info_url1));
    cinfo2.reset(new camera_info_manager::CameraInfoManager(nh, camera_name2, camera_info_url2));

    // check camera info is published
    if(camera_info_url1.size() == 0 || camera_info_url2.size() == 0){
        ROS_ERROR("No camera_info is published, check the topic list");
    }

    // define sensoray camera capture object
    /*for debug, we only define one object for now*/
    SensorayCam cam1(dev_name1);
//    SensorayCam cam2(dev_name2);

    // the main loop
    int count = 0;
    while(ros::ok() && nh.ok() && stopFlag == 0)// && count < 10)
    {
        ROS_INFO("ros is ok.");
        // read image
        if(!cam1.grab_image()){
            ROS_WARN("%s: loses frame.", cam1.dev_name);
            stopFlag = 1;
            continue;
        }
        ROS_INFO("grabed a image");
//        if(!cam2.grab_image()){
//            ROS_WARN("%s: loses frame.", cam2.dev_name);
//            continue;
//        }

        //convert image
        cv_bridge::CvImage cv_image1;
        cv::Mat imgbuf1(cv::Size(image_width, image_height), CV_8UC3, cam1.myImgPtr);
        cv_image1.image = cv::imdecode(imgbuf1,1);
        cv_image1.encoding = "bgr8";
        cv_image1.toImageMsg(img1);

//        cv_bridge::CvImage cv_image2;
//        cv::Mat imgbuf2(cv::Size(image_width, image_height), CV_8UC3, cam1.myImgPtr);
//        cv_image2.image = cv::imdecode(imgbuf2,1);
//        cv_image2.encoding = "bgr8";
//        cv_image2.toImageMsg(img1);

        // grab the camera info
        sensor_msgs::CameraInfoPtr ci1(new sensor_msgs::CameraInfo(cinfo1->getCameraInfo()));
        ci1->header.frame_id = img1.header.frame_id;
        ci1->header.stamp = img1.header.stamp;

//        sensor_msgs::CameraInfoPtr ci2(new sensor_msgs::CameraInfo(cinfo2->getCameraInfo()));
//        ci2->header.frame_id = img2.header.frame_id;
//        ci2->header.stamp = img2.header.stamp;

        // publish image
        img_pub1.publish(img1, *ci1);
//        img_pub2.publish(img2, *ci2);

        ROS_INFO("loop iteration: %d", count++);
    }

    ROS_INFO("will exit ...");
    return 1;
}
