
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
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>
#include "std_msgs/String.h"

sig_atomic_t stopFlag = 0;

// function declaration
void handler( int );
bool grab_image(SensorayCam *cam1, SensorayCam *cam2);
bool convert_image(SensorayCam *cam1, SensorayCam *cam2,
                   int image_width, int image_height,
                   sensor_msgs::Image &img1,sensor_msgs::Image &img2);


// the main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensoray_cam_stereo");
    ros::NodeHandle nh("~");

    signal(SIGINT, handler);

    // parameters
    std::string dev_name1, dev_name2, io_method_name, camera_name1, camera_name2, camera_info_url1, camera_info_url2;
    int image_width, image_height, framerate, b_size;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo1, cinfo2;

    // shared image message
    sensor_msgs::Image img1,img2;
    image_transport::CameraPublisher img_pub1,img_pub2;
    image_transport::ImageTransport it(nh);

    img_pub1 = it.advertiseCamera("/sensoray_cam_stereo/left/image_raw",1);
    img_pub2 = it.advertiseCamera("/sensoray_cam_stereo/right/image_raw",1);

    // get launch file specified parameter
    if(!nh.getParam("b_size", b_size)){
        nh.param("b_size", b_size, 2);
    }
    if(!nh.getParam("video_device1", dev_name1)){
        nh.param("video_device1", dev_name1, std::string("/dev/video0"));
    }
    if(!nh.getParam("video_device2", dev_name2)){
        nh.param("video_device2", dev_name2, std::string("/dev/video1"));
    }
    if(!nh.getParam("camera_info_url1", camera_info_url1)){
        nh.param("camera_info_url1", camera_info_url1, std::string("file://${ROS_HOME}/camera_info/calib_left.yaml"));
    }
    if(!nh.getParam("camera_info_url2", camera_info_url2)){
        nh.param("camera_info_url2", camera_info_url2, std::string("file://${ROS_HOME}/camera_info/calib_right.yaml"));
    }

    // grab the default parameters
    nh.param("io_method", io_method_name, std::string("mmap"));
    nh.param("image_width", image_width, 160*b_size);
    nh.param("image_height", image_height, 120*b_size);
    nh.param("framerate", framerate, 30);

    // load the camera info
    nh.param("camera_frame_id1", img1.header.frame_id, std::string("stereo_left"));
    nh.param("camera_frame_id2", img2.header.frame_id, std::string("stereo_right"));
    nh.param("camera_name1", camera_name1, std::string("stereo_left"));
    nh.param("camera_name2", camera_name2, std::string("stereo_right"));

    cinfo1.reset(new camera_info_manager::CameraInfoManager(nh, camera_name1, camera_info_url1));
    cinfo2.reset(new camera_info_manager::CameraInfoManager(nh, camera_name2, camera_info_url2));

    // check camera info is published
    if(camera_info_url1.size() == 0 || camera_info_url2.size() == 0){
        ROS_ERROR("No camera_info is published, check the topic list");
    }

    // define sensoray camera capture object
    SensorayCam cam1(dev_name1, b_size);
    SensorayCam cam2(dev_name2, b_size);

    int count = 0;
    while(stopFlag == 0){

        // get image from specified device
        grab_image(&cam1, &cam2);

        // convert image to sensor_msgs::Image
        if(!convert_image( &cam1, &cam2, image_width, image_height, img1, img2)) continue;

        // grab the camera info
        sensor_msgs::CameraInfoPtr ci1(new sensor_msgs::CameraInfo(cinfo1->getCameraInfo()));
        ci1->header.frame_id = img1.header.frame_id;
        ci1->header.stamp = img1.header.stamp;

        sensor_msgs::CameraInfoPtr ci2(new sensor_msgs::CameraInfo(cinfo2->getCameraInfo()));
        ci2->header.frame_id = img2.header.frame_id;
        ci2->header.stamp = img2.header.stamp;

        // publish image
        img_pub1.publish(img1, *ci1);
        img_pub2.publish(img2, *ci2);

        // counter
//        ROS_INFO("%d",count++);
    }
    return 1;
}

void handler( int )
{
    stopFlag = 1;
}

bool grab_image(SensorayCam *cam1, SensorayCam *cam2)
{
    int c1 = 0, c2 = 0;
    for(int count = 0; count < 10; count++){
        fd_set fds, ofds;
        struct timeval tv;
        int r;

        FD_ZERO (&fds);
        FD_ZERO (&ofds);
        FD_SET (cam1->fd, &fds);
        FD_SET (cam2->fd, &fds);

        /* Timeout. */
        tv.tv_sec = 5;
        tv.tv_usec = 0;

        // wait until the driver has captured data
        r = select (std::max(cam1->fd, cam2->fd) + 1, &fds, &ofds, NULL, &tv);
        if (-1 == r) {
            if (EINTR == errno)
                continue;
            errno_exit ("select");
        }

        if (0 == r) {
            fprintf (stderr, "select timeout\n");
            exit (EXIT_FAILURE);
        }

        if (c1 == 0){
            if (FD_ISSET(cam1->fd, &fds)) {
                if (cam1->read_frame ())
                    c1 = 1;
            }
        }
        if (c2 == 0){
            if (FD_ISSET(cam2->fd, &fds)) {
                if (cam2->read_frame ())
                    c2 = 1;
            }
        }

        if (c1 == 1 && c2 == 1){
            break;
        }
    }
}

bool convert_image(SensorayCam *cam1, SensorayCam *cam2,
                   int image_width, int image_height,
                   sensor_msgs::Image &img1,sensor_msgs::Image &img2)
{
    // check if the image pointer is null
    if(cam1->myImgPtr == NULL || cam2->myImgPtr == NULL){
        if(cam1->myImgPtr == NULL){
            ROS_WARN("lose a frame on cam 1 ");
        }
        if(cam2->myImgPtr == NULL){
            ROS_WARN("lose a frame on cam 2 ");
        }
        return 0;
    }

    cv_bridge::CvImage cv_image1;
    cv::Mat imgbuf1(cv::Size(image_width, image_height), CV_8UC3, cam1->myImgPtr);
    cv_image1.image = cv::imdecode(imgbuf1,1);
    cv_image1.encoding = "bgr8";
    cv_image1.toImageMsg(img1);
    img1.header.stamp = cam1->stamp;


    cv_bridge::CvImage cv_image2;
    cv::Mat imgbuf2(cv::Size(image_width, image_height), CV_8UC3, cam2->myImgPtr);
    cv_image2.image = cv::imdecode(imgbuf2,1);
    cv_image2.encoding = "bgr8";
    cv_image2.toImageMsg(img2);
    img2.header.stamp = cam1->stamp;

    return 1;
}
