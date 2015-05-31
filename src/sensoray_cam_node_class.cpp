/*********************************************************************
*
*    File:    ros2255drv_node
* Package:    ros2255drv
*  Author:    Weikun Zhen (weikunz AT cmu DOT edu)
*    Date:    2015-05-26
*  Detail:    This program is a driver for sensoray model 2255
*             video capture box in ROS (Robotics Operating System).
*
**********************************************************************/

#include <sensoray_cam/sensoray_cam.h>

// For communication to ROS
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>


#include <sstream>

class SensorayCamNode
{
public:
    // ROS node handle
    ros::NodeHandle nh;

    // shared image message
    sensor_msgs::CompressedImage img1, img2;
    image_transport::CameraPublisher image_pub1, image_pub2;
    
    // parameters
    std::string dev_name1, dev_name2, io_method_name, camera_name1, camera_name2, camera_info_url1,camera_info_url2;
    int image_width, image_height, framerate;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo1, cinfo2;

    // checker of dev name
    int duplicate;

    int count;

    std::string dev1;// = std::string("/dev/video0");
    std::string dev2;// = std::string("/dev/video1");
    SensorayCam* scam1;//(dev1);
    SensorayCam* scam2;//(dev2);

    // the constructor
    SensorayCamNode(ros::NodeHandle n): nh(n)
    {
        count = 0;

        dev1 = "/dev/video0";
        dev2 = "/dev/video1";
        scam1 = new SensorayCam(dev1);
//        scam2 = new SensorayCam(dev2);

        //signal(SIGINT, &scam1.sigHandler);
        // advertise the main image topic
        image_transport::ImageTransport it(nh);
        image_pub1 = it.advertiseCamera("/sensoray_cam_stereo/left/image_raw",1);
        image_pub2 = it.advertiseCamera("/sensoray_cam_stereo/right/image_raw",1);
        
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

        // pass the device name to sensoray_cam objects
//        duplicate = 0;
//        if(std::strcmp(dev_name1.c_str(),dev_name2.c_str()) == 0){
//            ROS_WARN("Duplicated device name, choose to open %s", dev_name1.c_str());
//            duplicate = 1;
//        }
//        std::strcpy(scam1.dev_name, dev_name1.c_str());
//        std::strcpy(scam2.dev_name, dev_name2.c_str());

        // start the devices
//        scam1.open_device();
//        scam1.init_device();
//        scam1.start_capturing();

//        if(duplicate == 0){
//            scam2.open_device();
//            scam2.init_device();
//            scam2.start_capturing();
//        }
    }

    // the destructor
    ~SensorayCamNode()
    {
        // close the devices
//        scam1.stop_capturing();
//        scam1.uninit_device();
//        scam1->close_device();
        delete scam1;
//        if(duplicate == 0){
//            scam2.stop_capturing();
//            scam2.uninit_device();
//            scam2.close_device();
//        }
        ROS_INFO("DESTRUCTED");
    }

    // read image and publish
    void read_and_publish_image(const ros::TimerEvent& e)
    {
        if(ros::ok() && count < 10)
        {
            ROS_INFO("read image count = %d", count++);
            // read the images
            if(!scam1->grab_image())
            {
                ROS_WARN("Device '%s' loses frame.", scam1->dev_name);
            }
            //        if(duplicate == 0){
            //            if(!scam2.grab_image())
            //            {
            //                ROS_WARN("Device '%s' loses frame.", scam2.dev_name);
            //            }
            //        }
        }
        else
        {
            delete scam1;
        }
    }

    bool run()
    {
        ros::Rate loop_rate(1);
        int count=0;
        while (count < 5 && nh.ok())
        {
//            read_and_publish_image();
//            ros::spinOnce();
            ROS_INFO("%d",count++);

           // loop_rate.sleep();
//            sleep(1);
            ROS_INFO("sleep ends");
        }
//        ros::Timer timer = nh.createTimer(ros::Duration(1.0/framerate), read_and_publish_image);

        return true;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensoray_cam");
    ros::NodeHandle n("~");

    SensorayCamNode a(n);
    ros::Timer timer = n.createTimer(ros::Duration(0.1), &SensorayCamNode::read_and_publish_image, &a);

//    a.run();

    ros::spin();
    return EXIT_SUCCESS;

}

