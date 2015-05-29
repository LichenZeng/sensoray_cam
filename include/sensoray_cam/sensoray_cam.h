#ifndef SENSORAY_CAM_SENSORAY_CAM_H
#define SENSORAY_CAM_SENSORAY_CAM_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <asm/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>

#include <ros/ros.h>

// Function prototypes for video control and info output
static void errno_exit(const char *s);
static int xioctl(int fd, int request, void *arg);
static volatile int g_stopping = 0;
static void sigHandler(int type)
{
    g_stopping = 1;
    ROS_INFO("stopping\n");

}

// Define class SensorayCam
class SensorayCam
{
public:
    // Define image I/O methods
    typedef enum {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
    } io_method;

    // Define image buffer struct
    struct buffer {
        void * start;
        size_t length;
    };

    SensorayCam();
    ~SensorayCam();

    char*           dev_name;
    io_method       io;
    buffer*         buffers = NULL;
    int             fd;
    unsigned int    n_buffers;
//    volatile int    g_stopping;
    int             G_quality;

    int             bPAL;
    int             bSize; // 4 CIFS(other settings 1 or 2)

//    void sigHandler(int type);
    int  read_frame(void);
    void process_image(const void *p);
    bool grab_image(void);
    void stop_capturing (void);
    void start_capturing (void);
    void uninit_device (void);
    void init_device (void);
    void close_device (void);
    void open_device (void);
//    void usage (FILE * fp,int argc,char ** argv);
//    int EnumVideo(int display);
//    int Check2255(char *devname);
//    void test_video_status(void);

private:
    void init_read(unsigned int buffer_size);
    void init_mmap(void);
    void init_userp	(unsigned int buffer_size);


    //const char short_options [] = "d:ehmruisf";

    /*const struct option long_options [] = {
        { "device",     required_argument,      NULL,           'd' },
        { "enumerate",  no_argument,            NULL,           'e' },
        { "help",       no_argument,            NULL,           'h' },
        { "mmap",       no_argument,            NULL,           'm' },
        { "read",       no_argument,            NULL,           'r' },
        { "userp",      no_argument,            NULL,           'u' },
        { "interp",     no_argument,            NULL,           'i' },
        { "serial",     no_argument,            NULL,           's' },
        { "firmware",     no_argument,            NULL,         'f' },
        { 0, 0, 0, 0 }
    };*/

    //const char *cardname = "s2255";
};

#endif
