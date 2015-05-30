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

// warpper function declaration
static void errno_exit(const char *s);
static int  xioctl(int fd, int request, void *arg);

// define class SensorayCam
class SensorayCam
{
public:
    // image I/O methods
    typedef enum {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
    } io_method;

    // image buffer struct
    struct buffer {
        void * start;
        size_t length;
    };

    SensorayCam();
    ~SensorayCam();

    // member variables
    char            dev_name[100];
    io_method       io;
    buffer*         buffers = NULL;
    int             fd;
    unsigned int    n_buffers;
    int             G_quality;
    int             bPAL;
    int             bSize; // 4 CIFS(other settings 1 or 2)

    // member functions
    int  read_frame(void);
    void process_image(const void *p);
    bool grab_image(void);
    void stop_capturing (void);
    void start_capturing (void);
    void uninit_device (void);
    void init_device (void);
    void close_device (void);
    void open_device (void);

private:
    void init_read(unsigned int buffer_size);
    void init_mmap(void);
    void init_userp	(unsigned int buffer_size);

};

#endif
