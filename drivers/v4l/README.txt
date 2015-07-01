November, 2014:
      Latest updates.
      Please note that some distributions of Linux may include a precompiled V4L driver.  This driver should be reasonably up to date on newer kernel version (3.12 and higher).  On older kernel versions (3.2 and lower), this driver which contains all bug fixes and new features is recommended.

June 27, 2012:
      VIDIOC_S_PARM capturemode fixed.

February 7, 2012:
      latest DSP firmware
      changed build scripts. No longer uses build.pl.
      If using old kernel versions(2.6.18-2.6.23), you must download 
      a backport from linuxtv OR use the non-V4L driver.

September 27, 2010:
V122:
      Latest DSP firmware 
      Build scripts streamlined
      JPEG enable module parameter
      Serial Number retrieval (see capture.c demo with "-s" option)
      Firmware retrieval (see capture.c demo with "-f" option)

      Jpeg disable description.
      xawtv on some systems chooses JPG capture by default. 
      This is problematic if the EVAL JPEG firmware is used as a
      black line through the middle of the image will be seen.
      A driver parameter was added to disable JPEG to prevent this
      behavior.  To disable JPEG:
         "modprobe -r s2255drv"
	 "modprobe s2255drv jpeg_enable=0"
      

March 8, 2010:
V121: support for 2257
      frame decimation added
      latest bug fixes

      works for 2.6.24-2.6.33 without updating V4L libraries(eg videodev2,
      videobuf-common modules).

      (2.6.18-2.6.23 requires full linuxtv update V4L libraries)

April 7, 2009:

V120: latest bug fixes and patches
      (important and recommended updates)
      Added latest linuxtv self build plus patch
      (Required for 2.6.18-2.6.23 and for 2.6.27+).
      New build.  Run ./build.pl instead of makefile.
      (If using non-eval JPEG firmware, please make sure to 
       copy it to /lib/firmware after installing the driver
        with build.pl)

	(Another recommended V4L2 display program to try is tvtime,
	"apt-get install tvtime" on Ubuntu)


V119: intermediate release (Beta)

Sensoray only supports V4L2 and not the obsolete V4L1(version one).

This driver should work with V4L2 apps.  See capture.c and jpeg.c
V118: fixed frame sequence number problem
      added V4L2_MODE_HIGHQUALITY for interpolated 4CIF output
      (see V4L2 API specs VIDIOC_S_PARAM, TABLE 2 and TABLE 5 for usage)
      added detection of video signal in VIDIOC_ENUMINPUT

Note: If you are using kernel 2.6.18-2.6.23, you can get a backport of
      the s2255drv V4L driver from http://linuxtv.org/hg/v4l-dvb.
      Download the tar.gz file and compile it.  This gives you 
      the latest bug fixes backported for all the V4L kernel
      modules.
      


V117: added JPEG quality control
To test JPEG capture, type "make jpeg" to compile, "./jpeg" to run

capture.c app is from from http://v4l2spec.bytesex.org/v4l2spec

Another Linux V4L2 app is xawtv. (see below)

Note: This driver is a supplement to s2255.ko.  It supports
      the V4L2 API.  The other driver uses a non-V4L2 API, which may
      or may not be preferred by some customers.

      Only one driver s2225.ko or s2255drv.ko(formerly s2255v.ko)
      	    can be used at one time(see below).

      Whether the V4L version 1 compatibility routines work
      is a function of the current kernel.
      Because V4L 1 is marked obsolete, it is highly recommended to
      use V4L2(version 2) exclusively.  
      videodev.h will very soon be removed in later versions of the kernel.

      To use the 2255 on older kernels, please see the non-V4L2 API.


FAQ
1) Q. Why does your device not work with VideoLan 0.8.6c, e,f?
   A. VideoLan 0.8.6 uses the V4L version one API, which is obsolete.
      VideoLan 0.8.6 V4L1 also assumes that all video devices are tuners.
      It exits when it fails to set a tuner channel.
      The good news is that VideoLan 9 has a V4L2 interface.  
      VLC9 has not been released(as of 04/23/2008), 
      but is available via svn/git or on Ubuntu Hardy through VLC's nightly
      builds.  
      This driver has been tested with the latest version of 
      VideoLan 9 as of April 23, 2008.

Rev notes: August 4,2008 
-changed name to s2255drv to match kernel
-code updated to kernel version(2.6.27)
-RGB conversions removed.  color conversions should be done in user
 space.  Use appropriate routines/libraries as desired.

Rev notes: April 21,2008 
-fixed intermittent deadlock problem on start/stop
-xawtv snapshots working

Dependencies:(kernel modules which must be loaded)
videobuf_vmalloc, videobuf_core, v4l2_common, usbcore, videodev

Installation

This version February 2008 was designed for the 2.6.24 kernel.
No backward revisions(2.6.22 for example) are planned at this time.


chmod 755 Makefile   (in case)
make
sudo make modules_install (installs driver, removes all old drivers and default kernel driver if present).
sudo modprobe s2255drv  (or reboot to load driver)


Example Usage

Opens xawtv on channel one(assumes no other V4L devices, such
      	       	           that the minor is zero)

xawtv -nodga -c /dev/video0

Opens a second xawtv instance on channel two(assumes no other V4L devices, such
      	       	           that the minor is one)
xawtv -nodga -c /dev/video1


Notes on CPU usage:

For maximum efficiency, capture in V4L2_PIX_FMT_YUV422P or V4L2_PIX_FMT_GREY
V4L2_PIX_FMT_YUYV has some software reordering.

The RGB capture modes use software conversion.  In order to get 2 channels of full color at full frame rate, the 2255 transfers across the USB bus in YUV format.  The current RGB conversion routines have been tested but may not be the most efficient(future TODO).

