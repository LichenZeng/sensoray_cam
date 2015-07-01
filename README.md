# sensory_cam
This program provides a ros interface to capture images via sensoray video capture card model 2255. 

## Install sensoray capture card driver on Linux. 
```
#!bash
        cd your_checkout_folder/sensoray_cam/drivers/v4l/
        chmod 755 Makefile
        make
        sudo make modules_install
        sudo modprobe s2255drv
```
You may need to reboot to load driver