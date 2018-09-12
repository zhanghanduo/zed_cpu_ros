zed_cpu_ros
===========
A simple zed camera driver which only use CPU and only publish left and right raw images, rectified image and its camera info.
This repo is forked from [willdzeng](https://github.com/willdzeng/zed_cpu_ros.git).

# Usage:
1. git the packge into your working space

    ```
    cd catkin_ws/src
    git clone https://github.com/zhanghanduo/zed_cpu_ros.git
    cd ..
    catkin_make
    ```
2. Get your calibration files:
    You can get your calibration files from zed or do a calibration your self by using ROS camera calibration package.

    (1). From zed:

    Find your zed calibration files in
    ```
    cd /usr/local/zed/settings
    ```
    or download from:
    http://calib.stereolabs.com/?SN=XXXX

    Note: XXXX is your last four digit S/N of your camera, make sure to change it!!

    put the .conf file into zed_cpu_ros/config folder

    update launch file configuration file name in zed_cpu_ros.launch into your SNXXXX.conf
    ```
    roscd zed_cpu_ros/launch
    ```
    change XXXX into the .conf file you have, for example 1010
    ```
    <arg name="config_file_location" default="$(find zed_cpu_ros)/config/SN1010.conf"/>
    ```

    (2). Do a calibration yourself:

    This option is suggested. Reference: http://wiki.ros.org/camera_calibration
    ```
    roslaunch zed_cpu_ros camera_calibration.launch
    ```
    After calibration:
    Find the left.yaml and right.yaml in the tar file and put them into the zed_cpu_ros/config folder.
    The calibration file will be loaded if you turn <load_zed_config> off in the launch file.

3. launch the code
    ```
    roslaunch zed_cpu_ros zed_cpu_ros.launch
    ```
## Launch file parameters

 Parameter                    |           Description                                       |              Value
------------------------------|-------------------------------------------------------------|-------------------------
 device_id                    | device_id selection                                         | int
 resolution                   | ZED Camera resolution                                       | '0': HD2K
 _                            | _                                                           | '1': HD1080
 _                            | _                                                           | '2': HD720
 _                            | _                                                           | '3': VGA
 frame_rate (for HD2K)        | Rate at which images are published (fps)                    | int (15)
 frame_rate (for HD1080)      | Rate at which images are published (fps)                    | int (15, 30)
 frame_rate (for HD720)       | Rate at which images are published (fps)                    | int (15, 30, 60)
 frame_rate (for VGA)         | Rate at which images are published (fps)                    | int (15, 30, 60, 100)
 left_frame_id                | Left Frame ID                                               | string
 right_frame_id               | Right Frame ID                                              | string
 load_zed_config              | Whether to use ZED calibration file                         | bool
 config_file_location         | The location of ZED calibration file                        | string
 rectify                      | Whether rectify image                                       | bool
 show_image                   | Whether to use opencv show image                            | bool
 encoding                     | image encoding                                              | string

# TODO:

- [ ] add the nodelet functionality.

# Author:
Zhang Handuo
