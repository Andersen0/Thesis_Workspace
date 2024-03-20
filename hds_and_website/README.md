# hds_and_website

To build the packages run:
```
colcon build --packages-select yolov6
```
and
```
colcon build --packages-select pyflask
```
To use this project you will need a few packages.

## For the Realsense depth-camera packages:

You can follow the installation section here : https://github.com/IntelRealSense/realsense-ros#installation

Or simply run:
```
sudo apt install ros-<ROS_DISTRO>-librealsense2*
```
and
```
sudo apt install ros-<ROS_DISTRO>-realsense2-*
```

For the YOLO package you will need a few packages like CV2 and pytorch, but I leave the installation up to you
You will probably get nice error messages for missing packages

## Before running the yolo prediction you have to launch the camera node. Run:
```
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true 
```

Among other things this makes sure the resolution and FOV of the RBG image and depth image is the same. This is required by the current yolo solution

## To run the Yolo-prediction by itself:
```
ros2 run yolov6 inferer
```

## If you get an error message that says:
**ImportError: /lib/x86_64-linux-gnu/libstdc++.so.6: cannot allocate memory in static TLS block**

You can run:
```
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libgcc_s.so.1:$LD_PRELOAD
```

# ras_reliability_backend

## packages

If this is your first time using this package remember to do the following:
```
sudo apt install ros-<ROS_DISTRO>-rosbridge-server
source /opt/ros/<ROS_DISTRO>/setup.bash
```


# Launching package

## To run the website together with the yolo prediction:
```
ros2 launch pyflask my_launch_file.launch
```

## If you want to change the yolo model:

The path to the .pt file and the .yaml file that contains the corresponding labels can be changed at the bottom of yolov6/yolov6/core/inferer.py

inferer.py is pretty much the only file you will want to edit for the yolo model. Except maybe the model .pt file and label yaml. Everything you will need is probably there.

You can also change which image topic is sent to the website and shown on your pc

(CHECK THIS) The possible images are:
raw image topic from depth camera

img_ori (full rbg with predictions)

pred_img (rgb image cropped with only predicitons)

pred_depth_img (depth image cropped with only predicitons, sections are used to calculate distance)




