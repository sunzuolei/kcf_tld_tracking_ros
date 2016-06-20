# kcf_tld_tracking_ros
Visual target tracking with KCF and TLD. 
Three packages including KCF, KCFtld and OpenTLD. you can run these three packages using Kinect.

# How to build
## Dependencies
* [ROS](http://www.ros.org)
* OpenCV 3.0
* CMake
* Kinect Dirver [freenect](http://wiki.ros.org/freenect_launch)

put the package into src folder of ros workspace then build it.  

#How to run
```
roslaunch KCF kcf.launch 
roslaunch KCFtld kcftld.launch
roslaunch OpenTLD opentld.launch
```
Thanks to [Klaus Haag](https://github.com/klahaag/CFtld)!
## Contributor
-------------------
- 张子洋: [zzy@mpig.com.cn](zzy@mpig.com.cn)

---------
Cheers!
:panda_face:
