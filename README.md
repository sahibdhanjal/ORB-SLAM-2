# HomeZone SLAM code ROS base
This code is based on ORB_SLAM2.

### Quick Setup
 - Please pre-install everything in the prerequisites section. You can use the install scripts in the **install** folder to install OpenCV and Pangolin. ROS would best be installed depending on the version you are developing on. 
 - Unzip **ORBVoc.tar.xz** in the *vocabulary* folder. 
 - In case you want to use different camera parameters, please change **camera_params.yaml** in the *config* folder. 
 - To build the package, clone the following in `..../catkin_ws/src/` use the following command from `..../catkin_ws/`:
`catkin_make --pkg orb_slam` **OR** `catkin build orb_slam`
 - To run the node, use the following command: `roslaunch orb_slam ORBSlam.launch`

# Prerequisites
We have tested the library in **Ubuntu 16.04** and **16.04**, but it should be easy to compile in other platforms. 
 - C++11 or C++0x Compiler - the new thread and chrono functionalities of C++11.
 - [Pangolin](https://github.com/stevenlovegrove/Pangolin) - used for visualization and user interface. General issues faced can be fixed by following [this document](https://woojjang.tistory.com/52).
 - OpenCV - used to manipulate images and features. **Required at least 2.4.3. Tested with OpenCV 2.4.11, OpenCV 3.2.0 and 4.1.0**.
 - Eigen3 - required by g2o (see thirdparty). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.
 - DBoW2 and g2o (Included in `include/thirdparty` folder) - modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations.
 - ROS - version Hydro or newer is needed.


## ORB-SLAM2 Authors 
[Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)).
The original implementation can be found [here](https://github.com/raulmur/ORB_SLAM2.git).

### Related Publications:

[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

