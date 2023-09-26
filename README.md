# AMPF_UbuntuPC


Since ZED-ros2-wrapper and crazyswarm2 requires Ubuntu 22.04,
Ubuntu 22.04 jammy is recommended.

Installation Order
------------------
- - -
1. zed sdk    
<https://www.stereolabs.com/docs/installation/linux/>

cuda will be automatically installed while zed sdk installed.   
In order to check your zed sdk installed correctly, going through following step is strongly recommended   
<https://www.stereolabs.com/docs/app-development/cpp/linux/>   

2. ROS2 Humble    
<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

3. zed-ros2-wrapper & zed-ros2-examples     
<https://github.com/stereolabs/zed-ros2-wrapper>    
<https://github.com/stereolabs/zed-ros2-examples>

5. crazyswarm2    
<https://github.com/IMRCLab/crazyswarm2>

6. webot and webot-ros2
sudo apt-get install ros-humble-webot-ros2*
when you run the launch file, webot will be automatically installed.

