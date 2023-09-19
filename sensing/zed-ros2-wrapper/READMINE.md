used command 
ZED 2i:
```bash
$ ros2 launch zed_wrapper zed2i.launch.py
```

config/common.yaml
depth_mode: 'NEURAL' -> accurate and smooth depth
pos_tracking_enabled: true -> for rviz tf
false for other mode like gnss, object detection etc..

