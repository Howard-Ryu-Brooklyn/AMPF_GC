from ultralytics import YOLO

# Load an official or custom model
# model = YOLO('yolov8n.pt')  # Load an official Detect model
# model = YOLO('yolov8n-seg.pt')  # Load an official Segment model
# model = YOLO('yolov8n-pose.pt')  # Load an official Pose model
model = YOLO('/home/humble/ros2_ws/src/AMPF_UbuntuPC/sensing/yolov8/runs/detect/epoch100/weights/best.pt')  # Load a custom trained model

# Perform tracking with the model
# results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True)  # Tracking with default tracker
# results = model(source="/home/humble/Documents/ZED/data/land_converted_left.mp4", show=True)  # predict
results = model.track(source="/home/humble/Documents/ZED/data/fly_converted_right.mp4", show=True, tracker="bytetrack.yaml")  # Tracking with ByteTrack tracker