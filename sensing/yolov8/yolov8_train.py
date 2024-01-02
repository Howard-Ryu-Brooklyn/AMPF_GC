from ultralytics import YOLO

# Load a model
# model = YOLO('yolov8n.yaml')  # build a new model from YAML
# model = YOLO('yolov8m.pt')  # load a pretrained model (recommended for training)
# model = YOLO('yolov8n.yaml').load('yolov8n.pt')  # build from YAML and transfer weights

# model = YOLO('/home/humble/ros2_ws/src/AMPF_UbuntuPC/sensing/yolov8/runs/detect/train/weights/best.pt')  # load a pretrained model (recommended for training)

model = YOLO('yolov8m.pt')  # load a pretrained model (recommended for training)

# Train the model
model.train(data='red_crazyflie_data/data.yaml', epochs=1000, imgsz=640)


