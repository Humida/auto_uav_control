# import lib
from ultralytics import YOLO

# import model
model = YOLO('yolov8n.pt')

# export model
model.export(format = 'engine')