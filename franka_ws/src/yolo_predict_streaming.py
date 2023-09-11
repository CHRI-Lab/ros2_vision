from ultralytics import YOLO
import cv2

# Load pretrained YOLOv8 model
# The yolov8m model may be too computational intensive
model = YOLO('yolov8n.pt')

# Define streaming source (e.g. camera dev number)
# May need to try out for different ports
camera_dev = '1'

# Return a list of Results objects
results = model(source=camera_dev, 
                show=True, 
                # save=True, 
                boxes=True
            )

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bbox outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
