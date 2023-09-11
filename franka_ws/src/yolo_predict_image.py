from ultralytics import YOLO
import cv2

# Load pretrained YOLOv8 model
model = YOLO('yolov8m.pt')

# Run batched inference on a list of images
source_images = ['sample_image_1.jpeg', 'sample_image_2.jpeg']

# Return a list of Results objects
results = model(source=source_images[1], 
                # conf=0.8,
                show=True, 
                save=True, 
                boxes=True
            )

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bbox outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
