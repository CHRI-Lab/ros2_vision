from ultralytics import YOLO
import cv2

# Load pretrained YOLOv8 model
model = YOLO('yolov8m.pt')
class_names = model.names

# Run batched inference on a list of images
source_images = ['sample_image_1.jpeg', 'sample_image_2.jpeg']

# Return a list of Results objects
results = model(source=source_images[0],
                # conf=0.8,
                show=True,
                save=True,
                boxes=True
                )

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bbox outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs

    con = result.boxes.conf
    class_ids = result.boxes.cls
    # x1 = int(result['xmin'])
    # y1 = int(result['ymin'])
    # x2 = int(result['xmax'])
    # y2 = int(result['ymax'])

    for class_id in class_ids:
        print("Class:", class_names[int(class_id)])
    print("\n-----------\n")
