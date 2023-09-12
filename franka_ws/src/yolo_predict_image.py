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

    confidences = result.boxes.conf
    class_ids = result.boxes.cls
    box_coordinates = result.boxes.xyxy

    for i in range(len(class_ids)):
        class_id = class_ids[i]
        coordinates = box_coordinates[i]

        x1, y1 = float(coordinates[0]), float(coordinates[1])
        x2, y2 = float(coordinates[2]), float(coordinates[3])

        print("Class:", class_names[int(class_id)])
        print("Confidence: {:.2f}".format(float(confidences[i])))
        print("Top left coordinates: ({:.2f}, {:.2f})".format(x1, y1))
        print("Bot right coordinates: ({:.2f}, {:.2f})".format(x2, y2))
        
        print("\n-----------\n")
