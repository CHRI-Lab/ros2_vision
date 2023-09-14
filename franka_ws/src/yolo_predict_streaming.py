from ultralytics import YOLO
import cv2

# Load pretrained YOLOv8 model
# The yolov8m model may be too computational intensive
model = YOLO('yolov8n.pt')
class_names = model.names

# Define streaming source (e.g. camera dev number)
# May need to try out for different ports
camera_dev = '1'

# Return a list of Results objects
results = model(source=camera_dev, 
                show=True, 
                # save=True, 
                boxes=True,
                stream=True
            )

# Process results list
for result in results:
    confidences = result.boxes.conf
    class_ids = result.boxes.cls
    box_coordinates = result.boxes.xyxy


    print("=============\n")
    for i in range(len(class_ids)):
        class_id = class_ids[i]
        coordinates = box_coordinates[i]

        x1, y1 = float(coordinates[0]), float(coordinates[1])
        x2, y2 = float(coordinates[2]), float(coordinates[3])

        print("Class:", class_names[int(class_id)])
        print("Confidence: {:.2f}".format(float(confidences[i])))
        print("Top left coordinates: ({:.2f}, {:.2f})".format(x1, y1))
        print("Bot right coordinates: ({:.2f}, {:.2f})".format(x2, y2))
        
        print("-----------\n")
