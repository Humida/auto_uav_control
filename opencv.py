import cv2
import time
from ultralytics import YOLO
import torch

# device = 'cuda' if torch.cuda.is_available() else 'cpu'
# print(f'Using device: {device}')

# Load the YOLOv8 model
model = YOLO('yolov8n.engine')
# num_devices = cv2.cuda.getCudaEnabledDeviceCount()
# cv2.cuda.setDevice(0)
# print(cv2.__version__)

# Open the video file
cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw, format = YUY2, width=640, height=480, framerate=30/1  ! videoconvert ! video/x-raw,format=BGR ! appsink")
# used to record the time when we processed last frame 
prev_frame_time = 0
  
# used to record the time at which we processed current frame 
new_frame_time = 0

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()
    # time when we finish processing for this frame 
    if success:
        
        new_frame_time = time.time() 

        fps = 1/(new_frame_time-prev_frame_time) 
        
        prev_frame_time = new_frame_time 
        
        # converting the fps into integer 
        fps = int(fps) 
    
        # converting the fps to string so that we can display it on frame 
        # by using putText function 
        fps = str(fps) 
        results = model.track(frame, persist=True)
        
        # frame_output =  result.plot()

        for result in results:
            box_coordinates = result.boxes.xyxy.cpu().numpy().astype(int).tolist()
            ids = result.boxes.id

            print(box_coordinates)
            if len(box_coordinates) != 0 and ids != None:
                ids_list = ids.cpu().numpy().astype(int).tolist()
                
                for box_coordinate in box_coordinates:
                    # id = int(id)
                    x1, y1, x2, y2 = box_coordinate

                    frame_output = cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 1)
      


        cv2.putText(frame, fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX , 3, (100, 255, 0), 3, cv2.LINE_AA) 
        cv2.imshow("YOLOv8", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()