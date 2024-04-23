import sys
import cv2
import time


def read_cam():
    cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw ,width=640, height=480, framerate=30/1  ! videoconvert ! video/x-raw,format=BGR ! appsink")
    fps = cap.get(cv2.CAP_PROP_FPS)
    # used to record the time when we processed last frame 
    
    
    prev_frame_time = 0 
    new_frame_time = 0
    
    font = cv2.FONT_HERSHEY_SIMPLEX 
    # time when we finish processing for this frame 
    
  
    # putting the FPS count on the frame 
  
    if cap.isOpened():
        
        cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
        while True:
            new_frame_time = time.time() 
  
            # Calculating the fps 
        
            # fps will be number of frame processed in given time frame 
            # since their will be most of time error of 0.001 second 
            # we will be subtracting it to get more accurate result 
            fps = 1/(new_frame_time-prev_frame_time) 
            prev_frame_time = new_frame_time 
        
            # converting the fps into integer 
            fps = int(fps) 
        
            # converting the fps to string so that we can display it on frame 
            # by using putText function 
            fps = str(fps) 
            ret_val, img = cap.read()
            cv2.putText(img, fps, (7, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA) 
            print(fps)
            cv2.imshow('demo',img)
            cv2.waitKey(10)
    else:
     print("camera open failed")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    read_cam()