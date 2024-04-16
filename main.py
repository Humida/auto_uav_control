import cv2
import numpy as np
from collections import defaultdict
from ultralytics import YOLO
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, 
                             Attitude, AttitudeRate)

model = YOLO("model.pt")

async def target_track():
    # define input source video
    cap = cv2.VideoCapture(0)
    track_history = defaultdict(lambda: [])
    while cap.isOpened:
        # read a frame from video
        success, frame = cap.read()
        
        if success:
            # run YOLOv8 tracking on the frame, persisting track between frames 
            results = model.track(frame, persist= True)

            # get the boxes and track IDs
            boxes = results[0].boxes.xywh.cpu()
            track_ids = results[0].boxes.id.int().cpu().tolist()

            # Visualize the results on the frame
            annotated_frame = results[0].plot()

            # Plot the tracks
            for box, track_id in zip(boxes, track_ids):
                x, y, w, h = box
                track = track_history[track_id]
                track.append((float(x), float(y)))  # x, y center point
            if len(track) > 30:  # retain 90 tracks for 90 frames
                track.pop(0)

            # Draw the tracking lines
            points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)


class mav_auto_control:
    def __init__(self) -> None:
        pass

    async def mav_auto_control():
        drone = System()
        await drone.connect(system_adress = "serial:///dev/ttyUSB0:115200")
        print(f"--- Waitting for drone connect ---")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"--- Connect to drone ---")


    async def algorigthm_control(self, boxes_target):
        pass



if __name__ == "__main__":
    # Run the asyncio loop
    pass