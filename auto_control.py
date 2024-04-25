import asyncio
import time
from ultralytics import YOLO
from mavsdk import System
from mavsdk.offboard import (OffboardError, Attitude)
import cv2

async def run():

    # Khởi tạo hệ thống MAVSDK
    model = YOLO("model.engine")
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0:115200")
    
    async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connect to drone --")
                break
            
    await drone.action.arm()
    asyncio.sleep(1)
                
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
    print(f"-- Start offboard")
        
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
        {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw, format = YUY2, width=640, height=480, framerate=30/1  ! videoconvert ! video/x-raw,format=BGR ! appsink")

    # await takeoff(drone = drone)
    # Vòng lặp chính
    while True:
        # Lấy ảnh từ camera UAV
        success, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Xử lý ảnh bằng YOLOv8 để phát hiện đối tượng
        results = model.track(frame, persist = True)
        
        x_O = 239
        y_O = 319
          
        data = results[0]
        index_max_conf = 0
        boxes = data.boxes.cpu().numpy()
        for index, box in enumerate(boxes):  
                if(float(box.conf) > float(boxes[index_max_conf].conf)):
                    index_max_conf = index

        id = data.boxes.id
        box_coordinate = data.boxes.xyxy
        print(boxes)
        print(box_coordinate)            
        if len(box_coordinate) >0:
            x1, y1, x2, y2 = box_coordinate[0].numpy().astype(int)
                
            x_center = int((x1 + x2)/2) - x_O
            y_center = int((y1 + y2)/2) - y_O
                
            pitch_angle =  round(y_center/240)*-90
            yaw_angle =  round(x_center/320)*-90
                
            print(pitch_angle, yaw_angle)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
            cv2.putText(frame, f"tank", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA, True)
            cv2.imshow('Tracking', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
                # land(drone)
            break
        
async def control_uav(drone):
    # Điều khiển UAV dựa trên vị trí tương đối của đối tượng
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.1))
    
async def takeoff(drone):
        thrust = 0  
        for i in range(100):
            thrust = int(i)*0.001
            await drone.offboard.set_attitude(0.0, 0.0, 0.0, thrust)
            await asyncio.sleep(0.2)
            
            
async def land(drone):
    thrust = 0.1
    for i in range(100):
        thrust = 0.1 - 0.01*int(i)
        await drone.offbroard.set_attitude(0.0, 0.0, 0.0, thrust)
            
    

if __name__ == "__main__":
    asyncio.run(run())
