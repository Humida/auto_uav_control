import cv2
import torch
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, Attitude, AttitudeRate)
from ultralytics import YOLO

# mavsdk api
class MAVSDKAutoControl:
    def __init__(self,serial_port) -> None:
        self.drone = System()
        self.serial_port = serial_port
        
    async def connect_pixhawk(self):
        await self.drone.connect(system_address = f"{self.serial_port}")
        print(f"-- Watting for drone connect --")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connect to drone --")
        print("bbb")
     
    async def start_offboad_mode(self):
    
        print(f"-- Arming --")
        await self.drone.action.arm()
        
        print(f"-- Setting initial set point")
        await self.drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
        
        print(f"-- Start offboard")
        try:
            await self.drone.offboard.start()
    
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
            {error._result.result}")
            print("-- Disarming")
            await self.drone.action.disarm()
            return
        
    async def takeoff(self):
        thrust = 0  
        for i in range(300):
            thrust = int(i)*0.001
            await self.drone.offboard.set_attitude(0.0, 0.0, 0.0, thrust)
            await asyncio.sleep(0.2)
            
    async def land(self):
        thrust = 0.3
        for i in range(300):
            thrust = thrust - int(i)*0.001
            await self.drone.offboard.set_attitude(0.0, 0.0, 0.0, thrust)
            await asyncio.sleep(0.2)
            
    async def attack_mode(self):
        await self.drone.offboard.set_attitude(0.0, 0.0, 0.0, 0.6)
        
    async def test_mode(self):
        await self.drone.offboard.set_attitude(0.0, 0.0, 0.0, 0.1)
        
    async def focus_object(self, pitch_angle, roll_angle, yaw_angle):
        await self.drone.offboard.set_attitude(pitch_angle, 0.0, yaw_angle, 0.5)
        await asyncio.sleep(0.03)

# Tracking, focus target and show realtime image
class TrackingAndFocus:
    def __init__(self, device) -> None:
        self.drone = MAVSDKAutoControl('serial:///dev/ttyUSB0:115200')
        self.device = device 
        self.source = cv2.VideoCapture(f"v4l2src device={self.device} ! video/x-raw, format = YUY2, width=640, height=480, framerate=30/1  ! videoconvert ! video/x-raw,format=BGR ! appsink")
        self.model = YOLO("model.engine")
    
    async def arm_drone(self):
        # await self.drone.connect_pixhawk()
        await self.drone.start_offboad_mode()
    
    def get_source_and_auto_tracking(self):
        print("get source")
        while self.source.isOpened():
            success, frame = self.source.read()
            if success:
                self.object_tracking(frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break
            
    def object_tracking(self, frame):
        x_O = 239
        y_O = 319
        
        index_max_conf = 0
        
        results = self.model.track(frame, persist = True, conf = 0.5)
        
        for index, result in enumerate(results):            
            if(float(result.boxes.conf) > float(results[index_max_conf])):
                index_max_conf = index
                
        object_target = result[index_max_conf]        
        box_coordinate = object_target.boxes.xyxy.cpu().numpy().astype(int).tolist()
        id = object_target.boxes.id
        id_str = str(id)
        conf = object_target.boxes.conf
        x1, y1, x2, y2 = box_coordinate
        
        self.show_result(box_coordinate, frame = frame, id = id_str, conf= conf)
        
        x_center = int((x1 + x2)/2) - x_O
        y_center = int((y1 + y2)/2) - y_O
        
        pitch_angle =  round(y_center/240)*-90
        yaw_angle =  round(x_center/320)*-90
        
        print(pitch_angle, yaw_angle)
        # self.drone.focus_object(pitch_angle,0, yaw_angle)
        
    def show_result(self, box_coordinate, frame, id, conf):
        x1, y1, x2, y2 = box_coordinate
        cv2.rectangel(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
        cv2.putText(frame, f"{id} {conf}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA, True)
        cv2.imshow("Tracking target", frame)        

async def run():
    
    object_auto_control = TrackingAndFocus("/dev/video0")
    await object_auto_control.arm_drone()
    object_auto_control.get_source_and_auto_tracking()

               
if __name__== "__main__":
    asyncio.run(run())
    
