import requests
import json
import time

class FusionController:
    def __init__(self, fusion_server_url="http://localhost:8080"):
        self.server_url = fusion_server_url
    
    def rotate(self, rx: float, ry: float, rz: float):
        command = {
            "command": "rotate",
            "rx": rx,
            "ry": ry,
            "rz": rz
        }
        
        return requests.post(self.server_url, json=command)
    
    def translate(self, tx: float, ty: float, tz: float):
        command = {
            "command": "translate",
            "tx": tx,
            "ty": ty,
            "tz": tz
        }
        
        return requests.post(self.server_url, json=command)
    
controller = FusionController()

# for i in range(5):
#     result = controller.rotate(2.0, 0.0, 0.0)
#     print(f"Rotation result: {result}")
#     time.sleep(0.1)

# for i in range(5):
#     result = controller.rotate(0.0, 2.0, 0.0)
#     print(f"Rotation result: {result}")
#     time.sleep(0.1)

############################ rotation #############################
# rotate on pitch, roll, yaw by 10.0 degrees respectively
for i in range(10):
    result = controller.rotate(1.0, 1.0, 1.0)
    print(f"Rotation result: {result.content.decode('utf-8')}")
    time.sleep(0.01)

# rotate on pitch by -10.0 degrees
for i in range(10):
    result = controller.rotate(-1.0, 0.0, 0.0)
    print(f"Rotation result: {result.content.decode('utf-8')}")
    time.sleep(0.01)

# rotate on roll by -10.0 degrees
for i in range(10):
    result = controller.rotate(0.0, -1.0, 0.0)
    print(f"Rotation result: {result.content.decode('utf-8')}")
    time.sleep(0.01)

# rotate on yaw by -10.0 degrees
for i in range(10):
    result = controller.rotate(0.0, 0.0, -1.0)
    print(f"Rotation result: {result.content.decode('utf-8')}")
    time.sleep(0.01)

############################ translation #############################
# translate on x, y, z by -1.0 respectively
for i in range(10):
    result = controller.translate(-0.1, -0.1, -0.1)
    print(f"Translation result: {result.content.decode('utf-8')}")
    time.sleep(0.01)

# translate on x by 1.0
for i in range(10):
    result = controller.translate(0.1, 0.0, 0.0)
    print(f"Translation result: {result.content.decode('utf-8')}")
    time.sleep(0.01)

# translate on y by 1.0
for i in range(10):
    result = controller.translate(0.0, 0.1, 0.0)
    print(f"Translation result: {result.content.decode('utf-8')}")
    time.sleep(0.01)

# translate on z by 1.0
for i in range(10):
    result = controller.translate(0.0, 0.0, 0.1)
    print(f"Translation result: {result.content.decode('utf-8')}")
    time.sleep(0.01)