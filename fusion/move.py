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
        
        response = requests.post(self.server_url, json=command)
        return response.json()
    
    def translate(self, tx: float, ty: float, tz: float):
        command = {
            "command": "translate",
            "tx": tx,
            "ty": ty,
            "tz": tz
        }
        
        response = requests.post(self.server_url, json=command)
        return response.json()
    
controller = FusionController()

# for i in range(5):
#     result = controller.rotate(2.0, 0.0, 0.0)
#     print(f"Rotation result: {result}")
#     time.sleep(0.1)

# for i in range(5):
#     result = controller.rotate(0.0, 2.0, 0.0)
#     print(f"Rotation result: {result}")
#     time.sleep(0.1)

for i in range(10):
    result = controller.rotate(1.0, 1.0, 1.0)
    print(f"Rotation result: {result}")
    time.sleep(0.01)

for i in range(10):
    result = controller.rotate(-1.0, -1.0, -1.0)
    print(f"Rotation result: {result}")
    time.sleep(0.01)

"""
for i in range(3):
    result = controller.translate("Body1", 0.1, 0.0, 0.0)
    print(f"Translation result: {result}")
    time.sleep(0.05)

for i in range(3):
    result = controller.translate("Body1", 0.0, 0.1, 0.0)
    print(f"Translation result: {result}")
    time.sleep(0.05)

for i in range(3):
    result = controller.translate("Body1", 0.0, 0.0, 0.1)
    print(f"Translation result: {result}")
    time.sleep(0.05)
"""