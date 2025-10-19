import requests
import json
import time

class FusionController:
    def __init__(self, fusion_server_url="http://localhost:8080"):
        self.server_url = fusion_server_url
    
    def rotate_body(self, body_name: str, rx: float, ry: float, rz: float):
        command = {
            "command": "rotate",
            "body_name": body_name,
            "rx": rx,
            "ry": ry,
            "rz": rz
        }
        
        response = requests.post(self.server_url, json=command)
        return response.json()
    
    def translate_body(self, body_name: str, tx: float, ty: float, tz: float):
        command = {
            "command": "translate",
            "body_name": body_name,
            "tx": tx,
            "ty": ty,
            "tz": tz
        }
        
        response = requests.post(self.server_url, json=command)
        return response.json()
    
controller = FusionController()
# result = controller.rotate_body("Body1", 15.0, 0.0, 0.0)
# print(f"Rotation result: {result}")

# result = controller.translate_body("Body1", 0.5, 0.0, 0.0)
# print(f"Rotation result: {result}")
# time.sleep(0.1)

# result = controller.translate_body("Body1", 0.0, 0.5, 0.0)
# print(f"Rotation result: {result}")
# time.sleep(0.1)

# result = controller.translate_body("Body1", 0.0, 0.0, 0.5)
# print(f"Rotation result: {result}")
# time.sleep(0.1)


# for i in range(5):
#     result = controller.rotate_body("Body1", 2.0, 0.0, 0.0)
#     print(f"Rotation result: {result}")
#     time.sleep(0.1)

# for i in range(5):
#     result = controller.rotate_body("Body1", 0.0, 2.0, 0.0)
#     print(f"Rotation result: {result}")
#     time.sleep(0.1)

for i in range(5):
    result = controller.rotate_body("Body1", 0.0, 0.0, 1.0)
    print(f"Rotation result: {result}")
    time.sleep(0.1)

for i in range(3):
    result = controller.translate_body("Body1", 0.1, 0.0, 0.0)
    print(f"Translation result: {result}")
    time.sleep(0.05)

for i in range(3):
    result = controller.translate_body("Body1", 0.0, 0.1, 0.0)
    print(f"Translation result: {result}")
    time.sleep(0.05)

for i in range(3):
    result = controller.translate_body("Body1", 0.0, 0.0, 0.1)
    print(f"Translation result: {result}")
    time.sleep(0.05)
