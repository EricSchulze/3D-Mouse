"""This file acts as the main module for this script."""

import adsk
import threading
import json
import socketserver
import math
import traceback
import time
from http.server import BaseHTTPRequestHandler

class FusionCommandHandler(BaseHTTPRequestHandler):
    def __init__(self, request, client_address, server):
        self.app = adsk.core.Application.get()
        self.ui  = self.app.userInterface
        super().__init__(request, client_address, server)

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        
        try:
            command_data = json.loads(post_data.decode('utf-8'))
            result = self.process_command(command_data)
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
            
        except Exception as e:
            self.send_error(400, str(e))
    
    def process_command(self, data):
        command = data.get('command')
        
        if command == 'rotate':
            return self.rotate(data)
        elif command == 'translate':
            return self.translate(data)
        else:
            return {"status": "failed", "error": f"Unknown command: {command}"}
    
    def get_camera_axes(self, camera):
        """
            Calculate camera's local coordinate system (Forward, Right, Up)
            Returns: normalized forward_vector, right_vector, up_vector
        """
        
        eye = camera.eye
        target = camera.target
        up_vector = camera.upVector
        
        # Forward vector (from eye to target)
        forward = eye.vectorTo(target)
        forward.normalize()
        
        # Right vector (cross product of forward and up)
        right = forward.crossProduct(up_vector)
        right.normalize()
        
        # Recalculate up vector to ensure orthogonal system
        up = right.crossProduct(forward)
        up.normalize()
        
        return forward, right, up

    def rotate_pitch(self, viewport, degree, rvec):
        camera = viewport.camera

        rotMatrix = adsk.core.Matrix3D.create()
        eye = camera.eye
        target = camera.target
        rotMatrix.setToRotation(math.radians(degree), rvec, target)
        eye.transformBy(rotMatrix)

        camera.eye = eye
        camera.isSmoothTransition = False
        viewport.camera = camera

    def rotate_yaw(self, viewport, degree, uvec):
        camera = viewport.camera

        rotMatrix = adsk.core.Matrix3D.create()
        eye = camera.eye
        target = camera.target
        rotMatrix.setToRotation(math.radians(degree), uvec, target)
        eye.transformBy(rotMatrix)

        camera.eye = eye
        camera.isSmoothTransition = False
        viewport.camera = camera

    def rotate_roll(self, viewport, degree, fvec, uvec):
        camera = viewport.camera

        rotMatrix = adsk.core.Matrix3D.create()
        eye = camera.eye
        rotMatrix.setToRotation(math.radians(degree), fvec, eye)
        uvec.transformBy(rotMatrix)

        camera.upVector = uvec
        camera.isSmoothTransition = False
        viewport.camera = camera

    def rotate(self, data):
        pitch = data.get('rx', 0.0)  # pitch
        roll = data.get('ry', 0.0)  # roll
        yaw = data.get('rz', 0.0)  # yaw

        viewport = self.app.activeViewport

        fvec, rvec, uvec = self.get_camera_axes(viewport.camera)
        self.rotate_pitch(viewport, pitch, rvec)
        self.rotate_yaw(viewport, yaw, uvec)
        self.rotate_roll(viewport, roll, fvec, uvec)

        adsk.doEvents()
        viewport.refresh()
        time.sleep(0.01)

        return {"status": "success", "operation": "orbit_rotate", "pitch": pitch, "roll": roll, "yaw": yaw}

    def translate_x(self, viewport, tx, rvec):
        camera = viewport.camera
        eye = camera.eye
        target = camera.target

        new_eye = adsk.core.Point3D.create(
            eye.x + rvec.x * tx,
            eye.y + rvec.y * tx,
            eye.z + rvec.z * tx
        )

        new_target = adsk.core.Point3D.create(
            target.x + rvec.x * tx,
            target.y + rvec.y * tx,
            target.z + rvec.z * tx
        )

        camera.eye = new_eye
        camera.target = new_target
        camera.isSmoothTransition = False
        viewport.camera = camera

    def translate_y(self, viewport, ty, fvec):
        camera = viewport.camera
        eye = camera.eye
        target = camera.target

        new_eye = adsk.core.Point3D.create(
            eye.x + fvec.x * ty,
            eye.y + fvec.y * ty,
            eye.z + fvec.z * ty
        )

        new_target = adsk.core.Point3D.create(
            target.x + fvec.x * ty,
            target.y + fvec.y * ty,
            target.z + fvec.z * ty
        )

        camera.eye = new_eye
        camera.target = new_target
        camera.isSmoothTransition = False
        viewport.camera = camera

    def translate_z(self, viewport, tz, uvec):
        camera = viewport.camera
        eye = camera.eye
        target = camera.target

        new_eye = adsk.core.Point3D.create(
            eye.x + uvec.x * tz,
            eye.y + uvec.y * tz,
            eye.z + uvec.z * tz
        )

        new_target = adsk.core.Point3D.create(
            target.x + uvec.x * tz,
            target.y + uvec.y * tz,
            target.z + uvec.z * tz
        )

        camera.eye = new_eye
        camera.target = new_target
        camera.isSmoothTransition = False
        viewport.camera = camera

    def translate(self, data):
        tx = data.get('tx', 0.0)
        ty = data.get('ty', 0.0)
        tz = data.get('tz', 0.0)

        viewport = self.app.activeViewport
        camera = viewport.camera

        eye = camera.eye
        target = camera.target
        up_vector = camera.upVector
        
        fvec, rvec, uvec = self.get_camera_axes(viewport.camera)
        self.translate_x(viewport, tx, rvec)
        self.translate_y(viewport, ty, fvec)
        self.translate_z(viewport, tz, uvec)

        adsk.doEvents()
        viewport.refresh()
        time.sleep(0.01)

        return {"status": "success", "operation": "orbit_rotate", "tx": tx, "ty": ty, "tz": tz}

    """
    def orbit_translate(self, data):
        tx = data.get('tx', 0.0)
        ty = data.get('ty', 0.0)
        tz = data.get('tz', 0.0)

        viewport = self.app.activeViewport
        camera = viewport.camera

        eye = camera.eye
        target = camera.target
        up_vector = camera.upVector
        
        # Calculate camera's right vector (local X-axis)
        forward = eye.vectorTo(target)
        forward.normalize()
        
        right = forward.crossProduct(up_vector)
        right.normalize()

        tx = right.x * tx
        ty = right.y * ty

        new_eye = adsk.core.Point3D.create(
            eye.x + tx,
            eye.y + ty,
            eye.z + tz
        )

        new_target = adsk.core.Point3D.create(
            target.x + tx,
            target.y + ty,
            target.z + tz
        )

        camera.eye = new_eye
        camera.target = new_target
        camera.isSmoothTransition = False
        viewport.camera = camera
        adsk.doEvents()
        viewport.refresh()
        time.sleep(0.01)

        return {"status": "success", "operation": "orbit_rotate", "tx": tx, "ty": ty, "tz": tz}
    """

class FusionHTTPServer:
    def __init__(self, port=8080):
        self.port = port
        self.server = None
        self.server_thread = None
    
    def start_server(self):
        """Start the HTTP server in a separate thread"""

        self.server = socketserver.TCPServer(('localhost', self.port), FusionCommandHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        print(f"Fusion 360 HTTP server started on port {self.port}")
    
    def stop_server(self):
        """Stop the HTTP server gracefully"""

        if self.server:
            self.server.shutdown()
            self.server.server_close()

        self.server_thread.join()
    
http_server = FusionHTTPServer(port=8080)

def run(_context: str):
    global http_server
    try:
        http_server.start_server()
    except:
        print(f'Failed:\n{traceback.format_exc()}')


def stop(context):
    global http_server
    print('Stopping HTTP server...')
    http_server.stop_server()
