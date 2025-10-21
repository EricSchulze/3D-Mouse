"""This file acts as the main module for this script."""

import adsk.core
import adsk.fusion
import threading
import json
import socketserver
import math
import queue
import traceback
import time
from typing import Optional
from http.server import HTTPServer, BaseHTTPRequestHandler

def rotate(body: adsk.fusion.BRepBody, rx: float, ry: float, rz: float) -> None:
    comp = body.parentComponent
    xaxis = comp.xConstructionAxis
    yaxis = comp.yConstructionAxis
    zaxis = comp.zConstructionAxis

    # Angle as ValueInput (most builds accept this form)
    rx_vi = adsk.core.ValueInput.createByReal(math.radians(rx))
    ry_vi = adsk.core.ValueInput.createByReal(math.radians(ry))
    rz_vi = adsk.core.ValueInput.createByReal(math.radians(rz))

    # Build input and define rotation
    move_feats = comp.features.moveFeatures
    sel = adsk.core.ObjectCollection.create()
    sel.add(body)
    mi = move_feats.createInput2(sel)

    mi.defineAsRotate(xaxis, rx_vi)
    move_feats.add(mi)
    mi.defineAsRotate(yaxis, ry_vi)
    move_feats.add(mi)
    mi.defineAsRotate(zaxis, rz_vi)
    move_feats.add(mi)


def translate(body: adsk.fusion.BRepBody, tx: float, ty: float, tz: float) -> None:
    comp = body.parentComponent

    # Create a translation vector
    tx_vi = adsk.core.ValueInput.createByReal(tx)
    ty_vi = adsk.core.ValueInput.createByReal(ty)
    tz_vi = adsk.core.ValueInput.createByReal(tz)

    # Create a move feature input
    move_feats = comp.features.moveFeatures
    sel = adsk.core.ObjectCollection.create()
    sel.add(body)
    mi = move_feats.createInput2(sel)

    # Define the translation
    mi.defineAsTranslateXYZ(tx_vi, ty_vi, tz_vi, True)
    move_feats.add(mi)

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
        # Access your rotate() and translate() functions here
        command_type = data.get('command')
        
        if command_type == 'rotate':
            return self.orbit_rotate(data)
        elif command_type == 'translate':
            return self.orbit_translate(data)
        else:
            raise ValueError(f"Unknown command: {command_type}")
    
    def get_camera_axes(self, camera):
        """Calculate camera's local coordinate system (Forward, Right, Up)"""
        
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

    def rotate_on_roll(self, roll):

        viewport = self.app.activeViewport
        camera = viewport.camera

        fvec, rvec, uvec = self.get_camera_axes(camera)

        rotMatrix = adsk.core.Matrix3D.create()
        eye = camera.eye
        rotMatrix.setToRotation(math.radians(roll), fvec, eye)
        uvec.transformBy(rotMatrix)

        camera.upVector = uvec
        camera.isSmoothTransition = False
        viewport.camera = camera
        adsk.doEvents()
        viewport.refresh()
        time.sleep(0.01)

    def orbit_rotate(self, data):
        pitch = data.get('rx', 0.0)  # pitch
        roll = data.get('ry', 0.0)  # roll
        yaw = data.get('rz', 0.0)  # yaw

        viewport = self.app.activeViewport
        camera = viewport.camera

        #### test
        fvec, rvec, uvec = self.get_camera_axes()

        rotMatrix = adsk.core.Matrix3D.create()
        eye = camera.eye
        target = camera.target
        # rotMatrix.setToRotation(math.radians(pitch), rvec, target)
        # rotMatrix.setToRotation(math.radians(yaw), uvec, target)
        rotMatrix.setToRotation(math.radians(roll), fvec, eye)
        uvec.transformBy(rotMatrix)
        # uvec.transformBy(rotMatrix)

        # target = adsk.core.Point3D.create(0,0,0)
        # up = adsk.core.Vector3D.create(0,0,1)
        # steps = 90
        
        # dist = camera.target.distanceTo(camera.eye)
    
        # for i in range(0, steps):
        #     eye = adsk.core.Point3D.create(dist * math.cos((math.pi*2) * (i/steps)), dist * math.sin((math.pi*2) * (i/steps)), 10)
            
        #     camera.eye = eye
        #     camera.target = target
        #     camera.upVector = up
        
        #     camera.isSmoothTransition = False
        #     viewport.camera = camera
        #     adsk.doEvents()
        #     viewport.refresh()
        #     time.sleep(0.01)

        #### end test

        """
        eye = camera.eye
        target = camera.target
        
        target_point = adsk.core.Point3D.create(target.x, target.y, target.z)
        transform = adsk.core.Matrix3D.create()

        transform.setToRotation(math.radians(rx), adsk.core.Vector3D.create(1, 0, 0), target_point)
        eye.transformBy(transform)

        transform.setToRotation(math.radians(ry), adsk.core.Vector3D.create(0, 1, 0), target_point)
        eye.transformBy(transform)

        transform.setToRotation(math.radians(rz), adsk.core.Vector3D.create(0, 0, 1), target_point)
        eye.transformBy(transform)
        """


        # Update camera
        # camera.eye = eye
        camera.upVector = uvec
        camera.isSmoothTransition = False
        viewport.camera = camera
        adsk.doEvents()
        viewport.refresh()
        time.sleep(0.01)

        return {"status": "success", "operation": "orbit_rotate", "pitch": pitch, "roll": roll, "yaw": yaw}

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
        viewport.camera = camera
        adsk.doEvents()
        time.sleep(0.01)

        return {"status": "success", "operation": "orbit_rotate", "tx": tx, "ty": ty, "tz": tz}

    def handle_rotate(self, data):
        rx = data.get('rx', 0.0)
        ry = data.get('ry', 0.0)
        rz = data.get('rz', 0.0)
        body_name = data.get('body_name')

        design = adsk.fusion.Design.cast(self.app.activeProduct)
        if not design:
            self.ui.messageBox('No active design.'); return

        root = design.rootComponent

        # Find a root-level body named "Body1"
        body = None
        for b in root.bRepBodies:
            if b.name == body_name:
                body = b
                break
        if not body:
            return {"status": "failed", "operation": "rotate", "body": body_name, "message": "Body not found"}
        # ui.messageBox(f'Root body "{body_name}" found.')
        rotate(body, rx, ry, rz)
        
        # ui.messageBox(f"rotate: {body_name}")

        return {"status": "success", "operation": "rotate", "body": body_name, "rx": rx, "ry": ry, "rz": rz}

    def handle_translate(self, data):
        tx = data.get('tx', 0.0)
        ty = data.get('ty', 0.0)
        tz = data.get('tz', 0.0)
        body_name = data.get('body_name')

        design = adsk.fusion.Design.cast(self.app.activeProduct)
        if not design:
            self.ui.messageBox('No active design.'); return

        root = design.rootComponent

        tz = data.get('tz', 0.0)
        body_name = data.get('body_name')

        design = adsk.fusion.Design.cast(self.app.activeProduct)
        if not design:
            self.ui.messageBox('No active design.'); return

        root = design.rootComponent

        # Find a root-level body named "Body1"
        body = None
        for b in root.bRepBodies:
            if b.name == body_name:
                body = b
                break
        if not body:
            return {"status": "failed", "operation": "rotate", "body": body_name, "message": "Body not found"}
        # ui.messageBox(f'Root body "{body_name}" found.')
        translate(body, tx, ty, tz)
        
        # ui.messageBox(f"rotate: {body_name}")

        return {"status": "success", "operation": "rotate", "body": body_name, "tx": tx, "ty": ty, "tz": tz}

class FusionHTTPServer:
    def __init__(self, port=8080):
        self.port = port
        self.command_queue = queue.Queue()
        self.server = None
        self.server_thread = None
        self.running = False
    
    def start_server(self):
        """Start the HTTP server in a separate thread"""
        self.running = True
        
        # Create server with custom handler
        self.server = socketserver.TCPServer(('localhost', self.port), FusionCommandHandler)
        
        # Start server thread
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        print(f"Fusion 360 HTTP server started on port {self.port}")
    
    def stop_server(self):
        """Stop the HTTP server"""
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        self.running = False
        self.server_thread.join()
    
    def process_command_queue(self):
        while not self.command_queue.empty():
            try:
                command = self.command_queue.get_nowait()
                self.execute_fusion_command(command)
            except queue.Empty:
                break
    
    def execute_fusion_command(self, command):
        if command['type'] == 'rotate':
            ui.messageBox("rotate")
        elif command['type'] == 'translate':
            ui.messageBox("translate")

http_server = FusionHTTPServer(port=8080)

def run(_context: str):
    global http_server
    try:
        http_server.start_server()
    except:
        ui.messageBox(f'Failed:\n{traceback.format_exc()}')


def stop(context):
    global http_server
    print('Stopping HTTP server...')
    http_server.stop_server()
