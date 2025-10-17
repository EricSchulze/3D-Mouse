# MyMoveBodyAddin.py
# Fusion 360 Python add-in that listens on localhost:5001 and
# moves a body named in the command via rotation + translation.

import adsk.core, adsk.fusion, traceback, math, json, socket, threading

_app = None
_ui = None
_handlers = []
_server_thread = None
_running = False
_EVENT_ID = 'ExternalMoveEvent'

def _log(msg):
    try:
        if _ui:
            _ui.messageBox(str(msg))
    except:
        pass

class ExternalMoveHandler(adsk.core.CustomEventHandler):
    def notify(self, args: adsk.core.CustomEventArgs):
        try:
            payload = args.additionalInfo or ''
            cmd = json.loads(payload)
            if not isinstance(cmd, dict):
                return
            if cmd.get('command') != 'move_body':
                return
            move_body(cmd)
        except:
            if _ui:
                _ui.messageBox('Move handler error:\n{}'.format(traceback.format_exc()))

def move_body(cmd: dict):
    """
    cmd example:
    {
      "command": "move_body",
      "name": "Body1",
      "rotate": { "axis":[0,0,1], "angle_deg":30, "pivot":"center" },
      "translate": { "dx_mm":10, "dy_mm":0, "dz_mm":0 },
      "mode": "feature"  # or "occurrence"
    }
    """
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        _log('No active design.')
        return

    root = design.rootComponent
    name = cmd.get('name', '')
    mode = cmd.get('mode', 'feature')  # "feature" => MoveFeature; "occurrence" => rigid transform

    # Locate body by name (in root or in occurrences).
    body = None
    body_in_asm = None
    occ_for_body = None

    for b in root.bRepBodies:
        if b.name == name:
            body = b
            break

    if body is None:
        for occ in root.allOccurrences:
            comp = occ.component
            for b in comp.bRepBodies:
                if b.name == name:
                    body_in_asm = b.createForAssemblyContext(occ)
                    occ_for_body = occ
                    break
            if body_in_asm:
                break

    if not body and not body_in_asm:
        _log(f'Body "{name}" not found.')
        return

    entity_to_move = body_in_asm if body_in_asm else body

    # Build rotation
    rot = adsk.core.Matrix3D.create()
    rotate = cmd.get('rotate', {})
    axis = rotate.get('axis', [0, 0, 1])
    angle_deg = float(rotate.get('angle_deg', 0.0))
    angle_rad = math.radians(angle_deg)

    # Pivot: "center" or explicit point {px_mm, py_mm, pz_mm}
    pivot = rotate.get('pivot', 'center')
    if isinstance(pivot, str) and pivot == 'center':
        bb = entity_to_move.boundingBox
        cpt = adsk.core.Point3D.create(
            (bb.minPoint.x + bb.maxPoint.x) * 0.5,
            (bb.minPoint.y + bb.maxPoint.y) * 0.5,
            (bb.minPoint.z + bb.maxPoint.z) * 0.5
        )
    else:
        # explicit pivot point in mm
        units_mgr = design.unitsManager
        px_cm = units_mgr.convert(float(pivot.get('px_mm', 0.0)), 'mm', 'cm')
        py_cm = units_mgr.convert(float(pivot.get('py_mm', 0.0)), 'mm', 'cm')
        pz_cm = units_mgr.convert(float(pivot.get('pz_mm', 0.0)), 'mm', 'cm')
        cpt = adsk.core.Point3D.create(px_cm, py_cm, pz_cm)

    axis_v = adsk.core.Vector3D.create(float(axis[0]), float(axis[1]), float(axis[2]))
    rot.setToRotation(angle_rad, axis_v, cpt)

    # Build translation (mm -> cm)
    translate = cmd.get('translate', {})
    units_mgr = design.unitsManager
    dx_cm = units_mgr.convert(float(translate.get('dx_mm', 0.0)), 'mm', 'cm')
    dy_cm = units_mgr.convert(float(translate.get('dy_mm', 0.0)), 'mm', 'cm')
    dz_cm = units_mgr.convert(float(translate.get('dz_mm', 0.0)), 'mm', 'cm')
    trans = adsk.core.Matrix3D.create()
    trans.translation = adsk.core.Vector3D.create(dx_cm, dy_cm, dz_cm)

    # Compose final transform: rotation followed by translation
    xform = rot.copy()
    xform.transformBy(trans)

    if mode == 'feature':
        # Parametric MoveFeature in the appropriate component context
        feats_container = (occ_for_body.component.features if occ_for_body else root.features)
        move_feats = feats_container.moveFeatures
        sel = adsk.core.ObjectCollection.create()
        sel.add(entity_to_move)
        move_input = move_feats.createInput(sel, xform)
        move_feats.add(move_input)
        _log(f'MoveFeature applied to "{name}".')
    else:
        # Rigid occurrence transform (no timeline feature)
        if not occ_for_body:
            _log('Occurrence move requires the body to be inside a component (not root).')
            return
        new_xf = occ_for_body.transform.copy()
        new_xf.transformBy(xform)
        occ_for_body.transform = new_xf
        _log(f'Occurrence transform applied to "{name}".')

def _server_loop(host='127.0.0.1', port=5001):
    # Background server thread to receive JSON commands (newline-delimited).
    global _running
    _running = True
    app = adsk.core.Application.get()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    try:
        while _running:
            srv.settimeout(1.0)
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(2.0)
                buf = b''
                while _running:
                    try:
                        chunk = conn.recv(4096)
                        if not chunk:
                            break
                        buf += chunk
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            payload = line.decode('utf-8', errors='ignore').strip()
                            if not payload:
                                continue
                            # Marshal to main thread via CustomEvent
                            try:
                                app.fireCustomEvent(_EVENT_ID, payload)
                            except:
                                pass
                    except socket.timeout:
                        continue
                    except:
                        break
    finally:
        try:
            srv.close()
        except:
            pass

def run(context):
    try:
        global _app, _ui, _server_thread
        _app = adsk.core.Application.get()
        _ui = _app.userInterface
        _ui.messageBox('Starting 3DMouseBodyControllerAddIn...')

        # Register the custom event handler
        ce = _app.registerCustomEvent(_EVENT_ID)
        handler = ExternalMoveHandler()
        ce.add(handler)
        _handlers.append(handler)

        # Start the TCP server thread
        _server_thread = threading.Thread(target=_server_loop, args=('127.0.0.1', 8080), daemon=True)
        _server_thread.start()

        # Optional: notify ready
        _ui.messageBox('Move add-in listening on 127.0.0.1:8080')
    except:
        if _ui:
            _ui.messageBox('Add-in run() failed:\n{}'.format(traceback.format_exc()))

def stop(context):
    try:
        global _running
        _running = False
        if _app:
            try:
                _app.unregisterCustomEvent(_EVENT_ID)
            except:
                pass
    except:
        if _ui:
            _ui.messageBox('Add-in stop() failed:\n{}'.format(traceback.format_exc()))