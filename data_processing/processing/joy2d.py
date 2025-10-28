import time
import numpy as np
import serial
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets

# =========================
# Configuration
# =========================

SERIAL_PORT = 'COM36'   # Change to your UART port
BAUD_RATE   = 115200

# Calibration durations
CALIB_SECONDS_MINMAX = 15.0   # min–max capture
CALIB_SECONDS_NEUTRAL = 5.0   # neutral rest capture

# Smoothing and buffers
BUFFER_SIZE = 800
N_SMOOTH    = 5                # moving average window for stability

# Base triangle geometry
SIDE        = 0.30             # triangle side length [m]
REST_HEIGHT = 0.20             # desired height above base [m] at rest

# Mapping normalized -> meters (per-axis, local frame)
XY_HALF_RANGE_M = 0.05         # local +/- range for x,y around rest (meters)
Z_HALF_RANGE_M  = 0.05         # local +/- range for z around REST_HEIGHT (meters)

# Optional: translation amplification for visualization (rotation unchanged)
TRANS_GAIN_XY = 3.0        # amplify XY translation component (1.0 = no change)
TRANS_GAIN_Z  = 1.0            # amplify Z translation component (1.0 = no change)

# Optional per-sensor axis sign flips in local frame (set to -1 if a sensor axis is reversed)
AXIS_SIGNS = [
    np.array([+1.0, +1.0, +1.0]),  # Sensor 1 local [x,y,z] signs
    np.array([+1.0, +1.0, +1.0]),  # Sensor 2 local [x,y,z] signs
    np.array([+1.0, +1.0, +1.0]),  # Sensor 3 local [x,y,z] signs
]

# =========================
# Geometry helpers
# =========================

def equilateral_triangle(side):
    """
    Return 3 vertices of an equilateral triangle centered at origin in the XY plane (z=0).
    Centroid at (0,0,0).
    """
    s = side
    v1 = np.array([+s/np.sqrt(3), 0.0, 0.0])
    v2 = np.array([-s/(2*np.sqrt(3)), +s/2.0, 0.0])
    v3 = np.array([-s/(2*np.sqrt(3)), -s/2.0, 0.0])
    return [v1, v2, v3]

BASE_CORNERS = equilateral_triangle(SIDE)       # world positions of base corners
BASE_CORNERS_A = np.vstack(BASE_CORNERS)
BASE_EDGES = [(0,1), (1,2), (2,0)]             # triangle outline

def build_local_frames_with_x_inward(base_corners):
    """
    For each base corner Bi, build an orthonormal local frame:
      - x_dir: inward toward triangle centroid (faces the center)
      - z_dir: global up (0,0,1)
      - y_dir: z_dir × x_dir to complete a right-handed triad (x × y = z)
    Returns list of 3 rotation matrices R_i (local->world).
    """
    B_centroid = np.mean(base_corners, axis=0)
    R_list = []
    for Bi in base_corners:
        x_dir = B_centroid - Bi
        xnorm = np.linalg.norm(x_dir)
        x_dir = x_dir / (xnorm + 1e-12)

        z_dir = np.array([0.0, 0.0, 1.0])
        # y = z × x for right-handed frame (x × y = z)
        y_dir = np.cross(z_dir, x_dir)
        ynorm = np.linalg.norm(y_dir)
        y_dir = y_dir / (ynorm + 1e-12)

        # Optionally re-orthonormalize z to ensure perfect orthogonality
        z_dir = np.cross(x_dir, y_dir)
        znorm = np.linalg.norm(z_dir)
        z_dir = z_dir / (znorm + 1e-12)

        # Assemble rotation (columns are world directions of local axes)
        R = np.column_stack([x_dir, y_dir, z_dir])  # world = R @ local
        R_list.append(R)
    return R_list

# =========================
# Math helpers
# =========================

def parse_uart_line(line):
    """
    Expect 9 comma-separated floats: x1,y1,z1, x2,y2,z2, x3,y3,z3
    Each triple is the raw sensor output for the relative position of upper corner i w.r.t. base corner i, in sensor's local axes.
    """
    try:
        vals = [float(x) for x in line.strip().split(',') if x]
        if len(vals) == 9:
            return vals
    except Exception:
        pass
    return None

def kabsch_rigid_transform(P, Q):
    """
    Compute rigid transform (R, t) that maps P -> Q (Nx3 points) using SVD (Kabsch).
    Returns R (3x3), t (3,). Exact for three non-colinear points.
    """
    P = np.asarray(P, dtype=float)
    Q = np.asarray(Q, dtype=float)
    assert P.shape == Q.shape and P.shape[1] == 3
    p_mean = P.mean(axis=0)
    q_mean = Q.mean(axis=0)
    Pc = P - p_mean
    Qc = Q - q_mean
    H = Pc.T @ Qc
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    # reflection safeguard
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1.0
        R = Vt.T @ U.T
    t = q_mean - R @ p_mean
    return R, t

# =========================
# Main application
# =========================

def main():
    pg.setConfigOptions(antialias=True)

    # Serial
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    # Qt app and 3D view
    app = QtWidgets.QApplication([])
    w = gl.GLViewWidget()
    w.setWindowTitle('Upper Triangle 6-DoF (X Inward Local Frames + 15s Min-Max + 5s Neutral)')
    w.setGeometry(50, 50, 1100, 800)
    w.show()

    # Camera
    w.setCameraPosition(distance=1.5, elevation=30, azimuth=45)
    w.opts['fov'] = 45

    # XY grid and axes
    grid = gl.GLGridItem()
    grid.rotate(90, 1, 0, 0)  # XY plane
    grid.setSize(x=SIDE*3.0, y=SIDE*3.0, z=1.0)
    w.addItem(grid)
    axes = gl.GLAxisItem()
    axes.setSize(x=SIDE*1.2, y=SIDE*1.2, z=SIDE*1.2)
    w.addItem(axes)

    # UI
    btn_minmax = QtWidgets.QPushButton('Calibrate Min-Max (15s)', w)
    btn_minmax.setGeometry(10, 10, 170, 32)
    btn_minmax.show()
    btn_neutral = QtWidgets.QPushButton('Capture Neutral (5s)', w)
    btn_neutral.setGeometry(190, 10, 160, 32)
    btn_neutral.show()
    status_lbl = QtWidgets.QLabel('Status: Ready', w)
    status_lbl.setGeometry(360, 10, 720, 32)
    status_lbl.setStyleSheet('color: white; background-color: rgba(0,0,0,0.4); padding: 4px;')
    status_lbl.show()

    # Base triangle visualization (static)
    base_edge_items = []
    for (i0, i1) in BASE_EDGES:
        item = gl.GLLinePlotItem(color=(1,1,1,0.8), width=2)
        item.setData(pos=np.array([BASE_CORNERS_A[i0], BASE_CORNERS_A[i1]], dtype=float))
        w.addItem(item)
        base_edge_items.append(item)
    base_scatter = gl.GLScatterPlotItem(pos=BASE_CORNERS_A, color=(1,1,1,0.9), size=7)
    w.addItem(base_scatter)

    # Upper triangle visualization (dynamic): wireframe + corner scatter
    plat_edge_items = [gl.GLLinePlotItem(color=(1,1,0,0.9), width=3) for _ in BASE_EDGES]
    for it in plat_edge_items:
        w.addItem(it)
    plat_scatter = gl.GLScatterPlotItem(pos=np.zeros((3,3)), color=(1,1,0,0.9), size=8)
    w.addItem(plat_scatter)

    # NEW: extruded 3D mesh for the upper triangle (created on first update)
    extruded_mesh = None

    # Leg lines (measured world vectors from base corners to measured upper corners)
    leg_items = [gl.GLLinePlotItem(), gl.GLLinePlotItem(), gl.GLLinePlotItem()]
    leg_colors = [(1,0,0,1), (0,1,0,1), (0,0,1,1)]
    for it in leg_items:
        w.addItem(it)

    # Measured upper corner markers (Qi, unamplified)
    qi_scatter = gl.GLScatterPlotItem(pos=np.zeros((3,3)), size=7)
    qi_scatter.setData(color=np.array([
        [1.0, 0.0, 0.0, 0.95],
        [0.0, 1.0, 0.0, 0.95],
        [0.0, 0.0, 1.0, 0.95],
    ]))
    w.addItem(qi_scatter)

    # Build local frames (X inward, Y = Z × X, Z up)
    R_local_to_world = build_local_frames_with_x_inward(BASE_CORNERS_A)
    S_local = [np.diag(AXIS_SIGNS[i]) for i in range(3)]  # per-sensor axis flips

    # Calibration state
    capturing_minmax  = False
    capturing_neutral = False
    minmax_start_time  = 0.0
    neutral_start_time = 0.0

    # Per-sensor per-axis min/max of RAW values (for normalization)
    calib_min = [None, None, None]  # arrays [x_min, y_min, z_min]
    calib_max = [None, None, None]  # arrays [x_max, y_max, z_max]
    # Per-sensor per-axis neutral offsets in NORMALIZED units
    neutral_offsets = [np.zeros(3), np.zeros(3), np.zeros(3)]

    # Platform body-frame corners P_body (3x3), learned from neutral pose (meters)
    have_body_shape = False
    P_body = None

    # Buffers
    data_buffer = []
    minmax_samples = [[], [], []]           # raw samples during min-max
    neutral_samples_norm = [[], [], []]     # normalized samples during neutral capture

    def start_minmax():
        nonlocal capturing_minmax, minmax_start_time, minmax_samples
        capturing_minmax = True
        minmax_start_time = time.time()
        minmax_samples = [[], [], []]
        status_lbl.setText(f'Calibrating Min-Max: {int(CALIB_SECONDS_MINMAX)}s remaining...')
    btn_minmax.clicked.connect(start_minmax)

    def start_neutral():
        nonlocal capturing_neutral, neutral_start_time, neutral_samples_norm
        # require min-max calibration first
        if not all(m is not None for m in calib_min) or not all(M is not None for M in calib_max):
            status_lbl.setText('Status: Complete Min-Max calibration first.')
            return
        capturing_neutral = True
        neutral_start_time = time.time()
        neutral_samples_norm = [[], [], []]
        status_lbl.setText(f'Capturing Neutral Rest: {int(CALIB_SECONDS_NEUTRAL)}s remaining...')
    btn_neutral.clicked.connect(start_neutral)

    def minmax_normalize(vec, vmin, vmax):
        vec = np.asarray(vec, dtype=float)
        vmin = np.asarray(vmin, dtype=float)
        vmax = np.asarray(vmax, dtype=float)
        denom = vmax - vmin
        out = np.zeros_like(vec)
        mask = denom > 1e-12
        out[mask] = (vec[mask] - vmin[mask]) / denom[mask]
        out = np.clip(out, 0.0, 1.0)
        out = (out * 2.0) - 1.0  # -> [-1, 1]
        return out

    def local_from_normalized(n_zero):
        """
        Map zero-centered normalized vector n_zero to local meters relative to the base corner:
          x_local = XY_HALF_RANGE_M * nx
          y_local = XY_HALF_RANGE_M * ny
          z_local = REST_HEIGHT + Z_HALF_RANGE_M * nz
        """
        nx, ny, nz = n_zero
        return np.array([XY_HALF_RANGE_M * nx,
                         XY_HALF_RANGE_M * ny,
                         REST_HEIGHT + Z_HALF_RANGE_M * nz], dtype=float)

    def update():
        nonlocal capturing_minmax, capturing_neutral
        nonlocal minmax_start_time, neutral_start_time
        nonlocal have_body_shape, P_body
        nonlocal extruded_mesh  # NEW

        # Read serial
        while ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore')
            except Exception:
                break
            vals = parse_uart_line(line)
            if not vals:
                continue
            # Preserve your current sensor reorder (as in your working code)
            vals = vals[6:9] + vals[3:6] + vals[0:3]  # flatten to 9 values
            data_buffer.append(vals)
            if len(data_buffer) > BUFFER_SIZE:
                data_buffer.pop(0)

            # Collect calibration samples
            if capturing_minmax:
                minmax_samples[0].append(vals[0:3])
                minmax_samples[1].append(vals[3:6])
                minmax_samples[2].append(vals[6:9])
            elif capturing_neutral:
                # Normalize with current min-max and store normalized for neutral offsets
                for i, col0 in enumerate([0, 3, 6]):
                    v_raw = np.array(vals[col0:col0+3], dtype=float)
                    v_norm = minmax_normalize(v_raw, calib_min[i], calib_max[i])
                    neutral_samples_norm[i].append(v_norm)

        # Min–max calibration timing/completion
        if capturing_minmax:
            elapsed = time.time() - minmax_start_time
            remaining = max(0.0, CALIB_SECONDS_MINMAX - elapsed)
            status_lbl.setText(f'Calibrating Min-Max: {remaining:.1f}s remaining...')
            if elapsed >= CALIB_SECONDS_MINMAX:
                capturing_minmax = False
                any_ok = False
                for i in range(3):
                    if len(minmax_samples[i]) > 0:
                        arr = np.array(minmax_samples[i], dtype=float)
                        vmin = np.min(arr, axis=0)
                        vmax = np.max(arr, axis=0)
                        same = np.isclose(vmax, vmin)
                        vmax[same] = vmin[same] + 1e-6
                        calib_min[i] = vmin
                        calib_max[i] = vmax
                        any_ok = True
                    else:
                        calib_min[i] = np.array([0.0, 0.0, 0.0])
                        calib_max[i] = np.array([1.0, 1.0, 1.0])
                if any_ok:
                    status_lbl.setText('Status: Min-Max complete. Now capture Neutral (5s).')
                else:
                    status_lbl.setText('Status: Min-Max failed (no data). Try again.')

        # Neutral capture timing/completion
        if capturing_neutral:
            elapsed = time.time() - neutral_start_time
            remaining = max(0.0, CALIB_SECONDS_NEUTRAL - elapsed)
            status_lbl.setText(f'Capturing Neutral Rest: {remaining:.1f}s remaining...')
            if elapsed >= CALIB_SECONDS_NEUTRAL:
                capturing_neutral = False
                # Compute per-sensor neutral offsets (mean normalized values)
                for i in range(3):
                    if len(neutral_samples_norm[i]) > 0:
                        arrn = np.array(neutral_samples_norm[i], dtype=float)
                        neutral_offsets[i] = np.mean(arrn, axis=0)
                    else:
                        neutral_offsets[i] = np.zeros(3)
                # Learn platform body-frame corners from neutral pose (meters), using local frames
                Q0_list = []
                for i in range(3):
                    local_rest = np.array([0.0, 0.0, REST_HEIGHT], dtype=float)
                    local_rest_flipped = S_local[i] @ local_rest
                    Qi0 = BASE_CORNERS_A[i] + R_local_to_world[i] @ local_rest_flipped
                    Q0_list.append(Qi0)
                Q0 = np.vstack(Q0_list)
                centroid = Q0.mean(axis=0)
                P_body = Q0 - centroid   # body-frame corners centered at neutral centroid
                have_body_shape = True
                status_lbl.setText('Status: Neutral offsets and body shape captured. Visualizing.')

        # Need data and completed calibrations to visualize
        if not data_buffer:
            return
        if not all(m is not None for m in calib_min) or not all(M is not None for M in calib_max):
            return
        if capturing_minmax or capturing_neutral or not have_body_shape or P_body is None:
            return

        arr = np.array(data_buffer, dtype=float)

        # Smooth last N samples per sensor
        def smooth_last(col0):
            sub = arr[-N_SMOOTH:, col0:col0+3] if arr.shape[0] >= N_SMOOTH else arr[:, col0:col0+3]
            return np.mean(sub, axis=0)

        v_raw = [smooth_last(0), smooth_last(3), smooth_last(6)]

        # Normalize, zero-center, map to local meters, then to world per sensor
        Q_world = []
        for i in range(3):
            n = minmax_normalize(v_raw[i], calib_min[i], calib_max[i])  # [-1, 1]
            n_zero = n - neutral_offsets[i]                              # rest ~ 0
            local_m = local_from_normalized(n_zero)                      # meters in local frame
            local_m_flipped = S_local[i] @ local_m                       # optional per-axis sign correction
            Qi = BASE_CORNERS_A[i] + R_local_to_world[i] @ local_m_flipped  # world corner
            Q_world.append(Qi)
        Q_world = np.vstack(Q_world)  # (3,3)

        # Optional: amplify translation only (rotation unchanged)
        if (TRANS_GAIN_XY != 1.0) or (TRANS_GAIN_Z != 1.0):
            # Amplify common-mode displacement relative to base corners
            r_mean = (Q_world - BASE_CORNERS_A).mean(axis=0)
            gain_vec = np.array([TRANS_GAIN_XY, TRANS_GAIN_XY, TRANS_GAIN_Z], dtype=float)
            delta = gain_vec * r_mean - r_mean
            Q_world_amp = Q_world + delta
        else:
            Q_world_amp = Q_world

        # Solve pose from body-frame corners -> amplified world corners
        R, t = kabsch_rigid_transform(P_body, Q_world_amp)

        # Update leg lines (base corner to measured Qi, unamplified)
        for i, (b, q, col, leg_item) in enumerate(zip(BASE_CORNERS_A, Q_world, leg_colors, leg_items)):
            leg_item.setData(pos=np.array([b, q], dtype=float), color=col, width=3)

        # Update measured upper corner markers (unamplified)
        qi_scatter.setData(pos=Q_world)

        # Update upper triangle wireframe by transforming body-frame corners (amplified translation, same rotation)
        V_world_amp = (R @ P_body.T).T + t
        plat_scatter.setData(pos=V_world_amp)
        for edge_item, (i0, i1) in zip(plat_edge_items, BASE_EDGES):
            edge_item.setData(pos=np.array([V_world_amp[i0], V_world_amp[i1]], dtype=float),
                              color=(1,1,0,0.9), width=3)

        # NEW: build/update a triangular prism extruded along the triangle's local normal
        # Create the mesh item once
        if extruded_mesh is None:
            extruded_mesh = gl.GLMeshItem(
                smooth=False,
                drawFaces=True,
                drawEdges=True,
                color=(1.0, 1.0, 0.0, 0.25)  # semi-transparent yellow
            )
            w.addItem(extruded_mesh)

        # Compute normal of the current upper triangle
        e1 = V_world_amp[1] - V_world_amp[0]
        e2 = V_world_amp[2] - V_world_amp[0]
        n = np.cross(e1, e2)
        nn = np.linalg.norm(n)
        if nn > 1e-12:
            n = n / nn
        else:
            n = np.array([0.0, 0.0, 1.0], dtype=float)  # fallback

        # Extrusion height equals SIDE (centered: +SIDE/2 and -SIDE/2)
        h = SIDE
        top = V_world_amp + 0.5 * h * n
        bot = V_world_amp - 0.5 * h * n

        # Vertex order: [top0, top1, top2, bot0, bot1, bot2]
        verts = np.vstack([top, bot]).astype(float)

        # Faces: 2 caps + 3 side quads split into triangles
        faces = np.array([
            [0, 1, 2],  # top cap
            [5, 4, 3],  # bottom cap (reverse winding)

            [0, 1, 4],  # side 0-1
            [0, 4, 3],

            [1, 2, 5],  # side 1-2
            [1, 5, 4],

            [2, 0, 3],  # side 2-0
            [2, 3, 5],
        ], dtype=np.int32)

        # Update the mesh geometry
        extruded_mesh.setMeshData(vertexes=verts, faces=faces)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)  # ~20 FPS

    # Run app
    app.exec_()
    ser.close()

if __name__ == "__main__":
    main()
