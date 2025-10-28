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
# Note: These are still used for Qi measurement visualization only; the virtual triangle uses rate-control
TRANS_GAIN_XY = 3.0
TRANS_GAIN_Z  = 1.0

# Optional per-sensor axis sign flips in local frame (set to -1 if a sensor axis is reversed)
AXIS_SIGNS = [
    np.array([+1.0, +1.0, +1.0]),  # Sensor 1 local [x,y,z] signs
    np.array([+1.0, +1.0, +1.0]),  # Sensor 2 local [x,y,z] signs
    np.array([+1.0, +1.0, +1.0]),  # Sensor 3 local [x,y,z] signs
]

# =========================
# RATE-CONTROL PARAMETERS (tuned for faster feel)
# =========================

# Deadzones: no motion until the measured deflection exceeds these per-axis thresholds
DEADZONE_TRANS = np.array([0.005, 0.005, 0.40], dtype=float)   # meters; large Z deadzone to suppress drift
DEADZONE_ROT_DEG = np.array([2.0, 2.0, 2.0], dtype=float)      # degrees
DEADZONE_ROT = np.deg2rad(DEADZONE_ROT_DEG)                    # radians

# Gains: strong XY translation, rotation; Z translation disabled for now
GAIN_TRANS = np.array([8.0, 8.0, 0.0], dtype=float)            # m/s per meter; set Z=0 until drift fixed
GAIN_ROT   = np.array([5.0, 5.0, 5.0], dtype=float)            # rad/s per rad

# Safety limits for commanded speeds
MAX_SPEED_TRANS = np.array([2.5, 2.5, 0.0], dtype=float)       # m/s; Z speed capped to 0 while disabled
MAX_SPEED_ROT   = np.array([10.0, 10.0, 10.0], dtype=float)    # rad/s

# Smoothing of the 6D deflection before deadzone/gain (lower for responsiveness)
ALPHA_SMOOTH_DEFLECTION = 0.05

# Global multiplier to scale both translation and rotation speeds
SPEED_SCALE = 4.0

# Optional: sublinear curve to boost small deflections (gamma < 1.0); set to 1.0 to disable
GAMMA_TRANS = 0.7
GAMMA_ROT   = 0.7

# =========================
# Geometry helpers
# =========================

def equilateral_triangle(side):
    s = side
    v1 = np.array([+s/np.sqrt(3), 0.0, 0.0])
    v2 = np.array([-s/(2*np.sqrt(3)), +s/2.0, 0.0])
    v3 = np.array([-s/(2*np.sqrt(3)), -s/2.0, 0.0])
    return [v1, v2, v3]

BASE_CORNERS = equilateral_triangle(SIDE)
BASE_CORNERS_A = np.vstack(BASE_CORNERS)
BASE_EDGES = [(0,1), (1,2), (2,0)]

def build_local_frames_with_x_inward(base_corners):
    B_centroid = np.mean(base_corners, axis=0)
    R_list = []
    for Bi in base_corners:
        x_dir = B_centroid - Bi
        xnorm = np.linalg.norm(x_dir)
        x_dir = x_dir / (xnorm + 1e-12)

        z_dir = np.array([0.0, 0.0, 1.0])
        y_dir = np.cross(z_dir, x_dir)
        ynorm = np.linalg.norm(y_dir)
        y_dir = y_dir / (ynorm + 1e-12)

        z_dir = np.cross(x_dir, y_dir)
        znorm = np.linalg.norm(z_dir)
        z_dir = z_dir / (znorm + 1e-12)

        R = np.column_stack([x_dir, y_dir, z_dir])  # world = R @ local
        R_list.append(R)
    return R_list

# =========================
# Math helpers
# =========================

def parse_uart_line(line):
    try:
        vals = [float(x) for x in line.strip().split(',') if x]
        if len(vals) == 9:
            return vals
    except Exception:
        pass
    return None

def kabsch_rigid_transform(P, Q):
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
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1.0
        R = Vt.T @ U.T
    t = q_mean - R @ p_mean
    return R, t

# =========================
# RATE-CONTROL math helpers
# =========================

def rotmat_to_rotvec(R):
    """
    Convert rotation matrix R to rotation vector r = axis * angle (radians).
    Numerically stable for small angles.
    """
    tr = np.clip(np.trace(R), -1.0, 3.0)
    cos_theta = (tr - 1.0) * 0.5
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)

    if theta < 1e-6:
        rx = 0.5 * (R[2,1] - R[1,2])
        ry = 0.5 * (R[0,2] - R[2,0])
        rz = 0.5 * (R[1,0] - R[0,1])
        return np.array([rx, ry, rz], dtype=float)

    denom = 2.0 * np.sin(theta)
    rx = (R[2,1] - R[1,2]) / denom
    ry = (R[0,2] - R[2,0]) / denom
    rz = (R[1,0] - R[0,1]) / denom
    axis = np.array([rx, ry, rz], dtype=float)
    norm_axis = np.linalg.norm(axis)
    if norm_axis > 1e-12:
        axis = axis / norm_axis
    return axis * theta

def deadzone_vector(x, dz):
    """
    Per-axis deadzone: sign(x) * max(0, |x| - dz)
    """
    x = np.asarray(x, dtype=float)
    dz = np.asarray(dz, dtype=float)
    out = np.zeros_like(x)
    for i in range(x.shape[0]):
        mag = abs(x[i])
        if mag > dz[i]:
            out[i] = np.sign(x[i]) * (mag - dz[i])
        else:
            out[i] = 0.0
    return out

def clip_per_axis(x, max_abs):
    x = np.asarray(x, dtype=float)
    max_abs = np.asarray(max_abs, dtype=float)
    out = np.clip(x, -max_abs, max_abs)
    return out

def apply_curve(x, gamma):
    """
    Sublinear curve to boost small commands: sign(x) * |x|**gamma
    gamma < 1.0 increases small values more than large ones.
    """
    x = np.asarray(x, dtype=float)
    if gamma == 1.0:
        return x
    return np.sign(x) * np.power(np.abs(x), gamma)

def scale_cmds(v_cmd, w_cmd, scale):
    return v_cmd * scale, w_cmd * scale

def quat_from_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # w, x, y, z
    axis = axis / norm
    half = 0.5 * angle
    s = np.sin(half)
    return np.array([np.cos(half), axis[0]*s, axis[1]*s, axis[2]*s], dtype=float)

def quat_mul(q1, q2):
    """
    Hamilton product q = q1 * q2, both wxyz
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)

def quat_normalize(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n

def quat_to_rotmat(q):
    w, x, y, z = q
    n = np.dot(q, q)
    if n < 1e-12:
        return np.eye(3)
    q = q / np.sqrt(n)
    w, x, y, z = q
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ], dtype=float)
    return R

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
    w.setWindowTitle('Upper Triangle 6-DoF Rate-Control (fast response)')
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

    btn_recenter = QtWidgets.QPushButton('Re-center Virtual Pose', w)
    btn_recenter.setGeometry(360, 10, 180, 32)
    btn_recenter.show()

    status_lbl = QtWidgets.QLabel('Status: Ready', w)
    status_lbl.setGeometry(550, 10, 540, 32)
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

    # Extruded 3D mesh for the upper triangle (created on first update)
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
    calib_min = [None, None, None]
    calib_max = [None, None, None]
    # Per-sensor per-axis neutral offsets in NORMALIZED units
    neutral_offsets = [np.zeros(3), np.zeros(3), np.zeros(3)]

    # Platform body-frame corners P_body (3x3), learned from neutral pose (meters)
    have_body_shape = False
    P_body = None

    # RATE-CONTROL: virtual pose state (persistent free-flying pose)
    virtual_pos = None            # 3-vector (world)
    virtual_quat = None           # quaternion [w,x,y,z]
    neutral_centroid = None       # world centroid at neutral

    # Deflection smoothing state (for rate-control)
    smooth_trans = np.zeros(3, dtype=float)
    smooth_rotvec = np.zeros(3, dtype=float)

    # Timing
    last_time = None

    # Buffers
    data_buffer = []
    minmax_samples = [[], [], []]
    neutral_samples_norm = [[], [], []]

    def start_minmax():
        nonlocal capturing_minmax, minmax_start_time, minmax_samples
        capturing_minmax = True
        minmax_start_time = time.time()
        minmax_samples = [[], [], []]
        status_lbl.setText(f'Calibrating Min-Max: {int(CALIB_SECONDS_MINMAX)}s remaining...')
    btn_minmax.clicked.connect(start_minmax)

    def start_neutral():
        nonlocal capturing_neutral, neutral_start_time, neutral_samples_norm
        if not all(m is not None for m in calib_min) or not all(M is not None for M in calib_max):
            status_lbl.setText('Status: Complete Min-Max calibration first.')
            return
        capturing_neutral = True
        neutral_start_time = time.time()
        neutral_samples_norm = [[], [], []]
        status_lbl.setText(f'Capturing Neutral Rest: {int(CALIB_SECONDS_NEUTRAL)}s remaining...')
    btn_neutral.clicked.connect(start_neutral)

    def recenter_virtual():
        nonlocal virtual_pos, virtual_quat, neutral_centroid
        if neutral_centroid is not None:
            virtual_pos = neutral_centroid.copy()
            virtual_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
            status_lbl.setText('Status: Virtual pose re-centered to neutral.')
        else:
            status_lbl.setText('Status: Capture Neutral first, then re-center.')
    btn_recenter.clicked.connect(recenter_virtual)

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
        nx, ny, nz = n_zero
        return np.array([XY_HALF_RANGE_M * nx,
                         XY_HALF_RANGE_M * ny,
                         REST_HEIGHT + Z_HALF_RANGE_M * nz], dtype=float)

    def update():
        nonlocal capturing_minmax, capturing_neutral
        nonlocal minmax_start_time, neutral_start_time
        nonlocal have_body_shape, P_body
        nonlocal extruded_mesh
        nonlocal virtual_pos, virtual_quat, neutral_centroid
        nonlocal smooth_trans, smooth_rotvec
        nonlocal last_time

        # Read serial
        while ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore')
            except Exception:
                break
            vals = parse_uart_line(line)
            if not vals:
                continue
            # Preserve your current sensor reorder
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

                # RATE-CONTROL: initialize virtual pose at neutral
                neutral_centroid = centroid.copy()
                virtual_pos = centroid.copy()
                virtual_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # identity
                last_time = time.monotonic()

                status_lbl.setText('Status: Neutral captured. Virtual pose initialized (rate-control mode).')

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
            local_m_flipped = S_local[i] @ local_m
            Qi = BASE_CORNERS_A[i] + R_local_to_world[i] @ local_m_flipped  # world corner
            Q_world.append(Qi)
        Q_world = np.vstack(Q_world)  # (3,3)

        # Optional: amplify translation only (rotation unchanged) for visualization of measured Qi
        if (TRANS_GAIN_XY != 1.0) or (TRANS_GAIN_Z != 1.0):
            r_mean = (Q_world - BASE_CORNERS_A).mean(axis=0)
            gain_vec = np.array([TRANS_GAIN_XY, TRANS_GAIN_XY, TRANS_GAIN_Z], dtype=float)
            delta = gain_vec * r_mean - r_mean
            Q_world_amp = Q_world + delta
        else:
            Q_world_amp = Q_world

        # Solve measured pose from body-frame -> world corners (use unamplified for control fidelity)
        R_meas, t_meas = kabsch_rigid_transform(P_body, Q_world)

        # Update leg lines (base corner to measured Qi, unamplified)
        for i, (b, q, col, leg_item) in enumerate(zip(BASE_CORNERS_A, Q_world, leg_colors, leg_items)):
            leg_item.setData(pos=np.array([b, q], dtype=float), color=col, width=3)

        # Update measured upper corner markers (unamplified)
        qi_scatter.setData(pos=Q_world)

        # =========================
        # RATE-CONTROL: compute velocities from deflection and integrate
        # =========================

        # Compute deflection in 6D: translation (meters) and rotation vector (radians)
        trans_defl = t_meas  # 3-vector
        rotvec_defl = rotmat_to_rotvec(R_meas)  # 3-vector (axis * angle)

        # Smooth deflection (optional)
        alpha = ALPHA_SMOOTH_DEFLECTION
        if alpha > 0.0:
            smooth_trans = (1.0 - alpha) * smooth_trans + alpha * trans_defl
            smooth_rotvec = (1.0 - alpha) * smooth_rotvec + alpha * rotvec_defl
            trans_defl_use = smooth_trans
            rotvec_defl_use = smooth_rotvec
        else:
            trans_defl_use = trans_defl
            rotvec_defl_use = rotvec_defl

        # Apply per-axis deadzones
        trans_after_dz = deadzone_vector(trans_defl_use, DEADZONE_TRANS)
        rot_after_dz   = deadzone_vector(rotvec_defl_use, DEADZONE_ROT)

        # Optional nonlinear curve to boost small deflections
        trans_after_curve = apply_curve(trans_after_dz, GAMMA_TRANS)
        rot_after_curve   = apply_curve(rot_after_dz, GAMMA_ROT)

        # Map to velocities
        v_cmd = GAIN_TRANS * trans_after_curve
        w_cmd = GAIN_ROT   * rot_after_curve

        # Global speed scale
        v_cmd, w_cmd = scale_cmds(v_cmd, w_cmd, SPEED_SCALE)

        # Clip to limits
        v_cmd = clip_per_axis(v_cmd, MAX_SPEED_TRANS)
        w_cmd = clip_per_axis(w_cmd, MAX_SPEED_ROT)

        # Integrate to update virtual pose
        now = time.monotonic()
        if last_time is None:
            dt = 0.05
        else:
            dt = max(0.0, min(0.2, now - last_time))  # clamp dt to avoid spikes
        last_time = now

        if virtual_pos is None or virtual_quat is None:
            # Initialize just in case
            virtual_pos = BASE_CORNERS_A.mean(axis=0) + np.array([0, 0, REST_HEIGHT], dtype=float)
            virtual_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        # Position integration
        virtual_pos = virtual_pos + v_cmd * dt

        # Orientation integration: small-angle delta quaternion from w_cmd*dt
        angle = np.linalg.norm(w_cmd) * dt
        if angle > 1e-12:
            axis = w_cmd / (np.linalg.norm(w_cmd) + 1e-12)
            dq = quat_from_axis_angle(axis, angle)
            virtual_quat = quat_mul(virtual_quat, dq)
            virtual_quat = quat_normalize(virtual_quat)

        # Convert virtual quaternion to rotation matrix
        R_virtual = quat_to_rotmat(virtual_quat)

        # World vertices of the virtual triangle
        V_virtual = (R_virtual @ P_body.T).T + virtual_pos

        # Update upper triangle wireframe (now driven by virtual pose)
        plat_scatter.setData(pos=V_virtual)
        for edge_item, (i0, i1) in zip(plat_edge_items, BASE_EDGES):
            edge_item.setData(pos=np.array([V_virtual[i0], V_virtual[i1]], dtype=float),
                              color=(1,1,0,0.9), width=3)

        # Extruded mesh update (from virtual pose)
        if extruded_mesh is None:
            extruded_mesh = gl.GLMeshItem(
                smooth=False,
                drawFaces=True,
                drawEdges=True,
                color=(1.0, 1.0, 0.0, 0.25)
            )
            w.addItem(extruded_mesh)

        e1 = V_virtual[1] - V_virtual[0]
        e2 = V_virtual[2] - V_virtual[0]
        n = np.cross(e1, e2)
        nn = np.linalg.norm(n)
        if nn > 1e-12:
            n = n / nn
        else:
            n = np.array([0.0, 0.0, 1.0], dtype=float)

        h = SIDE
        top = V_virtual + 0.5 * h * n
        bot = V_virtual - 0.5 * h * n
        verts = np.vstack([top, bot]).astype(float)

        faces = np.array([
            [0, 1, 2],
            [5, 4, 3],
            [0, 1, 4],
            [0, 4, 3],
            [1, 2, 5],
            [1, 5, 4],
            [2, 0, 3],
            [2, 3, 5],
        ], dtype=np.int32)

        extruded_mesh.setMeshData(vertexes=verts, faces=faces)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)  # ~20 FPS

    # Run app
    app.exec_()
    ser.close()

if __name__ == "__main__":
    main()
