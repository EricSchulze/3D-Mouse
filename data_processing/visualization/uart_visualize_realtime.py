
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets
import serial
import numpy as np

# UART serial port settings
SERIAL_PORT = 'COM128'  # Change to your UART port
BAUD_RATE = 115200

# Data buffer size
BUFFER_SIZE = 100

def parse_uart_line(line):
    try:
        values = [float(x) for x in line.strip().split(',') if x]
        if len(values) == 9:
            return values
    except Exception:
        pass
    return None

def main():
    # Set up serial connection
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    # Set up pyqtgraph 3D window
    app = QtWidgets.QApplication([])
    w = gl.GLViewWidget()
    w.setWindowTitle('Real-time UART 3D Visualization')
    w.setGeometry(0, 110, 800, 600)
    w.show()

    # Add axes grid
    grid = gl.GLGridItem()
    w.addItem(grid)

    # Arrow items for three sensors
    arrow1 = gl.GLLinePlotItem()
    arrow2 = gl.GLLinePlotItem()
    arrow3 = gl.GLLinePlotItem()
    w.addItem(arrow1)
    w.addItem(arrow2)
    w.addItem(arrow3)


    data_buffer = []
    # Calibration offsets for each sensor
    offset1 = None
    offset2 = None
    offset3 = None
    offset_count = 0
    CALIBRATION_FRAMES = 10  # Ignore first N frames for offset

    # Add calibration button
    btn = QtWidgets.QPushButton('Calibrate Zero')
    btn.setFixedWidth(120)
    btn.setFixedHeight(40)
    btn.move(10, 10)
    btn.setParent(w)
    btn.show()

    def update():
        while ser.in_waiting:
            line = ser.readline().decode('utf-8')
            values = parse_uart_line(line)
            if values:
                data_buffer.append(values)
                if len(data_buffer) > BUFFER_SIZE:
                    data_buffer.pop(0)
        if data_buffer:
            arr = np.array(data_buffer)
            # Each row: [x1,y1,z1, x2,y2,z2, x3,y3,z3]
            sensor1 = arr[:, 0:3]
            sensor2 = arr[:, 3:6]
            sensor3 = arr[:, 6:9]

            # Improved calibration: use mean of first CALIBRATION_FRAMES frames as offset
            nonlocal offset1, offset2, offset3, offset_count
            if offset_count < CALIBRATION_FRAMES and len(sensor1) >= CALIBRATION_FRAMES:
                offset1 = np.mean(sensor1[:CALIBRATION_FRAMES], axis=0)
                offset2 = np.mean(sensor2[:CALIBRATION_FRAMES], axis=0)
                offset3 = np.mean(sensor3[:CALIBRATION_FRAMES], axis=0)
                offset_count = CALIBRATION_FRAMES

            norm_sensor1 = sensor1 - offset1 if offset1 is not None else sensor1
            norm_sensor2 = sensor2 - offset2 if offset2 is not None else sensor2
            norm_sensor3 = sensor3 - offset3 if offset3 is not None else sensor3

            # Normalize to unit length (if not zero)
            def normalize(vec):
                norm = np.linalg.norm(vec)
                return vec / norm if norm > 0 else vec

            scale = 5  # Adjust this value for desired arrow length
            N = 5  # Moving average window size
            def smooth_vec(vecs):
                if len(vecs) >= N:
                    return np.mean(vecs[-N:], axis=0)
                elif len(vecs) > 0:
                    return vecs[-1]
                else:
                    return np.array([0,0,0])

            arrow_vec1 = scale * normalize(smooth_vec(norm_sensor1))
            arrow_vec2 = scale * normalize(smooth_vec(norm_sensor2))
            arrow_vec3 = scale * normalize(smooth_vec(norm_sensor3))

            # Draw arrows from origin to each sensor's normalized and scaled vector
            arrow1.setData(pos=np.array([[0,0,0], arrow_vec1]), color=(1,0,0,1), width=3)
            arrow2.setData(pos=np.array([[0,0,0], arrow_vec2]), color=(0,1,0,1), width=3)
            arrow3.setData(pos=np.array([[0,0,0], arrow_vec3]), color=(0,0,1,1), width=3)

    # Calibration button resets offsets
    def calibrate():
        nonlocal offset1, offset2, offset3, offset_count
        offset1 = None
        offset2 = None
        offset3 = None
        offset_count = 0
    btn.clicked.connect(calibrate)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)  # update every 50 ms

    app.exec_()
    ser.close()

if __name__ == '__main__':
    main()
