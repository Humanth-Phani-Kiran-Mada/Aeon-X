#!/usr/bin/env python3
"""
============================================================
 AEON X — AUX-9 PROJECT
 MODULE  : Raspberry Pi AI Decision Engine
 FIXED v1.1

 FIXES IN v1.1:
   [BUG-I]  VL53L0X_Simple.read_mm() returned a stub value (999) always.
            Replaced with real VL53L0X library call using
            adafruit_vl53l0x. If library not installed, falls back
            gracefully with a warning instead of silently returning 999.

   [BUG-J]  CameraPipeline: cv2.VideoCapture(0) opens the wrong camera
            on Pi — the Pi CSI camera is not /dev/video0 by default.
            With libcamera stack (Pi OS Bullseye+) you need:
            VideoCapture(0, cv2.CAP_V4L2) or use 'libcamera-vid'.
            Fixed: try cv2.CAP_V4L2 first, fallback to default index.

   [BUG-K]  SensorFusion._median_filter() created self._bufs dict inside
            the method using hasattr() — this works but is fragile.
            Moved _bufs initialization to __init__() properly.

   [BUG-L]  STM32Comm._parse_line() silently swallowed ValueError on
            malformed telemetry. Added specific error logging so bad
            packets are visible during debugging.

   [BUG-M]  MissionManager._tick() called self.mpu.read() and
            self.tof.read_mm() with no check for None — but self.mpu
            and self.tof are set to None if I2C init fails. The `if`
            check existed in _tick but self.tof.read_mm() was called
            unconditionally. Fixed the None guard.

   [BUG-N]  GPSReader thread: pynmea2.parse() can throw
            pynmea2.nmea.ChecksumError in addition to ParseError.
            Both were not caught. Fixed with broad Exception catch
            with type logging.

   [BUG-O]  NavigationEngine.decide() called _avoid_obstacle() which
            returns LEFT/RIGHT/REV/STOP, but the caller then
            passed this back through decide() as a NavDecision — the
            return value was correct but the obstacle state was not
            cleared between ticks, so the avoidance could loop.
            Added a clear obstacle state path.

   [BUG-P]  Signal handler called mission.emergency_stop() then
            mission.stop() — but stop() also calls send_command("STOP")
            and stm32.stop() which closes the serial port. If emergency_stop()
            already closed it via the thread, stop() raised an exception.
            Fixed with try/except around stop() in signal handler.

 INSTALL:
   pip3 install pyserial smbus2 pynmea2 opencv-python numpy
   pip3 install adafruit-circuitpython-vl53l0x
   OR: pip3 install VL53L0X   (Pimoroni version)
============================================================
"""

import time
import threading
import serial
import logging
import math
from dataclasses import dataclass, field
from typing      import Optional
from enum        import Enum, auto
from collections import deque
from datetime    import datetime

# Optional imports
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    import smbus2
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False

try:
    import pynmea2
    GPS_AVAILABLE = True
except ImportError:
    GPS_AVAILABLE = False

# BUG-I fix: try real VL53L0X library
VL53_AVAILABLE = False
try:
    import adafruit_vl53l0x
    import busio
    import board
    VL53_AVAILABLE = True
except ImportError:
    pass

# ============================================================
#  LOGGING
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s][%(levelname)s][%(name)s] %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(f'/tmp/aeon_x_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
    ]
)
log = logging.getLogger("AEON-X")

# ============================================================
#  CONSTANTS
# ============================================================
UART_PORT_STM32 = '/dev/serial0'
UART_BAUD_STM32 = 115200
GPS_UART_PORT   = '/dev/ttyUSB0'
GPS_BAUD        = 9600
I2C_BUS         = 1
MPU6050_ADDR    = 0x68

DANGER_CM       = 30
WARN_CM         = 60
COMMAND_RATE_HZ = 10
CAMERA_RATE_HZ  = 15

# ============================================================
#  ENUMS
# ============================================================
class VehicleMode(Enum):
    IDLE       = auto()
    AUTONOMOUS = auto()
    MANUAL     = auto()
    ESTOP      = auto()
    DEGRADED   = auto()

class NavDecision(Enum):
    STOP  = "STOP"
    FWD   = "FWD"
    REV   = "REV"
    LEFT  = "LEFT"
    RIGHT = "RIGHT"

# ============================================================
#  DATA CLASSES
# ============================================================
@dataclass
class STM32Telemetry:
    state:    int   = 0
    mode:     int   = 0
    front_cm: float = 999.0
    rear_cm:  float = 999.0
    left_cm:  float = 999.0
    right_cm: float = 999.0
    ir_left:  bool  = False
    ir_right: bool  = False
    timestamp: float = field(default_factory=time.time)

@dataclass
class IMUData:
    accel_x: float = 0.0; accel_y: float = 0.0; accel_z: float = 0.0
    gyro_x:  float = 0.0; gyro_y:  float = 0.0; gyro_z:  float = 0.0
    pitch:   float = 0.0; roll:    float = 0.0
    timestamp: float = field(default_factory=time.time)

@dataclass
class GPSData:
    latitude: float = 0.0; longitude: float = 0.0
    speed_knots: float = 0.0; heading: float = 0.0
    valid: bool = False
    timestamp: float = field(default_factory=time.time)

@dataclass
class CameraData:
    obstacle_detected:  bool  = False
    obstacle_direction: str   = "NONE"
    confidence:         float = 0.0
    timestamp: float = field(default_factory=time.time)

@dataclass
class FusedSensorState:
    front_cm: float = 999.0; rear_cm: float = 999.0
    left_cm:  float = 999.0; right_cm: float = 999.0
    ir_left:  bool = False;  ir_right: bool = False
    tof_cm:   float = 999.0
    pitch:    float = 0.0;   roll: float = 0.0
    gps:      GPSData = field(default_factory=GPSData)
    cam:      CameraData = field(default_factory=CameraData)
    obstacle_ahead: bool = False
    safe_to_move:   bool = True

# ============================================================
#  MPU6050
# ============================================================
class MPU6050:
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT = 0x3B
    GYRO_XOUT  = 0x43
    ACCEL_SCALE = 16384.0
    GYRO_SCALE  = 131.0

    def __init__(self, bus, addr=MPU6050_ADDR):
        self.bus = bus; self.addr = addr
        try:
            bus.write_byte_data(addr, self.PWR_MGMT_1, 0)
            log.info("[MPU6050] Initialized")
        except Exception as e:
            log.error(f"[MPU6050] Init failed: {e}")

    def _rw2c(self, reg):
        h = self.bus.read_byte_data(self.addr, reg)
        l = self.bus.read_byte_data(self.addr, reg + 1)
        v = (h << 8) + l
        return v - 65536 if v >= 0x8000 else v

    def read(self) -> IMUData:
        try:
            ax = self._rw2c(self.ACCEL_XOUT)     / self.ACCEL_SCALE
            ay = self._rw2c(self.ACCEL_XOUT + 2) / self.ACCEL_SCALE
            az = self._rw2c(self.ACCEL_XOUT + 4) / self.ACCEL_SCALE
            gx = self._rw2c(self.GYRO_XOUT)      / self.GYRO_SCALE
            gy = self._rw2c(self.GYRO_XOUT + 2)  / self.GYRO_SCALE
            gz = self._rw2c(self.GYRO_XOUT + 4)  / self.GYRO_SCALE
            pitch = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
            roll  = math.degrees(math.atan2(-ax, az))
            return IMUData(ax, ay, az, gx, gy, gz, pitch, roll)
        except Exception as e:
            log.warning(f"[MPU6050] Read error: {e}")
            return IMUData()

# ============================================================
#  VL53L0X — BUG-I fix: real library call
# ============================================================
class VL53L0X_Driver:
    def __init__(self):
        self._tof = None
        if VL53_AVAILABLE:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self._tof = adafruit_vl53l0x.VL53L0X(i2c)
                log.info("[VL53L0X] Initialized via adafruit library")
            except Exception as e:
                log.error(f"[VL53L0X] Init failed: {e}")
        else:
            log.warning("[VL53L0X] adafruit_vl53l0x not installed — ToF disabled")

    def read_mm(self) -> int:
        if self._tof is None:
            return 9999
        try:
            return self._tof.range  # Returns mm
        except Exception:
            return 9999

# ============================================================
#  GPS READER — BUG-N fix: catch ChecksumError too
# ============================================================
class GPSReader(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port; self.baud = baud
        self._data = GPSData(); self._lock = threading.Lock()
        self._running = False

    def run(self):
        if not GPS_AVAILABLE:
            log.warning("[GPS] pynmea2 not installed")
            return
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                self._running = True
                log.info(f"[GPS] Reading {self.port}")
                while self._running:
                    try:
                        raw = ser.readline()
                        line = raw.decode('ascii', errors='ignore').strip()
                        if line.startswith(('$GPRMC', '$GNRMC')):
                            msg = pynmea2.parse(line)
                            if msg.status == 'A':
                                with self._lock:
                                    self._data = GPSData(
                                        latitude=float(msg.latitude),
                                        longitude=float(msg.longitude),
                                        speed_knots=float(msg.spd_over_grnd or 0),
                                        heading=float(msg.true_course or 0),
                                        valid=True,
                                        timestamp=time.time()
                                    )
                    except Exception as e:
                        # BUG-N fix: log the exception type, not just ignore it
                        log.debug(f"[GPS] Parse error ({type(e).__name__}): {e}")
        except serial.SerialException as e:
            log.error(f"[GPS] Serial error: {e}")

    def get(self) -> GPSData:
        with self._lock: return self._data

    def stop(self): self._running = False

# ============================================================
#  CAMERA PIPELINE — BUG-J fix: CAP_V4L2 for Pi CSI camera
# ============================================================
class CameraPipeline(threading.Thread):
    def __init__(self, width=320, height=240):
        super().__init__(daemon=True)
        self.width = width; self.height = height
        self._data = CameraData(); self._lock = threading.Lock()
        self._running = False

    def run(self):
        if not CV2_AVAILABLE:
            log.warning("[Camera] OpenCV not available")
            return

        # BUG-J fix: try CAP_V4L2 backend first (required for Pi CSI camera)
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened():
            log.warning("[Camera] CAP_V4L2 failed, trying default backend")
            cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            log.error("[Camera] Could not open camera on index 0")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_RATE_HZ)

        log.info(f"[Camera] Started {self.width}x{self.height}")
        self._running = True

        bg = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=40, detectShadows=False)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        while self._running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05); continue

            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            fg      = bg.apply(blurred)
            fg      = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel)
            roi     = fg[int(self.height * 0.4):, :]
            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            obs = False; direction = "NONE"; confidence = 0.0
            if contours:
                largest = max(contours, key=cv2.contourArea)
                area    = cv2.contourArea(largest)
                if area > 500:
                    obs = True; confidence = min(area / 5000.0, 1.0)
                    M  = cv2.moments(largest)
                    cx = int(M['m10'] / M['m00']) if M['m00'] != 0 else self.width // 2
                    t  = self.width // 3
                    direction = "LEFT" if cx < t else "RIGHT" if cx > 2*t else "CENTER"

            with self._lock:
                self._data = CameraData(obs, direction, confidence, time.time())

            time.sleep(1.0 / CAMERA_RATE_HZ)

        cap.release()

    def get(self) -> CameraData:
        with self._lock: return self._data

    def stop(self): self._running = False

# ============================================================
#  STM32 COMM — BUG-L fix: log parse errors visibly
# ============================================================
class STM32Comm(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port; self.baud = baud
        self._ser = None
        self._tel  = STM32Telemetry()
        self._lock = threading.Lock()
        self._cmds = deque(maxlen=10)
        self._alerts = deque(maxlen=20)
        self._running = False

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=1.0)
            self._running = True
            log.info(f"[STM32] UART opened: {self.port}")
            while self._running:
                if self._ser.in_waiting:
                    try:
                        line = self._ser.readline().decode('utf-8', errors='ignore').strip()
                        if line: self._parse_line(line)
                    except serial.SerialException as e:
                        log.error(f"[STM32] Read error: {e}")
                if self._cmds:
                    cmd = self._cmds.popleft()
                    try:
                        self._ser.write(f"{cmd}\n".encode())
                        log.debug(f"[STM32→] {cmd}")
                    except serial.SerialException as e:
                        log.error(f"[STM32] Write error: {e}")
                time.sleep(0.005)
        except serial.SerialException as e:
            log.error(f"[STM32] Cannot open {self.port}: {e}")

    def _parse_line(self, line: str):
        # BUG-L fix: explicit error logging
        if line.startswith("TEL:"):
            parts = line[4:].split(',')
            if len(parts) != 8:
                log.warning(f"[STM32] Bad telemetry field count: {line}")
                return
            try:
                with self._lock:
                    self._tel = STM32Telemetry(
                        state=int(parts[0]), mode=int(parts[1]),
                        front_cm=float(parts[2]), rear_cm=float(parts[3]),
                        left_cm=float(parts[4]), right_cm=float(parts[5]),
                        ir_left=bool(int(parts[6])), ir_right=bool(int(parts[7])),
                        timestamp=time.time()
                    )
            except ValueError as e:
                log.warning(f"[STM32] Telemetry value error — line='{line}' err={e}")
        elif line.startswith("ALERT:"):
            msg = line[6:]
            self._alerts.append({"msg": msg, "ts": time.time()})
            log.warning(f"[ALERT] {msg}")
        elif line.startswith("STATE:"):
            log.info(f"[STM32] State: {line[6:]}")
        elif line.startswith("AEON_X:READY"):
            log.info("[STM32] Boot confirmed")

    def send_command(self, cmd: str): self._cmds.append(cmd)
    def get_telemetry(self) -> STM32Telemetry:
        with self._lock: return self._tel
    def get_alerts(self):
        a = list(self._alerts); self._alerts.clear(); return a
    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            try: self._ser.close()
            except: pass

# ============================================================
#  SENSOR FUSION — BUG-K fix: _bufs init in __init__
# ============================================================
class SensorFusion:
    def __init__(self):
        # BUG-K fix: proper initialization
        self._bufs = {k: deque(maxlen=5) for k in ('front','rear','left','right')}

    def _median(self, key: str, val: float) -> float:
        self._bufs[key].append(val)
        valid = [v for v in self._bufs[key] if v < 999]
        if not valid: return 999.0
        return sorted(valid)[len(valid) // 2]

    def fuse(self, stm32: STM32Telemetry, imu: IMUData,
             tof_mm: int, gps: GPSData, cam: CameraData) -> FusedSensorState:
        s = FusedSensorState()
        s.front_cm = self._median('front', stm32.front_cm)
        s.rear_cm  = self._median('rear',  stm32.rear_cm)
        s.left_cm  = self._median('left',  stm32.left_cm)
        s.right_cm = self._median('right', stm32.right_cm)
        s.ir_left  = stm32.ir_left;  s.ir_right = stm32.ir_right
        s.tof_cm   = tof_mm / 10.0 if tof_mm < 9000 else 999.0
        s.pitch    = imu.pitch;      s.roll  = imu.roll
        s.gps = gps; s.cam = cam

        obs_us  = s.front_cm < DANGER_CM and s.front_cm < 999
        obs_ir  = s.ir_left or s.ir_right
        obs_tof = s.tof_cm < DANGER_CM and s.tof_cm < 999
        obs_cam = cam.obstacle_detected and cam.obstacle_direction == "CENTER" and cam.confidence > 0.5

        s.obstacle_ahead = obs_us or obs_ir or obs_tof or obs_cam
        s.safe_to_move   = not s.obstacle_ahead
        return s

# ============================================================
#  NAVIGATION ENGINE — BUG-O fix: stateless per-tick decision
# ============================================================
class NavigationEngine:
    def decide(self, state: FusedSensorState) -> NavDecision:
        # IMU tilt safety
        if abs(state.pitch) > 30 or abs(state.roll) > 30:
            log.warning(f"[NAV] Tilt! P={state.pitch:.1f} R={state.roll:.1f}")
            return NavDecision.STOP

        if state.obstacle_ahead:
            return self._avoid(state)

        return NavDecision.FWD

    def _avoid(self, s: FusedSensorState) -> NavDecision:
        # BUG-O fix: pure function — no persistent state needed
        left_clear  = s.left_cm  > WARN_CM
        right_clear = s.right_cm > WARN_CM
        rear_clear  = s.rear_cm  > DANGER_CM

        if left_clear and s.left_cm >= s.right_cm:
            return NavDecision.LEFT
        if right_clear:
            return NavDecision.RIGHT
        if rear_clear:
            return NavDecision.REV
        return NavDecision.STOP

# ============================================================
#  MISSION MANAGER
# ============================================================
class MissionManager:
    def __init__(self):
        self.mode    = VehicleMode.IDLE
        self._lock   = threading.Lock()

        self.stm32 = STM32Comm(UART_PORT_STM32, UART_BAUD_STM32)

        self.mpu = None; self.tof = None; self.i2c_bus = None
        if I2C_AVAILABLE:
            try:
                self.i2c_bus = smbus2.SMBus(I2C_BUS)
                self.mpu     = MPU6050(self.i2c_bus)
                self.tof     = VL53L0X_Driver()
            except Exception as e:
                log.error(f"[I2C] Init failed: {e}")

        self.gps    = GPSReader(GPS_UART_PORT, GPS_BAUD)
        self.camera = CameraPipeline(320, 240)
        self.fusion = SensorFusion()
        self.nav    = NavigationEngine()

        self._last_cmd    = ""
        self._cmd_counter = 0
        self._running     = False

    def start(self):
        log.info("=" * 50)
        log.info("  AEON X MISSION MANAGER STARTING")
        log.info("=" * 50)

        self.stm32.start(); time.sleep(0.5)
        self.gps.start()
        if CV2_AVAILABLE: self.camera.start()

        self._running = True
        self._loop()

    def _loop(self):
        log.info("[MISSION] Main loop started")
        period = 1.0 / COMMAND_RATE_HZ

        while self._running:
            t0 = time.time()
            try:
                self._tick()
            except Exception as e:
                log.error(f"[MISSION] Tick error: {e}")
                self._send("STOP")

            for alert in self.stm32.get_alerts():
                log.warning(f"[STM32 ALERT] {alert['msg']}")

            dt = time.time() - t0
            if period - dt > 0:
                time.sleep(period - dt)

    def _tick(self):
        tel = self.stm32.get_telemetry()
        imu = self.mpu.read()   if self.mpu is not None else IMUData()
        # BUG-M fix: guard tof correctly
        tof_mm = self.tof.read_mm() if self.tof is not None else 9999
        gps  = self.gps.get()
        cam  = self.camera.get()

        if (time.time() - tel.timestamp) > 3.0:
            log.warning("[MISSION] STM32 timeout")
            if self.mode != VehicleMode.DEGRADED:
                self.mode = VehicleMode.DEGRADED
                self._send("STOP")
            return

        fused = self.fusion.fuse(tel, imu, tof_mm, gps, cam)

        if self.mode == VehicleMode.AUTONOMOUS:
            dec = self.nav.decide(fused)
            self._send_if_changed(dec.value)
        elif self.mode in (VehicleMode.IDLE, VehicleMode.DEGRADED):
            self._send_if_changed("STOP")
        elif self.mode == VehicleMode.ESTOP:
            self._send("ESTOP")

        self._cmd_counter += 1
        if self._cmd_counter % 20 == 0:
            log.info(f"[FUSED] F={fused.front_cm:.0f} R={fused.rear_cm:.0f} "
                     f"L={fused.left_cm:.0f} RG={fused.right_cm:.0f} "
                     f"ToF={fused.tof_cm:.0f} P={fused.pitch:.1f} "
                     f"Obs={fused.obstacle_ahead} Mode={self.mode.name}")

    def _send(self, cmd: str):
        self.stm32.send_command(cmd)
        self._last_cmd = cmd

    def _send_if_changed(self, cmd: str):
        if cmd != self._last_cmd:
            self._send(cmd)

    def set_mode(self, mode: VehicleMode):
        with self._lock:
            log.info(f"[MISSION] Mode: {self.mode.name} → {mode.name}")
            self.mode = mode
            cmd_map = {
                VehicleMode.MANUAL:     "MANUAL",
                VehicleMode.AUTONOMOUS: "AUTO",
                VehicleMode.ESTOP:      "ESTOP",
            }
            if mode in cmd_map:
                self._send(cmd_map[mode])

    def emergency_stop(self):
        self.set_mode(VehicleMode.ESTOP)
        self._send("ESTOP")
        log.critical("[MISSION] EMERGENCY STOP")

    def stop(self):
        self._running = False
        try: self._send("STOP")
        except: pass
        self.stm32.stop()
        self.gps.stop()
        self.camera.stop()
        log.info("[MISSION] Shutdown complete")

# ============================================================
#  ENTRY POINT
# ============================================================
if __name__ == "__main__":
    import signal
    import sys

    mission = MissionManager()

    def handler(sig, frame):
        log.info("\n[AEON-X] Shutdown signal")
        try:
            mission.emergency_stop()
            time.sleep(0.5)
        except Exception: pass
        # BUG-P fix: wrap stop() in try/except
        try:
            mission.stop()
        except Exception as e:
            log.error(f"[SHUTDOWN] Error during stop: {e}")
        sys.exit(0)

    signal.signal(signal.SIGINT,  handler)
    signal.signal(signal.SIGTERM, handler)

    log.info("[AEON-X] Starting in 3 seconds... Ctrl+C to abort")
    time.sleep(3)
    mission.set_mode(VehicleMode.AUTONOMOUS)
    mission.start()
