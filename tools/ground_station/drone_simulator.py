#!/usr/bin/env python3
"""
Drone-H743 High-Fidelity Flight Telemetry Simulator & Test Server for Serial-Studio.

Simulates:
- 3D Attitude (Roll, Pitch, Yaw, Quaternions, Angular Rates)
- 3-Axis Accelerometer (Static 1g + Flight Dynamics + Motor Vibration Harmonics)
- Barometer SPL06 (Altitude, Atmospheric Pressure, Vertical Velocity, Temperature)
- GPS M9N (Latitude, Longitude, Altitude, Ground Speed, Satellites, 3D Fix, HDOP)
- Magnetometer (Heading 0-360 deg)
- Power & System Status (Battery Voltage 3S/4S, Current, Capacity, Armed Status, CPU Load)
- Coaxial Motors & 4x Control Servos Output
- Two-way Interactive Command Handler (PING, ARM, DISARM, STATUS, CALIB, SERVO TEST)

Outputs standard Serial Studio CSV telemetry frames:
  /*$ROLL,PITCH,YAW,ROLL_RATE,PITCH_RATE,YAW_RATE,AX,AY,AZ,ALT,PRESS,VVEL,LAT,LON,GPS_ALT,SPEED,SATS,FIX,HEADING,VBAT,CURR,CAP,CPU,ARMED,M1,M2,S1,S2,S3,S4*/
"""

from __future__ import annotations

import math
import random
import socket
import sys
import threading
import time

DEFAULT_TCP_PORT = 6666
DEFAULT_UDP_PORT = 6668
UPDATE_RATE_HZ = 50.0

# Base Coordinates (Campus / Flight Field)
HOME_LAT = 31.230416
HOME_LON = 121.473701
HOME_ALT = 15.0

class DroneSimulator:
    def __init__(self, tcp_port: int = DEFAULT_TCP_PORT, udp_port: int = DEFAULT_UDP_PORT):
        self.tcp_port = tcp_port
        self.udp_port = udp_port
        self.running = False
        
        # State
        self.armed = False
        self.flight_mode = "MANUAL"
        self.battery_voltage = 12.4
        self.consumed_mah = 0.0
        
        # Flight variables
        self.time_elapsed = 0.0
        self.altitude = 0.0
        self.target_alt = 10.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        self.heading = 0.0
        
        # Servos & Motors
        self.m1_pwm = 0.0
        self.m2_pwm = 0.0
        self.s1_pos = 0.0
        self.s2_pos = 0.0
        self.s3_pos = 0.0
        self.s4_pos = 0.0
        
        # GPS
        self.lat = HOME_LAT
        self.lon = HOME_LON
        self.gps_alt = HOME_ALT
        self.ground_speed = 0.0
        self.sats = 18
        self.fix_type = 2 # 3D Fix

    def update_physics(self, dt: float):
        self.time_elapsed += dt
        t = self.time_elapsed

        if self.armed:
            # Flight mode dynamics
            self.m1_pwm = 55.0 + 8.0 * math.sin(t * 1.2) + random.uniform(-1.0, 1.0)
            self.m2_pwm = 54.0 + 8.0 * math.sin(t * 1.2 + 0.5) + random.uniform(-1.0, 1.0)
            
            # Smooth roll/pitch/yaw motion
            self.roll = 12.0 * math.sin(t * 0.8) + 2.0 * math.cos(t * 2.5)
            self.pitch = 8.0 * math.cos(t * 0.6) + 1.5 * math.sin(t * 2.1)
            self.yaw = (t * 10.0) % 360.0
            
            self.roll_rate = 12.0 * 0.8 * math.cos(t * 0.8) * 57.3 / 57.3
            self.pitch_rate = -8.0 * 0.6 * math.sin(t * 0.6)
            self.yaw_rate = 10.0
            
            # Altitude climb & hover
            if self.altitude < self.target_alt:
                self.altitude += dt * 1.5
            else:
                self.altitude = self.target_alt + 0.8 * math.sin(t * 0.5)
            self.vvel = 0.8 * 0.5 * math.cos(t * 0.5) if self.altitude >= self.target_alt else 1.5
            
            # GPS circular loiter
            radius = 0.00015
            self.lat = HOME_LAT + radius * math.cos(t * 0.1)
            self.lon = HOME_LON + radius * math.sin(t * 0.1)
            self.gps_alt = HOME_ALT + self.altitude
            self.ground_speed = 18.5 + 3.0 * math.sin(t * 0.3)
            
            # Servos active deflection
            self.s1_pos = -self.pitch * 1.2 + self.roll * 0.8 + random.uniform(-0.5, 0.5)
            self.s2_pos = self.pitch * 1.2 + self.roll * 0.8 + random.uniform(-0.5, 0.5)
            self.s3_pos = -self.roll * 1.2 + random.uniform(-0.5, 0.5)
            self.s4_pos = self.roll * 1.2 + random.uniform(-0.5, 0.5)
            
            # Battery discharge
            current = 14.5 + (self.m1_pwm + self.m2_pwm) * 0.15
            self.battery_voltage = max(10.2, 12.4 - (t * 0.003) - (current * 0.015))
            self.consumed_mah += (current * dt / 3.6)
        else:
            self.m1_pwm = 0.0
            self.m2_pwm = 0.0
            self.altitude = 0.0
            self.vvel = 0.0
            self.roll = 0.0 + random.uniform(-0.2, 0.2)
            self.pitch = 0.0 + random.uniform(-0.2, 0.2)
            self.yaw = (t * 1.0) % 360.0
            self.roll_rate = 0.0
            self.pitch_rate = 0.0
            self.yaw_rate = 0.0
            self.s1_pos = 0.0
            self.s2_pos = 0.0
            self.s3_pos = 0.0
            self.s4_pos = 0.0
            self.lat = HOME_LAT
            self.lon = HOME_LON
            self.gps_alt = HOME_ALT
            self.ground_speed = 0.0
            self.battery_voltage = 12.4
            current = 0.45

        self.current = current
        self.heading = self.yaw
        
        # Accelerometer simulation (gravity vector + motor vibration noise)
        vib_freq = 120.0 # 120 Hz motor harmonic
        vib_amp = 0.8 if self.armed else 0.02
        
        rad_roll = math.radians(self.roll)
        rad_pitch = math.radians(self.pitch)
        
        g = 9.80665
        self.ax = -g * math.sin(rad_pitch) + vib_amp * math.sin(t * vib_freq) + random.gauss(0, 0.05)
        self.ay = g * math.sin(rad_roll) * math.cos(rad_pitch) + vib_amp * math.cos(t * vib_freq) + random.gauss(0, 0.05)
        self.az = g * math.cos(rad_roll) * math.cos(rad_pitch) + vib_amp * math.sin(t * (vib_freq * 2)) + random.gauss(0, 0.05)
        
        # Barometer pressure (hypsometric formula: P = P0 * (1 - L*h/T0)^(g*M/(R*L)))
        p0 = 1013.25
        self.pressure = p0 * ((1.0 - (0.0065 * self.altitude) / 288.15) ** 5.255)
        self.temperature = 24.5 + 0.5 * math.sin(t * 0.1)

    def generate_csv_frame(self) -> str:
        """Build standard Serial-Studio CSV telemetry frame."""
        fields = [
            f"{self.roll:.2f}",
            f"{self.pitch:.2f}",
            f"{self.yaw:.2f}",
            f"{self.roll_rate:.2f}",
            f"{self.pitch_rate:.2f}",
            f"{self.yaw_rate:.2f}",
            f"{self.ax:.3f}",
            f"{self.ay:.3f}",
            f"{self.az:.3f}",
            f"{self.altitude:.2f}",
            f"{self.pressure:.2f}",
            f"{self.vvel:.2f}",
            f"{self.lat:.6f}",
            f"{self.lon:.6f}",
            f"{self.gps_alt:.2f}",
            f"{self.ground_speed:.2f}",
            f"{self.sats}",
            f"{self.fix_type}",
            f"{self.heading:.1f}",
            f"{self.battery_voltage:.2f}",
            f"{self.current:.2f}",
            f"{self.consumed_mah:.1f}",
            f"{42.0 + 3.0 * math.sin(self.time_elapsed * 0.4):.1f}", # CPU load %
            f"{1 if self.armed else 0}",
            f"{self.m1_pwm:.1f}",
            f"{self.m2_pwm:.1f}",
            f"{self.s1_pos:.1f}",
            f"{self.s2_pos:.1f}",
            f"{self.s3_pos:.1f}",
            f"{self.s4_pos:.1f}",
        ]
        return "/*$" + ",".join(fields) + "*/\n"

    def handle_command(self, cmd: str) -> str:
        cmd = cmd.strip().upper()
        if not cmd:
            return ""
        
        if cmd == "PING":
            return "PONG ok=1\n"
        elif cmd == "ARM":
            self.armed = True
            return "OK ARM: Motors armed, control active\n"
        elif cmd == "DISARM":
            self.armed = False
            return "OK DISARM: Motors disarmed, safe\n"
        elif cmd == "STATUS?":
            return f"STATUS armed={1 if self.armed else 0} vbat={self.battery_voltage:.2f} alt={self.altitude:.2f} mode={self.flight_mode}\n"
        elif cmd.startswith("CALIB"):
            return f"OK {cmd}: Calibration complete, offsets saved to flash\n"
        elif cmd.startswith("SERVO"):
            return "OK SERVO: Test sequence finished. All 4 channels normal.\n"
        elif cmd == "REBOOT":
            self.armed = False
            self.time_elapsed = 0.0
            return "READY system_boot=OK cfg_valid=1\n"
        else:
            return f"ACK: {cmd}\n"

    def run_tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind(("0.0.0.0", self.tcp_port))
            server.listen(5)
            server.settimeout(1.0)
            print(f"[Simulator] TCP Telemetry Server listening on port {self.tcp_port}...")
        except Exception as e:
            print(f"[Simulator] Could not bind TCP port {self.tcp_port}: {e}")
            return

        while self.running:
            try:
                client, addr = server.accept()
                print(f"[Simulator] Ground station client connected from {addr}")
                client.settimeout(0.1)
                
                dt = 1.0 / UPDATE_RATE_HZ
                last_time = time.time()
                
                while self.running:
                    now = time.time()
                    elapsed = now - last_time
                    if elapsed >= dt:
                        last_time = now
                        self.update_physics(elapsed)
                        frame = self.generate_csv_frame()
                        try:
                            client.sendall(frame.encode("utf-8"))
                        except Exception:
                            break
                    
                    # Receive incoming commands
                    try:
                        data = client.recv(1024)
                        if data:
                            lines = data.decode("utf-8", errors="ignore").splitlines()
                            for line in lines:
                                reply = self.handle_command(line)
                                if reply:
                                    client.sendall(reply.encode("utf-8"))
                    except socket.timeout:
                        pass
                    except Exception:
                        break
                    
                    time.sleep(0.005)
                
                client.close()
                print(f"[Simulator] Client {addr} disconnected")
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[Simulator] Server loop note: {e}")
                time.sleep(0.5)

        server.close()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run_tcp_server, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False


if __name__ == "__main__":
    port = DEFAULT_TCP_PORT
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    
    sim = DroneSimulator(tcp_port=port)
    sim.start()
    print("===================================================================")
    print(" 🚁 Drone-H743 Flight Control Telemetry Simulator Running")
    print(f" -> TCP Port: {port}")
    print(" -> Real-time 50Hz Attitude, Accel, Baro, GPS, Servos & Commands")
    print(" Press Ctrl+C to stop.")
    print("===================================================================")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sim.stop()
        print("\nSimulator stopped.")
