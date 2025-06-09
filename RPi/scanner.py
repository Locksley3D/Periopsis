from rplidar import RPLidar, RPLidarException
import serial
from time import sleep
import csv
import os
import pandas as pd
import numpy as np

LIDAR_PORT = '/dev/ttyUSB0'   # LIDAR USB port
PICO_PORT = '/dev/serial0'    # RP2040 UART

CSV_FILE = 'scan_data.csv'    # Output CSV file
PLY_FILE = 'point_cloud.ply'  # Output PLY file

debug_scan = False

def main():
    lidar = RPLidar(LIDAR_PORT)
    lidar.stop()
    lidar.stop_motor()

    # Prepare serial connection
    ser = serial.Serial(port=PICO_PORT, baudrate=115200, timeout=1)
    ser.write("3,0,100,100,\n".encode('utf-8'))  # Cyan LEDs for idle

    step_interval = input('Enter time between steps(recommended 2-15,More time = Higher resolution and longer scan time)\n')
    scan_type = input('Enter sweep type(1 - Half sweep, 2 - Half sweep)\n')
    cycle_multiplier = input('Enter scan cycles(>1)\n')
    
    # Prepare CSV
    with open(CSV_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Rotation', 'Angle (deg)', 'Distance (mm)'])

    ser.write("2,\n".encode('utf-8'))  # Zero encoder
    lidar.clear_input()
    lidar.start_motor()
    sleep(1)
    ser.write("3,0,255,0,\n".encode('utf-8'))  # Green LEDs
    sleep(0.25)
    ser.write(("1," + cycle_multiplier + "," + scan_type + "," + step_interval + ",\n").encode('utf-8'))  # Start scan
    print("Starting 3D scan with sweep type "+scan_type+", "+step_interval+" ms between steps and " + cycle_multiplier +"scan cycles")

    clearBuffer(ser)

    with open(CSV_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)

        while True:
            try:
                measurement_iter = lidar.iter_measurments()

                for measurement in measurement_iter:
                    if ser.in_waiting:
                        while ser.in_waiting:
                            try:
                                line = ser.readline().decode('utf-8', errors='ignore').strip()
                            except UnicodeDecodeError:
                                continue
                        if ',' not in line:
                            continue
                        header, data = line.split(",", 1)
                        if header == "2":
                            print("Scanning done")
                            raise StopIteration  # clean exit

                    _, quality, angle, distance = measurement
                    if quality > 0:
                        try:
                            rotation = int(data)
                            if debug_scan:
                                print(f"Rotation: {rotation/10} | Angle: {angle:.1f}° | Distance: {distance:.1f} mm")
                            writer.writerow([rotation/10, f"{angle:.1f}", f"{distance:.1f}"])
                        except ValueError:
                            pass

            except RPLidarException as e:
                print(f"Bad measurement skipped: {e}")
                lidar.clear_input()
                sleep(0.1)
                continue

            except StopIteration:
                break

    ser.write("3,0,100,100,\n".encode('utf-8'))  # Cyan LEDs for idle
    lidar.stop_motor()
    lidar.stop()
    lidar.disconnect()

    # Convert CSV to PLY
    print("Converting CSV to PLY...")
    process_csv_and_export_ply(CSV_FILE, PLY_FILE)
    print(f"PLY file created: {PLY_FILE}")

def clearBuffer(ser):
    while ser.in_waiting:
        try:
            ser.readline().decode('utf-8', errors='ignore').strip()
        except UnicodeDecodeError:
            continue

def polar_to_cartesian(rotation_deg, angle_deg, distance_mm):
    """Convert spherical coordinates to Cartesian (in mm)"""
    rot_rad = np.radians(rotation_deg)  # elevation
    ang_rad = np.radians(angle_deg)     # azimuth (lidar scan angle)
    
    x = distance_mm * np.sin(rot_rad) * np.cos(ang_rad)
    y = distance_mm * np.sin(rot_rad) * np.sin(ang_rad)
    z = distance_mm * np.cos(rot_rad)
    return x, y, z

def write_ply(filename, xs, ys, zs):
    """Write 3D point cloud data to a PLY file"""
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(xs)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for x, y, z in zip(xs, ys, zs):
            f.write(f"{x:.3f} {y:.3f} {z:.3f}\n")

def process_csv_and_export_ply(csv_path, ply_path):
    # Load CSV
    df = pd.read_csv(csv_path)
    df.columns = [col.strip().lower() for col in df.columns]

    rotation_col = [col for col in df.columns if "rotation" in col][0]
    angle_col = [col for col in df.columns if "angle" in col][0]
    distance_col = [col for col in df.columns if "distance" in col][0]

    df = df.astype({rotation_col: float, angle_col: float, distance_col: float})

    xs, ys, zs = [], [], []
    for _, row in df.iterrows():
        x, y, z = polar_to_cartesian(row[angle_col], row[rotation_col], row[distance_col])
        xs.append(x)
        ys.append(y)
        zs.append(z)

    write_ply(ply_path, xs, ys, zs)

if __name__ == '__main__':
    main()
