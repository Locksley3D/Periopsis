from rplidar import RPLidar, RPLidarException
import serial
from time import sleep
import csv
import os

LIDAR_PORT = '/dev/ttyUSB0'   # LIDAR USB port
PICO_PORT = '/dev/serial0'    # RP2040 UART

CSV_FILE = 'scan_data.csv'    # Output CSV file

def main():
    lidar = RPLidar(LIDAR_PORT)
    lidar.stop()
    lidar.stop_motor()

    # Prepare CSV file: create or empty if it exists
    ser = serial.Serial(port=PICO_PORT, baudrate=115200, timeout=1)

    ser.write("3,0,100,100,\n".encode('utf-8'))  # Cyan LEDs for idle

    cmd = input('Enter command\n')
    if cmd == "1":
        print("here1")
        
        with open(CSV_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Rotation', 'Angle (deg)', 'Distance (mm)'])

        ser.write("2,\n".encode('utf-8'))  # Zero encoder
        lidar.clear_input()
        lidar.start_motor()
        sleep(1)
        ser.write("3,0,255,0,\n".encode('utf-8'))  # Green LEDs
        sleep(0.25)
        ser.write("1,1,1,10,\n".encode('utf-8'))  # Start scan
        print("Starting 3D scan")

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
                                print(f"Rotation: {rotation/10} | Angle: {angle:.1f}° | Distance: {distance:.1f} mm")
                                writer.writerow([rotation/10, f"{angle:.1f}", f"{distance:.1f}"])
                            except ValueError:
                                pass  # malformed rotation data

                except RPLidarException as e:
                    print(f"Bad measurement skipped: {e}")
                    lidar.clear_input()
                    sleep(0.1)  # allow hardware to recover
                    continue  # restart scan

                except StopIteration:
                    break  # End scan

        ser.write("3,0,100,100,\n".encode('utf-8'))  # Cyan LEDs for idle
        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()

def clearBuffer(ser):
    while ser.in_waiting:
        try:
            ser.readline().decode('utf-8', errors='ignore').strip()
        except UnicodeDecodeError:
            continue

if __name__ == '__main__':
    main()
