"""Export raw GPS, IMU and wheel speed from the ROS 2 bag to CSV for the C++ filter.

Uses bag receive time and /imu/imu_uncompensated (same as the original Python version),
but nothing is converted or filtered: NaN fixes are kept, no frame flip,
no bias removal. That all happens in C++.

Usage (from the project root):
    python3 tools/export_csv.py path/to/my_ros2_bag_data.db3
Needs a sourced ROS 2 install (rosbag2_py). Writes data/gps.csv, data/imu.csv, data/wheel.csv.
"""
import csv
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

if len(sys.argv) != 2:
    sys.exit(__doc__)
BAG = sys.argv[1]
OUT = 'data'
GPS_TOPIC = '/gps/fix'
IMU_TOPIC = '/imu/imu_uncompensated'
TWIST_TOPIC = '/vehicle/twist'   # Dataspeed DBW: linear.x = wheel speed (m/s), 50 Hz
TOPICS = (GPS_TOPIC, IMU_TOPIC, TWIST_TOPIC)

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                        output_serialization_format='cdr'))
reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
# Only resolve our types: the bag also has dbw_mkz_msgs, which isn't installed.
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
         if t.name in TOPICS}

os.makedirs(OUT, exist_ok=True)
n_gps = n_imu = n_twist = 0
with open(f'{OUT}/gps.csv', 'w', newline='') as fg, open(f'{OUT}/imu.csv', 'w', newline='') as fi, \
        open(f'{OUT}/wheel.csv', 'w', newline='') as fw:
    gps = csv.writer(fg)
    imu = csv.writer(fi)
    wheel = csv.writer(fw)
    gps.writerow(['t', 'lat', 'lon', 'alt'])
    imu.writerow(['t', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])
    wheel.writerow(['t', 'v', 'wz'])
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, types[topic])
        t = f'{stamp * 1e-9:.9f}'  # absolute seconds; keep ns precision
        if topic == GPS_TOPIC:
            gps.writerow([t, repr(msg.latitude), repr(msg.longitude), repr(msg.altitude)])
            n_gps += 1
        elif topic == TWIST_TOPIC:
            tw = msg.twist
            wheel.writerow([t, repr(tw.linear.x), repr(tw.angular.z)])
            n_twist += 1
        else:
            a, w = msg.linear_acceleration, msg.angular_velocity
            imu.writerow([t] + [repr(v) for v in (a.x, a.y, a.z, w.x, w.y, w.z)])
            n_imu += 1

print(f'Wrote {n_gps} GPS, {n_imu} IMU and {n_twist} wheel-speed rows to {OUT}/')
