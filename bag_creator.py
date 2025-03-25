#!/usr/bin/env python3

import os
import cv2
import rospy
import rosbag
import argparse
import numpy as np
import pandas as pd
from cv_bridge import CvBridge
from datetime import datetime
import sensor_msgs.msg
from sensor_msgs.msg import Image, Imu, NavSatFix
from geometry_msgs.msg import Vector3
import tf.transformations
from std_msgs.msg import Header


def parse_timestamp(filename):
    """Extract timestamp from filename and convert to seconds"""
    # Assume filenames are named with nanosecond timestamps
    timestamp_ns = int(os.path.splitext(os.path.basename(filename))[0])
    return timestamp_ns


def create_header(timestamp_ns, frame_id):
    """Create ROS message header"""
    header = Header()
    #  Convert nanoseconds to ROS time (seconds and nanoseconds)
    header.stamp.secs = timestamp_ns // 1000000000
    header.stamp.nsecs = timestamp_ns % 1000000000
    header.frame_id = frame_id
    return header


def process_images(bag, image_folder, topic_name, frame_id="camera"):
    """Process image files and add to rosbag"""
    bridge = CvBridge()
    image_files = sorted([f for f in os.listdir(image_folder) if f.endswith(('.png', '.jpg', '.jpeg'))])
    
    print(f"Processed image files, total: {len(image_files)}")
    
    for i, img_file in enumerate(image_files):
        if i % 100 == 0:
            print(f"Processing image {i}/{len(image_files)}")
            
        img_path = os.path.join(image_folder, img_file)
        timestamp_ns = parse_timestamp(img_file)
        
        # Read image
        cv_img = cv2.imread(img_path)
        if cv_img is None:
            print(f"Warning: Unable to read image {img_path}, skipping")
            continue
            
        # Convert OpenCV image to ROS image message
        img_msg = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        img_msg.header = create_header(timestamp_ns, frame_id)
        
        # Write image message to bag
        bag.write(topic_name, img_msg, rospy.Time(img_msg.header.stamp.secs, img_msg.header.stamp.nsecs))


def process_imu_data(bag, imu_csv_path, topic_name, frame_id="imu"):
    """Process IMU data and add to rosbag"""
    # Read IMU CSV data
    imu_data = pd.read_csv(imu_csv_path)
    
    print(f"Process IMU data, total: {len(imu_data)} records")
    
    # Assume IMU data format is: timestamp, ax, ay, az, gx, gy, gz, [optional: qw, qx, qy, qz]
    # Adjust column names according to your data format
    timestamp_col = imu_data.columns[0]
    
    for i, (_, row) in enumerate(imu_data.iterrows()):
        if i % 1000 == 0:
            print(f"Processing IMU data {i}/{len(imu_data)}")
            
        timestamp_ns = int(row[timestamp_col])
        
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header = create_header(timestamp_ns, frame_id)
        
        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = float(row.get('ax', 0.0))
        imu_msg.linear_acceleration.y = float(row.get('ay', 0.0))
        imu_msg.linear_acceleration.z = float(row.get('az', 0.0))
        
        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = float(row.get('gx', 0.0))
        imu_msg.angular_velocity.y = float(row.get('gy', 0.0))
        imu_msg.angular_velocity.z = float(row.get('gz', 0.0))
        
        # If there is quaternion orientation data
        if all(col in imu_data.columns for col in ['qw', 'qx', 'qy', 'qz']):
            imu_msg.orientation.w = float(row['qw'])
            imu_msg.orientation.x = float(row['qx'])
            imu_msg.orientation.y = float(row['qy'])
            imu_msg.orientation.z = float(row['qz'])
        else:
            # If there is no quaternion, set it to unit quaternion
            imu_msg.orientation.w = 1.0
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
        
        # Set covariance (if unsure, set to a larger value)
        imu_msg.orientation_covariance = [0.01] * 9
        imu_msg.angular_velocity_covariance = [0.01] * 9
        imu_msg.linear_acceleration_covariance = [0.01] * 9
        
        # Write IMU message to bag
        bag.write(topic_name, imu_msg, rospy.Time(imu_msg.header.stamp.secs, imu_msg.header.stamp.nsecs))

def process_gnss_data(bag, gnss_csv_path, topic_name, frame_id="gnss"):
    """Process GNSS data and add to rosbag"""
    # Read GNSS CSV data
    gnss_data = pd.read_csv(gnss_csv_path)
    
    print(f"Process GNSS data, total: {len(gnss_data)} records")
    
    # Assume GNSS data is in the format of: timestamp, latitude, longitude, altitude, [optional: status, service]
    # Adjust column names to your data format
    timestamp_col = gnss_data.columns[0]
    timestamp_col = gnss_data.columns[0]
    
    for i, (_, row) in enumerate(gnss_data.iterrows()):
        if i % 100 == 0:
            print(f"Process GNSS data {i}/{len(gnss_data)}")
            
        timestamp_ns = int(row[timestamp_col])
        
        # Create NavSatFix message
        gnss_msg = NavSatFix()
        gnss_msg.header = create_header(timestamp_ns, frame_id)
        
        # Set position
        gnss_msg.latitude = float(row.get('latitude', 0.0))
        gnss_msg.longitude = float(row.get('longitude', 0.0))
        gnss_msg.altitude = float(row.get('altitude', 0.0))
        
        # Set status and service (adjust to your data)
        gnss_msg.status.status = int(row.get('status', 0))  # Defaults to STATUS_NO_FIX
        gnss_msg.status.service = int(row.get('service', 1))  # Defaults to SERVICE_GPS
        
        # Set the covariance (if you are unsure, you can set it to a larger value)
        gnss_msg.position_covariance = [0.0] * 9
        gnss_msg.position_covariance[0] = 10.0  # East-West variance
        gnss_msg.position_covariance[4] = 10.0  # North-South variance
        gnss_msg.position_covariance[8] = 10.0  # Altitude variance
        gnss_msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        # Write GNSS message to bag
        bag.write(topic_name, gnss_msg, rospy.Time(gnss_msg.header.stamp.secs, gnss_msg.header.stamp.nsecs))

def main():
    parser = argparse.ArgumentParser(description='Convert sensor data to ROS bag file')
    parser.add_argument('--output', type=str, required=True, help='Output bag file path')
    parser.add_argument('--image_folder', type=str, default='./cam0/image_raw', help='Folder containing images named with timestamps')
    parser.add_argument('--image_topic', type=str, default='/cam0/image_raw', help='Image topic name')
    parser.add_argument('--imu_csv', type=str, default='imu0.csv', help='IMU data CSV file path')
    parser.add_argument('--imu_topic', type=str, default='/imu0', help='IMU topic name')
    # parser.add_argument('--gnss_csv', type=str, help='GNSS data CSV file path')
    # parser.add_argument('--gnss_topic', type=str, default='/ublox_driver/receiver_lla', help='GNSS topic name')
    
    args = parser.parse_args()
    
    # Create bag file
    print(f"Create bag file: {args.output}")
    with rosbag.Bag(args.output, 'w') as bag:
        # Process image data
        if args.image_folder and os.path.exists(args.image_folder):
            process_images(bag, args.image_folder, args.image_topic)
        elif args.image_folder:
            print(f"Warning: Image folder {args.image_folder} does not exist")
        
        # Process IMU data
        if args.imu_csv and os.path.exists(args.imu_csv):
            process_imu_data(bag, args.imu_csv, args.imu_topic)
        elif args.imu_csv:
            print(f"Warning: IMU data file {args.imu_csv} does not exist")
    
    print(f"Bag file created: {args.output}")


if __name__ == '__main__':
    main()