# GNSS VINS ROS Bag Creator (Python version)

A Python tool to convert timestamp-named image files and IMU data into a ROS bag file. This utility simplifies the process of creating ROS bags from raw sensor data for robotics and autonomous systems development.

## Overview

This tool allows you to:

- Convert a folder of timestamp-named images into ROS Image messages
- Convert IMU data from CSV files into ROS IMU messages
- Convert GNSS data from CSV files into ROS NavSatFix messages
- Package all sensor data into a single, time-synchronized ROS bag file

The resulting ROS bag can be used for simulation, algorithm testing, and sensor fusion development.

## Requirements

- ROS (tested with ROS Noetic, but should work with other versions)
- Python 3.6 or higher
- OpenCV
- NumPy
- Pandas
- cv_bridge
- rosbag

## Installation

1. Clone this repository:

```bash
git clone https://github.com/RuijieXu0408/gnss_vins_rosbag_creator.git
cd gnss_vins_rosbag_creator
```

1. Make the script executable:

```bash
chmod +x bag_creator.py
```

1. Install required dependencies:

```bash
pip install numpy pandas opencv-python 
```

1. Ensure your ROS environment is set up:

```bash
source /opt/ros/noetic/setup.bash  # Replace "noetic" with your ROS distribution
```

## Data Format Requirements

### Directory Structure

Recommended data structure:

```
dataset/
├── cam0/             # Folder containing timestamp-named images
│   ├── 1705396656652587231.png
│   ├── 1705396656752587432.png
│   └── ...
├── imu/
│   └── imu0.csv      # IMU sensor data
└── gnss/
    └── gnss0.csv     # GNSS/GPS data
```

### Image Files

- Images should be named with nanosecond-precision Unix timestamps, e.g., `1705396656652587231.png`
- Supported formats: PNG, JPG, JPEG
- All images should be in the same folder

### IMU Data (CSV)

The IMU CSV file should contain the following columns:

- First column: timestamp (nanoseconds)
- `ax`, `ay`, `az`: Linear acceleration (m/s²)
- `gx`, `gy`, `gz`: Angular velocity (rad/s)
- Optional: `qw`, `qx`, `qy`, `qz`: Orientation quaternion

Example:

```
timestamp,ax,ay,az,gx,gy,gz
1705396656652587231,0.1,0.2,0.3,0.01,0.02,0.03
1705396656752587432,0.11,0.22,0.31,0.011,0.021,0.031
```

### GNSS Data (CSV)

The GNSS CSV file should contain the following columns:

- First column: timestamp (nanoseconds)
- `latitude`, `longitude`, `altitude`: Position data
- Optional: `status`, `service`: GNSS status and service information

Example:

```
timestamp,latitude,longitude,altitude
1705396656652587231,40.7128,-74.0060,10.5
1705396656752587432,40.7129,-74.0061,10.6
```

## Usage

Basic usage:

```bash
./bag_creator.py --output dataset.bag --image_folder dataset/cam0 --imu_csv dataset/imu/imu0.csv
```

Full options:

```bash
./bag_creator.py \
  --output dataset.bag \
  --image_folder dataset/cam0 \
  --image_topic /cam0/image_raw \
  --imu_csv dataset/imu/imu0.csv \
  --imu_topic /imu0 \
  --gnss_csv dataset/gnss/gnss0.csv \
  --gnss_topic /ublox_driver/receiver_lla
```

### Command-line Arguments

| Argument         | Description                              | Default                      |
| ---------------- | ---------------------------------------- | ---------------------------- |
| `--output`       | Output ROS bag file path                 | (Required)                   |
| `--image_folder` | Folder containing timestamp-named images | `/cam0/image_raw`            |
| `--image_topic`  | ROS topic name for image messages        | `/cam0/image_raw`            |
| `--imu_csv`      | Path to IMU data CSV file                | `imu0.csv`                   |
| `--imu_topic`    | ROS topic name for IMU messages          | `/imu0`                      |
| `--gnss_csv`     | Path to GNSS data CSV file               | (Optional)                   |
| `--gnss_topic`   | ROS topic name for GNSS messages         | `/ublox_driver/receiver_lla` |

## Verifying the Created ROS Bag

You can verify the contents of your newly created ROS bag using the following commands:

```bash
# Display bag information
rosbag info dataset.bag

# Play back the bag
rosbag play dataset.bag

# In another terminal, list available topics
rostopic list

# View messages on a specific topic
rostopic echo /cam0/image_raw
rostopic echo /imu0
rostopic echo /ublox_driver/receiver_lla
```

## Tips for Large Datasets

- Processing large datasets can be memory-intensive. Consider splitting the conversion into smaller batches if needed.
- For image-heavy datasets, you may want to downsample the images or process only a subset to reduce file size.
- If your original timestamps are in a different format, modify the `parse_timestamp` function accordingly.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Thanks to the ROS community for providing the tools and libraries that make this possible.
