# Dataset Collection
These datasets were collected using the `stateLogger.py` ROS node running on the F1/10 autonomous vehicle platform. The vehicle was remotely operated at different speeds and doing different maneuvers. A sample of this collection process can be seen in this [YouTube video](https://www.youtube.com/watch?v=ZXK1WLFKrMk).
# Dataset Description
### Timestamps
All dataset data files are timestamped with seconds and nanoseconds, as they were recorded from stamped ROS messages. These timestamps are standardized to be the first two columns for all files, where the first column is the seconds of that record and the second column is the nanoseconds of that column. 

### Command Data
This csv file includes the ROS ackerman message being sent to the VESC controller to control the car. This csv file logs the following data with the following column naming:
| Column       | Description |
| ----------- | ----------- |
|         | Pandas generated index, should be ignored|
|    S    | Timestamp seconds of this record       |
|    ns   | Timestamp nanoseconds of this record        |
| V | Velocity ackerman command sent to VESC in meters per second|
| delta | Steering ackerman command sent to VESC in radians|

### IMU Data
This csv file includes the IMU data read from the VESC. This data is calibrated and can be used as is for different IMU estimates of orientations and accelerations. This csv file logs the following data with the following column naming:
| Column       | Description |
| ----------- | ----------- |
|         | Pandas generated index, should be ignored|
|    S    | Timestamp seconds of this record       |
|    ns   | Timestamp nanoseconds of this record        |
| phi_r | Euler roll angle in degrees|
| phi_p | Euler pitch angle in degrees|
| phi_y | Euler yaw angle in degrees|
| ax | Longitudinal (x-axis) acceleration in meters per second squared|
| ay | Lateral (y-axis) acceleration in meters per second squared|
| az | Vertical (z-axis) acceleration in meters per second squared|
| wx | Longitudinal (x-axis) angular frequency in radians per second|
| wy | Lateral (y-axis) angular frequency in radians per second|
| wz | Vertical (z-axis) angular frequency in radians per second|
| q.x | Orientation quaternion x component|
| q.y | Orientation quaternion y component|
| q.z | Orientation quaternion z component|
| q.w | Orientation quaternion w component|

### PF Odometry Data
This csv file includes the odometry data induced from the particle filter. This csv file logs the following data with the following column naming:
| Column       | Description |
| ----------- | ----------- |
|         | Pandas generated index, should be ignored|
|    S    | Timestamp seconds of this record       |
|    ns   | Timestamp nanoseconds of this record        |
| vx | Longitudinal (x-axis) velocity in meters per second|
| vy | Lateral (y-axis) velocity in meters per second|
| vz | Vertical (z-axis) velocity in meters per second|
| wx | Longitudinal (x-axis) angular frequency in radians per second|
| wy | Lateral (y-axis) angular frequency in radians per second|
| wz | Vertical (z-axis) angular frequency in radians per second|
| x | Longitudinal (x-axis) position in meters|
| y | Lateral (y-axis) position in meters|
| q.x | Orientation quaternion x component|
| q.y | Orientation quaternion y component|
| q.z | Orientation quaternion z component|
| q.w | Orientation quaternion w component|

### Odometry Data
This csv file includes the odometry data induced from ROS. This csv file logs the following data with the following column naming:
| Column       | Description |
| ----------- | ----------- |
|         | Pandas generated index, should be ignored|
|    S    | Timestamp seconds of this record       |
|    ns   | Timestamp nanoseconds of this record        |
| vx | Longitudinal (x-axis) velocity in meters per second|
| vy | Lateral (y-axis) velocity in meters per second|
| vz | Vertical (z-axis) velocity in meters per second|
| wx | Longitudinal (x-axis) angular frequency in radians per second|
| wy | Lateral (y-axis) angular frequency in radians per second|
| wz | Vertical (z-axis) angular frequency in radians per second|
| x | Longitudinal (x-axis) position in meters|
| y | Lateral (y-axis) position in meters|
| q.x | Orientation quaternion x component|
| q.y | Orientation quaternion y component|
| q.z | Orientation quaternion z component|
| q.w | Orientation quaternion w component|

### PF Pose Data
This csv file includes the localized position induced from the particle filter. This csv file logs the following data with the following column naming:
| Column       | Description |
| ----------- | ----------- |
|         | Pandas generated index, should be ignored|
|    S    | Timestamp seconds of this record       |
|    ns   | Timestamp nanoseconds of this record        |
| x | Longitudinal (x-axis) position in meters|
| y | Lateral (y-axis) position in meters|
| q.x | Orientation quaternion x component|
| q.y | Orientation quaternion y component|
| q.z | Orientation quaternion z component|
| q.w | Orientation quaternion w component|

## Changes to Datasets:
Datasets 1 and 2 include only the csv files mentioned thus far. Datasets 3 through 5 include additional csv files that were added after realizing their need. The changes are the following:
1. **LIDAR Scan Data:** Added to datasets 3 and 4 in an effort to later perform Friction-SLAM. Ultimately not used in this code, but left here for future use.
2. **Sensor Core Data:** Added to dataset 5. This was introduced upon further inspection of filter performance in dataset 3. The messages sent via ackerman messages to the VESC do NOT necessarily translate to actual speed by the VESC. This depends on a host of factors including transmission slip, control delay, as well as VESC saturation. As such, we use the PF's speed estimate for datasets 1 through 4, while we use the motor speed inferred from the motor's RPM for dataset 5.

### Scan Data:
This csv file includes the Hakuyo LIDAR scans recorded through ROS. Notably, the logger does not include the intensities measurements as they are not logged by the Hakuyo LIDAR. If your LIDAR does log this information to ROS, you will need to change `scan_callback` starting at line 85 in `stateLogger.py`. Reach out through discussions for help on this topic. This csv file logs the following data with the following column naming:
| Column       | Description |
| ----------- | ----------- |
|         | Pandas generated index, should be ignored|
|    S    | Timestamp seconds of this record       |
|    ns   | Timestamp nanoseconds of this record        |
| amin | minimum angle of the LIDAR in radians|
| amax | maximum angle of the LIDAR in radians|
| ai | angle increment of the LIDAR in radians|
| ti | time increment of the LIDAR in seconds|
| st | scan time of the LIDAR in seconds|
| rmin | maximuim range of the LIDAR in meters|
| rmax | minimum range of the LIDAR in meters|
| r0...r1080 | LIDAR range measurements corresponding to each index of the scan|

### Core Data:
This csv file includes the raw sensor data read from the VESC. Not all information from the sensors raw message from ROS were recorded. Only those that could be relevant to our estimator were recorded. Feel free to modify  `core_callback` starting at line 48 in `stateLogger.py` to include all other information in the message. Reach out through discussions for help on this topic. This csv file logs the following data with the following column naming:
| Column       | Description |
| ----------- | ----------- |
|         | Pandas generated index, should be ignored|
|    S    | Timestamp seconds of this record       |
|    ns   | Timestamp nanoseconds of this record        |
| cM | Average motor current in Amps|
| cI | Average input current in Amps|
| DC | Duty cycle (0 to 1)|
| RPM| Motor electrical speed in RPM|
| VI | Input voltage in volts|
| ED | Energy drawn from input in watt-hour|
| ER | Energy regenerated to input in watt-hour|
