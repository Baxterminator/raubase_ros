# Task shared data 

You have at you disposal several result and sensors data. Each one of them may be needing a requirement to be available.

The data are gathered inside the structure **SharedData** (can be found in the file `raubase_ros/plan/data/shared_data.py`) which is shared accross all tasks. On this page you can find all available values. To use them, replace `SharedData` by `self.data`.

## Sensors 

### `SharedData.encoders` 

**Type:** [`raubase_msgs/DataEncoder`](../../messages/data)

**Requires:** `Requirement.ENCODERS`
    
It contains the latest received encoders values.

### `SharedData.ir` 

**Type:** [`Dict[int, raubase_msgs/DataDistance]`](../../messages/data)

**Requires:** `Requirement.DISTANCE`
    
It contains the latest received IR sensors values. Since there could be several installed IR sensors on the robot, the informations are store in a dict where the key id the index of the sensor.

### `SharedData.last_img` 

**Type:** [`sensor_msgs/CompressedImage`](https://docs.ros2.org/latest/api/sensor_msgs/msg/CompressedImage.html)

**Requires:** `Requirement.CAMERA`
    
Contains the latest available image from the camera. Should be decrypted through the cv_bridge package to get a raw image.

### `SharedData.servos` 

**Type:** [`raubase_ros/DataServo`](../../messages/data)

**Requires:** `Requirement.SERVOS`
    
Contains the latest servos positions.

## Results

### `SharedData.odometry` 

**Type:** [`raubase_msgs/ResultOdometry`](../../messages/results)

**Requires:** `Requirement.ODOMETRY`
    
Get the latest computed and received odometry.


### `SharedData.last_yolo` 

**Type:** [`raubase_msgs/ResultYolo`](../../messages/results)

**Requires:** `Requirement.YOLO`
    
Get the latest computed and received YOLO classification.

### `SharedData.last_aruco` 

**Type:** [`raubase_msgs/ResultOdometry`](../../messages/results)

**Requires:** `Requirement.ARUCO`
    
Get the latest computed and received the latest detected ArUco codes.

## Special

### `SharedData.distance` 

**Type:** `float`

**Requires:** `Requirement.ODOMETRY`
    
Internal register computing the distance since the latest call of `SharedData.reset_distance()` method, or since the beginning of the task otherwise.

### `SharedData.time_origin` 

**Type:** `float`

**Requires:** `Requirement.NONE`
    
Origin of the time for computing duration. This register is reset at the beginning of the task and on call of the method `SharedData.reset_time()`.

### `SharedData.time_elapsed` 

**Type:** `float`

**Requires:** `Requirement.NONE`

Auto-incrementing time register since the time saved in `SharedData.time_origin`. This value we be reset on task beginning or on method call `SharedData.reset_time()`.

### `SharedData.task_time` 

**Type:** `float`

**Requires:** `Requirement.NONE`

Auto-incrementing time register since the beginning of the task.

## Methods

### `SharedData.reset_distance() -> None` 

This method reset the distance register to 0.

### `SharedData.reset_time() -> None` 

This method reset both `SharedData.time_origin` (to now) and `SharedData.time_elapsed` (to 0). Its aim is to measure actions time and do some conditions on them.
