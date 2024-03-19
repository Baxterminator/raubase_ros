# Task requirements

You will find here a complete list of the requirement that you can use for your tasks. All the requirements are contained in one enumeration **Requirements** which inherits from the **BaseRequirement** interface.

The implementation of these conditions can be found in the file `raubase_ros/plan/data/requirements.py` in the package.

You can combine the requirements with the integer (+) operator, or the binary (|)  or operator. 

## Available requirements

### `Requirement.NONE`

This value is the default value meaning nothing. If your task doesn't need anything, this is what she should return.

### `Requirement.ENCODERS`

This requirement ask for the encoders data to be available.

### `Requirement.DISTANCE`

This ask for the availability of the IR distance sensors.

### `Requirement.ODOMETRY`

Ask for the odometry results of the raubase software.

### `Requirement.CAMERA`

Ask for grabbing the camera images and information (matrix, ...).

### `Requirement.YOLO`

Ask for the result of the YOLO-base image classifier.

### `Requirement.ARUCO`

Ask for the result of the ArUco code lector.

### `Requirement.MOVE`

Ask for move functions to be initialized.

### `Requirement.LINE`

Ask for the function to follow the line to be initialized.


## Implementing new requirements

If you want to implement you own requirement, you can make a new class inheriting the **BaseRequirement** interface. You can define the enumeration value by calling the `BaseRequirement.next` method which implement a global counter to make each requirement unique. If you do not use this method, you will live with the risk of getting override by another requirement.

```Python
class MyRequirement(BaseRequirement):
    Requirement_A = BaseRequirement.next()
    Requirement_B = BaseRequirement.next()
    Requirement_C = BaseRequirement.next()
```