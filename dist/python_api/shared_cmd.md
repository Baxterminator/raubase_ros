# Task shared controls 

You have at you disposal several commands interacting with different part of the Raubase software. Each one of them may be needing a requirement to be available.

The data are gathered inside the structure **ControlWrapper** (can be found in the file `raubase_ros/plan/data/shared_control.py`) which is shared accross all tasks. On this page you can find all available commands. To use them, replace `ControlWrapper` by `self.control`.

## Movement

### `ControlWrapper.set_vel_h(velocity, heading) -> None`

Move the robot according to the given velocity and heading command.

**Requires**: `Requirement.MOVE`

**Params**: 

  - `float` velocity: the linear velocity command (in m/s)
  - `float` heading: the absolute heading the robot should have (in rad)

### `ControlWrapper.set_vel_h(velocity, turn rate) -> None`

Move the robot according to the given velocity and turn rate command.

**Requires**: `Requirement.MOVE`

**Params**: 

  - `float` velocity: the linear velocity command (in m/s)
  - `float` turn rate: the turn rate to apply (in rad/s)

### `ControlWrapper.follow_line(egde, offset) -> None`

Automatically follow the line, especially the defined edge. Apply an offset to the line center.

**Requires**: `Requirement.LINE`

**Params**: 

  - `bool` edge: if True, follow the right edge, follow the left edge otherwise
  - `float` offset: the offset at which to follow the line (in m)

### `ControlWrapper.controller_input(mode, force) -> None`

Configure the mixer to use the given mode in input.

**Requires**: `InternalRequirement.CONTROLLER` (needed by `Requirement.MOVE`)

**Params**: 

  - `str` mode: the name of the input the mixer has to switch to
  - `bool` force: should force for sending the message (prevent sending two time the same message one after another)